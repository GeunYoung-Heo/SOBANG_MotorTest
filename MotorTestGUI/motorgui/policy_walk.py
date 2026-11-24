"""정책 보행: SOBANG_Locomotion의 실물 보행 정책(policies/real_walk)을 Radxa에서 돌리고 GUI에서 시작·명령·정지한다.

정책 계산과 50 Hz 루프는 Radxa의 중계 안에서 돈다 (radxa/walk_policy.py → RL 세션의 run_walk_test()).
GUI는 관절 변환값(순서·부호·영점)과 옵션을 보내고, 상태(10 Hz)를 받아 보여 주고 기록한다.

순서 (RL 절차서 S2 → S3):
  1. 보행 시작: 4 s 동안 지금 자세에서 서기 자세로 이동 → 드라이버 PD로 유지 (로봇은 매단 상태)
  2. 발이 바닥에 닿으면 '정책 켜기': 직전 행동 0으로 정책 시작, 20 ms마다. 켜는 순간부터 설정한 속도로 앞으로 걷는다
  3. 속도: GUI는 앞으로만 걷게 한다 (vx 0~0.5 m/s, 기본 0.1, vy = wz = 0). 걷는 중에 바꾸면 바로 Radxa로 간다
  4. 멈춤 / Esc / 모두 힘 풀기: 정책이 바로 끝나고 모든 관절 IDLE. Radxa의 기록(.npz)을 logs/policy_walk_*/로 가져옴
Radxa 쪽 안전 정지: 몸통 기울기 20° 초과(정책 중), 드라이버 오류·heartbeat 끊김·엔코더 응답 없음·IMU 끊김,
10 ms 넘게 늦은 주기 5번 연속, GUI 연결 끊김·0.5 s 무응답.

토크 한계 6 N·m (체크박스, 기본 켬): ODrive 0.5.1은 CAN으로 전류 한계를 못 바꿔서(실측 Iq 명령 18.00 A),
Radxa가 보내기 직전에 목표를 잘라 드라이버 PD 토크를 ±6 N·m 안에 둔다 (radxa/walk_policy.py BridgeIO.clamp, 근사).

기록 (logs/policy_walk_<시각>/): meta.json, events.txt, status.csv, real_walk.npz·real_walk_io.npz(Radxa),
gui_frames.csv, gui_log.txt. 분석: python analysis/policy_walk_report.py (README 12절).
"""
import csv
import json
import math
import queue
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

from . import protocol as P
from .config import GUI_ROOT

POLICY_YAML = GUI_ROOT / "radxa" / "policies" / "real_walk" / "policy.yaml"
LOG_DIR = GUI_ROOT / "logs"
COMMAND_LIMITS = (0.5, 0.25, 1.0)          # 학습 범위 (real_walk.py COMMAND_LIMITS)
FORWARD_MAX = 0.5                          # GUI는 앞으로만 걷게 함: vx 0~0.5 m/s, vy = wz = 0 (2026-11-24 사용자 요청)
FORWARD_DEFAULT = 0.1                      # 천천히 (RL 절차서 S4의 첫 단계)
TORQUE_LIMIT_NM = 6.0                      # 학습 토크 한계 (관절)
STATE_KO = {"idle": "꺼짐", "starting": "시작 중", "ramp": "서기 자세로 이동 중", "hold": "서기 자세 유지 · 정책 대기",
            "walk": "정책 보행 중", "stopped": "멈춤", "error": "시작 못 함"}


def read_policy_yaml(path=POLICY_YAML):
    """policy.yaml에서 joints와 default_joint_positions만 읽는다 (GUI에 PyYAML이 없어도 되게)."""
    joints, stand, key = [], [], None
    for raw in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw.rstrip()
        if not line or line.lstrip().startswith("#"):
            continue
        if not line.startswith((" ", "-")):
            key = line.split(":", 1)[0].strip()
            continue
        item = line.strip()
        if item.startswith("- "):
            if key == "joints":
                joints.append(item[2:].strip())
            elif key == "default_joint_positions":
                stand.append(float(item[2:]))
    return joints, stand


def read_policy_gains(path=POLICY_YAML):
    """policy.yaml의 joint_kp, joint_kd (관절 PD, 드라이버 게인과 같음)."""
    out, key = {"joint_kp": [], "joint_kd": []}, None
    for raw in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw.rstrip()
        if not line or line.lstrip().startswith("#"):
            continue
        if not line.startswith((" ", "-")):
            key = line.split(":", 1)[0].strip()
            continue
        if key in out and line.strip().startswith("- "):
            out[key].append(float(line.strip()[2:]))
    return out["joint_kp"], out["joint_kd"]


class FrameRecorder:
    """정책 보행 중 GUI가 받은 CAN 프레임 가운데 분석에 쓸 것을 gui_frames.csv로 남긴다. 새 요청은 보내지 않는다.

    정책 중에도 GUI는 Iq(관절마다 10 Hz, 선택한 관절 50 Hz)·Vbus(2 s)를 요청하고 heartbeat를 받는다. 엔코더는
    Radxa 루프의 요청에 대한 응답이 그대로 들어온다 (50 Hz × 12). CAN 스레드(CanLink._handle)는 줄을 큐에
    넣기만 하고, 파일 쓰기는 이 클래스의 스레드가 한다 (CAN 수신을 막지 않게).
    kind: enc (pos turn, vel turn/s), iq (Iq_setpoint, Iq_measured A), hb (axis_error, axis_state), vbus (V),
          motor_err, enc_err.  t_gui = App.now() (status.csv, events.txt와 같은 시계)
    """

    def __init__(self, path):
        self.q = queue.SimpleQueue()
        self.f = open(path, "w", encoding="utf-8", newline="")
        self.f.write("t_gui,node,kind,a,b\n")
        self.closed = False
        threading.Thread(target=self._run, daemon=True, name="policy-frames").start()

    def tap(self, now, node, cmd, data):
        """CAN 스레드에서 프레임마다 불림."""
        if self.closed:
            return
        cs = P.cs
        if cmd == cs.GET_ENCODER_ESTIMATES and len(data) >= 8:
            kind, (a, b) = "enc", P.parse_two_floats(data)
        elif cmd == cs.GET_IQ and len(data) >= 8:
            kind, (a, b) = "iq", P.parse_two_floats(data)
        elif cmd == cs.HEARTBEAT and len(data) >= 5:
            kind, (a, b) = "hb", P.parse_heartbeat(data)
        elif cmd == cs.GET_VBUS and len(data) >= 4:
            kind, a, b = "vbus", P.parse_float(data), ""
        elif cmd == cs.GET_MOTOR_ERROR and len(data) >= 4:
            kind, a, b = "motor_err", P.parse_motor_error(data), ""
        elif cmd == cs.GET_ENCODER_ERROR and len(data) >= 4:
            kind, a, b = "enc_err", P.parse_u32(data), ""
        else:
            return
        self.q.put(f"{now:.4f},{node},{kind},{a},{b}\n")

    def close(self):
        self.closed = True
        self.q.put(None)

    def _run(self):
        t_flush = time.monotonic()
        try:
            while True:
                try:
                    line = self.q.get(timeout=1.0)
                except queue.Empty:
                    line = ""
                if line is None:
                    break
                if line:
                    self.f.write(line)
                if time.monotonic() - t_flush > 1.0:
                    self.f.flush()
                    t_flush = time.monotonic()
        finally:
            self.f.close()


class PolicyWalk:
    def __init__(self, app):
        self.app = app
        self.lock = threading.Lock()
        self.state = "idle"
        self.status = {}
        self.reason = ""
        self.joint_names, self.standing = read_policy_yaml()
        self.kp, self.kd = read_policy_gains()
        self.command = [FORWARD_DEFAULT, 0.0, 0.0]
        self.torque_clamp = True                # 토크 한계 6 N·m (Radxa 목표 자르기)
        self.remote_io_log = None
        self.floor_t = None
        self.t_start = None
        self.events = deque(maxlen=300)
        self.log_dir = None
        self.remote_log = None
        self.local_log = None
        self.fetch_note = ""
        self._csv = None
        self._csv_f = None
        self._frames = None                 # FrameRecorder (정책 보행 중 + 멈춘 뒤 2 s)

    # ------------------------------------------------------------------ 조회
    @property
    def active(self):
        return self.state in ("starting", "ramp", "hold", "walk")

    def target_q(self):
        """정책 목표 {관절 이름: rad} (3D의 목표 겹침용), 없으면 None."""
        q = self.status.get("target_q") if self.active else None
        if not q or len(q) != len(self.joint_names):
            return None
        return dict(zip(self.joint_names, q))

    def _event(self, text):
        t = self.app.now()
        self.events.append((t, text))
        self.app.log(f"정책 보행: {text}")
        if self.log_dir is not None:
            with open(self.log_dir / "events.txt", "a", encoding="utf-8") as f:
                f.write(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] (t_gui {t:.3f}) {text}\n")

    # ------------------------------------------------------------------ 기록 보조 (gui_frames.csv, gui_log.txt)
    def _start_frames(self):
        old, self._frames = self._frames, None
        if old is not None:
            old.close()
        try:
            self._frames = FrameRecorder(self.log_dir / "gui_frames.csv")
        except OSError as e:
            self.app.log(f"정책 보행: gui_frames.csv를 만들지 못함 ({e})")
            return
        self.app.link.frame_tap = self._frames.tap

    def _end_frames(self, rec, log_dir, t_start):
        """멈춘 뒤 2 s에 부름 (그 사이 heartbeat·에러 응답도 남김)."""
        if rec is not None:
            if self.app.link.frame_tap == rec.tap:
                self.app.link.frame_tap = None
            rec.close()
            if self._frames is rec:
                self._frames = None
        self._save_gui_log(log_dir, t_start)

    def _save_gui_log(self, log_dir, t_start):
        """GUI '기록' 탭의 줄(GUI·CAN·[Radxa]) 중 이번 실행 것을 gui_log.txt로."""
        if log_dir is None:
            return
        rows = [(t, "GUI", s) for t, s in list(self.app.logs)] + [(t, "CAN", s) for t, s in list(self.app.link.logs)]
        t_from = -math.inf if t_start is None else t_start - 2.0
        rows = sorted(r for r in rows if r[0] >= t_from)
        try:
            with open(log_dir / "gui_log.txt", "w", encoding="utf-8") as f:
                for t, src, s in rows:
                    f.write(f"{t:10.3f} [{src}] {s}\n")
        except OSError:
            pass

    # ------------------------------------------------------------------ 시작 전 점검
    def preflight(self):
        app, link, cfg = self.app, self.app.link, self.app.cfg
        probs = []
        if self.active:
            return ["이미 정책 보행 중입니다"]
        if not link.connected or link.radxa is None:
            return ["Radxa 경유로 연결해야 합니다 (정책은 Radxa에서 돕니다)"]
        pol = link.radxa.info.get("policy")
        if pol is None:
            probs.append("Radxa 중계가 정책 보행을 모릅니다 (끊고 다시 연결하면 새 파일이 복사됨)")
        else:
            if pol.get("missing"):
                probs.append(f"Radxa에 없는 패키지: {', '.join(pol['missing'])} (README 12절)")
            if not pol.get("files"):
                probs.append("Radxa에 정책 파일이 없습니다 (끊고 다시 연결)")
        if app.walk.active:
            probs.append("규칙 기반 보행 테스트 중입니다")
        if app.sweep_active:
            probs.append("가동범위 기록 중입니다")
        now = app.now()
        names = set(cfg.names)
        if set(self.joint_names) != names:
            probs.append("정책 관절 이름이 설정과 다릅니다")
            return probs
        bad = {"응답 없음": [], "영점 없음": [], "방향 미확인": [], "토크 켜짐": [], "에러": []}
        for name in self.joint_names:
            j = cfg.joint(name)
            ns = link.nodes[j.node]
            if not ns.online(now) or ns.pos is None or link.bus_of(j.node) is None:
                bad["응답 없음"].append(j.short)
                continue
            if not j.calibrated:
                bad["영점 없음"].append(j.short)
            if not j.dir_checked:
                bad["방향 미확인"].append(j.short)
            if ns.state == P.STATE_CLOSED_LOOP or app.cmdr.get(name) is not None:
                bad["토크 켜짐"].append(j.short)
            if ns.axis_error:
                bad["에러"].append(j.short)
        for k, v in bad.items():
            if v:
                hint = {"토크 켜짐": " (먼저 '모두 힘 풀기')", "방향 미확인": " ('영점·방향 설정' 탭)",
                        "에러": " ('명령' 탭의 에러 지우기)"}.get(k, "")
                probs.append(f"{k}: {', '.join(v)}{hint}")
        s = link.imu.snapshot()
        if s["state"] != "ok" or s["age"] is None or s["age"] > 0.3:
            probs.append(f"IMU가 '받는 중'이 아닙니다 ({s['state']}; 'IMU 자세' 탭)")
        return probs

    def notes(self):
        """시작을 막지는 않는 주의 사항."""
        out = []
        q = self.app.link.imu.fresh_q()
        if q is not None:
            from .imu import gravity_body
            g = gravity_body(q)
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, -g[2]))))
            if tilt > 10.0:
                out.append(f"몸통이 {tilt:.0f}° 기울어 있습니다. 정책 중 20°를 넘으면 Radxa가 멈춥니다")
        if not self.torque_clamp:
            out.append("토크 한계 6 N·m가 꺼져 있습니다: 드라이버가 18 A(무릎·고관절 약 18 N·m, 발목 약 23 N·m)까지 내서 "
                       "학습 조건과 다르고, 12 A SMPS가 버티지 못할 수 있습니다")
        return out

    # ------------------------------------------------------------------ 명령
    def _send(self, d):
        conn = self.app.link.radxa
        if conn is None:
            return "Radxa 연결이 없습니다"
        return conn.policy_send(d)

    def start(self):
        probs = self.preflight()
        if probs:
            return "시작할 수 없습니다: " + " / ".join(probs)
        app, link, cfg = self.app, self.app.link, self.app.cfg
        joints = []
        for name in self.joint_names:
            j = cfg.joint(name)
            ns = link.nodes[j.node]
            joints.append({"name": name, "node": j.node, "bus": link.bus_of(j.node), "sign": j.direction,
                           "zero_turns": j.zero_turn - ns.k})
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_dir = LOG_DIR / f"policy_walk_{stamp}"
        self.log_dir.mkdir(parents=True, exist_ok=True)
        wall, t_gui = datetime.now(), app.now()
        meta = {"start": wall.isoformat(timespec="seconds"), "start_ms": wall.isoformat(timespec="milliseconds"),
                "t_gui_at_start_ms": round(t_gui, 4), "policy": "SOBANG_Locomotion policies/real_walk",
                "command_at_start": list(self.command), "gui_selected_node": link.selected_node,
                "torque_limit_6Nm": self.torque_clamp,
                "torque_clamp": ({"method": "radxa_target", "limit_nm": TORQUE_LIMIT_NM, "kp": self.kp, "kd": self.kd}
                                 if self.torque_clamp else None),
                "drive_current_lim_A": 18.0, "joints": joints, "standing_pose_rad": self.standing,
                "radxa": link.radxa.info, "imu_axes": cfg.imu_axes, "imu_bus": cfg.imu_i2c_bus}
        (self.log_dir / "meta.json").write_text(json.dumps(meta, indent=1, ensure_ascii=False), encoding="utf-8")
        self._csv_f = open(self.log_dir / "status.csv", "w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._csv_f)
        self._csv.writerow(["t_gui", "phase", "cycles", "read_ms", "period_ms", "tilt_deg", "imu_age_ms", "vx", "vy", "wz",
                            "torque_clamp", "clamped_joints", "clamp_frac", "miss"]
                           + [f"target_{n}" for n in self.joint_names])
        self.status, self.reason, self.fetch_note = {}, "", ""
        self.remote_log = self.local_log = self.remote_io_log = None
        self.floor_t = None
        self.t_start = app.now()
        with self.lock:
            self.state = "starting"
        link.passive_encoder = True
        self._start_frames()
        self._event(f"시작 요청 (앞으로 {self.command[0]:.2f} m/s, 토크 한계 6 N·m "
                    f"{'켬 (Radxa 목표 자르기)' if self.torque_clamp else '끔 (드라이버 18 A)'}): "
                    + ", ".join(f"{cfg.joint(j['name']).short} 부호 {j['sign']:+d} 영점 {j['zero_turns']:.4f}"
                                for j in joints))
        err = self._send({"cmd": "start", "joints": joints, "torque_clamp": self.torque_clamp,
                          "limit_nm": TORQUE_LIMIT_NM, "command": list(self.command),
                          "log_name": f"real_walk_{stamp}"})
        if err:
            self._finish("error", err)
        return err

    def floor(self):
        if self.state != "hold":
            return "서기 자세를 유지하는 중에만 정책을 켭니다"
        self.floor_t = self.app.now()
        self._event("정책 켜기 (발이 바닥에 닿음)")
        return self._send({"cmd": "floor"})

    def set_forward(self, vx):
        """앞으로 걷는 속도 [m/s], 0~0.5. 옆·회전은 0 (GUI는 앞으로만 걷게 함)."""
        v = [max(0.0, min(FORWARD_MAX, round(float(vx), 3))), 0.0, 0.0]
        if v == self.command:
            return None
        self.command = v
        if not self.active or self.state == "starting":     # 시작 명령에 넣어 보냄 (Radxa가 아직 준비 전일 수 있음)
            return None
        self._event(f"앞으로 걷는 속도 {v[0]:.2f} m/s")
        return self._send({"cmd": "command", "v": v})

    def stop(self, why="GUI에서 멈춤"):
        if not self.active:
            return None
        self._event(f"멈춤 요청: {why}")
        return self._send({"cmd": "stop", "why": why})

    def on_idle_all(self, reason):
        """Esc·모두 힘 풀기: GUI의 IDLE과 함께 정책 멈춤도 보낸다 (중계는 IDLE 프레임만 봐도 멈춤)."""
        if self.active:
            self.stop(reason)

    def on_disconnect(self):
        if self.active:
            self._finish("stopped", "연결 끊김 (Radxa가 정책을 멈추고 힘을 풂)")

    # ------------------------------------------------------------------ Radxa에서 (수신 스레드)
    def on_message(self, d):
        kind = d.get("type")
        if kind == "started":
            self.remote_log = d.get("log")
            with self.lock:
                self.state = "ramp"
            self._event("Radxa에서 시작: 서기 자세로 4 s 이동")
        elif kind == "status":
            self.status = d
            phase = d.get("phase")
            with self.lock:
                if self.state in ("starting", "ramp", "hold", "walk") and phase in ("ramp", "hold", "walk"):
                    if phase != self.state:
                        self.state = phase
                        self._event({"hold": "서기 자세 도달, 유지 중 (발이 바닥에 닿으면 '정책 켜기')",
                                     "walk": "정책 보행 시작", "ramp": "서기 자세로 이동"}[phase])
            if self._csv is not None:
                c = d.get("command") or [0, 0, 0]
                self._csv.writerow([f"{self.app.now():.3f}", phase, d.get("n"), d.get("read_ms"), d.get("period_ms"),
                                    d.get("tilt"), d.get("imu_age_ms"), *c, d.get("torque_clamp"),
                                    d.get("clamped_joints"), d.get("clamp_frac"), d.get("miss")]
                                   + list(d.get("target_q") or [""] * 12))
        elif kind == "floor":
            pass
        elif kind == "stopped":
            self.remote_log = d.get("log") or self.remote_log
            self.remote_io_log = d.get("io_log")
            if d.get("sent"):
                self._event(f"목표를 자른 주기 {d.get('clamp_cycles', 0)} / {d.get('sent')}")
            self._finish("stopped", d.get("reason") or "")
            if d.get("log"):
                threading.Thread(target=self._fetch_log, daemon=True, name="policy-log").start()
        elif kind == "error":
            msg = d.get("msg") or "오류"
            if self.state == "starting":
                self._finish("error", msg)
            else:
                self._event(f"⚠ {msg}")

    def _finish(self, state, reason):
        with self.lock:
            was = self.state
            self.state = state
            self.reason = reason
        self.app.link.passive_encoder = False
        if was != state or reason:
            self._event(("멈춤: " if state == "stopped" else "시작 못 함: ") + reason)
            if state == "stopped" and was in ("hold", "walk", "ramp"):
                self.app.link.alerts.append(f"정책 보행 멈춤: {reason}")
        if self._csv_f is not None:
            self._csv_f.close()
            self._csv_f, self._csv = None, None
        # radxa-rx 스레드에서 불릴 수 있으므로 기다리지 않고 Timer로: 멈춘 뒤 2 s까지 프레임을 남기고 닫음
        tm = threading.Timer(2.0, self._end_frames, args=(self._frames, self.log_dir, self.t_start))
        tm.daemon = True
        tm.start()

    def _fetch_log(self):
        log_dir, t_start = self.log_dir, self.t_start
        conn = self.app.link.radxa
        if conn is None or log_dir is None or not self.remote_log:
            self.fetch_note = "Radxa 기록을 가져오지 못했습니다 (연결 없음)"
            return
        dst = log_dir / "real_walk.npz"
        err = conn.fetch_file(self.remote_log, dst)
        if err:
            self.fetch_note = f"Radxa 기록을 가져오지 못했습니다: {err} (Radxa {self.remote_log})"
        else:
            self.local_log = dst
            self.fetch_note = f"Radxa 기록 → logs/{log_dir.name}/real_walk.npz ({dst.stat().st_size // 1024} KB)"
        if self.remote_io_log:                      # 보낸 목표(자른 목표) 기록
            err_io = conn.fetch_file(self.remote_io_log, log_dir / "real_walk_io.npz")
            self.fetch_note += " + real_walk_io.npz" if not err_io else f" (real_walk_io.npz 실패: {err_io})"
        self._event(self.fetch_note)
        self._save_gui_log(log_dir, t_start)
