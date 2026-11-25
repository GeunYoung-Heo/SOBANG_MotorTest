"""정책 보행: SOBANG_Locomotion의 실물 보행 정책을 Radxa에서 돌리고 GUI에서 시작·명령·정지한다.

정책 계산과 50 Hz 루프는 Radxa의 중계 안에서 돈다 (radxa/walk_policy.py → RL 세션의 run_walk_test()).
GUI는 정책 이름, 관절 변환값(순서·부호·영점)과 옵션을 보내고, 상태(10 Hz)를 받아 보여 주고 기록한다.

정책 (radxa/policies/<이름>/, GUI가 연결할 때 Radxa에 복사): 탭의 드롭다운으로 고른다.
  real_clamp12 / real_clamp9 / real_clamp6 (2026-11-25, Radxa 목표 자르기를 넣고 학습, 한계 = policy.yaml torque_clamp_nm)
  real_walk (2026-11-24 시험한 정책, 드라이버 6 N·m로 학습, 참고용)
RL 절차서의 권장 순서: real_clamp12부터. 버스 전압이 21.5 V 아래로 떨어지거나 SMPS가 흔들리면 9 → 6으로 내림.

순서 (RL 절차서 S2 → S3):
  1. 보행 시작: 4 s 동안 지금 자세에서 서기 자세로 이동 → 드라이버 PD로 유지 (로봇은 매단 상태)
  2. 발이 바닥에 닿으면 '정책 켜기': 직전 행동 0으로 정책 시작, 20 ms마다. 켜는 순간부터 설정한 속도로 앞으로 걷는다
     켜기 전에 하네스를 느슨하게 하고 IMU 앞 숙임이 2° 안인지 본다 (넘으면 GUI가 한 번 더 묻는다)
  3. 속도: GUI는 앞으로만 걷게 한다 (vx 0~0.5 m/s, 기본 0.1, vy = wz = 0). 걷는 중에 바꾸면 바로 Radxa로 간다
  4. 멈춤 / Esc / 모두 힘 풀기: 정책이 바로 끝나고 모든 관절 IDLE. Radxa의 기록(.npz)을 logs/policy_walk_*/로 가져옴
안전 정지 (2026-11-25 RL 절차서 기준):
  Radxa: 몸통 기울기 30° 초과(정책 중, 전에는 20°), 관절 추종 오차 |보낸 목표 − 엔코더| 30° 초과 0.5 s,
         드라이버 오류·heartbeat 끊김·엔코더 응답 없음·IMU 끊김, 10 ms 넘게 늦은 주기 5번 연속, GUI 연결 끊김·0.5 s 무응답.
  GUI:   버스 전압(Vbus)이 21.5 V 아래 → 모두 힘 풀기 (정책 중에는 Vbus를 0.5 s마다 읽음).

토크 한계: ODrive 0.5.1은 CAN으로 전류 한계를 못 바꿔서(실측 Iq 명령 18.00 A), Radxa가 보내기 직전에 목표를 잘라
드라이버 PD 토크를 ±L 안에 둔다 (radxa/walk_policy.py BridgeIO.clamp, 근사). real_clamp*는 L = 정책 값으로 항상 켬,
real_walk만 체크박스(6 N·m).

기록 (logs/policy_walk_<시각>_<정책>_<L>Nm/): meta.json, events.txt, status.csv, real_walk.npz·real_walk_io.npz(Radxa),
gui_frames.csv, gui_log.txt. 분석: python analysis/policy_walk_report.py (README 12절).
"""
import csv
import hashlib
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

POLICIES_DIR = GUI_ROOT / "radxa" / "policies"
DEFAULT_POLICY = "real_clamp12"             # RL 절차서 2026-11-25: 첫 시험은 real_clamp12
POLICY_YAML = POLICIES_DIR / "real_walk" / "policy.yaml"
LOG_DIR = GUI_ROOT / "logs"
COMMAND_LIMITS = (0.5, 0.25, 1.0)          # 학습 범위 (real_walk.py COMMAND_LIMITS)
FORWARD_MAX = 0.5                          # GUI는 앞으로만 걷게 함: vx 0~0.5 m/s, vy = wz = 0 (2026-11-24 사용자 요청)
FORWARD_DEFAULT = 0.1                      # 천천히 (RL 절차서 S4의 첫 단계)
TORQUE_LIMIT_NM = 6.0                      # real_walk(11-24)의 학습 토크 한계 (관절), 체크박스로 켜는 자르기 한계
# 안전 정지 기준 (RL 절차서 2026-11-25 '안전 정지 기준'; Radxa가 앞의 둘, GUI가 Vbus를 본다)
TILT_LIMIT_DEG = 30.0                      # 정책 중 몸통 기울기 (전에는 20°)
TRACK_LIMIT_DEG, TRACK_TIME_S = 30.0, 0.5  # 관절 추종 오차 |보낸(자른) 목표 − 엔코더|
VBUS_STOP_V = 21.5                         # 버스 전압 (절차서 '공통 중단 기준')
LEAN_OK_DEG = 2.0                          # 정책 켜기 직전 IMU 앞 숙임 (절차서 11-25 체크리스트)
STATE_KO = {"idle": "꺼짐", "starting": "시작 중", "ramp": "서기 자세로 이동 중", "hold": "서기 자세 유지 · 정책 대기",
            "walk": "정책 보행 중", "stand": "정책 끔 · 서기 자세로 돌아가는 중", "stopped": "멈춤", "error": "시작 못 함"}
ACTIVE_STATES = ("starting", "ramp", "hold", "walk", "stand")
# 드롭다운 설명 (RL 레포 docs/results/real_walk/DEPLOY.md 14절, MuJoCo 결과). 순서 = 목록 순서
POLICY_INFO = {
    "real_clamp12": ("11-25 · 토크 12 N·m · 권장 첫 시험",
                     "MuJoCo: 추종 넘어짐 0/24, 실물 비슷한 조건 20° 넘음 0/50, 걷는 중 앞 숙임 −0.4~+4.7°,\n"
                     "정책 켤 때 첫 움직임 7°, 평균 토크 1.2 / 2.4 N·m (vx 0.1 / 0.2). 셋 중 가장 강하고 첫 움직임이 가장 작음."),
    "real_clamp9": ("11-25 · 토크 9 N·m",
                    "MuJoCo: 추종 넘어짐 0/24, 20° 넘음 0/50, 걷는 중 앞 숙임 −3.1~+1.3°, 첫 움직임 13–16°,\n"
                    "평균 토크 2.5 / 3.3 N·m. 12 N·m에서 버스 전압이 21.5 V 아래로 떨어지거나 SMPS가 흔들리면 이것으로."),
    "real_clamp6": ("11-25 · 토크 6 N·m",
                    "MuJoCo: 추종 넘어짐 0/24, 20° 넘음 0/50, 걷는 중 앞 숙임 −2.4~+2.5°, 첫 움직임 13°,\n"
                    "옆걸음·회전 추종이 약함. 9 N·m 다음 단계."),
    "real_walk": ("11-24 시험한 정책 · 참고",
                  "2026-11-24 실물 시험 정책 (드라이버 6 N·m로 학습). 6 N·m 자르기에서 0.8초 만에 앞으로 넘어짐.\n"
                  "자르기는 아래 체크박스로 켜고 끔."),
}


def read_policy_scalar(path, key):
    """policy.yaml 맨 위 'key: 값' (숫자면 float). 없으면 None."""
    for raw in Path(path).read_text(encoding="utf-8").splitlines():
        if raw.startswith(key + ":"):
            v = raw.split(":", 1)[1].split("#", 1)[0].strip()
            try:
                return float(v)
            except ValueError:
                return v or None
    return None


def list_policies():
    """GUI에 있는 정책 (radxa/policies/<이름>/policy.onnx + policy.yaml): [{name, dir, clamp_nm, label, tip}].

    순서: POLICY_INFO에 적은 순서 (real_clamp12, 9, 6, real_walk), 그 밖의 폴더는 이름순으로 뒤에."""
    found = {}
    if POLICIES_DIR.is_dir():
        for d in sorted(POLICIES_DIR.iterdir()):
            if (d / "policy.onnx").is_file() and (d / "policy.yaml").is_file():
                lim = read_policy_scalar(d / "policy.yaml", "torque_clamp_nm")
                found[d.name] = {"name": d.name, "dir": d, "clamp_nm": lim if isinstance(lim, float) else None}
    order = [n for n in POLICY_INFO if n in found] + [n for n in found if n not in POLICY_INFO]
    out = []
    for n in order:
        p = found[n]
        label, tip = POLICY_INFO.get(n, ("", ""))
        if not label:
            label = f"토크 {p['clamp_nm']:g} N·m" if p["clamp_nm"] else "자르기 값 없음 (체크박스 6 N·m)"
        p["label"], p["tip"] = f"{n} — {label}", tip
        out.append(p)
    return out


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
        self.policies = list_policies()
        self.policy_name = None
        self.clamp_nm = None                    # 고른 정책의 torque_clamp_nm (real_clamp*), real_walk는 None
        names = [p["name"] for p in self.policies]
        self._load_policy(DEFAULT_POLICY if DEFAULT_POLICY in names else (names[0] if names else "real_walk"))
        self.command = [FORWARD_DEFAULT, 0.0, 0.0]
        self.torque_clamp = True                # real_walk용 체크박스 (토크 한계 6 N·m, Radxa 목표 자르기)
        self.started = {}                       # Radxa가 'started'로 알린 실제 설정 (정책, 한계, 기울기 정지 …)
        self.vbus_min = None                    # 이번 실행에서 읽은 가장 낮은 버스 전압 (node, V)
        self.lean_at_floor = None
        self._vbus_tripped = False
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

    # ------------------------------------------------------------------ 정책 고르기
    def _load_policy(self, name):
        path = POLICIES_DIR / name / "policy.yaml"
        self.joint_names, self.standing = read_policy_yaml(path)
        self.kp, self.kd = read_policy_gains(path)
        lim = read_policy_scalar(path, "torque_clamp_nm")
        self.clamp_nm = lim if isinstance(lim, float) and lim > 0 else None
        self.policy_name = name

    def select_policy(self, name):
        """드롭다운에서 고름. 정책 보행 중에는 못 바꿈. 오류 메시지 또는 None."""
        if name == self.policy_name:
            return None
        if self.active:
            return "정책 보행 중에는 정책을 바꿀 수 없습니다"
        if name not in [p["name"] for p in self.policies]:
            return f"정책 {name}이 없습니다 (radxa/policies/)"
        try:
            self._load_policy(name)
        except (OSError, ValueError) as e:
            return f"정책 {name}을 읽지 못했습니다: {e}"
        self.app.log(f"정책 보행: 정책 {name} 고름 (토크 한계 {self.limit_nm:g} N·m"
                     f"{'' if self.trained_clamp else ', 체크박스'})")
        return None

    @property
    def trained_clamp(self):
        """정책이 목표 자르기를 넣고 학습했는가 (real_clamp*): 그러면 자르기는 항상 켬, 한계는 정책 값."""
        return self.clamp_nm is not None

    @property
    def clamp_on(self):
        return True if self.trained_clamp else bool(self.torque_clamp)

    @property
    def limit_nm(self):
        return self.clamp_nm if self.trained_clamp else TORQUE_LIMIT_NM

    def policy_info(self):
        return next((p for p in self.policies if p["name"] == self.policy_name), None)

    # ------------------------------------------------------------------ 조회
    @property
    def active(self):
        return self.state in ACTIVE_STATES

    def lean(self):
        """IMU 자세로 본 몸통 (앞 숙임°, 오른쪽 기울기°, 전체 기울기°). IMU 값이 없으면 None.

        앞으로 숙이면 몸통 좌표의 중력 x > 0, 오른쪽으로 기울면 y < 0 (RL 절차서 4절)."""
        q = self.app.link.imu.fresh_q()
        if q is None:
            return None
        from .imu import gravity_body
        g = gravity_body(q)
        fwd = math.degrees(math.asin(max(-1.0, min(1.0, g[0]))))
        right = math.degrees(math.asin(max(-1.0, min(1.0, -g[1]))))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -g[2]))))
        return fwd, right, tilt

    def floor_check(self):
        """'정책 켜기' 전에 물어볼 것 (RL 절차서: 하네스 느슨히, 정책 직전 IMU 앞 숙임 2° 안). 없으면 None."""
        ln = self.lean()
        if ln is None:
            return "IMU 자세 값이 없습니다. 앞 숙임을 확인할 수 없습니다"
        if abs(ln[0]) > LEAN_OK_DEG:
            return (f"IMU 앞 숙임이 {ln[0]:+.1f}°입니다 (기준 {LEAN_OK_DEG:g}° 안). 하네스가 체중을 받치거나 "
                    "몸통이 기울어 있을 수 있습니다. 하네스를 느슨하게 하고 몸통을 세운 뒤 켜는 것이 좋습니다")
        return None

    def tick(self):
        """App.update()마다: 버스 전압 감시 (RL 절차서 공통 중단 기준 21.5 V). 정책 중 한 드라이버라도 넘으면 모두 힘 풀기."""
        if not self.active or self.t_start is None:
            return
        link, cfg = self.app.link, self.app.cfg
        low = None
        for name in self.joint_names:
            ns = link.nodes.get(cfg.joint(name).node)
            if ns is None or ns.vbus is None or ns.t_vbus < self.t_start:
                continue
            if self.vbus_min is None or ns.vbus < self.vbus_min[1]:
                self.vbus_min = (ns.node, float(ns.vbus))
            if ns.vbus < VBUS_STOP_V and (low is None or ns.vbus < low[1]):
                low = (ns.node, float(ns.vbus))
        if low is not None and not self._vbus_tripped:
            self._vbus_tripped = True               # 'stopped'가 올 때까지 매 프레임 다시 부르지 않게
            self.app.idle_all(f"버스 전압 {low[1]:.1f} V (node {low[0]}) < {VBUS_STOP_V:g} V")

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
            remote = pol.get("policies")
            if remote is None:
                probs.append("Radxa 중계가 옛 판이라 정책을 고를 수 없습니다 (끊고 다시 연결하면 새 판이 복사됨)")
            else:
                r = next((p for p in remote if p.get("name") == self.policy_name), None)
                if r is None:
                    probs.append(f"Radxa에 정책 {self.policy_name}이 없습니다 (끊고 다시 연결하면 복사됨)")
                elif r.get("torque_clamp_nm") != self.clamp_nm:
                    probs.append(f"Radxa의 {self.policy_name} 자르기 한계({r.get('torque_clamp_nm')})가 GUI({self.clamp_nm})와 "
                                 "다릅니다 (끊고 다시 연결)")
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
        ln = self.lean()
        if ln is not None and ln[2] > 10.0:
            out.append(f"몸통이 {ln[2]:.0f}° 기울어 있습니다. 정책 중 {TILT_LIMIT_DEG:g}°를 넘으면 Radxa가 멈춥니다")
        if not self.clamp_on:
            out.append("토크 한계 6 N·m가 꺼져 있습니다: 드라이버가 18 A(무릎·고관절 약 18 N·m, 발목 약 23 N·m)까지 내서 "
                       "학습 조건과 다르고, 12 A SMPS가 버티지 못할 수 있습니다")
        if self.policy_name == "real_walk":
            out.append("real_walk는 11-24에 시험한 정책입니다. 11-25 RL 절차서의 시험 정책은 real_clamp12 / 9 / 6입니다")
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
        name, lim, clamp_on = self.policy_name, self.limit_nm, self.clamp_on
        tag = f"{name}_{lim:g}Nm" if clamp_on else f"{name}_noclamp"     # RL 절차서: 기록 이름에 정책과 limit_nm
        self.log_dir = LOG_DIR / f"policy_walk_{stamp}_{tag}"
        self.log_dir.mkdir(parents=True, exist_ok=True)
        wall, t_gui = datetime.now(), app.now()
        onnx = POLICIES_DIR / name / "policy.onnx"
        meta = {"start": wall.isoformat(timespec="seconds"), "start_ms": wall.isoformat(timespec="milliseconds"),
                "t_gui_at_start_ms": round(t_gui, 4), "policy": f"SOBANG_Locomotion policies/{name}",
                "policy_name": name, "policy_onnx_sha256": hashlib.sha256(onnx.read_bytes()).hexdigest()[:16],
                "policy_trained_clamp_nm": self.clamp_nm,
                "command_at_start": list(self.command), "gui_selected_node": link.selected_node,
                "torque_limit_6Nm": clamp_on and lim == TORQUE_LIMIT_NM,   # 옛 키 (11-24 기록과 같은 뜻)
                "torque_clamp": ({"method": "radxa_target", "limit_nm": lim, "kp": self.kp, "kd": self.kd,
                                  "source": "policy.yaml torque_clamp_nm" if self.trained_clamp else "GUI 체크박스"}
                                 if clamp_on else None),
                "stop_limits": {"tilt_deg": TILT_LIMIT_DEG, "track_deg": TRACK_LIMIT_DEG, "track_s": TRACK_TIME_S,
                                "vbus_v": VBUS_STOP_V, "lean_ok_deg": LEAN_OK_DEG},
                "drive_current_lim_A": 18.0, "joints": joints, "standing_pose_rad": self.standing,
                "radxa": link.radxa.info, "imu_axes": cfg.imu_axes, "imu_bus": cfg.imu_i2c_bus}
        (self.log_dir / "meta.json").write_text(json.dumps(meta, indent=1, ensure_ascii=False), encoding="utf-8")
        self._csv_f = open(self.log_dir / "status.csv", "w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._csv_f)
        self._csv.writerow(["t_gui", "phase", "cycles", "read_ms", "period_ms", "tilt_deg", "imu_age_ms", "vx", "vy", "wz",
                            "torque_clamp", "clamped_joints", "clamp_frac", "miss", "limit_nm", "track_err_deg",
                            "vbus_min_v"]
                           + [f"target_{n}" for n in self.joint_names])
        self.status, self.reason, self.fetch_note, self.started = {}, "", "", {}
        self.remote_log = self.local_log = self.remote_io_log = None
        self.floor_t = self.lean_at_floor = self.vbus_min = None
        self._vbus_tripped = False
        self.t_start = app.now()
        with self.lock:
            self.state = "starting"
        link.passive_encoder = True
        self._start_frames()
        self._event(f"시작 요청 (정책 {name}, 앞으로 {self.command[0]:.2f} m/s, "
                    + (f"토크 한계 {lim:g} N·m 켬 (Radxa 목표 자르기"
                       + (", 정책 학습 값)" if self.trained_clamp else ")") if clamp_on else "토크 한계 끔 (드라이버 18 A)")
                    + f", 정지: 기울기 {TILT_LIMIT_DEG:g}°·추종 오차 {TRACK_LIMIT_DEG:g}° {TRACK_TIME_S:g} s·"
                      f"Vbus {VBUS_STOP_V:g} V): "
                    + ", ".join(f"{cfg.joint(j['name']).short} 부호 {j['sign']:+d} 영점 {j['zero_turns']:.4f}"
                                for j in joints))
        err = self._send({"cmd": "start", "policy": name, "joints": joints, "torque_clamp": clamp_on,
                          "limit_nm": lim, "tilt_limit_deg": TILT_LIMIT_DEG, "command": list(self.command),
                          "log_name": f"real_walk_{stamp}_{tag}"})
        if err:
            self._finish("error", err)
        return err

    def floor(self):
        if self.state != "hold":
            return "서기 자세를 유지하는 중에만 정책을 켭니다"
        self.floor_t = self.app.now()
        ln = self.lean()
        self.lean_at_floor = ln
        self._event("정책 켜기 (발이 바닥에 닿음" + (f", IMU 앞 숙임 {ln[0]:+.1f}° 오른쪽 {ln[1]:+.1f}°" if ln else
                                                 ", IMU 자세 없음") + ")")
        return self._send({"cmd": "floor"})

    def stand(self):
        """제자리에 서기: 정책을 끄고 1 s 동안 서기 자세로 돌아가 드라이버 PD로 유지 (힘은 풀지 않음).
        다시 걸으려면 '정책 켜기'. 서기 자세 유지 중에도 기울기 30° 정지는 켜져 있다."""
        if self.state != "walk":
            return "정책 보행 중에만 '제자리에 서기'를 합니다"
        self._event("제자리에 서기 요청 (정책 끄고 서기 자세로 1 s, 드라이버 PD로 유지)")
        return self._send({"cmd": "stand"})

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
            self.started = d
            with self.lock:
                self.state = "ramp"
            got = (d.get("policy"), d.get("torque_clamp"), d.get("limit_nm"), d.get("tilt_limit_deg"))
            want = (self.policy_name, self.clamp_on, self.limit_nm, TILT_LIMIT_DEG)
            self._event("Radxa에서 시작: 서기 자세로 4 s 이동"
                        + (f" (정책 {got[0]}, 토크 한계 {'%g N·m' % got[2] if got[1] else '끔'}, 기울기 정지 {got[3]}°, "
                           f"추종 오차 정지 {d.get('track_limit_deg')}° {d.get('track_time_s')} s)" if got[0] else
                           " (Radxa가 정책 이름을 알리지 않음: 옛 중계)"))
            if got[0] and got != want:
                self._event(f"⚠ Radxa 설정이 GUI와 다릅니다: Radxa {got}, GUI {want}")
        elif kind == "status":
            self.status = d
            phase = d.get("phase")
            with self.lock:
                if self.state in ACTIVE_STATES and phase in ("ramp", "hold", "walk", "stand"):
                    if phase != self.state:
                        self.state = phase
                        self._event({"hold": "서기 자세 도달, 유지 중 (발이 바닥에 닿으면 '정책 켜기')",
                                     "walk": "정책 보행 시작", "ramp": "서기 자세로 이동",
                                     "stand": "정책 끔, 서기 자세로 돌아가는 중 (1 s)"}[phase])
            if self._csv is not None:
                c = d.get("command") or [0, 0, 0]
                vb = self.vbus_min
                self._csv.writerow([f"{self.app.now():.3f}", phase, d.get("n"), d.get("read_ms"), d.get("period_ms"),
                                    d.get("tilt"), d.get("imu_age_ms"), *c, d.get("torque_clamp"),
                                    d.get("clamped_joints"), d.get("clamp_frac"), d.get("miss"), d.get("limit_nm"),
                                    d.get("track_err_deg"), "" if vb is None else f"{vb[1]:.2f}"]
                                   + list(d.get("target_q") or [""] * 12))
        elif kind in ("floor", "stand"):
            pass
        elif kind == "stopped":
            self.remote_log = d.get("log") or self.remote_log
            self.remote_io_log = d.get("io_log")
            if d.get("sent"):
                vb = self.vbus_min
                self._event(f"목표를 자른 주기 {d.get('clamp_cycles', 0)} / {d.get('sent')}, 추종 오차 최대 "
                            f"{d.get('track_err_max_deg', '?')}°, 버스 전압 최저 "
                            + (f"{vb[1]:.2f} V (node {vb[0]})" if vb else "읽지 못함"))
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
            if state == "stopped" and was in ("hold", "walk", "ramp", "stand"):
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
