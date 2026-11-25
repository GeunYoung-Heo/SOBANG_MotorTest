"""정책 보행 기록 분석 (PC): MotorTestGUI/logs/policy_walk_*/ 를 읽어 보고서(report.md)·요약(summary.json)·그림을 만든다.

실행 (PowerShell):
  & "$env:USERPROFILE\\odrv051\\Scripts\\Activate.ps1"
  cd "$env:USERPROFILE\\Desktop\\ASMR_ActuatorTest\\MotorTestGUI"
  python analysis\\policy_walk_report.py --list                    # 기록 목록과 한 줄 요약
  python analysis\\policy_walk_report.py                           # 가장 최근 기록 하나
  python analysis\\policy_walk_report.py logs\\policy_walk_20261125_101500 [다른 폴더 ...]
  python analysis\\policy_walk_report.py --all                     # 모든 기록
  python analysis\\policy_walk_report.py --fetch <폴더>            # real_walk.npz가 없으면 Radxa에서 가져옴 (ssh radxa)
결과: <기록 폴더>\\analysis\\report.md, summary.json, overview.png, joints.png, torque.png, last_seconds.png, spectrum.png

읽는 파일 (있는 것만): real_walk.npz (Radxa run_walk_test 매 주기; target = 정책이 낸 목표), status.csv (10 Hz 상태),
events.txt, meta.json (torque_clamp가 있으면 Radxa 목표 자르기), gui_frames.csv (GUI가 받은 Iq·Vbus·heartbeat·엔코더 응답),
gui_log.txt (GUI '기록' 탭), real_walk_io.npz (BridgeIO가 보낸 목표 target_sent·clamped, 맞춤용 t_send·t_read·pre_loop;
있으면 추종 오차·PD 토크를 보낸 목표로 계산하고 잘림 비율을 보고).
토크: ODrive 0.5.1은 CAN으로 current_lim을 못 바꾸므로 드라이버 한계는 18 A(--drive-current-a)로 보고, 학습 한계 6 N·m를
넘은 비율을 따로 센다.
판정 기준은 RL 세션의 "첫 실물 보행 시험" 절차서 (S2 서기 자세 1°, S3 기울기 5°·6 N·m 1 s, 공통 30 ms 등)와
같은 절차서에 적힌 시뮬 보행 값(목표−실제 상위 1 % 23°, 최대 50°, 12° 초과 최장 0.26 s, 관절 속도 최대 11 rad/s)이다.
"""
import argparse
import csv
import json
import math
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
GUI_ROOT = HERE.parent
sys.path.insert(0, str(HERE))
from onnx_numpy import OnnxMlp  # noqa: E402

LOG_ROOT = GUI_ROOT / "logs"
POLICIES_DIR = GUI_ROOT / "radxa" / "policies"
POLICY_DIR = POLICIES_DIR / "real_walk"          # meta.json에 policy_name이 없는 기록 (2026-11-24)
MJCF = GUI_ROOT / "assets" / "asmr2026-humanoid-v1" / "mjcf" / "asmr2026_humanoid.xml"
CONFIG = GUI_ROOT / "config" / "robot_config.json"

RAD_PER_TURN = 2.0 * math.pi / 15.0
KT = {"6512": 0.067, "5010": 0.0836}       # Nm/A 모터축, 벤치 실측 (BasicTest/bench/params.py)
PHASE_KO = ("이동", "유지", "정책")
PHASE_COLOR = ("#e4e4e4", "#d8e9f6", "#fde4cc")
COMMAND_LIMITS = np.array([0.5, 0.25, 1.0])

# RL 절차서 기준과 시뮬 보행 값
S2_TRACK_DEG = 1.0
S3_TILT_DEG = 5.0
STOP_TILT_DEG = 20.0            # 2026-11-24 기록. 11-25부터는 meta.json stop_limits.tilt_deg (30°)
SAT_RUN_S = 1.0
PERIOD_BAD_MS = 30.0
SIM_ERR_P99_DEG = 23.0          # real_walk 시뮬 보행 (목표 − 실제)
SIM_ERR_MAX_DEG = 50.0
SIM_ERR12_RUN_S = 0.26
# real_clamp* (RL DEPLOY.md 14절, MuJoCo, 자르기 후 목표 − 엔코더): 상위 1 % 13–19°, 최대 19–27°, 12° 초과 최장 0.28 s
SIM_CLAMP_ERR_P99_DEG = 19.0
SIM_CLAMP_ERR_MAX_DEG = 27.0
SIM_CLAMP_ERR12_RUN_S = 0.28
SIM_QD_MAX = 11.0
OVERSPEED_QD = 25.0
TRAIN_ZERO_DEG = 2.0
TRAIN_IMU_TILT_DEG = 2.0
FIRST_STEP_DEG = 4.0
VBUS_LOW = 21.5
DRIVE_CURRENT_A = 18.0          # 드라이버 current_lim (CAN으로 못 바꿈). meta.json drive_current_lim_A가 있으면 그 값
DRIVE_CURRENT_OVERRIDE = None   # --drive-current-a (USB로 바꿨을 때)
IMU_KIN_OFFSET_DEG = 2.0        # IMU 피치 − 다리 기구학 피치 차이 경고 기준 (학습 장착 기울기 ±2°)

try:  # 에러 이름 (없으면 16진수만)
    sys.path[:0] = [str(GUI_ROOT), str(GUI_ROOT.parent / "BasicTest")]
    from motorgui import protocol as _P
    AXIS_ERRORS, MOTOR_ERRORS, ENCODER_ERRORS = _P.AXIS_ERRORS, _P.MOTOR_ERRORS, _P.ENCODER_ERRORS
    STATE_NAMES = _P.STATE_NAMES
except Exception:  # noqa: BLE001
    AXIS_ERRORS = MOTOR_ERRORS = ENCODER_ERRORS = {}
    STATE_NAMES = {1: "IDLE", 8: "CLOSED_LOOP_CONTROL"}


def decode(code, table):
    code = int(code)
    if not code:
        return "0"
    names = [n for bit, n in table.items() if code & bit]
    rest = code & ~sum(b for b in table if code & b)
    return " | ".join(names + ([f"0x{rest:X}"] if rest else [])) + f" (0x{code:X})"


# ---------------------------------------------------------------------------------------------- 읽기
def _num(s):
    s = s.split(" #", 1)[0].strip()
    if s.startswith("[") and s.endswith("]"):          # 한 줄 목록: [0.1, 0.2]
        return [_num(x) for x in s[1:-1].split(",") if x.strip()]
    try:
        return float(s)
    except ValueError:
        return s


def read_policy_yaml(path=POLICY_DIR / "policy.yaml"):
    """맨 위 수준의 스칼라와 목록만 읽는다 (PyYAML 없이)."""
    out, key = {}, None
    for raw in Path(path).read_text(encoding="utf-8").splitlines():
        s = raw.strip()
        if not s or s.startswith("#"):
            continue
        if not raw.startswith((" ", "-")):
            k, _, v = raw.partition(":")
            key = k.strip()
            out[key] = _num(v.strip()) if v.strip() else []
        elif raw.startswith("- ") and isinstance(out.get(key), list):
            out[key].append(_num(s[2:].strip()))
    return out


def mjcf_ranges(path=MJCF):
    if not Path(path).exists():
        return {}
    xml = Path(path).read_text(encoding="utf-8")
    out = {}
    for m in re.finditer(r"<joint[^>]*>", xml):
        tag = m.group(0)
        n, r = re.search(r'name="([^"]+)"', tag), re.search(r'range="([^"]+)"', tag)
        if n and r:
            out[n.group(1)] = tuple(float(x) for x in r.group(1).split())
    return out


def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return math.nan


EV_RE = re.compile(r"^\[(\d\d:\d\d:\d\d(?:\.\d+)?)\]\s*(?:\(t_gui ([-\d.]+)\)\s*)?(.*)$")


def short(name):
    side = "L" if "_left_" in name else "R"
    part = name.replace("leg_left_", "").replace("leg_right_", "").replace("_joint", "").replace("_", " ")
    return f"{side} {part}"


def pct(x, p):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, p)) if x.size else math.nan


def amax(x):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return float(x.max()) if x.size else math.nan


def longest_run(mask):
    best = cur = 0
    for m in np.asarray(mask, dtype=bool):
        cur = cur + 1 if m else 0
        best = max(best, cur)
    return best


def f(x, d=2):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "–"
    return f"{x:.{d}f}"


def table(headers, rows):
    out = "| " + " | ".join(headers) + " |\n|" + "---|" * len(headers) + "\n"
    return out + "".join("| " + " | ".join(str(c) for c in r) + " |\n" for r in rows)


def projected_gravity(quat):
    w, x, y, z = np.asarray(quat, dtype=np.float64).T
    return -np.stack([2 * (x * z - w * y), 2 * (w * x + y * z), w * w - x * x - y * y + z * z], axis=1)


def body_rates_from_quat(quat, dt):
    """연속 쿼터니언(몸통→세계)에서 몸통 좌표 각속도 [rad/s]: 2·vec(q_k* ⊗ q_k+1)/dt."""
    q0, q1 = quat[:-1], quat[1:]
    aw, ax, ay, az = q0[:, 0], -q0[:, 1], -q0[:, 2], -q0[:, 3]
    bw, bx, by, bz = q1.T
    dw = aw * bw - ax * bx - ay * by - az * bz
    dx = aw * bx + ax * bw + ay * bz - az * by
    dy = aw * by - ax * bz + ay * bw + az * bx
    dz = aw * bz + ax * by - ay * bx + az * bw
    s = np.where(dw < 0, -1.0, 1.0)
    return 2.0 * np.stack([dx, dy, dz], axis=1) * s[:, None] / dt[:, None]


class Run:
    """한 기록 폴더."""

    def __init__(self, d):
        self.dir = Path(d)
        self.name = self.dir.name
        self.meta = self._json("meta.json")
        self.events = []
        p = self.dir / "events.txt"
        if p.exists():
            for line in p.read_text(encoding="utf-8").splitlines():
                m = EV_RE.match(line)
                if m:
                    self.events.append((m.group(1), fnum(m.group(2)) if m.group(2) else None, m.group(3)))
        self.status = []
        p = self.dir / "status.csv"
        if p.exists():
            with open(p, encoding="utf-8", newline="") as fh:
                self.status = list(csv.DictReader(fh))
        self.npz = None
        p = self.dir / "real_walk.npz"
        if p.exists():
            with np.load(p, allow_pickle=False) as z:
                self.npz = {k: z[k] for k in z.files}
        # Radxa BridgeIO가 따로 남기는 기록 (목표 자르기를 넣은 뒤): target_sent (보낸 목표, rad), clamped (bool)
        self.io_npz = None
        p = self.dir / "real_walk_io.npz"
        if p.exists():
            with np.load(p, allow_pickle=False) as z:
                self.io_npz = {k: z[k] for k in z.files}
        self.frames = self._frames()
        p = self.dir / "gui_log.txt"
        self.gui_log = p.read_text(encoding="utf-8").splitlines() if p.exists() else []

    def _json(self, name):
        p = self.dir / name
        try:
            return json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}
        except json.JSONDecodeError:
            return {}

    def _frames(self):
        p = self.dir / "gui_frames.csv"
        if not p.exists():
            return None
        rows = {}
        with open(p, encoding="utf-8") as fh:
            next(fh, None)
            for line in fh:
                parts = line.rstrip("\n").split(",")
                if len(parts) < 5:
                    continue
                rows.setdefault(parts[2], []).append((fnum(parts[0]), int(parts[1]), fnum(parts[3]), fnum(parts[4])))
        return {k: np.array(v, dtype=float) for k, v in rows.items()}

    @property
    def stop_reason_gui(self):
        for _, _, text in reversed(self.events):
            if text.startswith(("멈춤: ", "시작 못 함: ")):
                return text
        return None


# ---------------------------------------------------------------------------------------------- 분석
class Analysis:
    def __init__(self, run, policy_dir=None):
        self.run = run
        if policy_dir is None:              # 기록의 meta.json이 고른 정책 (11-25부터), 없으면 real_walk
            name = run.meta.get("policy_name") if run.meta else None
            policy_dir = POLICIES_DIR / name if name and (POLICIES_DIR / name / "policy.yaml").exists() else POLICY_DIR
        self.policy_name = Path(policy_dir).name
        self.findings = []          # (등급, 문장)  등급: "⚠" 문제·기준 넘음, "ℹ" 참고, "✓" 기준 통과
        self.sections = []          # (제목, 마크다운)
        self.summary = {"folder": run.name}
        self.y = read_policy_yaml(Path(policy_dir) / "policy.yaml")
        self.policy_dir = Path(policy_dir)
        self.ranges = mjcf_ranges()
        try:
            self.cfg = json.loads(CONFIG.read_text(encoding="utf-8")).get("joints", {})
        except (OSError, json.JSONDecodeError):
            self.cfg = {}
        self._setup()

    def add(self, level, text):
        self.findings.append((level, text))

    # ------------------------------------------------------------------ 기본 배열
    def _setup(self):
        run, y, z = self.run, self.y, self.run.npz
        self.names = [str(n) for n in z["joints"]] if z is not None and "joints" in z else list(y.get("joints", []))
        self.short = [short(n) for n in self.names]
        self.stand = np.array(y.get("default_joint_positions", [0.0] * 12), dtype=float)
        self.kp = np.array(y.get("joint_kp", [100.6] * 12), dtype=float)
        self.kd = np.array(y.get("joint_kd", [7.99] * 12), dtype=float)
        mj = {j["name"]: j for j in run.meta.get("joints", [])}
        self.node = [int(mj[n]["node"]) if n in mj else int(self.cfg.get(n, {}).get("node", -1)) for n in self.names]
        self.motor = [self.cfg.get(n, {}).get("motor", "5010" if "ankle" in n else "6512") for n in self.names]
        tl = run.meta.get("torque_limit_6Nm", True)
        self.torque_limit_on = bool(tl)            # GUI 옵션 (ODrive 0.5.1에서는 실제 전류 한계를 바꾸지 못함, 아래)
        # 학습 토크 한계와 그에 해당하는 전류: real_walk는 policy.yaml effort_limits (6 N·m),
        # real_clamp*는 학습에 넣은 Radxa 목표 자르기 한계 torque_clamp_nm (effort_limits는 드라이버 한계 18/22.6)
        self.clamp_nm = y.get("torque_clamp_nm") if isinstance(y.get("torque_clamp_nm"), (int, float)) else None
        if self.clamp_nm:
            self.train_limit = np.full(12, float(self.clamp_nm))
            self.sim_err = (SIM_CLAMP_ERR_P99_DEG, SIM_CLAMP_ERR_MAX_DEG, SIM_CLAMP_ERR12_RUN_S)
        else:
            self.train_limit = np.array(y.get("effort_limits", [6.0] * 12), dtype=float)
            self.sim_err = (SIM_ERR_P99_DEG, SIM_ERR_MAX_DEG, SIM_ERR12_RUN_S)
        self.lim_txt = f"{float(np.max(self.train_limit)):g} N·m"
        stop = run.meta.get("stop_limits") if isinstance(run.meta.get("stop_limits"), dict) else {}
        self.stop_tilt = float(stop.get("tilt_deg", STOP_TILT_DEG))
        self.stop_track = (float(stop.get("track_deg", 30.0)), float(stop.get("track_s", 0.5))) if stop else None
        self.cur_limit = self.train_limit / 15.0 / np.array([KT[m] for m in self.motor])
        # 드라이버의 실제 한계: ODrive 0.5.1 CAN Simple의 0x00F는 vel_limit만 바꾼다 (current_lim 쓰기 없음,
        # 2026-11-24 실물 Iq 명령이 18.00 A까지 나와 확인) → 드라이버 설정 current_lim(기본 18 A)이 그대로 걸림
        self.drive_current = DRIVE_CURRENT_A if DRIVE_CURRENT_OVERRIDE is None else DRIVE_CURRENT_OVERRIDE
        if DRIVE_CURRENT_OVERRIDE is None and isinstance(run.meta.get("drive_current_lim_A"), (int, float)):
            self.drive_current = float(run.meta["drive_current_lim_A"])
        self.effort = self.drive_current * 15.0 * np.array([KT[m] for m in self.motor])
        self.torque_sign = np.array([float(self.cfg.get(n, {}).get("torque_sign", 1)) for n in self.names])
        self.has_npz = z is not None and len(z.get("time", [])) >= 2
        if not self.has_npz:
            return
        self.t = z["time"].astype(float)
        self.N = len(self.t)
        st = z["state"].astype(int)
        self.obs = z["observation"].astype(np.float64)
        self.pol = (st == 2) | (np.abs(self.obs).sum(axis=1) > 0)
        self.ph = np.where(self.pol, 2, np.minimum(st, 1))
        self.sign = z["signs"].astype(float)
        self.zero = z["zero_turns"].astype(float)
        self.pos, self.vel = z["pos_turns"].astype(float), z["vel_turns"].astype(float)
        self.q = self.sign * (self.pos - self.zero) * RAD_PER_TURN
        self.qd = self.sign * self.vel * RAD_PER_TURN
        self.target = z["target"].astype(float)
        self.action = z["action"].astype(float)
        self.command = z["command"].astype(float)
        self.quat = z["quat"].astype(float)
        self.gyro = z["gyro"].astype(float)
        self.loop_ms = z["loop_ms"].astype(float)
        self.reason = str(z["reason"])
        self.grav = projected_gravity(self.quat)
        gn = np.linalg.norm(self.grav, axis=1)
        self.tilt = np.degrees(np.arccos(np.clip(-self.grav[:, 2] / np.where(gn > 0, gn, 1), -1, 1)))
        self.lean_fwd = np.degrees(np.arctan2(self.grav[:, 0], -self.grav[:, 2]))     # 앞으로 숙이면 +
        self.lean_right = np.degrees(np.arctan2(-self.grav[:, 1], -self.grav[:, 2]))  # 오른쪽으로 기울면 +
        self.dt = float(np.median(np.diff(self.t))) if self.N > 1 else 0.02
        self.err = np.full_like(self.q, np.nan)                 # 직전 목표 − 지금 실제 (드라이버 PD가 보는 오차)
        # 드라이버가 받은 목표: 자르기 기록이 있으면 보낸 목표, 없으면 정책 목표 (npz target = 정책이 낸 원래 목표)
        self.target_sent, self.clamped, self.io_note = self._align_io()
        self.target_drive = self.target_sent if self.target_sent is not None else self.target
        self.err[1:] = self.target_drive[:-1] - self.q[1:]
        self.tau_raw = np.full_like(self.q, np.nan)             # 드라이버 PD 토크 추정 (한계 전)
        self.tau_raw[1:] = self.kp * self.err[1:] - self.kd * self.qd[1:]
        self.tau = np.clip(self.tau_raw, -self.effort, self.effort)
        self.sat = np.abs(self.tau) >= self.train_limit     # 학습 한계 6 N·m 이상 (학습 때는 여기서 잘림)
        idx = np.flatnonzero(self.pol)
        self.k0 = int(idx[0]) if idx.size else None             # 정책 첫 주기
        # 정책 구간 [(시작, 끝)]: '제자리에 서기'(11-25) 뒤 다시 '정책 켜기'를 하면 한 기록에 여러 개
        self.pol_segs = []
        k = 0
        while k < self.N:
            if self.pol[k]:
                e = k
                while e + 1 < self.N and self.pol[e + 1]:
                    e += 1
                self.pol_segs.append((k, e + 1))
                k = e + 1
            else:
                k += 1

    def _align_io(self):
        """real_walk_io.npz의 target_sent·clamped를 npz 주기에 맞춘다. send_targets 호출은 루프 전 1번이 더 있고,
        errors()로 멈추면 끝에 1번이 더 있을 수 있어, 자르지 않은 행에서 target_sent == target인 어긋남(0–3행)을 찾는다.
        돌려줌: (target_sent N×12 또는 None, clamped N×12 또는 None, 설명)"""
        io = self.run.io_npz
        if io is None:
            return None, None, None
        if "target_sent" not in io:
            return None, None, f"real_walk_io.npz에 target_sent 없음 (키: {sorted(io)})"
        ts = np.asarray(io["target_sent"], dtype=float)
        cl = np.asarray(io["clamped"], dtype=bool) if "clamped" in io else np.zeros(ts.shape, bool)
        # 시각으로: t_send (send 호출마다 perf_counter 절대값), t_read (read_joints 호출마다, 주기 시작 직후),
        # pre_loop (루프 전 첫 send). npz time = 주기 시작 − t_start이므로 t_read에서 t_start를 찾고, 주기 k의 send는
        # [주기 k 시작, 주기 k+1 시작) 안의 첫 send 행.
        if "t_send" in io and "t_read" in io and len(io["t_send"]) == len(ts):
            t_send, t_read = np.asarray(io["t_send"], float), np.asarray(io["t_read"], float)
            keep = ~np.asarray(io["pre_loop"], bool) if "pre_loop" in io else np.ones(len(ts), bool)
            for i in range(min(3, len(t_read))):
                off = t_read[i] - self.t[0]
                j = np.clip(np.searchsorted(t_read, self.t + off), 0, len(t_read) - 1)
                if np.median(np.abs(t_read[j] - (self.t + off))) > 0.002:
                    continue
                starts = self.t + off
                ends = np.append(starts[1:], starts[-1] + 2 * self.dt)
                tsk, idx = t_send[keep], np.flatnonzero(keep)
                r = np.searchsorted(tsk, starts)
                ok = (r < len(tsk)) & (tsk[np.minimum(r, len(tsk) - 1)] < ends)
                if ok.mean() > 0.95:
                    rows = idx[np.minimum(r, len(tsk) - 1)]
                    sent = np.where(ok[:, None], ts[rows], self.target)     # 못 찾은 주기는 정책 목표
                    return sent, np.where(ok[:, None], cl[rows], False), \
                        f"보낸 목표를 t_send·t_read 시각으로 맞춤 ({int(ok.sum())}/{self.N}주기)"
        best = None
        for o in range(0, min(3, len(ts) - self.N) + 1):
            a, c = ts[o:o + self.N], cl[o:o + self.N]
            # 목표가 매 주기 바뀌는 정책 구간의 자르지 않은 관절만 비교 (유지 구간은 목표가 일정해 어긋나도 같아 보임)
            free = ~c & self.pol[:, None]
            if free.sum() < 20:
                free = ~c
            if free.sum() < 5:
                continue
            d = float(np.mean(np.abs(a - self.target)[free]))
            if best is None or d < best[0]:
                best = (d, o)
        if best is None or best[0] > 1e-4:
            return None, None, (f"real_walk_io.npz({len(ts)}행)를 npz({self.N}주기)에 맞추지 못함 → 정책 목표로 계산"
                                + (f" (최소 차이 {best[0]:.1e} rad)" if best else ""))
        o = best[1]
        return ts[o:o + self.N], cl[o:o + self.N], f"보낸 목표 {len(ts)}행 중 {o}번째부터 npz {self.N}주기와 맞춤"

    def phase_mask(self, p):
        return self.ph == p

    # ------------------------------------------------------------------ 섹션들
    def overview(self):
        run = self.run
        files = [p for p in ("real_walk.npz", "real_walk_io.npz", "status.csv", "events.txt", "meta.json", "gui_frames.csv",
                             "gui_log.txt") if (run.dir / p).exists()]
        lines = [f"- 폴더: `{run.dir}`", f"- 파일: {', '.join(files)}"]
        m = run.meta
        if m:
            clamp = m.get("torque_clamp")
            lines.append(f"- 시작: {m.get('start_ms') or m.get('start')}, 시작 명령: {m.get('command_at_start', '–')}, "
                         f"IMU 축: {m.get('imu_axes')}")
            lines.append(f"- 정책: {self.policy_name}"
                         + (f" (학습에 넣은 자르기 {self.clamp_nm:g} N·m)" if self.clamp_nm else " (학습 토크 한계 effort_limits)")
                         + (f", 정지 기준: {m['stop_limits']}" if m.get("stop_limits") else ", 정지 기준: 기울기 20° (11-24)"))
            lines.append(f"- 토크: 드라이버 전류 한계 {self.drive_current:g} A (CAN으로 못 바꿈). Radxa 목표 자르기: "
                         + (f"{clamp.get('method')} {clamp.get('limit_nm')} N·m" if isinstance(clamp, dict) else "없음")
                         + (f" ({self.io_note})" if self.has_npz and self.io_note else ""))
            if isinstance(clamp, dict) and self.has_npz and self.target_sent is None:
                self.add("⚠", f"meta.json에 목표 자르기가 켜져 있지만 보낸 목표 기록을 쓰지 못함: {self.io_note or 'real_walk_io.npz 없음'}. "
                              "추종 오차·토크는 정책 목표로 계산함.")
        if run.npz is not None and not self.has_npz:
            r0 = str(run.npz.get("reason", ""))
            lines.append(f"- real_walk.npz는 있지만 주기 기록이 {len(run.npz.get('time', []))}개 → 시작 직후 멈춤. "
                         f"Radxa 멈춘 이유: `{r0}`")
            self.add("⚠", f"루프가 시작 직후 멈춤 (주기 기록 {len(run.npz.get('time', []))}개): {r0}")
        elif not self.has_npz:
            lines.append("- **real_walk.npz 없음** → status.csv·events.txt만 봄. "
                         f"Radxa에서 가져오기: `python analysis\\policy_walk_report.py --fetch {run.dir}`")
            self.add("⚠", "Radxa 매 주기 기록(real_walk.npz)이 없습니다 (가져오기 실패 또는 시작 실패). "
                          f"GUI 기록: {run.stop_reason_gui or '멈춘 이유 없음'}")
        else:
            dur = {p: float(self.phase_mask(p).sum()) * self.dt for p in range(3)}
            lines.append(f"- 주기 {self.N}개, {self.t[-1] + self.dt:.1f} s: 이동 {dur[0]:.1f} s → 유지 {dur[1]:.1f} s → "
                         f"정책 {dur[2]:.1f} s")
            self.summary.update(cycles=self.N, dur_ramp=dur[0], dur_hold=dur[1], dur_policy=dur[2])
            lines.append(f"- Radxa 멈춘 이유 (npz): `{self.reason}`")
            if dur[2] == 0:
                self.add("ℹ", "정책을 켜지 않은 기록입니다 (이동·유지만).")
        gr = run.stop_reason_gui
        if gr:
            lines.append(f"- GUI 이벤트: `{gr}`")
        self.summary["reason_npz"] = self.reason if self.has_npz else None
        self.summary["reason_gui"] = gr
        self._classify_stop()
        # 설정 일치
        if self.has_npz and run.meta.get("joints"):
            mj = run.meta["joints"]
            ds = max(abs(float(j["sign"]) - s) for j, s in zip(mj, self.sign))
            dz = max(abs(float(j["zero_turns"]) - zt) for j, zt in zip(mj, self.zero))
            order_ok = [j["name"] for j in mj] == self.names
            lines.append(f"- meta.json ↔ npz: 관절 순서 {'같음' if order_ok else '**다름**'}, 부호 차이 {ds:g}, "
                         f"영점 차이 {dz:.2e} turn")
            if not order_ok or ds or dz > 1e-6:
                self.add("⚠", "meta.json과 npz의 관절 순서·부호·영점이 다릅니다.")
        if self.has_npz:
            sp = np.abs(self.run.npz["standing_pose"].astype(float) - self.stand).max()
            if sp > 1e-6:
                self.add("⚠", f"npz의 서기 자세가 policy.yaml과 {sp:.2e} rad 다릅니다 (정책 파일 버전 확인).")
        self.sections.append(("개요", "\n".join(lines) + "\n"))

    def _classify_stop(self):
        r = (self.reason if self.has_npz else "") or ""
        g = self.run.stop_reason_gui or ""
        if r.startswith("tilt"):
            self.add("⚠", f"몸통 기울기 {self.stop_tilt:g}° 초과로 Radxa가 멈춤 ({r}). '멈추기 직전' 절에서 어느 쪽으로 넘어갔는지 확인.")
        elif "추종 오차" in r:
            self.add("⚠", f"관절 추종 오차 정지 (보낸 목표 − 엔코더가 기준을 넘은 채 이어짐): {r}")
        elif "버스 전압" in g:
            self.add("⚠", f"버스 전압이 낮아 GUI가 멈춤: {g}. RL 절차서: 토크 한계가 낮은 정책으로 내림 (12 → 9 → 6).")
        elif r.startswith("driver errors"):
            self.add("⚠", f"드라이버 문제로 멈춤: {r}")
        elif "late" in r:
            self.add("⚠", f"주기 지연으로 멈춤: {r} (타이밍 절 확인)")
        elif r == "stop requested":
            if g:
                self.add("ℹ", f"운영자·GUI 쪽에서 멈춤: {g}")
        elif r:
            self.add("⚠", f"멈춘 이유: {r}")
        if "오류" in g or "시작 못 함" in g:
            self.add("⚠", f"GUI: {g}")

    def timing(self):
        out = []
        if self.has_npz:
            per = np.diff(self.t) * 1000.0
            ph = self.ph[1:]
            rows = []
            for p in range(3):
                m = ph == p
                if m.sum() < 2:
                    continue
                x, lm = per[m], self.loop_ms[1:][m]
                rows.append([PHASE_KO[p], m.sum(), f(pct(x, 50)), f(pct(x, 99)), f(amax(x)), int((x > 25).sum()),
                             int((x > PERIOD_BAD_MS).sum()), longest_run(x > PERIOD_BAD_MS), f(pct(lm, 50)),
                             f(pct(lm, 99)), f(amax(lm))])
            out.append("Radxa 루프 (npz): 주기 = 이웃한 주기 시작 간격, 작업 = 한 주기에서 읽기+정책+전송에 든 시간\n")
            out.append(table(["단계", "주기 수", "주기 p50 ms", "p99", "최대", ">25 ms", f">{PERIOD_BAD_MS:g} ms",
                              f">{PERIOD_BAD_MS:g} 연속", "작업 p50 ms", "p99", "최대"], rows))
            n30 = int((per > PERIOD_BAD_MS).sum())
            self.summary.update(period_p99=pct(per, 99), period_max=amax(per), n_period_over30=n30,
                                loop_p99=pct(self.loop_ms, 99))
            if n30:
                self.add("⚠", f"주기가 {PERIOD_BAD_MS:g} ms를 넘은 곳 {n30}번 (최대 {amax(per):.1f} ms, 연속 최대 "
                              f"{longest_run(per > PERIOD_BAD_MS)}번). 절차서 공통 중단 기준은 '30 ms 이상이 이어짐'.")
            else:
                self.add("✓", f"주기 최대 {amax(per):.1f} ms (30 ms 넘은 적 없음).")
            # 엔코더 정체 응답: 움직이는 중인데 pos·vel이 직전 주기와 완전히 같음 → 그 주기에 새 응답을 못 받음
            rep = (self.pos[1:] == self.pos[:-1]) & (self.vel[1:] == self.vel[:-1]) & (np.abs(self.vel[1:]) > 1e-3)
            self.stale = np.vstack([np.zeros((1, 12), bool), rep])
            rows = []
            for j in range(12):
                c = [int((self.stale[:, j] & self.phase_mask(p)).sum()) for p in range(3)]
                rows.append([self.short[j], self.node[j], *c, longest_run(self.stale[:, j])])
            tot = int(self.stale.sum())
            out.append("\n엔코더 정체 응답 (움직이는 중인데 값이 직전 주기와 완전히 같음 = 8 ms 안에 새 응답 못 받음, 지난 값 사용)\n")
            if tot:
                ns = self.stale.sum(axis=1)
                out.append(f"정체 응답이 있는 주기 {int((ns > 0).sum())}개, 그중 6개 넘는 관절이 한꺼번에 정체 (읽기를 통째로 놓침, "
                           f"보통 Radxa 쪽 지연) {int((ns > 6).sum())}개\n")
                out.append(table(["관절", "node", "이동", "유지", "정책", "연속 최대"], [r for r in rows if sum(r[2:5])]))
                pol_rate = (self.stale & self.pol[:, None]).sum() / max(1, self.pol.sum() * 12)
                self.add("⚠" if pol_rate > 0.01 or longest_run(self.stale.any(axis=1)) >= 3 else "ℹ",
                         f"엔코더 정체 응답 {tot}번 (정책 중 {100 * pol_rate:.2f} %). 5주기 연속이면 Radxa가 멈춤.")
            else:
                out.append("없음\n")
            self.summary["stale_encoder"] = tot
            # IMU 값이 주기 사이에 안 바뀜 (200 Hz 입력이면 늘 바뀌어야 함)
            qsame = np.all(self.quat[1:] == self.quat[:-1], axis=1)
            if qsame.any():
                self.add("⚠" if longest_run(qsame) >= 2 else "ℹ",
                         f"IMU 쿼터니언이 직전 주기와 같은 주기 {int(qsame.sum())}번 (연속 최대 {longest_run(qsame)}). "
                         "imu_reader가 멈칫한 것.")
            self.summary["imu_same"] = int(qsame.sum())
        if self.run.status:
            rows = []
            st = self.run.status
            for p, key in enumerate(("ramp", "hold", "walk")):
                rs = [r for r in st if r.get("phase") == key]
                if not rs:
                    continue
                rd = [fnum(r["read_ms"]) for r in rs]
                age = [fnum(r["imu_age_ms"]) for r in rs]
                miss = [fnum(r["miss"]) for r in rs]
                rows.append([PHASE_KO[p], len(rs), f(pct(rd, 50)), f(pct(rd, 99)), f(amax(rd)), f(pct(age, 50)),
                             f(pct(age, 99)), f(amax(age)), f(amax(miss), 0)])
            out.append("\nGUI가 받은 상태 (status.csv, 10 Hz 표본): 엔코더 12개 읽기 시간, 정책이 쓴 IMU 값의 나이\n")
            out.append(table(["단계", "표본", "읽기 p50 ms", "p99", "최대", "IMU 나이 p50 ms", "p99", "최대",
                              "놓침 연속 최대"], rows))
            rd_all = [fnum(r["read_ms"]) for r in st]
            if amax(rd_all) >= 8.0:
                self.add("ℹ", f"엔코더 읽기가 8 ms 기다림 한도에 닿은 표본이 있습니다 (최대 {amax(rd_all):.1f} ms).")
        self.sections.append(("1. 타이밍·통신", "\n".join(out)))

    def imu(self):
        if not self.has_npz:
            return
        out = []
        qn = np.linalg.norm(self.quat, axis=1)
        rows = []
        for p in range(3):
            m = self.phase_mask(p)
            if not m.any():
                continue
            g = self.grav[m]
            rows.append([PHASE_KO[p], f(np.mean(self.tilt[m]), 1), f(pct(self.tilt[m], 95), 1), f(amax(self.tilt[m]), 1),
                         f(np.mean(self.lean_fwd[m]), 1), f(np.mean(self.lean_right[m]), 1),
                         " ".join(f(v, 3) for v in g.mean(axis=0)),
                         " ".join(f(v, 3) for v in np.degrees(self.gyro[m].mean(axis=0))),
                         " ".join(f(v, 1) for v in np.degrees(np.percentile(np.abs(self.gyro[m]), 99, axis=0)))])
        out.append(table(["단계", "기울기 평균°", "p95", "최대", "앞 숙임 평균°", "오른쪽 기울기 평균°", "중력 평균 (x y z)",
                          "자이로 평균 °/s (x y z)", "|자이로| p99 °/s"], rows))
        out.append(f"\n쿼터니언 크기 {qn.min():.4f}~{qn.max():.4f}. 앞으로 숙이면 중력 x > 0, 오른쪽으로 기울면 중력 y < 0 "
                   "(절차서 4절). 이동·유지는 매단 상태라 몸통이 하네스에 따라 기울 수 있음.\n")
        hold = self.phase_mask(1)
        if hold.sum() > 25:
            lf, lr = float(np.mean(self.lean_fwd[hold])), float(np.mean(self.lean_right[hold]))
            self.summary.update(hold_lean_fwd=lf, hold_lean_right=lr)
            if math.hypot(lf, lr) > TRAIN_IMU_TILT_DEG:
                self.add("ℹ", f"유지(매단 상태) 중 몸통 기울기 앞 {lf:+.1f}°, 오른쪽 {lr:+.1f}°. 하네스 때문일 수도 있음. "
                              f"바닥에 똑바로 세웠을 때도 {TRAIN_IMU_TILT_DEG:g}°를 넘으면 장착 기울기(학습 ±2°) 확인.")
        if self.pol.any():
            tp = self.tilt[self.pol]
            frac5 = float((tp > S3_TILT_DEG).mean())
            self.summary.update(policy_tilt_max=amax(tp), policy_tilt_frac5=frac5)
            if amax(tp) > S3_TILT_DEG:
                self.add("⚠", f"정책 중 기울기 최대 {amax(tp):.1f}° (5° 넘은 시간 {100 * frac5:.0f} %). S3 통과 기준은 5° 안.")
            else:
                self.add("✓", f"정책 중 기울기 최대 {amax(tp):.1f}° (S3 기준 5° 안).")
        # 자이로와 쿼터니언 변화의 일관성 (축·단위가 맞는지)
        if self.N > 50:
            dt = np.diff(self.t)
            wq = body_rates_from_quat(self.quat, dt)
            wg = 0.5 * (self.gyro[1:] + self.gyro[:-1])
            rows = []
            for a, ax in enumerate("xyz"):
                s = float(np.std(wq[:, a]))
                if s < 0.03:
                    rows.append([ax, f(s, 3), "움직임 작음", "–"])
                    continue
                c = float(np.corrcoef(wq[:, a], wg[:, a])[0, 1])
                k = float(np.dot(wq[:, a], wg[:, a]) / np.dot(wq[:, a], wq[:, a]))
                rows.append([ax, f(s, 3), f(c, 3), f(k, 2)])
                # 표본 시각이 달라(쿼터니언 차분 vs 순간 자이로) 걸을 때는 상관이 0.7–0.9까지 내려감 → 축·부호 오류만 잡음
                if c < 0.5 or k < 0:
                    self.add("⚠", f"자이로 {ax}축이 쿼터니언 변화와 안 맞음 (상관 {c:.2f}, 기울기 {k:.2f}). 축·부호·단위 확인.")
            out.append("\n자이로 ↔ 쿼터니언 변화 (같은 축이면 상관·기울기 > 0; 걸을 때는 표본 시각 차이로 0.7–0.9도 정상. 축 혼동·부호 "
                       "오류 점검용)\n")
            out.append(table(["축", "쿼터니언 각속도 표준편차 rad/s", "상관", "기울기 (자이로/쿼터니언)"], rows))
        self._imu_vs_kinematics(out)
        self.sections.append(("2. IMU", "\n".join(out)))

    def _kin_gravity(self):
        """다리 기구학(MJCF)으로, 디딤발(세계 높이가 낮은 발) 발바닥이 바닥에 평평할 때의 몸통 좌표 중력 방향.
        mujoco나 MJCF가 없으면 None."""
        if hasattr(self, "_kin_cache"):
            return self._kin_cache
        self._kin_cache = None
        try:
            import mujoco
            M = mujoco.MjModel.from_xml_path(str(MJCF))
        except Exception:  # noqa: BLE001
            return None
        D = mujoco.MjData(M)
        qadr = [M.jnt_qposadr[M.joint(n).id] for n in self.names]
        free = M.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE

        def fk(q):
            D.qpos[:] = 0.0
            if free:
                D.qpos[3] = 1.0
            D.qpos[qadr] = q
            mujoco.mj_kinematics(M, D)
            return {s: (D.site(f"{s}_foot").xmat.reshape(3, 3).copy(), D.site(f"{s}_foot").xpos - D.body("base").xpos)
                    for s in ("left", "right")}

        r0 = {s: v[0] for s, v in fk(np.zeros(12)).items()}       # 0 자세: 발바닥이 몸통과 평행
        g = np.zeros((self.N, 3))
        for k in range(self.N):
            o = fk(self.q[k])
            w, x, y, z = self.quat[k]
            zrow = np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])   # 세계 z축 (몸통 좌표)
            s = min(o, key=lambda side: zrow @ o[side][1])
            g[k] = o[s][0] @ r0[s].T @ np.array([0.0, 0.0, -1.0])
        self._kin_cache = g
        return g

    def _imu_vs_kinematics(self, out):
        gk = self._kin_gravity()
        if gk is None:
            out.append("\nIMU ↔ 다리 기구학: mujoco 또는 MJCF가 없어 건너뜀.\n")
            return
        kf = np.degrees(np.arctan2(gk[:, 0], -gk[:, 2]))
        kr = np.degrees(np.arctan2(-gk[:, 1], -gk[:, 2]))
        masks = []
        hold_idx = np.flatnonzero(self.ph == 1)
        if hold_idx.size:
            masks.append(("유지 마지막 3 s (정책 켜기 직전)", (self.ph == 1) & (self.t > self.t[hold_idx[-1]] - 3.0)))
        if self.pol.any():
            masks.append(("정책", self.pol))
        rows, diffs = [], {}
        for lab, m in masks:
            df, dr = float(np.mean(self.lean_fwd[m] - kf[m])), float(np.mean(self.lean_right[m] - kr[m]))
            diffs[lab] = (df, dr)
            rows.append([lab, f(float(self.lean_fwd[m].mean()), 1), f(float(kf[m].mean()), 1), f(df, 1),
                         f(float(self.lean_right[m].mean()), 1), f(float(kr[m].mean()), 1), f(dr, 1)])
        out.append("\nIMU ↔ 다리 기구학: 디딤발 발바닥이 바닥에 평평하다고 보고 관절 각(MJCF)으로 계산한 몸통 기울기와 IMU를 비교. "
                   "서기 자세는 hip+knee+ankle pitch 합이 0이라 발이 평평하면 몸통이 수직. 차이가 일정하면 IMU 장착(또는 가속도계) "
                   "기울기, 영점 오차 합, 발이 평평하지 않음(하네스가 몸통을 당김) 중 하나. 발이 바닥에 닿아 체중이 실렸을 때만 의미.\n")
        out.append(table(["구간", "IMU 앞°", "기구학 앞°", "차이°", "IMU 오른쪽°", "기구학 오른쪽°", "차이°"], rows))
        key = "유지 마지막 3 s (정책 켜기 직전)"
        if key in diffs:
            df, dr = diffs[key]
            self.summary.update(imu_minus_kin_fwd_hold=df, imu_minus_kin_right_hold=dr)
            if abs(df) > IMU_KIN_OFFSET_DEG or abs(dr) > IMU_KIN_OFFSET_DEG:
                self.add("⚠", f"정책 켜기 직전(발이 바닥에 닿은 서기 자세) IMU가 다리 기구학보다 앞 {df:+.1f}°, 오른쪽 {dr:+.1f}° "
                              f"기울어 있음 (학습 IMU 장착 기울기 ±2°). 수평계로 몸통 실제 기울기 확인 필요.")

    def standing_check(self):
        """S2: 서기 자세 이동·유지."""
        if not self.has_npz:
            return
        hold = np.flatnonzero(self.phase_mask(1))
        ramp = np.flatnonzero(self.phase_mask(0))
        out = []
        q0 = np.degrees(self.q[0])
        out.append("시작 자세 (첫 주기, deg): " + ", ".join(f"{s} {v:+.1f}" for s, v in zip(self.short, q0)) + "\n")
        if hold.size < 10:
            out.append("유지 구간이 짧아 S2 판정 안 함.\n")
            self.sections.append(("3. 서기 자세 (S2)", "\n".join(out)))
            return
        t_hold0 = self.t[hold[0]]
        steady = hold[self.t[hold] >= t_hold0 + 0.5]
        if steady.size < 5:
            steady = hold
        e = np.degrees(self.err[steady])
        qm = np.degrees(self.q[steady].mean(axis=0))
        rows, bad = [], []
        for j in range(12):
            me = float(np.mean(e[:, j]))
            rows.append([self.short[j], f(math.degrees(self.stand[j]), 2), f(qm[j], 2), f(me, 2), f(float(np.std(e[:, j])), 2),
                         f(float(np.mean(self.tau[steady, j])), 2),
                         f(qm[j] + qm[j + 6], 2) if j < 6 else ""])
            if abs(me) > S2_TRACK_DEG:
                bad.append(f"{self.short[j]} {me:+.1f}°")
        out.append(f"유지 구간 {self.t[steady[0]]:.1f}~{self.t[steady[-1]]:.1f} s ({steady.size}주기, 유지 시작 0.5 s 뒤부터). "
                   "목표−실제 = 서기 자세 − 엔코더 (드라이버 PD가 남긴 오차), τ = Kp·오차 − Kd·속도\n")
        out.append(table(["관절", "서기 목표°", "실제 평균°", "목표−실제 평균°", "표준편차°", "PD 토크 평균 N·m", "좌우 합° (L+R)"],
                         rows))
        sym = [abs(qm[j] + qm[j + 6]) for j in range(6)]
        out.append(f"\n좌우 합: 오른쪽 관절은 왼쪽의 거울(부호 반대)이라 대칭이면 0. 최대 {max(sym):.2f}°.\n")
        if ramp.size:
            r_end = ramp[-1]
            if r_end + 1 < self.N:
                ee = np.degrees(self.target[r_end] - self.q[r_end + 1])
                out.append("이동 끝 (서기 목표 도달 직후) 목표−실제°: " +
                           ", ".join(f"{s} {v:+.1f}" for s, v in zip(self.short, ee)) + "\n")
        self.summary.update(s2_max_err=float(np.max(np.abs(e.mean(axis=0)))), s2_sym_max=float(max(sym)))
        if bad:
            self.add("⚠", f"S2: 서기 자세 목표와 1° 넘게 차이: {', '.join(bad)} (마찰 0.36 N·m ≈ 0.2°, 다리 무게 포함).")
        else:
            self.add("✓", f"S2: 유지 중 모든 관절이 서기 목표 1° 안 (최대 {np.max(np.abs(e.mean(axis=0))):.2f}°).")
        if max(sym) > TRAIN_ZERO_DEG:
            worst = int(np.argmax(sym))
            self.add("⚠", f"서기 자세 좌우 비대칭: {self.short[worst].split(' ', 1)[1]} L+R = {qm[worst] + qm[worst + 6]:+.1f}° "
                          f"(목표는 대칭). 영점 오차 의심 (학습 ±{TRAIN_ZERO_DEG:g}°).")
        self.sections.append(("3. 서기 자세 (S2)", "\n".join(out)))

    def joints_policy(self):
        if not self.has_npz or self.pol.sum() < 5:
            return
        m = self.pol.copy()
        for k0, _ in self.pol_segs:            # 구간마다 첫 주기는 직전 목표가 서기 자세
            m[k0] = False
        out = []
        rows = []
        for j in range(12):
            n = self.names[j]
            qd_ = np.degrees(self.q[self.pol, j])
            lo, hi = self.ranges.get(n, (math.nan, math.nan))
            margin = min(qd_.min() - math.degrees(lo), math.degrees(hi) - qd_.max()) if math.isfinite(lo) else math.nan
            ae = np.abs(np.degrees(self.err[m, j]))
            run12 = longest_run(ae > 12.0) * self.dt
            sat = self.sat[m, j]
            satrun = longest_run(sat) * self.dt
            qdm = amax(np.abs(self.qd[self.pol, j]))
            rows.append([self.short[j], f"{qd_.min():+.0f}~{qd_.max():+.0f}", f(margin, 1), f(pct(ae, 95), 1), f(pct(ae, 99), 1),
                         f(amax(ae), 1), f(run12, 2), f(pct(np.abs(self.tau[m, j]), 95), 2), f(100 * sat.mean(), 1),
                         f(satrun, 2), f(qdm, 1), int(self.stale[self.pol, j].sum()) if hasattr(self, "stale") else "–"]
                        + ([f(100 * float(self.clamped[m, j].mean()), 1),
                            f(amax(np.abs(np.degrees(self.target[m, j] - self.target_drive[m, j]))), 1)]
                           if self.clamped is not None else []))
            if math.isfinite(margin) and margin < 3.0:
                self.add("⚠", f"{self.short[j]}가 MJCF 관절 범위 끝 {margin:.1f}° 안까지 감 (기구 끝·영점 확인).")
            sp99, smax, srun = self.sim_err
            if pct(ae, 99) > sp99 or amax(ae) > smax or run12 > 2 * srun:
                self.add("⚠", f"{self.short[j]} 추종 오차가 시뮬보다 큼: p99 {pct(ae, 99):.0f}° (시뮬 {sp99:g}°), 최대 "
                              f"{amax(ae):.0f}° ({smax:g}°), 12° 초과 최장 {run12:.2f} s ({srun:g} s).")
            if satrun > SAT_RUN_S:
                self.add("⚠", f"{self.short[j]}가 학습 한계 {self.train_limit[j]:g} N·m 이상을 {satrun:.2f} s 연속 냄 "
                              "(S3 중단 기준 1 s).")
            if sat.mean() > 0.05:
                self.add("⚠", f"{self.short[j]} 토크가 학습 한계 {self.train_limit[j]:g} N·m를 넘은 시간 {100 * sat.mean():.0f} % "
                              f"(최대 {amax(np.abs(self.tau[m, j])):.1f} N·m; 학습 때는 {self.train_limit[j]:g}에서 잘림"
                              + (", 정책 주기마다 한 번" if self.clamp_nm else "") + ").")
            if qdm > SIM_QD_MAX:
                self.add("⚠" if qdm > 0.8 * OVERSPEED_QD else "ℹ",
                         f"{self.short[j]} 관절 속도 최대 {qdm:.1f} rad/s (시뮬 보행 최대 11, 드라이버 과속 오류 25).")
        sp99, smax, srun = self.sim_err
        out.append(f"정책 {self.policy_name}, 정책 중 (첫 주기 제외). 오차 = 직전 목표 − 지금 실제 (deg), τ = 드라이버 PD 토크 "
                   f"추정 (N·m, 드라이버 전류 한계 {self.drive_current:g} A로 자름; 학습 한계는 {self.lim_txt}). 시뮬 보행: 오차 "
                   f"p99 {sp99:g}°, 최대 약 {smax:g}°, 12° 초과 최장 {srun:g} s, 관절 속도 최대 11 rad/s.\n")
        clamp_cols = ["목표 잘림 %", "잘린 양 최대°"] if self.clamped is not None else []
        if self.clamped is not None:
            out.append(f"목표 자르기 기록이 있어 오차·τ는 드라이버로 보낸(자른) 목표로 계산. '{self.lim_txt} 넘음'은 자른 뒤에도 "
                       "주기 사이에 남은 초과 (20 ms 사이 관절이 움직여서). 잘린 양 = 정책 목표 − 보낸 목표.\n")
            if self.stop_track is not None and self.target_sent is not None:
                # Radxa의 추종 오차 정지와 같은 값: 같은 주기의 |보낸 목표 − 읽은 각도|
                same = np.abs(np.degrees(self.target_drive[self.pol] - self.q[self.pol]))
                lim_d, lim_s = self.stop_track
                runs = [longest_run(same[:, j] > lim_d) * self.dt for j in range(12)]
                jw = int(np.argmax(same.max(axis=0)))
                self.summary["same_cycle_track_err_max_deg"] = round(float(same.max()), 2)
                lvl = "⚠" if max(runs) >= 0.5 * lim_s else ("ℹ" if same.max() > 0.6 * lim_d else "✓")
                self.add(lvl, f"추종 오차 정지 기준({lim_d:g}°·{lim_s:g} s, 같은 주기 |보낸 목표 − 엔코더|): 최대 "
                              f"{same.max():.1f}° ({self.short[jw]}), {lim_d:g}° 넘은 최장 {max(runs):.2f} s.")
            cm = self.clamped[m].mean(axis=0)
            self.summary["clamped_frac"] = [round(float(x), 4) for x in cm]
            top = [j for j in np.argsort(-cm)[:4] if cm[j] > 0.01]
            if top:
                self.add("ℹ", "Radxa가 목표를 자른 비율 (정책 중): " + ", ".join(f"{self.short[j]} {100 * cm[j]:.0f} %" for j in top))
        out.append(table(["관절", "범위°", "MJCF 끝까지 여유°", "|오차| p95", "p99", "최대", "12° 초과 최장 s", "|τ| p95",
                          f"{self.lim_txt} 넘음 %", f"{self.lim_txt} 넘음 최장 s", "|q̇| 최대 rad/s", "정체 응답"] + clamp_cols,
                         rows))
        # 속도 추정 점검: ODrive vel_estimate vs 위치 차분
        qdiff = np.gradient(self.q, self.t, axis=0)
        cs = []
        for j in range(12):
            a, b = qdiff[self.pol, j], self.qd[self.pol, j]
            cs.append(float(np.corrcoef(a, b)[0, 1]) if np.std(a) > 1e-3 and np.std(b) > 1e-3 else math.nan)
        out.append("\n관절 속도 (ODrive vel_estimate) ↔ 위치 차분 상관: " +
                   ", ".join(f"{s} {f(c, 2)}" for s, c in zip(self.short, cs)) + "\n")
        bad = [self.short[j] for j, c in enumerate(cs) if math.isfinite(c) and c < 0.7]
        if bad:
            self.add("⚠", f"관절 속도와 위치 변화가 잘 안 맞는 관절: {', '.join(bad)} (관측 21–32 단위·부호 확인).")
        self.summary["policy_err_p99_max"] = max(pct(np.abs(np.degrees(self.err[m, j])), 99) for j in range(12))
        self.summary["policy_sat_run_max"] = max(longest_run(self.sat[m, j]) for j in range(12)) * self.dt
        self.sections.append(("4. 관절 (정책 중)", "\n".join(out)))

    def policy_io(self):
        if not self.has_npz or not self.pol.any():
            return
        P = self.pol
        idx = np.flatnonzero(P)
        out = []
        # 관측이 기록된 다른 값과 같은지 (관측 조립 확인)
        prev = np.zeros((idx.size, 12))
        for i, k in enumerate(idx):
            if k > 0 and P[k - 1]:
                prev[i] = np.clip(self.action[k - 1], -20, 20)
        checks = {
            "명령 0–2": self.obs[P, 0:3] - np.clip(self.command[P], -COMMAND_LIMITS, COMMAND_LIMITS),
            "각속도 3–5": self.obs[P, 3:6] - self.gyro[P],
            "중력 6–8": self.obs[P, 6:9] - self.grav[P],
            "관절 각 9–20": self.obs[P, 9:21] - (self.q[P] - self.stand),
            "관절 속도 21–32": self.obs[P, 21:33] - self.qd[P],
            "직전 행동 33–44": self.obs[P, 33:45] - prev,
            "목표 = 서기 + 0.25·clip(행동)": self.target[P] - (self.stand + 0.25 * np.clip(self.action[P], -20, 20)),
        }
        rows = []
        for k, v in checks.items():
            d = float(np.abs(v).max())
            rows.append([k, f"{d:.1e}", "같음" if d < 1e-4 else "**다름**"])
            if d >= 1e-4:
                self.add("⚠", f"관측·목표 조립이 기록된 값과 다름: {k} 최대 차이 {d:.2e} (코드 확인 필요).")
        # 재생: 같은 관측으로 정책을 다시 계산
        try:
            mlp = OnnxMlp(self.policy_dir / "policy.onnx")
            rep = mlp(self.obs[P].astype(np.float32))
            d = float(np.abs(rep - self.action[P]).max())
            rows.append(["정책 재생 (PC numpy) vs Radxa 행동", f"{d:.1e}", "같음" if d < 1e-3 else "**다름**"])
            self.summary["replay_diff"] = d
            if d >= 1e-3:
                self.add("⚠", f"PC에서 같은 관측으로 계산한 행동이 Radxa 기록과 {d:.2e} 다름 (정책 파일이 다를 수 있음).")
        except Exception as e:  # noqa: BLE001
            rows.append(["정책 재생", "–", f"건너뜀: {e}"])
        out.append("관측 조립 점검 (기록된 값끼리 비교)\n")
        out.append(table(["항목", "최대 차이", "결과"], rows))
        # 첫 주기
        k0 = self.k0
        dev = np.degrees(self.target[k0] - self.stand)
        out.append(f"\n정책 첫 주기 (t = {self.t[k0]:.2f} s): 중력 {np.round(self.grav[k0], 3).tolist()}, 각속도 rad/s "
                   f"{np.round(self.gyro[k0], 3).tolist()}, 관절 − 서기 최대 {np.degrees(np.abs(self.obs[k0, 9:21]).max()):.1f}°, "
                   f"명령 {np.round(self.command[k0], 2).tolist()}\n")
        out.append("첫 목표 − 서기 자세 (deg): " + ", ".join(f"{s} {v:+.1f}" for s, v in zip(self.short, dev)) + "\n")
        # 이상적 관측(명령 0, 똑바로, 정지)에서의 첫 목표: policy.yaml reference_first_action (real_clamp*), 없으면 4°
        ref = self.y.get("reference_first_action")
        ref_deg = (float(np.degrees(0.25 * np.abs(np.clip(np.array(ref, dtype=float), -20, 20))).max())
                   if isinstance(ref, list) and len(ref) == 12 else None)
        first_lim = (ref_deg + 4.0) if ref_deg is not None else FIRST_STEP_DEG
        if np.abs(dev).max() > first_lim:
            self.add("ℹ", f"정책 첫 목표가 서기 자세에서 최대 {np.abs(dev).max():.1f}° ({self.short[int(np.argmax(np.abs(dev)))]}). "
                          + (f"이상적 관측의 기준 첫 목표는 최대 {ref_deg:.1f}°." if ref_deg is not None
                             else "오프라인 점검(이상적 관측)에서는 4° 안."))
        # 행동 통계·떨림 (주파수는 가장 긴 정책 구간 하나로: 구간을 이어 붙이면 이음매가 주파수에 섞임)
        a = self.action[P]
        tgt = np.degrees(self.target[P])
        fs = 1.0 / self.dt
        rows = []
        n = idx.size
        dom, hf = [math.nan] * 12, [math.nan] * 12
        lk0, lk1 = max(self.pol_segs, key=lambda s: s[1] - s[0])
        tgt_l = np.degrees(self.target[lk0:lk1])
        if lk1 - lk0 >= 100:
            w = np.hanning(lk1 - lk0)
            freqs = np.fft.rfftfreq(lk1 - lk0, self.dt)
            self.spec = (freqs, [])
            for j in range(12):
                x = tgt_l[:, j] - tgt_l[:, j].mean()
                p = np.abs(np.fft.rfft(x * w)) ** 2
                self.spec[1].append(p)
                band = (freqs >= 0.3) & (freqs <= 5.0)
                dom[j] = float(freqs[band][np.argmax(p[band])]) if band.any() else math.nan
                tot = p[freqs > 0.2].sum()
                hf[j] = float(p[freqs > 8.0].sum() / tot) if tot > 0 else math.nan
        for j in range(12):
            aj = a[:, j]
            da = np.diff(aj)
            ratio = float(np.std(da) / np.std(aj)) if np.std(aj) > 1e-6 and aj.size > 2 else math.nan
            rows.append([self.short[j], f(pct(np.abs(aj), 50), 2), f(pct(np.abs(aj), 99), 2), f(amax(np.abs(aj)), 2),
                         f(100 * float((np.abs(aj) >= 20).mean()), 1), f(amax(np.abs(tgt[:, j] - math.degrees(self.stand[j]))), 1),
                         f(dom[j], 2), f(hf[j], 2), f(ratio, 2)])
        out.append(f"\n행동 (정책 출력, 배율 전) {n}주기. 진동수 = 목표의 주 성분 (0.3–5 Hz), 8 Hz 넘는 비율 = 떨림 지표, "
                   "Δa/a = 이웃 주기 차이의 표준편차 ÷ 행동 표준편차 (1 Hz 부드러운 움직임 ≈ 0.13, 번갈아 튀면 ≈ 2)\n")
        out.append(table(["관절", "|a| p50", "p99", "최대", "±20 잘림 %", "목표−서기 최대°", "주 진동수 Hz", "8 Hz 넘는 비율",
                          "Δa/a"], rows))
        clip = float((np.abs(a) >= 20).mean())
        if clip > 0.01:
            self.add("⚠", f"정책 행동이 ±20 한계에 잘린 비율 {100 * clip:.1f} % (관측이 학습 범위를 벗어났을 수 있음).")
        # 떨림: 8 Hz 위 성분의 진폭(rms, deg)이 크고 비율도 클 때만 (보행 움직임이 작은 hip roll이 비율만으로 걸리지 않게)
        hf_amp = [math.sqrt(h) * float(np.std(tgt_l[:, j])) if math.isfinite(h) else math.nan for j, h in enumerate(hf)]
        bad = [j for j in range(12) if math.isfinite(hf[j]) and hf[j] > 0.3 and hf_amp[j] > 1.5]
        if bad:
            self.add("⚠", "고주파 떨림 (목표의 8 Hz 위 성분): " + ", ".join(f"{self.short[j]} {hf_amp[j]:.1f}° rms" for j in bad)
                     + " (절차서 '빠르게 떪': 속도 단위, vel_ff, 루프 지터, 게인).")
        self.summary["target_hf_rms_deg"] = [round(x, 2) if math.isfinite(x) else None for x in hf_amp]
        if math.isfinite(dom[2]):
            self.summary["gait_hz_left_hip_pitch"] = dom[2]
        # 정책 구간 (정책 켜기 → 서기 버튼·멈춤)
        prow = []
        for si, (k0, k1) in enumerate(self.pol_segs):
            nxt = self.ph[k1] if k1 < self.N else None
            end = "서기 버튼" if nxt == 0 else ("유지" if nxt == 1 else f"기록 끝 ({self.reason})")
            body = slice(min(k0 + 25, k1 - 1), k1)
            prow.append([si + 1, f"{self.t[k0]:.1f}", f((k1 - k0) * self.dt, 1), " ".join(f(v, 2) for v in self.command[k0]),
                         f(float(self.lean_fwd[k0]), 1), f(float(np.mean(self.lean_fwd[body])), 1), f(amax(self.tilt[k0:k1]), 1),
                         f(float(np.degrees(np.abs(self.target[k0] - self.stand)).max()), 1), end])
        out.append(f"\n정책 구간 {len(self.pol_segs)}개 (주파수 분석은 가장 긴 구간 {self.t[lk0]:.1f}~{self.t[lk1 - 1] + self.dt:.1f} s)\n")
        out.append(table(["#", "시작 t s", "길이 s", "명령", "켤 때 앞 숙임°", "앞 숙임 평균° (0.5 s 뒤)", "최대 기울기°",
                          "첫 목표−서기 최대°", "끝"], prow))
        self.summary["policy_segments"] = len(self.pol_segs)
        # 명령 구간 (정책 구간 안에서만 이어 붙임)
        segs = []
        for k0, k1 in self.pol_segs:
            i = k0
            while i < k1:
                j = i
                while j + 1 < k1 and np.array_equal(self.command[j + 1], self.command[i]):
                    j += 1
                kk = np.arange(i, j + 1)
                segs.append([f"{self.t[kk[0]]:.1f}~{self.t[kk[-1]] + self.dt:.1f}", f((kk.size) * self.dt, 1),
                             " ".join(f(v, 2) for v in self.command[i]), f(float(np.mean(self.gyro[kk, 2])), 3),
                             f(float(np.mean(self.tilt[kk])), 1), f(amax(self.tilt[kk]), 1),
                             f(float(np.mean(self.lean_fwd[kk])), 1), f(float(np.mean(self.lean_right[kk])), 1)])
                i = j + 1
        out.append("\n명령 구간 (속도 측정이 없어 vx는 영상으로 확인; 회전은 자이로 z로 비교 가능, 시뮬 0.8 → 0.85 rad/s)\n")
        out.append(table(["t s", "길이 s", "명령 vx vy wz", "자이로 z 평균 rad/s", "기울기 평균°", "최대°", "앞 숙임 평균°",
                          "오른쪽 기울기 평균°"], segs))
        self.sections.append(("5. 정책 입력·출력", "\n".join(out)))

    def drivers(self):
        """gui_frames.csv: GUI가 받은 Iq·Vbus·heartbeat (정책 중에도 GUI가 요청하는 값)."""
        fr = self.run.frames
        if fr is None:
            self.sections.append(("6. 드라이버 (GUI가 받은 값)", "gui_frames.csv 없음 (GUI 기록 추가 전 기록).\n"))
            return
        out = []
        node_j = {n: j for j, n in enumerate(self.node)}
        off, how = self._align(fr)
        out.append(f"시각 맞추기: {how}\n")
        tmap = (lambda tg: tg - off) if off is not None else None
        # heartbeat
        hb = fr.get("hb")
        if hb is not None and hb.size:
            first, changes, err_seen = {}, [], {}
            for n in sorted(set(hb[:, 1].astype(int))):
                r = hb[hb[:, 1] == n]
                prev = None
                for t, _, err, state in r:
                    cur = (int(err), int(state))
                    tt = tmap(t) if tmap else t
                    if prev is None:
                        first[n] = cur
                    elif cur != prev:
                        changes.append((tt, n, cur))
                    if cur[0] and (prev is None or cur[0] != prev[0]):
                        who = self.short[node_j[n]] if n in node_j else f"node {n}"
                        err_seen.setdefault(cur[0], []).append((tt, who))
                    prev = cur
            for code, lst in err_seen.items():
                ts = [x[0] for x in lst]
                self.add("⚠", f"axis_error {decode(code, AXIS_ERRORS)}: {len(lst)}개 관절, t {min(ts):.2f}"
                              f"{'' if max(ts) - min(ts) < 0.005 else f'~{max(ts):.2f}'} s ({', '.join(x[1] for x in sorted(lst))})"
                         + (" → 여러 드라이버가 동시에 = 전원(버스 전압) 문제" if code & 0x2 and len(lst) >= 3 else ""))
            groups = {}                        # 0.1 s 안에 같은 상태로 바뀐 node는 한 줄로
            for t, n, cur in sorted(changes):
                key = next((k for k in groups if k[1] == cur and abs(k[0] - t) < 0.1), None) or (t, cur)
                groups.setdefault(key, []).append(n)
            init = {}
            for n, cur in first.items():
                init.setdefault(cur, []).append(n)
            out.append("\nheartbeat 처음 상태: " + "; ".join(
                f"{STATE_NAMES.get(c[1], c[1])}{' 에러 ' + decode(c[0], AXIS_ERRORS) if c[0] else ''} "
                f"(node {','.join(map(str, ns))})" for c, ns in init.items()) + "\n")
            rows = [[f"{t:.2f}" + ("" if tmap else " (GUI 시각)"), STATE_NAMES.get(cur[1], cur[1]),
                     decode(cur[0], AXIS_ERRORS),
                     f"{len(ns)}개: " + ", ".join(self.short[node_j[n]] if n in node_j else str(n) for n in ns)]
                    for (t, cur), ns in groups.items()]
            out.append("heartbeat 상태 변화\n")
            out.append(table(["t s", "상태", "axis_error", "관절"], rows[:60]) if rows else "없음\n")
        for kind, tab, label in (("motor_err", MOTOR_ERRORS, "motor.error"), ("enc_err", ENCODER_ERRORS, "encoder.error")):
            r = fr.get(kind)
            if r is not None and r.size:
                for t, n, code, _ in r:
                    if code:
                        self.add("⚠", f"node {int(n)} {label} {decode(code, tab)}")
        # Iq
        iq = fr.get("iq")
        if iq is not None and iq.size:
            rows, over_iq = [], []
            for j in range(12):
                r = iq[iq[:, 1] == self.node[j]]
                if not r.size:
                    continue
                t, iset, imeas = r[:, 0], r[:, 2], r[:, 3]
                if tmap is not None and self.has_npz:
                    tt = tmap(t)
                    k = np.clip(np.searchsorted(self.t, tt) - 1, 0, self.N - 1)
                    inrun = (tt >= self.t[0]) & (tt <= self.t[-1] + self.dt)
                    polm = inrun & self.pol[k]
                else:
                    k, inrun, polm = None, np.ones(len(t), bool), np.ones(len(t), bool)
                sel = polm if polm.sum() >= 5 else inrun
                if sel.sum() < 2:
                    continue
                span = float(t[sel].max() - t[sel].min())
                rate = (sel.sum() - 1) / span if span > 0 else math.nan
                lim = self.cur_limit[j]
                near = float((np.abs(iset[sel]) >= 0.95 * lim).mean())
                slope = corr = math.nan
                if k is not None:
                    tau_iq = self.sign[j] * self.torque_sign[j] * imeas[sel] * KT[self.motor[j]] * 15.0
                    tau_pd = self.tau[k[sel], j]
                    ok = np.isfinite(tau_pd)
                    if ok.sum() > 10 and np.std(tau_pd[ok]) > 0.05:
                        corr = float(np.corrcoef(tau_pd[ok], tau_iq[ok])[0, 1])
                        slope = float(np.dot(tau_pd[ok], tau_iq[ok]) / np.dot(tau_pd[ok], tau_pd[ok]))
                rows.append([self.short[j], self.node[j], f(rate, 0), f(pct(np.abs(imeas[sel]), 95), 2), f(amax(np.abs(imeas[sel])), 2),
                             f(amax(np.abs(iset[sel])), 2), f(lim, 2), f(100 * near, 1), f(slope, 2), f(corr, 2)])
                if amax(np.abs(iset[sel])) > 1.1 * lim:
                    over_iq.append(f"{self.short[j]} {amax(np.abs(iset[sel])):.1f} A")
            if over_iq:
                cl6512 = float(self.train_limit[0]) / 15.0 / KT["6512"]
                cl5010 = float(self.train_limit[4]) / 15.0 / KT["5010"]
                self.add("⚠", f"Iq 명령이 학습 한계 {self.lim_txt}에 해당하는 전류(6512 {cl6512:.2f} A, 발목 {cl5010:.2f} A)를 넘음: "
                         + ", ".join(over_iq)
                         + f". 드라이버 전류 한계는 {self.drive_current:g} A 그대로 (ODrive 0.5.1의 CAN 0x00F는 속도 한계만 바꿈)"
                         + ("; 목표 자르기는 정책 주기마다 한 번이라 주기 사이에는 넘을 수 있음 (학습도 같음)." if self.clamp_nm else "."))
            out.append(f"\nIq (정책 중 표본; 없으면 기록 전체). {self.lim_txt} 전류 = 학습 토크 한계에 해당하는 전류, '넘음 %' = "
                       "|Iq 명령|이 그 95 % 이상인 표본. τ(Iq) = 방향 × torque_sign × Iq × kt × 15 를 드라이버 PD 추정 τ와 비교 "
                       "(기울기 ≈ 1, 상관 ≈ 1이면 PD 모델이 맞음)\n")
            out.append(table(["관절", "node", "표본 Hz", "|Iq| p95 A", "|Iq| 최대", "|Iq 명령| 최대", f"{self.lim_txt} 전류 A",
                              "넘음 %", "τ(Iq)/τ(PD) 기울기", "상관"], rows))
            self.iq = iq
        vb = fr.get("vbus")
        if vb is not None and vb.size:
            v = vb[:, 2]
            out.append(f"\n버스 전압 (Vbus, 2 s마다 node별): 최소 {v.min():.2f} V, 중앙 {np.median(v):.2f} V, 최대 {v.max():.2f} V, "
                       f"표본 {v.size}개\n")
            self.summary.update(vbus_min=float(v.min()), vbus_max=float(v.max()))
            if np.median(v) > 18 and v.min() < VBUS_LOW:
                self.add("⚠", f"배터리 전압 최소 {v.min():.2f} V < {VBUS_LOW} V (절차서 공통 중단 기준).")
            if v.max() > np.median(v) + 1.5:
                self.add("ℹ", f"버스 전압이 중앙값보다 {v.max() - np.median(v):.1f} V 높게 튄 표본 (회생 전류 확인).")
        self.vbus = vb
        self.sections.append(("6. 드라이버 (GUI가 받은 값)", "\n".join(out)))

    def _align(self, fr):
        """GUI 시각 t_gui → npz 시각. 엔코더 응답 값이 npz pos_turns와 정확히 같은 주기를 찾아 맞춘다."""
        self.offset = None
        if not self.has_npz:
            return None, "npz 없음 → GUI 시각 그대로"
        enc = fr.get("enc")
        pairs = []
        if enc is not None and enc.size:
            node_j = {n: j for j, n in enumerate(self.node)}
            lut = {}
            for j in range(12):
                vals, first, counts = np.unique(self.pos[:, j], return_index=True, return_counts=True)
                for v, i, c in zip(vals, first, counts):
                    if c == 1:
                        lut[(j, float(v))] = int(i)
            for t, n, a, _ in enc:
                j = node_j.get(int(n))
                if j is not None:
                    k = lut.get((j, float(a)))
                    if k is not None:
                        pairs.append(t - self.t[k])
        if len(pairs) >= 20:
            pairs = np.array(pairs)
            off = float(np.median(pairs))
            self.offset = off
            return off, (f"엔코더 응답 {len(pairs)}개가 npz 값과 정확히 일치 → t_npz = t_gui − {off:.3f} s "
                         f"(차이 p5–p95 {1000 * (np.percentile(pairs, 5) - off):+.0f}~{1000 * (np.percentile(pairs, 95) - off):+.0f} ms)")
        st = [(fnum(r["t_gui"]), fnum(r["cycles"])) for r in self.run.status]
        d = [tg - self.t[int(n) - 2] for tg, n in st if math.isfinite(n) and 2 <= n < self.N + 2]
        if d:
            off = float(np.median(d))
            self.offset = off
            return off, f"status.csv의 주기 번호로 맞춤 (±20 ms): t_npz = t_gui − {off:.3f} s"
        return None, "맞출 수 없음 → GUI 시각 그대로"

    def last_seconds(self, window=2.0):
        if not self.has_npz:
            return
        t_end = self.t[-1]
        ks = np.flatnonzero(self.t >= t_end - window)
        rows = []
        for k in ks[::max(1, int(round(0.2 / self.dt)))].tolist() + [int(ks[-1])]:
            e = np.abs(np.degrees(self.err[k])) if k > 0 else np.zeros(12)
            ta = np.abs(self.tau[k]) if k > 0 else np.zeros(12)
            je, jt = int(np.nanargmax(e)), int(np.nanargmax(ta))
            rows.append([f"{self.t[k]:.2f}", PHASE_KO[self.ph[k]], f(self.tilt[k], 1), f(self.lean_fwd[k], 1),
                         f(self.lean_right[k], 1), f"{self.short[je]} {e[je]:.0f}", f"{self.short[jt]} {ta[jt]:.1f}",
                         int(self.sat[k].sum()), f(self.loop_ms[k], 1), " ".join(f(v, 2) for v in self.command[k])])
        out = [f"마지막 {window:g} s (0.2 s 간격 + 마지막 주기). 멈춘 이유: `{self.reason}`. 안전 정지를 일으킨 주기는 기록되지 않음 "
               "(기록 전에 멈춤).\n",
               table(["t s", "단계", "기울기°", "앞 숙임°", "오른쪽°", "최대 |오차|°", "최대 |τ| N·m", "포화 관절 수", "작업 ms", "명령"],
                     rows)]
        self.sections.append(("7. 멈추기 직전", "\n".join(out)))

    def events_and_logs(self):
        out = []
        if self.run.events:
            out.append("events.txt\n```")
            for wall, tg, text in self.run.events:
                extra = ""
                if tg is not None and getattr(self, "offset", None) is not None:
                    extra = f" [npz {tg - self.offset:+.2f} s]"
                out.append(f"{wall}{extra} {text[:400]}")
            out.append("```")
        interesting = [ln for ln in self.run.gui_log if any(k in ln for k in ("⚠", "[Radxa]", "오류", "에러", "error", "Error",
                                                                           "끊", "IDLE", "보호"))]
        if interesting:
            out.append("\ngui_log.txt 중 경고·Radxa 줄\n```")
            out += interesting[-80:]
            out.append("```")
        if out:
            self.sections.append(("8. 이벤트·GUI 기록", "\n".join(out) + "\n"))

    # ------------------------------------------------------------------ 그림
    def plots(self, outdir):
        if not self.has_npz:
            return []
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams.update({"font.family": "Malgun Gothic", "axes.unicode_minus": False, "font.size": 8})
        t, files = self.t, []

        def shade(ax):
            i = 0
            while i < self.N:
                j = i
                while j + 1 < self.N and self.ph[j + 1] == self.ph[i]:
                    j += 1
                ax.axvspan(t[i], t[j] + self.dt, color=PHASE_COLOR[self.ph[i]], lw=0, zorder=0)
                i = j + 1

        # 1. 개요
        fig, ax = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
        shade(ax[0])
        ax[0].plot(t, self.tilt, "k", lw=1, label="기울기")
        ax[0].plot(t, self.lean_fwd, "C0", lw=0.8, label="앞 숙임")
        ax[0].plot(t, self.lean_right, "C3", lw=0.8, label="오른쪽 기울기")
        ax[0].axhline(S3_TILT_DEG, color="orange", ls=":", lw=0.8)
        ax[0].axhline(self.stop_tilt, color="red", ls=":", lw=0.8)
        ax[0].set_ylabel("deg")
        ax[0].legend(loc="upper left", ncol=3)
        shade(ax[1])
        ax[1].plot(t, self.command[:, 0], "C0", label="명령 vx m/s")
        ax[1].plot(t, self.command[:, 2], "C2", label="명령 wz rad/s")
        for a_, c, lb in ((0, "C1", "자이로 x"), (1, "C4", "자이로 y"), (2, "C2", "자이로 z")):
            ax[1].plot(t, self.gyro[:, a_], c, lw=0.6, alpha=0.6, ls="--" if a_ == 2 else "-", label=lb)
        ax[1].set_ylabel("m/s, rad/s")
        ax[1].legend(loc="upper left", ncol=5)
        shade(ax[2])
        ax[2].plot(t[1:], np.diff(t) * 1000, "k", lw=0.8, label="주기 ms")
        ax[2].plot(t, self.loop_ms, "C0", lw=0.8, label="작업 ms")
        ax[2].axhline(PERIOD_BAD_MS, color="red", ls=":", lw=0.8)
        ax[2].set_ylabel("ms")
        ax[2].legend(loc="upper left", ncol=2)
        shade(ax[3])
        ax[3].plot(t, np.abs(np.degrees(self.err)).max(axis=1), "C3", lw=0.8, label="최대 |오차| deg")
        ax[3].plot(t, np.abs(self.tau).max(axis=1) * 5, "C0", lw=0.8, label="최대 |τ| ×5 N·m")
        if hasattr(self, "stale"):
            ns = self.stale.sum(axis=1)
            ax[3].plot(t[ns > 0], ns[ns > 0] * 10, "k.", ms=4, label="정체 응답 수 ×10")
        ax[3].legend(loc="upper left", ncol=3)
        ax[3].set_xlabel("t [s] (회색 이동, 파랑 유지, 주황 정책)")
        fig.suptitle(f"{self.run.name}  멈춘 이유: {self.reason}")
        fig.tight_layout()
        files.append(self._save(fig, outdir / "overview.png"))

        # 2. 관절 (왼쪽 열 L, 오른쪽 열 R)
        def grid(fn, title, path):
            fig, axs = plt.subplots(6, 2, figsize=(12, 13), sharex=True)
            for j in range(12):
                a_ = axs[j % 6, j // 6]
                shade(a_)
                fn(a_, j)
                a_.set_title(self.short[j], fontsize=8, loc="left")
            axs[-1, 0].set_xlabel("t [s]")
            axs[-1, 1].set_xlabel("t [s]")
            fig.suptitle(title)
            fig.tight_layout(rect=(0, 0, 1, 0.98))
            return self._save(fig, path)

        def jplot(a_, j):
            a_.plot(t, np.degrees(self.target[:, j]), color="C1", lw=0.9, ls="--", label="정책 목표")
            if self.target_sent is not None:
                a_.plot(t, np.degrees(self.target_sent[:, j]), color="C2", lw=0.8, ls="-.", label="보낸 목표 (자름)")
            a_.plot(t, np.degrees(self.q[:, j]), "k", lw=0.8, label="실제")
            a_.axhline(math.degrees(self.stand[j]), color="gray", ls=":", lw=0.7)
            lo, hi = self.ranges.get(self.names[j], (None, None))
            ylo, yhi = a_.get_ylim()
            for lim in (lo, hi):
                if lim is not None and ylo - 15 < math.degrees(lim) < yhi + 15:
                    a_.axhline(math.degrees(lim), color="red", lw=0.8)
            if j == 0:
                a_.legend(loc="upper left", ncol=2)
        files.append(grid(jplot, "관절 각 (deg): 검정 실제, 주황 점선 목표, 회색 점선 서기 자세, 빨강 MJCF 범위 끝",
                          outdir / "joints.png"))

        def tplot(a_, j):
            a_.plot(t, self.tau[:, j], "C0", lw=0.7, label="PD 추정")
            a_.axhline(self.effort[j], color="red", ls=":", lw=0.8)
            a_.axhline(-self.effort[j], color="red", ls=":", lw=0.8)
            iq = getattr(self, "iq", None)
            if iq is not None and self.offset is not None:
                r = iq[iq[:, 1] == self.node[j]]
                if r.size:
                    a_.plot(r[:, 0] - self.offset, self.sign[j] * self.torque_sign[j] * r[:, 3] * KT[self.motor[j]] * 15.0,
                            ".", color="C3", ms=2, label="Iq 실측")
            if j == 0:
                a_.legend(loc="upper left", ncol=2)
        files.append(grid(tplot, "관절 토크 (N·m): 파랑 드라이버 PD 추정, 빨강 점 Iq 실측 (있으면), 점선 한계", outdir / "torque.png"))

        # 3. 마지막 3 s
        ks = self.t >= self.t[-1] - 3.0
        fig, axs = plt.subplots(4, 2, figsize=(12, 9), sharex=True)
        for c, side in enumerate((0, 6)):
            a_ = axs[0, c]
            if c == 0:
                a_.plot(t[ks], self.tilt[ks], "k", label="기울기°")
                a_.plot(t[ks], self.lean_fwd[ks], "C0", label="앞 숙임°")
                a_.plot(t[ks], self.lean_right[ks], "C3", label="오른쪽°")
            else:
                for ax_i, col, lb in ((0, "C1", "자이로 x"), (1, "C4", "자이로 y"), (2, "C2", "자이로 z")):
                    a_.plot(t[ks], np.degrees(self.gyro[ks, ax_i]), col, label=f"{lb} °/s")
            a_.legend(loc="upper left", ncol=3)
            for r, j in enumerate((side + 2, side + 3, side + 4)):
                a_ = axs[r + 1, c]
                a_.plot(t[ks], np.degrees(self.target[ks, j]), "C1--", lw=1, label="목표")
                a_.plot(t[ks], np.degrees(self.q[ks, j]), "k", lw=1, label="실제")
                a2 = a_.twinx()
                a2.plot(t[ks], self.tau[ks, j], "C0", lw=0.6, alpha=0.6)
                a2.set_ylim(-1.2 * self.effort[j], 1.2 * self.effort[j])
                a_.set_title(f"{self.short[j]} (파랑 τ, 오른쪽 축)", fontsize=8, loc="left")
        fig.suptitle(f"멈추기 직전 3 s: {self.reason}")
        fig.tight_layout()
        files.append(self._save(fig, outdir / "last_seconds.png"))

        # 4. 스펙트럼
        if getattr(self, "spec", None):
            freqs, ps = self.spec
            fig, axs = plt.subplots(1, 2, figsize=(12, 4), sharey=True)
            for j in range(12):
                axs[j // 6].plot(freqs[1:], np.log10(ps[j][1:] + 1e-9), lw=0.8, label=self.short[j])
            axs[0].set_ylabel("log10 파워")
            for a_ in axs:
                a_.axvline(8.0, color="red", ls=":", lw=0.8)
                a_.set_xlabel("Hz")
                a_.legend(fontsize=7)
            fig.suptitle("정책 중 목표 각도의 파워 스펙트럼 (8 Hz 위가 크면 떨림)")
            fig.tight_layout()
            files.append(self._save(fig, outdir / "spectrum.png"))
        return files

    @staticmethod
    def _save(fig, path):
        import matplotlib.pyplot as plt
        fig.savefig(path, dpi=110)
        plt.close(fig)
        return path.name

    # ------------------------------------------------------------------ 전체
    def run_all(self, outdir):
        steps = [self.overview, self.timing, self.imu, self.standing_check, self.joints_policy, self.policy_io, self.drivers,
                 self.last_seconds, self.events_and_logs]
        for s in steps:
            try:
                s()
            except Exception as e:  # noqa: BLE001 - 한 절이 실패해도 나머지는 만든다
                import traceback
                self.sections.append((f"(분석 실패: {s.__name__})", f"```\n{traceback.format_exc()}\n```\n"))
                self.add("⚠", f"분석 도구 오류 ({s.__name__}): {type(e).__name__}: {e}")
        outdir.mkdir(parents=True, exist_ok=True)
        try:
            figs = self.plots(outdir)
        except Exception as e:  # noqa: BLE001
            figs = []
            self.add("⚠", f"그림 만들기 실패: {type(e).__name__}: {e}")
        order = {"⚠": 0, "ℹ": 1, "✓": 2}
        fnd = sorted(self.findings, key=lambda x: order[x[0]])
        md = [f"# 정책 보행 기록 분석: {self.run.name}\n",
              "## 한눈에\n", "\n".join(f"- {lv} {tx}" for lv, tx in fnd) + "\n"]
        for title, body in self.sections:
            md.append(f"## {title}\n\n{body}")
        if figs:
            md.append("## 그림\n\n" + "\n".join(f"![{p}]({p})" for p in figs) + "\n")
        (outdir / "report.md").write_text("\n".join(md), encoding="utf-8")
        self.summary["findings"] = [f"{lv} {tx}" for lv, tx in fnd]
        (outdir / "summary.json").write_text(json.dumps(self.summary, indent=1, ensure_ascii=False, default=float),
                                             encoding="utf-8")
        return fnd


# ---------------------------------------------------------------------------------------------- 실행
def all_runs():
    return sorted(p for p in LOG_ROOT.glob("policy_walk_*") if p.is_dir())


def one_line(d):
    r = Run(d)
    z = r.npz
    if z is not None and len(z.get("time", [])):
        st = z["state"].astype(int)
        pol = (st == 2) | (np.abs(z["observation"]).sum(axis=1) > 0)
        dt = float(np.median(np.diff(z["time"]))) if len(z["time"]) > 1 else 0.02
        ph = f"이동 {(st == 0).sum() * dt:4.1f}s 유지 {((st == 1) & ~pol).sum() * dt:5.1f}s 정책 {pol.sum() * dt:5.1f}s"
        reason = str(z["reason"])
    elif z is not None:
        ph, reason = "주기 기록 0개 (시작 직후 멈춤)" + " " * 13, str(z.get("reason", ""))
    else:
        ph, reason = "npz 없음" + " " * 26, ""
    gui = r.stop_reason_gui or ""
    extra = " +frames" if r.frames is not None else ""
    return f"{d.name}  {ph}  {reason[:40]:40s}  {gui[:60]}{extra}"


def fetch(d):
    m = re.search(r"policy_walk_(\d{8}_\d{6}(?:_[A-Za-z0-9_\-]+)?)$", d.name)   # 11-25부터 _<정책>_<한계>Nm 붙음
    if not m or (d / "real_walk.npz").exists():
        return None
    remote = f"radxa:~/motor_bridge/logs/real_walk_{m.group(1)}.npz"
    r = subprocess.run(["scp", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", remote, str(d / "real_walk.npz")],
                       capture_output=True, text=True)
    return f"{remote} → {'가져옴' if r.returncode == 0 else '실패: ' + (r.stderr or '').strip()}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="*", help="기록 폴더 (없으면 가장 최근 것)")
    ap.add_argument("--list", action="store_true", help="기록 목록과 한 줄 요약만")
    ap.add_argument("--all", action="store_true", help="모든 기록 분석")
    ap.add_argument("--fetch", action="store_true", help="real_walk.npz가 없으면 Radxa에서 scp로 가져옴")
    ap.add_argument("--policy-dir", default=None,
                    help="정책 폴더 (없으면 기록의 meta.json policy_name, 그것도 없으면 radxa/policies/real_walk)")
    ap.add_argument("--drive-current-a", type=float, default=None,
                    help="드라이버 current_lim [A] (토크 추정을 자르는 값; 없으면 meta.json drive_current_lim_A, 그것도 없으면 18)")
    a = ap.parse_args()
    globals()["DRIVE_CURRENT_OVERRIDE"] = a.drive_current_a
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    runs = all_runs()
    if a.list:
        for d in runs:
            print(one_line(d))
        if not runs:
            print(f"{LOG_ROOT}에 policy_walk_* 기록이 없습니다")
        return
    dirs = [Path(x) for x in a.dirs] if a.dirs else (runs if a.all else runs[-1:])
    if not dirs:
        print(f"{LOG_ROOT}에 policy_walk_* 기록이 없습니다")
        return
    for d in dirs:
        if not d.is_dir():
            print(f"폴더가 아님: {d}")
            continue
        if a.fetch:
            note = fetch(d)
            if note:
                print(note)
        an = Analysis(Run(d), a.policy_dir)
        out = d / "analysis"
        fnd = an.run_all(out)
        print(f"== {d.name}  →  {out / 'report.md'}")
        for lv, tx in fnd:
            print(f"  {lv} {tx}")


if __name__ == "__main__":
    main()
