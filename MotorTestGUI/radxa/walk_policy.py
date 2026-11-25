"""Radxa: MotorTestGUI '정책 보행'. RL 세션의 run_walk_test()(real_walk_loop.py, 고치지 않은 사본)를 CAN 중계 안에서 돌린다.

can_bridge.py가 GUI의 POLICY_CMD를 받으면 이 파일을 불러온다 (numpy, onnxruntime, PyYAML이 필요한 것은 이 파일뿐).
run_walk_test()가 요구하는 것을 중계 위에 구현한다:
  io.read_joints()     12개에 Get Encoder Estimates(RTR)를 보내고 응답을 기다림 (최대 8 ms)
  io.send_targets()    Set Input Pos, vel_ff = 0, torque_ff = 0 (학습과 같게). 목표 자르기로 토크 한계
  io.enable()          CLOSED_LOOP
  io.idle()            IDLE 두 번
  io.errors()          heartbeat의 axis_error, 위치 제어가 아님, heartbeat·엔코더·IMU 끊김, 관절 추종 오차
  imu                  imu_reader.py가 내는 값 (중계가 줄마다 넣어 줌): quaternion, angular_velocity [deg/s]
정책 단계는 run_walk_test()가 부르는 함수로 안다: floor_ready()를 부르면 '서기 자세 유지', get_command()를 부르면 '정책'.

정책: policies/<이름>/policy.onnx + policy.yaml. GUI가 start에 이름을 넣어 고른다 (2026-11-25: real_clamp12 / 9 / 6,
11-24 real_walk). 관측·행동·관절 순서·서기 자세는 모두 같고, 바뀌는 것은 정책 파일과 자르기 한계뿐이다.

토크 한계 (목표 자르기, 2026-11-24): ODrive 0.5.1 CAN은 전류 한계를 바꾸지 못한다 (0x00F는 vel_limit만, 실측으로
Iq 명령이 18.00 A까지 나옴). 그래서 보내기 직전에 관절마다 목표를 자른다: 드라이버 PD 토크
Kp·(목표 − q) − Kd·q̇ 가 ±L 안에 들게 (q, q̇는 이번 주기에 읽은 값, Kp·Kd는 policy.yaml = 드라이버 게인).
다음 목표까지 20 ms 동안은 드라이버가 PD를 계속 돌리므로, 그 사이 관절이 밀리면 L을 넘을 수 있다 (근사).
  - real_clamp* 정책: 이 자르기를 넣고 학습했다. L = policy.yaml의 torque_clamp_nm (12, 9, 6). 항상 켬 (끌 수 없음).
  - real_walk (11-24): 드라이버 토크 6 N·m로 학습. GUI 체크박스로 켜고 끔, L = 6.
  직전 행동 관측은 자르기 전 정책 출력 그대로 (run_walk_test가 하는 대로, 학습과 같음).
기록: 같은 이름 + _io.npz (target_raw 정책 목표, target_sent 보낸 목표, clamped, t_send, t_read, pre_loop).

안전 (중계 쪽):
  - GUI가 멈춤을 보내거나, GUI에서 IDLE 프레임이 오거나(Esc·모두 힘 풀기), GUI 연결이 끊기거나 0.5 s 조용하면
    stop_requested()가 True가 되고 중계가 바로 모든 node를 IDLE로 둔다. run_walk_test()도 다음 주기에 끝나며 IDLE.
  - 멈춘 뒤에는 목표를 보내지 않고, enable()도 하지 않는다 (같은 잠금 안에서 확인).
  - 2026-11-25 RL 절차서의 정지 기준: 정책 중 몸통 기울기 30° 초과 (run_walk_test의 tilt_limit_deg, 전에는 20°),
    관절 추종 오차 |보낸(자른) 목표 − 엔코더| 30° 초과가 0.5 s 이어지면 멈춤 (errors()에서; 서기 자세 이동·유지 중에도).
"""
import importlib.util
import math
import os
import struct
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
POLICIES_DIR = os.path.join(HERE, "policies")
DEFAULT_POLICY = "real_walk"                 # GUI가 이름을 안 보냈을 때 (11-24 GUI와 같게)
LOG_DIR = os.path.join(HERE, "logs")

HEARTBEAT, SET_AXIS_STATE, GET_ENCODER_ESTIMATES, SET_INPUT_POS = 0x001, 0x007, 0x009, 0x00C
STATE_IDLE, STATE_CLOSED_LOOP = 1, 8
F_RTR = 2
READ_TIMEOUT_S = 0.008          # 12개 읽기 실측 p50 4 ms, p99 10 ms (latency 실험)
MISS_LIMIT = 5                  # 한 관절 엔코더 응답이 이만큼 연속으로 늦으면 멈춤
HEARTBEAT_TIMEOUT_S = 0.5       # heartbeat 10 Hz
ENABLE_GRACE_S = 0.6            # CLOSED_LOOP로 바뀌는 데 줄 시간
IMU_TIMEOUT_S = 0.1
TORQUE_LIMIT_NM = 6.0           # real_walk(11-24)의 학습 토크 한계 (관절). 목표 자르기로 근사
TILT_LIMIT_DEG = 30.0           # 정책 중 몸통 기울기 정지 (RL 절차서 2026-11-25, 전에는 20°)
TRACK_LIMIT_DEG = 30.0          # 관절 추종 오차 정지: |보낸(자른) 목표 − 엔코더|가 이 값을 넘은 채로
TRACK_TIME_S = 0.5              #   이 시간 이어지면 (RL 절차서 2026-11-25, 자르기 후 목표 기준)
REFERENCE_FIRST_ACTION = {      # 서기 자세·명령 0·똑바로·정지에서의 첫 행동 (RL 세션, onnxruntime 1.22.1)
    "real_walk": [-0.08326, 0.03751, -0.06501, 0.26643, 0.15041, 0.20079,
                  0.03692, -0.06964, -0.04834, -0.17642, -0.24063, -0.11180]}   # 새 정책은 policy.yaml에 있음


def missing_deps():
    return [m for m in ("numpy", "onnxruntime", "yaml") if importlib.util.find_spec(m) is None]


def _yaml_scalar(path, key):
    """policy.yaml에서 맨 위 'key: 값' 한 줄을 읽는다 (PyYAML 없이, 중계가 연결 정보를 만들 때 씀). 없으면 None."""
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                if line.startswith(key + ":"):
                    v = line.split(":", 1)[1].split("#", 1)[0].strip()
                    try:
                        return float(v)
                    except ValueError:
                        return v
    except OSError:
        return None
    return None


def valid_policy_name(name):
    return isinstance(name, str) and 0 < len(name) <= 64 and all(c.isalnum() or c in "_-" for c in name)


def policy_dir(name):
    """정책 폴더 경로. 이름이 이상하거나 파일이 없으면 ValueError."""
    if not valid_policy_name(name):
        raise ValueError(f"정책 이름이 이상합니다: {name!r}")
    d = os.path.join(POLICIES_DIR, name)
    if not all(os.path.exists(os.path.join(d, f)) for f in ("policy.onnx", "policy.yaml")):
        raise ValueError(f"Radxa에 정책 {name}이 없습니다 (끊고 다시 연결하면 GUI가 복사함)")
    return d


def list_policies():
    """Radxa에 있는 정책: [{name, torque_clamp_nm (없으면 None)}] (이름순)."""
    out = []
    try:
        names = sorted(os.listdir(POLICIES_DIR))
    except OSError:
        return out
    for name in names:
        d = os.path.join(POLICIES_DIR, name)
        if valid_policy_name(name) and all(os.path.exists(os.path.join(d, f)) for f in ("policy.onnx", "policy.yaml")):
            lim = _yaml_scalar(os.path.join(d, "policy.yaml"), "torque_clamp_nm")
            out.append({"name": name, "torque_clamp_nm": lim if isinstance(lim, float) else None})
    return out


def policy_files_ok():
    return bool(list_policies()) and \
        all(os.path.exists(os.path.join(HERE, f)) for f in ("real_walk.py", "real_walk_loop.py"))


class FeedImu:
    """run_walk_test()가 읽는 IMU (Bmi088Imu와 같은 속성). 중계가 imu_reader.py의 표본 줄로 채운다."""

    def __init__(self):
        import numpy as np
        self.np = np
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.angular_velocity = np.zeros(3)        # deg/s (Bmi088Imu와 같은 단위)
        self.t = None

    def update(self, d):
        q, w = d.get("q"), d.get("w")
        if d.get("state") != "ok" or q is None or w is None:
            return
        self.quaternion = self.np.array(q, dtype=float)
        self.angular_velocity = self.np.degrees(self.np.array(w, dtype=float))
        self.t = time.perf_counter()

    def age(self):
        return math.inf if self.t is None else time.perf_counter() - self.t


class BridgeIO:
    def __init__(self, bridge, joints, stop_ev, imu, joint_map, kp, kd, torque_clamp=True, limit_nm=TORQUE_LIMIT_NM,
                 track_limit_deg=TRACK_LIMIT_DEG, track_time_s=TRACK_TIME_S):
        import numpy as np
        self.np = np
        self.bridge = bridge
        self.joints = joints                       # 정책 순서: name, node, bus
        self.nodes = [int(j["node"]) for j in joints]
        self.bus = {int(j["node"]): int(j["bus"]) for j in joints}
        self.idx = {n: i for i, n in enumerate(self.nodes)}
        # 토크 한계 (목표 자르기): 관절 rad 기준. joint_map = real_walk.JointMap
        self.joint_map = joint_map
        self.kp, self.kd = np.asarray(kp, dtype=float), np.asarray(kd, dtype=float)
        self.torque_clamp = bool(torque_clamp)
        self.limit_nm = float(limit_nm)
        self.last_turns = self.last_tps = None     # 이번 주기에 읽은 값 (자르기 기준)
        self.last_clamped = 0                      # 마지막 send에서 잘린 관절 수
        self.clamp_cycles = 0                      # 하나라도 잘린 send 수
        self.log = {"t_read": [], "t_send": [], "pre_loop": [], "target_raw": [], "target_sent": [], "clamped": []}
        # 추종 오차 정지: |보낸 목표 − 이번 주기에 읽은 각도| (관절 rad)가 한계를 넘기 시작한 시각 (관절마다)
        self.track_limit = math.radians(float(track_limit_deg))
        self.track_time_s = float(track_time_s)
        self.track_since = [None] * 12
        self.track_err = np.zeros(12)                # 마지막 send의 |목표 − 각도| [rad]
        self.track_err_max = 0.0                     # 기록 전체에서 가장 큰 값 [rad]
        self.track_trip = None                       # (관절 번호, 오차 rad, 이어진 s)
        self.stop_ev = stop_ev
        self.imu = imu
        self.cond = threading.Condition()
        self.pos = [None] * 12
        self.vel = [0.0] * 12
        self.t_enc = [0.0] * 12
        self.miss = [0] * 12
        self.hb = getattr(bridge, "hb", None)      # 중계가 늘 모아 두는 heartbeat: node -> (시각, axis_error, state)
        if self.hb is None:
            self.hb = {}
        self.enabled_at = None
        self.reported = set()
        self.last_read_ms = 0.0
        self.last_period_ms = 0.0
        self._t_last_read = None
        self.targets = None                        # 마지막으로 보낸 목표 [turn]
        self.n_sent = 0

    # ---- 중계 장치 스레드에서
    def on_frame(self, bus, can_id, flags, dlc, data):
        if flags:                                  # 확장 ID, RTR 요청은 무시
            return
        node, cmd = can_id >> 5, can_id & 0x1F
        i = self.idx.get(node)
        if i is None:
            return
        if cmd == GET_ENCODER_ESTIMATES and dlc >= 8:
            pos, vel = struct.unpack_from("<ff", data, 0)
            with self.cond:
                self.pos[i], self.vel[i], self.t_enc[i] = pos, vel, time.perf_counter()
                self.cond.notify_all()
        elif cmd == HEARTBEAT and dlc >= 5 and not hasattr(self.bridge, "hb"):
            self.hb[node] = (time.monotonic(), struct.unpack_from("<I", data, 0)[0], data[4])

    def _send(self, node, cmd, payload=b"", rtr=False):
        dev = self.bridge.devices[self.bus[node]]
        dev.send((node << 5) | cmd, F_RTR if rtr else 0, 8, payload.ljust(8, b"\x00"))

    # ---- run_walk_test()가 부름
    def read_joints(self):
        t_req = time.perf_counter()
        self.log["t_read"].append(t_req)
        if self._t_last_read is not None:
            self.last_period_ms = 1000.0 * (t_req - self._t_last_read)
        self._t_last_read = t_req
        # 처음(아직 값이 없는 관절이 있을 때)은 0.2 s까지 다시 요청하며 기다림. 그 뒤로는 주기마다 8 ms까지
        first = any(p is None for p in self.pos)
        deadline = t_req + (0.2 if first else READ_TIMEOUT_S)
        t_ask = 0.0
        with self.cond:
            while True:
                now = time.perf_counter()
                if now - t_ask > 0.04 and (t_ask == 0.0 or first):
                    for i, n in enumerate(self.nodes):
                        if t_ask == 0.0 or self.t_enc[i] < t_req:
                            try:
                                self._send(n, GET_ENCODER_ESTIMATES, rtr=True)
                            except OSError:
                                pass
                    t_ask = now
                if all(t >= t_req for t in self.t_enc) or now >= deadline:
                    break
                self.cond.wait(min(deadline - now, 0.04))
            fresh = [t >= t_req for t in self.t_enc]
            pos, vel = list(self.pos), list(self.vel)
        self.last_read_ms = 1000.0 * (time.perf_counter() - t_req)
        for i, f in enumerate(fresh):
            self.miss[i] = 0 if f else self.miss[i] + 1
        if any(p is None for p in pos):
            missing = [self.joints[i]["name"] for i, p in enumerate(pos) if p is None]
            raise RuntimeError(f"엔코더 응답 없음: {', '.join(missing)}")
        self.last_turns, self.last_tps = self.np.array(pos, dtype=float), self.np.array(vel, dtype=float)
        return self.last_turns.copy(), self.last_tps.copy()

    def clamp(self, q_target):
        """관절 목표(rad)를 드라이버 PD 토크 Kp·(목표 − q) − Kd·q̇ 가 ±limit 안에 들게 자른다. (보낼 목표, 잘림 bool)"""
        np = self.np
        q_target = np.asarray(q_target, dtype=float)
        if not self.torque_clamp or self.last_turns is None:
            return q_target, np.zeros(12, dtype=bool)
        q = self.joint_map.joint_positions(self.last_turns).astype(float)
        qd = self.joint_map.joint_velocities(self.last_tps).astype(float)
        lo = q + (-self.limit_nm + self.kd * qd) / self.kp
        hi = q + (self.limit_nm + self.kd * qd) / self.kp
        sent = np.clip(q_target, lo, hi)
        return sent, np.abs(sent - q_target) > 1e-9

    def send_targets(self, turns):
        if self.stop_ev.is_set():
            return
        np = self.np
        raw = self.joint_map.joint_positions(np.asarray(turns, dtype=float)).astype(float)
        sent, clamped = self.clamp(raw)
        out = self.joint_map.motor_turns(sent) if clamped.any() else np.asarray(turns, dtype=float)
        self.log["t_send"].append(time.perf_counter())
        self.log["pre_loop"].append(self.n_sent == 0)       # run_walk_test가 루프 전에 한 번 (지금 자세 유지)
        self.log["target_raw"].append(raw)
        self.log["target_sent"].append(sent)
        self.log["clamped"].append(clamped)
        self.last_clamped = int(clamped.sum())
        self.clamp_cycles += bool(clamped.any())
        self._check_tracking(sent)
        self.targets = [float(t) for t in out]
        self.n_sent += 1
        for n, t in zip(self.nodes, self.targets):
            try:
                self._send(n, SET_INPUT_POS, struct.pack("<fhh", t, 0, 0))
            except OSError:
                pass

    def _check_tracking(self, sent):
        """보낸 목표(관절 rad)와 이번 주기에 읽은 각도의 차이가 한계를 track_time_s 넘게 이어졌는지 본다 (errors()가 보고)."""
        if self.last_turns is None:
            return
        np = self.np
        q = self.joint_map.joint_positions(self.last_turns).astype(float)
        err = np.abs(np.asarray(sent, dtype=float) - q)
        self.track_err = err
        self.track_err_max = max(self.track_err_max, float(err.max()))
        now = time.perf_counter()
        for i, e in enumerate(err):
            if e > self.track_limit:
                if self.track_since[i] is None:
                    self.track_since[i] = now
                elif now - self.track_since[i] >= self.track_time_s and self.track_trip is None:
                    self.track_trip = (i, float(e), now - self.track_since[i])
            else:
                self.track_since[i] = None

    def save_log(self, path):
        """보낸 목표 기록 (_io.npz). 주기 맞추기는 t_read(read_joints마다)·t_send(send마다)·pre_loop로."""
        np = self.np
        lg = self.log
        np.savez(path, t_read=np.array(lg["t_read"]), t_send=np.array(lg["t_send"]),
                 pre_loop=np.array(lg["pre_loop"], dtype=bool),
                 target_raw=np.array(lg["target_raw"]).reshape(-1, 12), target_sent=np.array(lg["target_sent"]).reshape(-1, 12),
                 clamped=np.array(lg["clamped"], dtype=bool).reshape(-1, 12),
                 torque_clamp=self.torque_clamp, limit_nm=self.limit_nm, kp=self.kp, kd=self.kd,
                 track_limit_deg=math.degrees(self.track_limit), track_time_s=self.track_time_s,
                 joints=np.array([j["name"] for j in self.joints]))

    def enable(self):
        with self.bridge.policy_lock:              # GUI의 IDLE과 엇갈려 다시 켜지지 않게
            if self.stop_ev.is_set():
                raise RuntimeError("시작 전에 멈춤 요청을 받았습니다")
            for n in self.nodes:
                self._send(n, SET_AXIS_STATE, struct.pack("<I", STATE_CLOSED_LOOP))
            self.enabled_at = time.monotonic()

    def idle(self):
        for _ in range(2):
            for n in self.nodes:
                try:
                    self._send(n, SET_AXIS_STATE, struct.pack("<I", STATE_IDLE))
                except OSError:
                    pass

    def errors(self):
        now = time.monotonic()
        out = []
        for j in self.joints:
            n = int(j["node"])
            hb = self.hb.get(n)
            if hb is None or now - hb[0] > HEARTBEAT_TIMEOUT_S:
                out.append(f"{j['name']} (node {n}) heartbeat 끊김")
            elif hb[1]:
                out.append(f"{j['name']} (node {n}) axis_error 0x{hb[1]:X}")
            elif self.enabled_at is not None and now - self.enabled_at > ENABLE_GRACE_S and hb[2] != STATE_CLOSED_LOOP:
                out.append(f"{j['name']} (node {n}) 상태 {hb[2]} (위치 제어 아님)")
        for i, m in enumerate(self.miss):
            if m >= MISS_LIMIT:
                out.append(f"{self.joints[i]['name']} 엔코더 응답 {m}주기 연속 없음")
        if self.imu.age() > IMU_TIMEOUT_S:
            out.append(f"IMU 값 끊김 ({1000 * min(self.imu.age(), 9.999):.0f} ms)")
        if self.track_trip is not None:
            i, e, dt = self.track_trip
            out.append(f"{self.joints[i]['name']} 추종 오차 {math.degrees(e):.0f}°가 {dt:.2f} s 이어짐 "
                       f"(기준 {math.degrees(self.track_limit):.0f}°·{self.track_time_s:g} s, 보낸 목표 − 엔코더)")
        return out


class PolicyRun:
    """정책 보행 한 번. start() → (서기 자세 이동 → 유지 → floor() 뒤 정책) → stop() 또는 안전 정지."""

    def __init__(self, bridge, opts, report):
        import numpy as np
        if HERE not in sys.path:
            sys.path.insert(0, HERE)
        import real_walk
        import real_walk_loop
        self.np, self.real_walk, self.loop = np, real_walk, real_walk_loop
        self.bridge, self.report = bridge, report
        joints = opts["joints"]
        import yaml
        self.policy_name = str(opts.get("policy") or DEFAULT_POLICY)
        self.policy_dir = policy_dir(self.policy_name)
        with open(os.path.join(self.policy_dir, "policy.yaml")) as f:
            pcfg = yaml.safe_load(f)
        order = list(pcfg["joints"])
        if [j["name"] for j in joints] != order:
            raise ValueError(f"관절 순서가 policy.yaml과 다릅니다: {[j['name'] for j in joints]}")
        self.joint_map = real_walk.JointMap([float(j["sign"]) for j in joints],
                                            [float(j["zero_turns"]) for j in joints])
        self.stop_ev = threading.Event()
        self.floor_ev = threading.Event()
        self.stand_ev = threading.Event()        # GUI '제자리에 서기': 정책 끄고 서기 자세로 돌아가 유지
        self.command = [0.0, 0.0, 0.0]          # 정책을 켜는 순간부터 쓰는 속도 명령 (GUI가 start에 넣어 보냄)
        v = opts.get("command")
        if isinstance(v, list) and len(v) == 3 and all(isinstance(x, (int, float)) and math.isfinite(x) for x in v):
            self.command = [float(x) for x in v]
        self.phase = "ramp"
        self.stop_reason = None
        self.imu = bridge.policy_imu
        if self.imu.age() > IMU_TIMEOUT_S:
            raise RuntimeError("IMU 값이 없습니다 (Radxa의 IMU가 '받는 중'이어야 함)")
        # 토크 한계: 목표 자르기 (Kp·Kd = policy.yaml joint_kp·joint_kd = 드라이버 게인의 관절 값)
        #   real_clamp*: policy.yaml의 torque_clamp_nm으로 학습 → 항상 켬, 그 값 (GUI 값은 무시)
        #   real_walk:   GUI 체크박스 (기본 켬, 6 N·m)
        train_clamp = pcfg.get("torque_clamp_nm")
        if isinstance(train_clamp, (int, float)) and train_clamp > 0:
            clamp_on, limit = True, float(train_clamp)
        else:
            clamp_on, limit = bool(opts.get("torque_clamp", True)), float(opts.get("limit_nm", TORQUE_LIMIT_NM))
        if not (math.isfinite(limit) and 0.5 <= limit <= 30.0):
            raise ValueError(f"토크 한계가 이상합니다: {limit}")
        self.trained_clamp = isinstance(train_clamp, (int, float)) and train_clamp > 0
        tilt = opts.get("tilt_limit_deg", TILT_LIMIT_DEG)
        self.tilt_limit = float(tilt) if isinstance(tilt, (int, float)) and 5.0 <= tilt <= 45.0 else TILT_LIMIT_DEG
        self.io = BridgeIO(bridge, joints, self.stop_ev, self.imu, self.joint_map, pcfg["joint_kp"], pcfg["joint_kd"],
                           torque_clamp=clamp_on, limit_nm=limit)
        os.makedirs(LOG_DIR, exist_ok=True)
        name = "".join(c for c in str(opts.get("log_name", "real_walk")) if c.isalnum() or c in "_-") or "real_walk"
        self.log_path = os.path.join(LOG_DIR, name + ".npz")
        self.io_log_path = os.path.join(LOG_DIR, name + "_io.npz")
        self.thread = threading.Thread(target=self._run, daemon=True, name="policy")
        self.done = threading.Event()

    # run_walk_test()가 부르는 함수
    def _floor_ready(self):
        self.phase = "hold"
        return self.floor_ev.is_set()

    def _get_command(self):
        self.phase = "walk"
        return list(self.command)

    def _stand_requested(self):
        """정책 중 매 주기 불림. '제자리에 서기'가 왔으면 한 번만 True: 정책을 끄고 서기 자세로 1 s 동안 돌아간 뒤 유지.
        다시 걸으려면 GUI에서 '정책 켜기' (floor)."""
        if not self.stand_ev.is_set():
            return False
        self.stand_ev.clear()
        self.floor_ev.clear()
        self.phase = "stand"
        return True

    def start(self):
        self.thread.start()
        threading.Thread(target=self._status_loop, daemon=True, name="policy-status").start()

    def stop(self, why):
        if self.stop_reason is None:
            self.stop_reason = why
        self.stop_ev.set()

    @property
    def active(self):
        return not self.done.is_set()

    def _run(self):
        reason = None
        try:
            reason = self.loop.run_walk_test(self.io, self.imu, self.joint_map, get_command=self._get_command,
                                             floor_ready=self._floor_ready, log_path=self.log_path,
                                             policy_dir=self.policy_dir, stop_requested=self.stop_ev.is_set,
                                             tilt_limit_deg=self.tilt_limit, stand_requested=self._stand_requested)
        except Exception as e:  # noqa: BLE001 - 시작 실패 등도 GUI에 알리고 힘을 푼다
            reason = f"오류: {type(e).__name__}: {e}"
            self.io.idle()
        finally:
            if reason == "stop requested" and self.stop_reason:
                reason = self.stop_reason
            try:
                self.io.save_log(self.io_log_path)
            except Exception as e:  # noqa: BLE001 - 보조 기록이 실패해도 멈춤 보고는 한다
                self.report({"type": "error", "msg": f"보낸 목표 기록(_io.npz)을 저장하지 못함: {e}"})
            self.phase = "stopped"
            self.done.set()
            size = os.path.getsize(self.log_path) if os.path.exists(self.log_path) else 0
            io_ok = os.path.exists(self.io_log_path)
            self.report({"type": "stopped", "reason": reason, "log": self.log_path if size else None,
                         "log_size": size, "io_log": self.io_log_path if io_ok else None,
                         "clamp_cycles": self.io.clamp_cycles, "sent": self.io.n_sent, "policy": self.policy_name,
                         "track_err_max_deg": round(math.degrees(self.io.track_err_max), 1)})

    def describe(self):
        """GUI에 'started'로 알리는 실제 설정 (Radxa가 정한 값)."""
        io = self.io
        return {"policy": self.policy_name, "torque_clamp": io.torque_clamp, "limit_nm": io.limit_nm,
                "trained_clamp": self.trained_clamp, "tilt_limit_deg": self.tilt_limit,
                "track_limit_deg": round(math.degrees(io.track_limit), 3), "track_time_s": io.track_time_s}

    def _status_loop(self):
        while not self.done.wait(0.1):
            io = self.io
            q = None
            if io.targets is not None:
                q = [round(float(x), 4) for x in self.joint_map.joint_positions(io.targets)]
            g = self.real_walk.projected_gravity(self.imu.quaternion)
            self.report({"type": "status", "phase": self.phase, "n": io.n_sent,
                         "ramp": min(1.0, io.n_sent / self.loop.RAMP_STEPS),
                         "read_ms": round(io.last_read_ms, 2), "period_ms": round(io.last_period_ms, 2),
                         "tilt": round(self.real_walk.tilt_deg(g), 2), "imu_age_ms": round(1000 * min(self.imu.age(), 9.9), 1),
                         "command": [round(c, 3) for c in self.command], "target_q": q,
                         "torque_clamp": io.torque_clamp, "limit_nm": io.limit_nm, "clamped_joints": io.last_clamped,
                         "clamp_frac": round(io.clamp_cycles / max(1, io.n_sent), 3),
                         "track_err_deg": round(math.degrees(float(io.track_err.max())), 1),
                         "track_err_max_deg": round(math.degrees(io.track_err_max), 1),
                         "miss": max(io.miss) if io.miss else 0})


def check_policy(name=DEFAULT_POLICY):
    """Radxa에서 정책 파일 점검 (모터 없이): RL 세션의 test_real_walk_policy.py와 같은 기준 출력, 한 스텝 시간."""
    import numpy as np
    import yaml
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    import real_walk
    d = policy_dir(name)
    with open(os.path.join(d, "policy.yaml")) as f:
        pcfg = yaml.safe_load(f)
    ref = pcfg.get("reference_first_action") or REFERENCE_FIRST_ACTION.get(name)
    if ref is None:
        raise ValueError(f"{name}: 기준 첫 행동(reference_first_action)이 없습니다")
    ref = np.array(ref, dtype=float)
    p = real_walk.RealWalkPolicy(d)
    p.reset()
    target = p.step([0.0, 0.0, 0.0], np.zeros(3), real_walk.projected_gravity([1.0, 0, 0, 0]), p.standing_pose,
                    np.zeros(12))
    diff = float(np.abs(p.last_action - ref).max())
    rng = np.random.default_rng(0)
    times = []
    p.reset()
    for _ in range(1000):
        q = p.standing_pose + rng.normal(0.0, 0.05, 12)
        t0 = time.perf_counter()
        p.step(rng.uniform(-0.3, 0.3, 3), rng.normal(0.0, 0.3, 3), real_walk.projected_gravity([1.0, 0, 0, 0]), q,
               rng.normal(0.0, 1.0, 12))
        times.append(time.perf_counter() - t0)
    times = 1000.0 * np.array(times)
    import onnxruntime
    lim = pcfg.get("torque_clamp_nm")
    return {"policy": name, "ok": diff < 1e-3, "diff": diff, "onnxruntime": onnxruntime.__version__,
            "torque_clamp_nm": lim if lim is not None else f"없음 (GUI 체크박스, {TORQUE_LIMIT_NM:g} N·m)",
            "first_target_deg": [round(float(x), 2) for x in np.degrees(target - p.standing_pose)],
            "step_ms_median": round(float(np.median(times)), 3), "step_ms_p99": round(float(np.percentile(times, 99)), 3)}


if __name__ == "__main__":       # Radxa에서: python3 ~/motor_bridge/walk_policy.py [정책 이름 ...] (없으면 전부)
    import json
    miss = missing_deps()
    if miss:
        print("없는 패키지:", miss)
        sys.exit(2)
    names = sys.argv[1:] or [p["name"] for p in list_policies()]
    results = [check_policy(n) for n in names]
    print(json.dumps(results, ensure_ascii=False, indent=1))
    print("ALL OK" if all(r["ok"] for r in results) else "MISMATCH")
    sys.exit(0 if all(r["ok"] for r in results) else 1)
