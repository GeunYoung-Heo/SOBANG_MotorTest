"""Radxa: MotorTestGUI '정책 보행'. RL 세션의 run_walk_test()(real_walk_loop.py, 고치지 않은 사본)를 CAN 중계 안에서 돌린다.

can_bridge.py가 GUI의 POLICY_CMD를 받으면 이 파일을 불러온다 (numpy, onnxruntime, PyYAML이 필요한 것은 이 파일뿐).
run_walk_test()가 요구하는 것을 중계 위에 구현한다:
  io.read_joints()     12개에 Get Encoder Estimates(RTR)를 보내고 응답을 기다림 (최대 8 ms)
  io.send_targets()    Set Input Pos, vel_ff = 0, torque_ff = 0 (학습과 같게). (선택) 목표 자르기로 토크 6 N·m 한계
  io.enable()          CLOSED_LOOP
  io.idle()            IDLE 두 번
  io.errors()          heartbeat의 axis_error, 위치 제어가 아님, heartbeat·엔코더·IMU 끊김
  imu                  imu_reader.py가 내는 값 (중계가 줄마다 넣어 줌): quaternion, angular_velocity [deg/s]
정책 단계는 run_walk_test()가 부르는 함수로 안다: floor_ready()를 부르면 '서기 자세 유지', get_command()를 부르면 '정책'.

토크 한계 (목표 자르기, 2026-11-24): ODrive 0.5.1 CAN은 전류 한계를 바꾸지 못한다 (0x00F는 vel_limit만, 실측으로
Iq 명령이 18.00 A까지 나옴). 그래서 보내기 직전에 관절마다 목표를 자른다: 드라이버 PD 토크
Kp·(목표 − q) − Kd·q̇ 가 ±6 N·m 안에 들게 (q, q̇는 이번 주기에 읽은 값, Kp·Kd는 policy.yaml = 드라이버 게인).
다음 목표까지 20 ms 동안은 드라이버가 PD를 계속 돌리므로, 그 사이 관절이 밀리면 6 N·m를 넘을 수 있다 (근사).
기록: 같은 이름 + _io.npz (target_raw 정책 목표, target_sent 보낸 목표, clamped, t_send, t_read, pre_loop).

안전 (중계 쪽):
  - GUI가 멈춤을 보내거나, GUI에서 IDLE 프레임이 오거나(Esc·모두 힘 풀기), GUI 연결이 끊기거나 0.5 s 조용하면
    stop_requested()가 True가 되고 중계가 바로 모든 node를 IDLE로 둔다. run_walk_test()도 다음 주기에 끝나며 IDLE.
  - 멈춘 뒤에는 목표를 보내지 않고, enable()도 하지 않는다 (같은 잠금 안에서 확인).
"""
import importlib.util
import math
import os
import struct
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
POLICY_DIR = os.path.join(HERE, "policies", "real_walk")
LOG_DIR = os.path.join(HERE, "logs")

HEARTBEAT, SET_AXIS_STATE, GET_ENCODER_ESTIMATES, SET_INPUT_POS = 0x001, 0x007, 0x009, 0x00C
STATE_IDLE, STATE_CLOSED_LOOP = 1, 8
F_RTR = 2
READ_TIMEOUT_S = 0.008          # 12개 읽기 실측 p50 4 ms, p99 10 ms (latency 실험)
MISS_LIMIT = 5                  # 한 관절 엔코더 응답이 이만큼 연속으로 늦으면 멈춤
HEARTBEAT_TIMEOUT_S = 0.5       # heartbeat 10 Hz
ENABLE_GRACE_S = 0.6            # CLOSED_LOOP로 바뀌는 데 줄 시간
IMU_TIMEOUT_S = 0.1
TORQUE_LIMIT_NM = 6.0           # 학습 토크 한계 (관절). 목표 자르기로 근사


def missing_deps():
    return [m for m in ("numpy", "onnxruntime", "yaml") if importlib.util.find_spec(m) is None]


def policy_files_ok():
    return all(os.path.exists(os.path.join(POLICY_DIR, f)) for f in ("policy.onnx", "policy.yaml")) and \
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
    def __init__(self, bridge, joints, stop_ev, imu, joint_map, kp, kd, torque_clamp=True, limit_nm=TORQUE_LIMIT_NM):
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
        self.targets = [float(t) for t in out]
        self.n_sent += 1
        for n, t in zip(self.nodes, self.targets):
            try:
                self._send(n, SET_INPUT_POS, struct.pack("<fhh", t, 0, 0))
            except OSError:
                pass

    def save_log(self, path):
        """보낸 목표 기록 (_io.npz). 주기 맞추기는 t_read(read_joints마다)·t_send(send마다)·pre_loop로."""
        np = self.np
        lg = self.log
        np.savez(path, t_read=np.array(lg["t_read"]), t_send=np.array(lg["t_send"]),
                 pre_loop=np.array(lg["pre_loop"], dtype=bool),
                 target_raw=np.array(lg["target_raw"]).reshape(-1, 12), target_sent=np.array(lg["target_sent"]).reshape(-1, 12),
                 clamped=np.array(lg["clamped"], dtype=bool).reshape(-1, 12),
                 torque_clamp=self.torque_clamp, limit_nm=self.limit_nm, kp=self.kp, kd=self.kd,
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
        with open(os.path.join(POLICY_DIR, "policy.yaml")) as f:
            pcfg = yaml.safe_load(f)
        order = list(pcfg["joints"])
        if [j["name"] for j in joints] != order:
            raise ValueError(f"관절 순서가 policy.yaml과 다릅니다: {[j['name'] for j in joints]}")
        self.joint_map = real_walk.JointMap([float(j["sign"]) for j in joints],
                                            [float(j["zero_turns"]) for j in joints])
        self.stop_ev = threading.Event()
        self.floor_ev = threading.Event()
        self.command = [0.0, 0.0, 0.0]          # 정책을 켜는 순간부터 쓰는 속도 명령 (GUI가 start에 넣어 보냄)
        v = opts.get("command")
        if isinstance(v, list) and len(v) == 3 and all(isinstance(x, (int, float)) and math.isfinite(x) for x in v):
            self.command = [float(x) for x in v]
        self.phase = "ramp"
        self.stop_reason = None
        self.imu = bridge.policy_imu
        if self.imu.age() > IMU_TIMEOUT_S:
            raise RuntimeError("IMU 값이 없습니다 (Radxa의 IMU가 '받는 중'이어야 함)")
        # 토크 한계 6 N·m: 목표 자르기 (Kp·Kd = policy.yaml joint_kp·joint_kd = 드라이버 게인의 관절 값)
        clamp_on = bool(opts.get("torque_clamp", True))
        self.io = BridgeIO(bridge, joints, self.stop_ev, self.imu, self.joint_map, pcfg["joint_kp"], pcfg["joint_kd"],
                           torque_clamp=clamp_on, limit_nm=float(opts.get("limit_nm", TORQUE_LIMIT_NM)))
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
                                             policy_dir=POLICY_DIR, stop_requested=self.stop_ev.is_set)
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
                         "clamp_cycles": self.io.clamp_cycles, "sent": self.io.n_sent})

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
                         "torque_clamp": io.torque_clamp, "clamped_joints": io.last_clamped,
                         "clamp_frac": round(io.clamp_cycles / max(1, io.n_sent), 3),
                         "miss": max(io.miss) if io.miss else 0})


def check_policy():
    """Radxa에서 정책 파일 점검 (모터 없이): RL 세션의 test_real_walk_policy.py와 같은 기준 출력, 한 스텝 시간."""
    import numpy as np
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    import real_walk
    ref = np.array([-0.08326, 0.03751, -0.06501, 0.26643, 0.15041, 0.20079,
                    0.03692, -0.06964, -0.04834, -0.17642, -0.24063, -0.11180])
    p = real_walk.RealWalkPolicy(POLICY_DIR)
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
    return {"ok": diff < 1e-3, "diff": diff, "onnxruntime": onnxruntime.__version__,
            "first_target_deg": [round(float(x), 2) for x in np.degrees(target - p.standing_pose)],
            "step_ms_median": round(float(np.median(times)), 3), "step_ms_p99": round(float(np.percentile(times, 99)), 3)}


if __name__ == "__main__":       # Radxa에서: python3 ~/motor_bridge/walk_policy.py
    import json
    miss = missing_deps()
    if miss:
        print("없는 패키지:", miss)
        sys.exit(2)
    print(json.dumps(check_policy(), ensure_ascii=False, indent=1))
