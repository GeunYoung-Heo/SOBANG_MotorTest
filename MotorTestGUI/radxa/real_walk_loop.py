"""50 Hz loop of the real walking test: ramp to the standing pose, hold it, run the policy, stop safely, save a log.

The team's CAN and IMU code plugs in through `io` and `imu`; this file has no driver code. Copy it next to
policy/real_walk.py (or keep the repository layout) and call run_walk_test() from the team's program.

io must provide (positions in motor turns, policy joint order of policy.yaml):
    read_joints() -> (pos_turns[12], vel_turns_per_s[12])   ODrive pos_estimate / vel_estimate, newest values
    send_targets(pos_turns[12])                             SET_INPUT_POS with vel_ff = 0, torque_ff = 0
    enable()                                                all axes CLOSED_LOOP_CONTROL (position mode)
    idle()                                                  all axes IDLE
    errors() -> list[str]                                   driver errors seen since the last call, [] if none
imu: Bmi088Imu (robot/bmi088.py) after is_calibrated, or anything with quaternion (w, x, y, z) and
angular_velocity (deg/s) in the base frame.

MotorTestGUI copy (RL repo 5be5fd2) + one addition, 2026-11-25: stand_requested(). When it returns True while the
policy runs, the policy stops, the targets move in STAND_STEPS from the measured pose to the standing pose and the
drivers' PD holds it ("hold" again; floor_ready() starts the policy again with policy.reset()). The tilt stop stays
on in that return ramp and the hold after it (the robot is on the floor). The log marks the return ramp as state 0.
With the default stand_requested this file behaves exactly like the RL repo's.
"""

import time

import numpy as np

try:  # real_walk.py copied next to this file
    from real_walk import RealWalkPolicy, projected_gravity, stand_targets, tilt_deg
except ImportError:  # repository layout: load policy/real_walk.py by path (the package need not be installed)
    import importlib.util
    import os
    _path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "berkeley_humanoid_lite_lowlevel", "policy",
                         "real_walk.py")
    _spec = importlib.util.spec_from_file_location("real_walk", _path)
    _real_walk = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_real_walk)
    RealWalkPolicy, projected_gravity = _real_walk.RealWalkPolicy, _real_walk.projected_gravity
    stand_targets, tilt_deg = _real_walk.stand_targets, _real_walk.tilt_deg

PERIOD = 0.02  # s, policy_dt
RAMP_STEPS = 200  # 4 s from the current pose to the standing pose (robot hanging)
TILT_LIMIT_DEG = 30.0  # 11-25: was 20; training ends episodes at 45 deg, the real robot recovered from 16.7 deg
MAX_LATE_CYCLES = 5  # consecutive cycles more than 10 ms late -> stop
STAND_STEPS = 50  # MotorTestGUI: 1 s from the walking pose back to the standing pose (stand_requested)


def run_walk_test(io, imu, joint_map, get_command, floor_ready, log_path="real_walk_log.npz",
                  policy_dir="policies/real_walk", stop_requested=lambda: False, tilt_limit_deg=TILT_LIMIT_DEG,
                  stand_requested=lambda: False):
    """Run the test until stop_requested(), a safety stop or Ctrl-C; then all axes go IDLE and the log is saved.

    get_command() -> (vx m/s, vy m/s, wz rad/s), called every policy step (0 at first; clipped to the training ranges).
    floor_ready() -> True once the operator has lowered the robot onto the floor; the policy starts at that step.
    """
    policy = RealWalkPolicy(policy_dir)
    log = {key: [] for key in ("time", "state", "command", "observation", "action", "target", "pos_turns",
                               "vel_turns", "quat", "gyro", "loop_ms")}
    reason = "stop requested"
    turns, _ = io.read_joints()
    io.send_targets(turns)  # hold the current pose so enabling does not jump
    io.enable()
    plan = list(stand_targets(joint_map.joint_positions(turns), policy.standing_pose, RAMP_STEPS))
    state, late = "ramp", 0
    on_floor = False  # MotorTestGUI: set once the policy has run (the stand ramp and hold after it keep the tilt stop)
    t_start = next_time = time.perf_counter()
    try:
        while not stop_requested():
            cycle_start = time.perf_counter()
            turns, turns_per_s = io.read_joints()
            q, qd = joint_map.joint_positions(turns), joint_map.joint_velocities(turns_per_s)
            quat = np.array(imu.quaternion, dtype=np.float64)
            gyro = np.deg2rad(np.array(imu.angular_velocity, dtype=np.float32))
            gravity = projected_gravity(quat)
            command = np.zeros(3, dtype=np.float32)
            if state == "walk" and stand_requested():  # MotorTestGUI: policy off, back to the standing pose on PD
                plan = list(stand_targets(q, policy.standing_pose, STAND_STEPS))
                state = "stand"
            phase = state  # the state that makes this cycle's target

            if on_floor and state in ("stand", "hold") and tilt_deg(gravity) > tilt_limit_deg:
                reason = f"tilt {tilt_deg(gravity):.1f} deg"
                break
            if state in ("ramp", "stand"):
                target = plan.pop(0)
                if not plan:
                    state = "hold"
            elif state == "hold":  # standing pose on the drivers' PD only, until the robot is on the floor
                target = policy.standing_pose
                if floor_ready():
                    policy.reset()
                    state = "walk"
            if state == "walk":
                on_floor = True
                if tilt_deg(gravity) > tilt_limit_deg:
                    reason = f"tilt {tilt_deg(gravity):.1f} deg"
                    break
                command = np.asarray(get_command(), dtype=np.float32)
                target = policy.step(command, gyro, gravity, q, qd)

            io.send_targets(joint_map.motor_turns(target))
            errors = io.errors()
            if errors:
                reason = f"driver errors {errors}"
                break

            log["time"].append(cycle_start - t_start)
            log["state"].append(("ramp", "hold", "walk").index("ramp" if phase == "stand" else phase))
            log["command"].append(command)
            log["observation"].append(policy.observation_buffer[0].copy() if state == "walk" else np.zeros(45))
            log["action"].append(policy.last_action.copy() if state == "walk" else np.zeros(12))
            log["target"].append(np.asarray(target, dtype=np.float32))
            log["pos_turns"].append(np.asarray(turns, dtype=np.float64))
            log["vel_turns"].append(np.asarray(turns_per_s, dtype=np.float64))
            log["quat"].append(quat)
            log["gyro"].append(gyro)
            log["loop_ms"].append(1000.0 * (time.perf_counter() - cycle_start))

            next_time += PERIOD
            delay = next_time - time.perf_counter()
            if delay > 0.0:
                time.sleep(delay)
                late = 0
            elif delay < -0.01:
                late += 1
                if late >= MAX_LATE_CYCLES:
                    reason = f"{late} cycles in a row more than 10 ms late"
                    break
                next_time = time.perf_counter()  # do not try to catch up
    except KeyboardInterrupt:
        reason = "Ctrl-C"
    finally:
        io.idle()
        np.savez(log_path, reason=reason, joints=policy.joint_names, standing_pose=policy.standing_pose,
                 signs=joint_map.signs, zero_turns=joint_map.zero_turns,
                 **{key: np.array(value) for key, value in log.items()})
        print(f"stopped: {reason}; log -> {log_path}")
    return reason
