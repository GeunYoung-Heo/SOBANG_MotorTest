"""Runs the real walking test policy (policies/real_walk, task Velocity-ASMR-Biped-Real-v0) on the robot computer.

Needs numpy, onnxruntime and PyYAML only (no torch, no omegaconf), so this file can be copied to the robot on its own.
It does not talk to the motor drivers or the IMU. The caller reads them and converts the readings to the policy's
joint convention:
    order   policy.yaml `joints` (left hip roll, hip yaw, hip pitch, knee, ankle pitch, ankle roll, then the right leg)
    sign    + as in the simulation model (see the first-walk procedure's joint figures)
    units   rad and rad/s on the joint side, angle 0 = the model's zero pose
JointMap does this for drivers that report motor turns. The caller sends the returned joint targets as position
commands with velocity and torque feedforward 0, every 20 ms (policy_dt).

Observation (45 values, the policy group of Velocity-ASMR-Biped-Real-v0):
    0-2    velocity command vx, vy (m/s), wz (rad/s)
    3-5    base angular velocity (rad/s, base frame: x forward, y left, z up)
    6-8    projected gravity: unit gravity direction in the base frame, (0, 0, -1) when upright
    9-20   joint positions minus the standing pose (rad)
    21-32  joint velocities (rad/s)
    33-44  previous action (clipped, before scaling); zeros when the policy starts
Action (12): joint target = standing pose + 0.25 * clip(action, -20, 20).

Usage:
    policy = RealWalkPolicy("policies/real_walk")
    for target in stand_targets(q_now, policy.standing_pose, 200):   # 4 s ramp to the standing pose
        send(target); wait 20 ms
    policy.reset()
    every 20 ms:
        target = policy.step(command, gyro_rad_s, projected_gravity(quat), q, qd)
        send(target)
"""

import os

import numpy as np
import onnxruntime as ort
import yaml

# velocity command ranges of the training (biped env_cfg: lin_vel_x, lin_vel_y, ang_vel_z)
COMMAND_LIMITS = np.array([0.5, 0.25, 1.0], dtype=np.float32)


def projected_gravity(quat: np.ndarray) -> np.ndarray:
    """Unit gravity direction in the base frame from the base orientation quat (w, x, y, z, base to world), e.g.
    Bmi088Imu.quaternion; the same as quat_rotate_inverse(quat, (0, 0, -1))."""
    w, x, y, z = np.asarray(quat, dtype=np.float64)
    return -np.array([2.0 * (x * z - w * y), 2.0 * (w * x + y * z), w * w - x * x - y * y + z * z], dtype=np.float32)


def tilt_deg(gravity: np.ndarray) -> float:
    """Base tilt from upright (deg) from the projected gravity."""
    return float(np.degrees(np.arccos(np.clip(-gravity[2] / np.linalg.norm(gravity), -1.0, 1.0))))


def stand_targets(start: np.ndarray, goal: np.ndarray, steps: int = 200) -> np.ndarray:
    """Joint targets that move linearly from start to goal in `steps` policy steps (BHL's RL_INIT; 200 = 4 s at
    50 Hz). Row k is sent at step k; the last row is goal."""
    alpha = np.arange(1, steps + 1, dtype=np.float32)[:, None] / steps
    return (1.0 - alpha) * np.asarray(start, dtype=np.float32) + alpha * np.asarray(goal, dtype=np.float32)


class JointMap:
    """Driver readings in motor turns <-> policy joint angles, per joint in policy order:
        q = sign * (turns - zero_turns) * 2 pi / gear_ratio,   turns = sign * q * gear_ratio / (2 pi) + zero_turns

    sign (+1 or -1) makes + the simulation model's direction; zero_turns is the motor position (turns) at the model's
    joint angle 0, from the team's zeroing. The ODrive pos_estimate and vel_estimate are motor turns and turns/s.
    """

    def __init__(self, signs, zero_turns, gear_ratio: float = 15.0):
        self.signs = np.asarray(signs, dtype=np.float64)
        self.zero_turns = np.asarray(zero_turns, dtype=np.float64)
        self.scale = 2.0 * np.pi / gear_ratio
        if self.signs.shape != (12,) or self.zero_turns.shape != (12,) or not np.all(np.abs(self.signs) == 1.0):
            raise ValueError("signs (+1/-1) and zero_turns need 12 values in policy order")

    def joint_positions(self, turns) -> np.ndarray:
        return (self.signs * (np.asarray(turns, dtype=np.float64) - self.zero_turns) * self.scale).astype(np.float32)

    def joint_velocities(self, turns_per_s) -> np.ndarray:
        return (self.signs * np.asarray(turns_per_s, dtype=np.float64) * self.scale).astype(np.float32)

    def motor_turns(self, joint_positions) -> np.ndarray:
        return self.signs * np.asarray(joint_positions, dtype=np.float64) / self.scale + self.zero_turns


class RealWalkPolicy:
    """The walking policy with its observation and action processing (the same as RlController.update)."""

    def __init__(self, policy_dir: str = "policies/real_walk"):
        with open(os.path.join(policy_dir, "policy.yaml")) as f:
            cfg = yaml.safe_load(f)
        self.joint_names = list(cfg["joints"])
        self.standing_pose = np.array(cfg["default_joint_positions"], dtype=np.float32)
        self.action_scale = float(cfg["action_scale"])
        self.action_limits = (float(cfg["action_limit_lower"]), float(cfg["action_limit_upper"]))
        self.policy_dt = float(cfg["policy_dt"])
        self.num_observations = int(cfg["num_observations"])
        assert self.num_observations == 45 and len(self.joint_names) == 12 and int(cfg["history_length"]) == 0

        self.session = ort.InferenceSession(os.path.join(policy_dir, "policy.onnx"),
                                            providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.observation_buffer = np.zeros((1, self.num_observations), dtype=np.float32)
        self.prev_actions = np.zeros(12, dtype=np.float32)
        self.last_action = np.zeros(12, dtype=np.float32)  # raw policy output of the last step, for logging

    def reset(self):
        """Call right before the first step (previous action = 0, as at the start of a training episode)."""
        self.prev_actions[:] = 0.0

    def observation(self, command, base_ang_vel, gravity, joint_pos, joint_vel) -> np.ndarray:
        """The 45 policy inputs; the command is clipped to the training ranges."""
        command = np.clip(np.asarray(command, dtype=np.float32), -COMMAND_LIMITS, COMMAND_LIMITS)
        return np.concatenate([
            command,
            np.asarray(base_ang_vel, dtype=np.float32),
            np.asarray(gravity, dtype=np.float32),
            np.asarray(joint_pos, dtype=np.float32) - self.standing_pose,
            np.asarray(joint_vel, dtype=np.float32),
            self.prev_actions,
        ])

    def step(self, command, base_ang_vel, gravity, joint_pos, joint_vel) -> np.ndarray:
        """One policy step: returns the 12 joint targets (rad, policy order and signs).

        command (vx m/s, vy m/s, wz rad/s); base_ang_vel rad/s in the base frame (Bmi088Imu.angular_velocity is deg/s);
        gravity from projected_gravity(); joint_pos rad and joint_vel rad/s from JointMap or the team's conversion.
        """
        self.observation_buffer[0] = self.observation(command, base_ang_vel, gravity, joint_pos, joint_vel)
        self.last_action[:] = self.session.run(None, {self.input_name: self.observation_buffer})[0][0]
        self.prev_actions[:] = np.clip(self.last_action, *self.action_limits)
        return self.standing_pose + self.action_scale * self.prev_actions
