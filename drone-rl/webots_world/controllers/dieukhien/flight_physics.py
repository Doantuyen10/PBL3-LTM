"""Cascaded PID flight controller and 6-DOF physics simulator for DJI Mavic 2 Pro in Webots."""
import math
import numpy as np


def _clamp(val, low, high):
    return max(low, min(high, float(val)))


def _wrap_pi(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class MavicPhysicsPilot:
    """Controls Mavic2Pro purely via 4 propeller motor velocities and onboard IMU/GPS/Gyro."""

    # Empirical Mavic 2 Pro physical constants from Cyberbotics mavic2pro.c
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0
    MAX_MOTOR_VEL = 570.0  # Mavic2Pro.proto RotationalMotor maxVelocity is 576 rad/s

    # Outer-loop horizontal position/velocity cascade gains
    KP_POS = 0.65
    KP_VEL = 0.55
    MAX_HORIZ_SPEED = 5.0
    MAX_PITCH_DIST = 1.8
    MAX_ROLL_DIST = 1.2
    MAX_YAW_DIST = 0.8

    def __init__(self, robot, timestep_ms):
        self.robot = robot
        self.timestep_ms = int(timestep_ms)
        self.dt = self.timestep_ms / 1000.0

        self.imu = robot.getDevice('inertial unit')
        self.gps = robot.getDevice('gps')
        self.gyro = robot.getDevice('gyro')
        self.compass = robot.getDevice('compass')
        self.camera_roll = robot.getDevice('camera roll')
        self.camera_pitch = robot.getDevice('camera pitch')
        self.front_left_led = robot.getDevice('front left led')
        self.front_right_led = robot.getDevice('front right led')

        self.fl_motor = robot.getDevice('front left propeller')
        self.fr_motor = robot.getDevice('front right propeller')
        self.rl_motor = robot.getDevice('rear left propeller')
        self.rr_motor = robot.getDevice('rear right propeller')
        self.motors = (self.fl_motor, self.fr_motor, self.rl_motor, self.rr_motor)

        self.available = (
            self.imu is not None
            and self.gps is not None
            and self.gyro is not None
            and all(m is not None for m in self.motors)
        )

        self.prev_pos = None
        self.vel = np.zeros(3, dtype=float)
        self.yaw_ref = None
        self.last_motor_speeds = (0.0, 0.0, 0.0, 0.0)

        if self.available:
            self.imu.enable(self.timestep_ms)
            self.gps.enable(self.timestep_ms)
            self.gyro.enable(self.timestep_ms)
            if self.compass is not None:
                self.compass.enable(self.timestep_ms)
            for motor in self.motors:
                motor.setPosition(float('inf'))
                motor.setVelocity(1.0)

    def read_state(self):
        """Read 3D position, velocity, attitude (roll, pitch, yaw) and body angular rates."""
        pos = np.asarray(self.gps.getValues(), dtype=float)
        roll, pitch, yaw = [float(x) for x in self.imu.getRollPitchYaw()]
        roll_vel, pitch_vel, yaw_vel = [float(x) for x in self.gyro.getValues()]

        if self.prev_pos is None or not np.isfinite(self.prev_pos).all():
            self.vel = np.zeros(3, dtype=float)
        else:
            inst_vel = (pos - self.prev_pos) / max(self.dt, 1e-6)
            self.vel = 0.7 * self.vel + 0.3 * inst_vel
        self.prev_pos = pos.copy()

        if self.yaw_ref is None and np.isfinite(yaw):
            self.yaw_ref = yaw

        return pos, self.vel.copy(), (roll, pitch, yaw), (roll_vel, pitch_vel, yaw_vel)

    def step_attitude_and_thrust(self, target_pos, target_vel_xy=None, sim_time=0.0):
        """Compute 4-rotor velocities to track target_pos [x, y, z] and target_vel_xy [vx, vy]."""
        pos, vel, (roll, pitch, yaw), (roll_vel, pitch_vel, yaw_vel) = self.read_state()
        target_pos = np.asarray(target_pos, dtype=float)
        if target_vel_xy is None:
            target_vel_xy = np.zeros(2, dtype=float)
        else:
            target_vel_xy = np.asarray(target_vel_xy, dtype=float)

        # Stabilize gimbal and blink LEDs if available
        if self.camera_roll is not None:
            self.camera_roll.setPosition(_clamp(-0.115 * roll_vel, -0.5, 0.5))
        if self.camera_pitch is not None:
            self.camera_pitch.setPosition(_clamp(-0.1 * pitch_vel + 0.7, -0.5, 1.7))
        if self.front_left_led is not None and self.front_right_led is not None:
            led_state = int(sim_time) % 2
            self.front_left_led.set(led_state)
            self.front_right_led.set(1 - led_state)

        # 1. Vertical / Altitude PID (with smooth high-climb boost up to 30 m)
        alt_err = float(target_pos[2] - pos[2])
        clamped_diff_alt = _clamp(alt_err + self.K_VERTICAL_OFFSET, -1.0, 1.0)
        vertical_input = self.K_VERTICAL_P * (clamped_diff_alt ** 3)
        if alt_err > 1.0:
            vertical_input += _clamp(1.8 * (alt_err - 1.0), 0.0, 8.5)
        elif alt_err < -1.0:
            vertical_input += _clamp(1.2 * (alt_err + 1.0), -4.0, 0.0)
        vertical_input -= _clamp(0.45 * vel[2], -3.0, 3.0)

        # 2. Outer horizontal position + velocity cascade controller in World XY
        pos_err_xy = target_pos[:2] - pos[:2]
        v_des_xy = self.KP_POS * pos_err_xy + 0.85 * target_vel_xy
        v_des_norm = float(np.linalg.norm(v_des_xy))
        if v_des_norm > self.MAX_HORIZ_SPEED:
            v_des_xy = v_des_xy * (self.MAX_HORIZ_SPEED / v_des_norm)

        vel_err_xy = v_des_xy - vel[:2]
        cmd_world_x = self.KP_VEL * vel_err_xy[0] + 0.18 * pos_err_xy[0]
        cmd_world_y = self.KP_VEL * vel_err_xy[1] + 0.18 * pos_err_xy[1]

        # Rotate World XY command into Drone Body frame (forward, left)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        cmd_forward = cmd_world_x * cos_y + cmd_world_y * sin_y
        cmd_left = -cmd_world_x * sin_y + cmd_world_y * cos_y

        # In Mavic2Pro: negative pitch_disturbance moves forward; positive roll_disturbance moves left
        pitch_disturbance = _clamp(-cmd_forward, -self.MAX_PITCH_DIST, self.MAX_PITCH_DIST)
        roll_disturbance = _clamp(cmd_left, -self.MAX_ROLL_DIST, self.MAX_ROLL_DIST)

        # Reduce horizontal tilt during steep initial climb so vertical thrust dominates
        if abs(alt_err) > 4.0:
            scale = _clamp(1.0 - (abs(alt_err) - 4.0) / 15.0, 0.25, 1.0)
            pitch_disturbance *= scale
            roll_disturbance *= scale

        # 3. Heading (Yaw) hold controller
        yaw_target = self.yaw_ref if self.yaw_ref is not None else 0.0
        yaw_err = _wrap_pi(yaw_target - yaw)
        yaw_disturbance = _clamp(0.8 * yaw_err - 0.35 * yaw_vel, -self.MAX_YAW_DIST, self.MAX_YAW_DIST)

        # 4. Inner Attitude PID + Motor Mixing (matching mavic2pro.c)
        roll_input = self.K_ROLL_P * _clamp(roll, -1.0, 1.0) + roll_vel + roll_disturbance
        pitch_input = self.K_PITCH_P * _clamp(pitch, -1.0, 1.0) + pitch_vel + pitch_disturbance
        yaw_input = yaw_disturbance

        fl = _clamp(self.K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input, 0.0, self.MAX_MOTOR_VEL)
        fr = _clamp(self.K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input, 0.0, self.MAX_MOTOR_VEL)
        rl = _clamp(self.K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input, 0.0, self.MAX_MOTOR_VEL)
        rr = _clamp(self.K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input, 0.0, self.MAX_MOTOR_VEL)

        self.fl_motor.setVelocity(fl)
        self.fr_motor.setVelocity(-fr)
        self.rl_motor.setVelocity(-rl)
        self.rr_motor.setVelocity(rr)
        self.last_motor_speeds = (fl, -fr, -rl, rr)
        return pos


class MavicRigidBodySim:
    """6-DOF rigid-body quadrotor dynamics matching Webots Mavic2Pro.proto for offline physics tests."""

    def __init__(self, initial_pos=(0.0, 0.0, 0.085), initial_rpy=(0.0, 0.0, -math.pi), dt=0.008):
        self.dt = float(dt)
        self.pos = np.asarray(initial_pos, dtype=float).copy()
        self.vel = np.zeros(3, dtype=float)
        self.rpy = np.asarray(initial_rpy, dtype=float).copy()
        self.omega = np.zeros(3, dtype=float)
        self.motor_vel = np.zeros(4, dtype=float)  # [fl, fr, rl, rr]

    def step_physics(self, fl, fr, rl, rr):
        # Convert signed Webots motor commands (fl > 0, fr < 0, rl < 0, rr > 0) to positive rotor speeds
        w_fl = max(0.0, float(fl))
        w_fr = max(0.0, -float(fr))
        w_rl = max(0.0, -float(rl))
        w_rr = max(0.0, float(rr))
        self.motor_vel[:] = [w_fl, w_fr, w_rl, w_rr]

        # Mean rotor speed relative to Mavic2Pro hover speed (68.5 + 0.6^3 * 3.0 = 69.148 rad/s)
        w_hover = 69.148
        w_mean = 0.25 * (w_fl + w_fr + w_rl + w_rr)

        # Differential rotor commands driving roll, pitch, yaw angular accelerations
        u_roll = 0.25 * ((-w_fl + w_fr - w_rl + w_rr))
        u_pitch = 0.25 * ((w_fl + w_fr - w_rl - w_rr))
        u_yaw = 0.25 * ((-w_fl + w_fr + w_rl - w_rr))

        # Attitude dynamics (damped rigid body in Webots defaultDamping=0.5)
        alpha_roll = 1.8 * u_roll - 6.0 * self.omega[0]
        alpha_pitch = 1.8 * u_pitch - 6.0 * self.omega[1]
        alpha_yaw = 0.9 * u_yaw - 5.0 * self.omega[2]

        self.omega[0] += alpha_roll * self.dt
        self.omega[1] += alpha_pitch * self.dt
        self.omega[2] += alpha_yaw * self.dt

        self.rpy[0] = _clamp(self.rpy[0] + self.omega[0] * self.dt, -0.75, 0.75)
        self.rpy[1] = _clamp(self.rpy[1] + self.omega[1] * self.dt, -0.75, 0.75)
        self.rpy[2] = _wrap_pi(self.rpy[2] + self.omega[2] * self.dt)

        roll, pitch, yaw = self.rpy
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        # Total specific thrust (m/s^2) in body Z, calibrated to hover at w_hover = 69.148
        g_eff = 9.81
        thrust_acc = g_eff * ((w_mean / w_hover) ** 2)

        # Project thrust vector into World XYZ using roll, pitch, yaw and linear damping
        acc_forward = -g_eff * math.tan(pitch) * 1.35
        acc_left = g_eff * math.tan(roll) * 1.35
        ax = acc_forward * cos_y - acc_left * sin_y - 0.85 * self.vel[0]
        ay = acc_forward * sin_y + acc_left * cos_y - 0.85 * self.vel[1]
        az = thrust_acc * math.cos(roll) * math.cos(pitch) - g_eff - 0.95 * self.vel[2]

        self.vel[0] += ax * self.dt
        self.vel[1] += ay * self.dt
        self.vel[2] += az * self.dt
        self.pos += self.vel * self.dt
        if self.pos[2] < 0.0:
            self.pos[2] = 0.0
            self.vel[2] = max(0.0, self.vel[2])
        return self.pos.copy()
