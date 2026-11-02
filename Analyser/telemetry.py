"""
Telemetry Collector for Physics Engine Analysis
Samples frame-by-frame physical quantities: kinetic energy, velocity, angular speed,
penetration depth, jitter frequency, and trajectory history.
"""

import math
from collections import defaultdict

class BodySample:
    __slots__ = (
        'id', 'shape', 'x', 'y', 'z', 'vx', 'vy', 'vz',
        'speed', 'angle', 'angular_velocity', 'mass', 'is_static'
    )

    def __init__(self, body_id, shape, x, y, z, vx, vy, vz, angle, ang_vel, mass, is_static):
        self.id = body_id
        self.shape = shape
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.vx = float(vx)
        self.vy = float(vy)
        self.vz = float(vz)
        self.speed = math.hypot(vx, vy, vz) if hasattr(math, 'hypot') else math.sqrt(vx*vx + vy*vy + vz*vz)
        self.angle = float(angle)
        self.angular_velocity = float(ang_vel)
        self.mass = float(mass)
        self.is_static = bool(is_static)


class FrameTelemetry:
    def __init__(self, frame_idx, time_sec):
        self.frame = frame_idx
        self.time = time_sec
        self.bodies = {}
        self.total_kinetic_energy = 0.0
        self.max_speed = 0.0
        self.max_penetration = 0.0
        self.active_count = 0
        self.sleeping_count = 0


class TelemetryRecorder:
    def __init__(self, floor_y=0.0):
        self.floor_y = floor_y
        self.frames = []
        self.body_histories = defaultdict(list)

    def record_frame(self, frame_idx, time_sec, bodies, floor_y=None):
        if floor_y is not None:
            self.floor_y = floor_y

        ft = FrameTelemetry(frame_idx, time_sec)
        total_ke = 0.0
        max_spd = 0.0
        max_pen = 0.0
        active_cnt = 0
        sleep_cnt = 0

        for b in bodies:
            # Extract properties (support both 2D and 3D, dict or object)
            b_id = getattr(b, 'id', 0)
            shape = getattr(b, 'shape_type', 'box')
            pos = getattr(b, 'position', None)
            vel = getattr(b, 'velocity', None)

            if pos is not None:
                x = getattr(pos, 'x', 0.0)
                y = getattr(pos, 'y', 0.0)
                z = getattr(pos, 'z', 0.0)
            else:
                x = getattr(b, 'x', 0.0)
                y = getattr(b, 'y', 0.0)
                z = getattr(b, 'z', 0.0)

            if vel is not None:
                vx = getattr(vel, 'x', 0.0)
                vy = getattr(vel, 'y', 0.0)
                vz = getattr(vel, 'z', 0.0)
            else:
                vx = getattr(b, 'vx', 0.0)
                vy = getattr(b, 'vy', 0.0)
                vz = getattr(b, 'vz', 0.0)

            angle = getattr(b, 'angle', 0.0)
            ang_vel = getattr(b, 'angular_velocity', 0.0)
            mass = getattr(b, 'mass', 1.0)
            is_stat = getattr(b, 'is_static', False)

            sample = BodySample(b_id, shape, x, y, z, vx, vy, vz, angle, ang_vel, mass, is_stat)
            ft.bodies[b_id] = sample
            self.body_histories[b_id].append((time_sec, sample))

            if not is_stat:
                speed_sq = vx * vx + vy * vy + vz * vz
                spd = math.sqrt(speed_sq)
                max_spd = max(max_spd, spd)

                # Linear kinetic energy: 0.5 * m * v^2
                ke = 0.5 * mass * speed_sq
                # Angular energy: 0.5 * I * w^2 (approximate I)
                if ang_vel != 0.0:
                    radius = getattr(b, 'radius', 1.0)
                    inertia = 0.5 * mass * radius * radius
                    ke += 0.5 * inertia * (ang_vel * ang_vel)

                total_ke += ke

                if spd < 15.0:
                    sleep_cnt += 1
                else:
                    active_cnt += 1

                # Check floor penetration (supports both 2D Y-down and 3D Y-up)
                radius = getattr(b, 'radius', getattr(b, 'height', 20.0) * 0.5)
                if self.floor_y > 0.0:
                    if self.floor_y < 10.0:
                        # 3D World (metric space, Y-axis points upward)
                        bottom_y = y - radius
                        if bottom_y < self.floor_y:
                            pen = (self.floor_y - bottom_y) * 25.0  # scaled to px
                            max_pen = max(max_pen, pen)
                    else:
                        # 2D Screen space (Y-axis points downward)
                        bottom_y = y + radius
                        if bottom_y > self.floor_y:
                            pen = bottom_y - self.floor_y
                            max_pen = max(max_pen, pen)

        ft.total_kinetic_energy = total_ke
        ft.max_speed = max_spd
        ft.max_penetration = max_pen
        ft.active_count = active_cnt
        ft.sleeping_count = sleep_cnt

        self.frames.append(ft)
        return ft

    # -----------------------------------------------------------------------
    # Statistical Telemetry Aggregation
    # -----------------------------------------------------------------------
    def get_energy_curve(self):
        """Returns list of (time, total_kinetic_energy)"""
        return [(f.time, f.total_kinetic_energy) for f in self.frames]

    def get_max_penetration_over_time(self):
        """Returns list of (time, max_penetration)"""
        return [(f.time, f.max_penetration) for f in self.frames]

    def calculate_jitter_index(self, body_id):
        """
        Calculates velocity oscillation frequency and jitter metric.
        High values indicate resting contact chatter or constraint fighting.
        """
        history = self.body_histories.get(body_id, [])
        if len(history) < 10:
            return 0.0

        sign_flips = 0
        micro_movement_energy = 0.0
        prev_vy = 0.0

        for _, sample in history:
            vy = sample.vy
            if (vy > 1.0 and prev_vy < -1.0) or (vy < -1.0 and prev_vy > 1.0):
                sign_flips += 1
            if sample.speed < 40.0:
                micro_movement_energy += sample.speed
            prev_vy = vy

        flip_rate = sign_flips / float(len(history))
        avg_micro_motion = micro_movement_energy / float(len(history))
        return flip_rate * avg_micro_motion

    def calculate_settling_time(self, speed_threshold=25.0):
        """Finds the timestamp when all dynamic bodies stayed under speed_threshold"""
        if not self.frames:
            return 0.0

        last_active_time = self.frames[0].time
        for f in self.frames:
            if f.max_speed > speed_threshold:
                last_active_time = f.time

        return last_active_time
