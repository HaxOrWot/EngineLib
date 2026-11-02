"""
Physics Inspector & Anomaly Detector
Analyzes sampled telemetry to detect physical defects:
- Tunneling / floor sinking
- Explosive kinetic energy instability
- Resting contact chatter & micro-jitter
- Gravity acceleration deviation
- Unsettled sliding creep
- Joint constraint elongation / sagging
"""

import math

class Anomaly:
    SEV_CRITICAL = "CRITICAL"
    SEV_WARNING = "WARNING"
    SEV_INFO = "INFO"

    def __init__(self, code, severity, title, description, timestamp=0.0, metric_value=0.0, threshold=0.0):
        self.code = code
        self.severity = severity
        self.title = title
        self.description = description
        self.timestamp = timestamp
        self.metric_value = metric_value
        self.threshold = threshold

    def to_dict(self):
        return {
            "code": self.code,
            "severity": self.severity,
            "title": self.title,
            "description": self.description,
            "timestamp": round(self.timestamp, 3),
            "metric_value": round(self.metric_value, 2),
            "threshold": round(self.threshold, 2),
        }


class PhysicsInspector:
    def __init__(self, telemetry_recorder, expected_gravity=640.0, floor_y=610.0):
        self.recorder = telemetry_recorder
        self.expected_gravity = expected_gravity
        self.floor_y = floor_y
        self.anomalies = []

    def inspect_all(self):
        self.anomalies.clear()
        self._check_floor_sinking_and_tunneling()
        self._check_energy_explosions()
        self._check_resting_jitter()
        self._check_unsettled_creep()
        self._check_gravity_consistency()
        return self.anomalies

    def _check_floor_sinking_and_tunneling(self):
        max_pen = 0.0
        worst_frame = None

        for f in self.recorder.frames:
            if f.max_penetration > max_pen:
                max_pen = f.max_penetration
                worst_frame = f

        if max_pen > 14.0:
            self.anomalies.append(Anomaly(
                code="ERR_TUNNELING_OR_DEEP_SINK",
                severity=Anomaly.SEV_CRITICAL,
                title="Deep Floor Penetration or Tunneling Detected",
                description=(
                    f"A rigid body penetrated {max_pen:.1f}px beneath the floor at t={worst_frame.time:.2f}s. "
                    "Indicates Baumgarte position correction is under-scaled or substeps are insufficient for high-speed impacts."
                ),
                timestamp=worst_frame.time if worst_frame else 0.0,
                metric_value=max_pen,
                threshold=14.0
            ))
        elif max_pen > 5.0:
            self.anomalies.append(Anomaly(
                code="WARN_FLOOR_SINKING",
                severity=Anomaly.SEV_WARNING,
                title="Moderate Contact Sinking Observed",
                description=(
                    f"Resting blocks sank up to {max_pen:.1f}px into the floor surface at t={worst_frame.time:.2f}s. "
                    "Consider increasing solver iterations or tuning the penetration slop allowance."
                ),
                timestamp=worst_frame.time if worst_frame else 0.0,
                metric_value=max_pen,
                threshold=5.0
            ))

    def _check_energy_explosions(self):
        frames = self.recorder.frames
        if len(frames) < 5:
            return

        for i in range(2, len(frames)):
            prev_ke = frames[i - 1].total_kinetic_energy
            curr_ke = frames[i].total_kinetic_energy

            # If energy abruptly doubles (> 200%) without external explosions
            if prev_ke > 200.0 and curr_ke > prev_ke * 2.2 and (curr_ke - prev_ke) > 1500.0:
                self.anomalies.append(Anomaly(
                    code="ERR_KINETIC_ENERGY_EXPLOSION",
                    severity=Anomaly.SEV_CRITICAL,
                    title="Sudden Kinetic Energy Explosion",
                    description=(
                        f"Total kinetic energy spiked violently from {prev_ke:.0f}J to {curr_ke:.0f}J "
                        f"at t={frames[i].time:.2f}s. Indicates constraint over-correction or overlapping restitution springiness."
                    ),
                    timestamp=frames[i].time,
                    metric_value=curr_ke,
                    threshold=prev_ke * 2.0
                ))
                break  # Record highest onset

    def _check_resting_jitter(self):
        max_jitter = 0.0
        worst_body_id = None

        for b_id in self.recorder.body_histories.keys():
            j = self.recorder.calculate_jitter_index(b_id)
            if j > max_jitter:
                max_jitter = j
                worst_body_id = b_id

        if max_jitter > 22.0:
            self.anomalies.append(Anomaly(
                code="ERR_RESTING_CHATTER_JITTER",
                severity=Anomaly.SEV_CRITICAL,
                title="Severe Resting Contact Jitter (Chatter)",
                description=(
                    f"Body ID {worst_body_id} exhibits continuous micro-oscillation (jitter index {max_jitter:.1f}). "
                    "The contact solver is perpetually fighting gravity without settling into sleep."
                ),
                metric_value=max_jitter,
                threshold=22.0
            ))
        elif max_jitter > 9.0:
            self.anomalies.append(Anomaly(
                code="WARN_RESTING_MICRO_VIBRATION",
                severity=Anomaly.SEV_WARNING,
                title="Noticeable Micro-Vibrations in Stacks",
                description=(
                    f"Resting body ID {worst_body_id} has minor vibration (jitter index {max_jitter:.1f}). "
                    "Resting velocity threshold or sleeping velocity threshold should be raised."
                ),
                metric_value=max_jitter,
                threshold=9.0
            ))

    def _check_unsettled_creep(self):
        settling_time = self.recorder.calculate_settling_time(speed_threshold=20.0)
        total_time = self.recorder.frames[-1].time if self.recorder.frames else 0.0

        if total_time > 4.0 and settling_time >= total_time - 0.2:
            self.anomalies.append(Anomaly(
                code="WARN_PERPETUAL_SLIDING_CREEP",
                severity=Anomaly.SEV_WARNING,
                title="Perpetual Sliding Creep / Failure to Settle",
                description=(
                    f"Bodies failed to come to a complete stop even after {total_time:.1f}s of simulation. "
                    "Coulomb friction or rolling resistance damping is insufficient to arrest horizontal drift."
                ),
                timestamp=settling_time,
                metric_value=settling_time,
                threshold=total_time * 0.8
            ))

    def _check_gravity_consistency(self):
        # Scan freefall segments
        for b_id, history in self.recorder.body_histories.items():
            if len(history) < 15:
                continue

            for i in range(2, len(history) - 2):
                t1, s1 = history[i]
                t2, s2 = history[i + 2]
                dt = t2 - t1

                if dt > 0.02 and abs(s1.vx) < 5.0 and s1.vy > 50.0 and s2.vy > s1.vy:
                    accel_y = (s2.vy - s1.vy) / dt
                    err = abs(accel_y - self.expected_gravity)
                    if err > self.expected_gravity * 0.35 and err > 150.0:
                        self.anomalies.append(Anomaly(
                            code="WARN_GRAVITY_ACCEL_MISMATCH",
                            severity=Anomaly.SEV_WARNING,
                            title="Gravity Acceleration Mismatch",
                            description=(
                                f"Observed vertical acceleration {accel_y:.1f}px/s² deviates from expected {self.expected_gravity:.1f}px/s². "
                                "Check scaling factor PPM or air drag damping."
                            ),
                            timestamp=t1,
                            metric_value=accel_y,
                            threshold=self.expected_gravity
                        ))
                        return
