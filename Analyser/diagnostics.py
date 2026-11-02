"""
Diagnostics Engine & Recommendation Generator
Translates detected physical anomalies into concrete, actionable engineering prescriptions:
specifies exact subsystems, parameter adjustments, and code-level fixes.
"""

class Recommendation:
    def __init__(self, title, subsystem, root_cause, fix_instructions, priority="HIGH"):
        self.title = title
        self.subsystem = subsystem
        self.root_cause = root_cause
        self.fix_instructions = fix_instructions
        self.priority = priority


class DiagnosticEngine:
    def __init__(self, inspector):
        self.inspector = inspector
        self.anomalies = []
        self.recommendations = []
        self.health_score = 100

    def evaluate(self):
        self.anomalies = self.inspector.inspect_all()
        self.recommendations.clear()
        self.health_score = 100

        # Tally deductions
        for an in self.anomalies:
            if an.severity == "CRITICAL":
                self.health_score -= 25
            elif an.severity == "WARNING":
                self.health_score -= 10
            else:
                self.health_score -= 5

        self.health_score = max(0, min(100, self.health_score))

        # Generate targeted prescriptions for each anomaly
        seen_codes = set()
        for an in self.anomalies:
            if an.code in seen_codes:
                continue
            seen_codes.add(an.code)

            if an.code == "ERR_TUNNELING_OR_DEEP_SINK":
                self.recommendations.append(Recommendation(
                    title="Enhance Positional Projection & Increase Solver Sub-stepping",
                    subsystem="ContactSolver / World Substeps",
                    root_cause="High-speed impact velocities overcome Baumgarte penetration allowance within a single timestep.",
                    fix_instructions=[
                        "1. Increase world sub-steps from 2-3 to 4-5 via set_world_substeps(4).",
                        "2. Clamp max penetration correction per iteration in ContactSolver to prevent aggressive repulsion bounce.",
                        "3. Ensure the floor AABB size along depth (Z) is at least 10.0m to prevent edge clipping."
                    ],
                    priority="CRITICAL"
                ))
            elif an.code == "WARN_FLOOR_SINKING":
                self.recommendations.append(Recommendation(
                    title="Fine-Tune Baumgarte Factor & Lower Slop Allowance",
                    subsystem="ContactSolver",
                    root_cause="PENETRATION_SLOP is permitting visible sinking before positional correction engages.",
                    fix_instructions=[
                        "1. Reduce PENETRATION_SLOP from 0.005 (5mm) down to 0.002 (2mm).",
                        "2. Set BAUMGARTE_FACTOR to 0.40 - 0.50 for firm contact support without jitter.",
                        "3. Run 6 to 8 solver iterations per substep for rigid stacked towers."
                    ],
                    priority="MEDIUM"
                ))
            elif an.code == "ERR_KINETIC_ENERGY_EXPLOSION":
                self.recommendations.append(Recommendation(
                    title="Eliminate Constraint Springiness & Overlapping Restitution",
                    subsystem="ContactSolver / Restitution",
                    root_cause="Elastic restitution is firing on resting/overlapping contacts, generating artificial energy.",
                    fix_instructions=[
                        "1. Zero restitution (e = 0.0) whenever relative impact velocity is below RESTING_VEL_THRESHOLD (0.35 m/s).",
                        "2. Decouple position projection from velocity integration (use XPBD position projection).",
                        "3. Clamp impulse magnitude in ContactSolver so contact impulses cannot exceed external load."
                    ],
                    priority="CRITICAL"
                ))
            elif an.code in ("ERR_RESTING_CHATTER_JITTER", "WARN_RESTING_MICRO_VIBRATION"):
                self.recommendations.append(Recommendation(
                    title="Strengthen Resting Velocity Clamping & Sleeping Subsystem",
                    subsystem="Integrator / ContactSolver",
                    root_cause="Microscopic velocity bounces cause bodies to continuously oscillate around zero velocity.",
                    fix_instructions=[
                        "1. Increase RESTING_VEL_THRESHOLD in ContactSolver to 0.40 m/s.",
                        "2. In Integrator, zero both velocity and angular velocity when speed is below SLEEP_VELOCITY_THRESHOLD for 0.4s.",
                        "3. Apply angular damping (body.angular_velocity *= 0.95) for resting contacts to halt rotational flutter."
                    ],
                    priority="HIGH"
                ))
            elif an.code == "WARN_PERPETUAL_SLIDING_CREEP":
                self.recommendations.append(Recommendation(
                    title="Apply Static Friction Locking & Rolling Drag",
                    subsystem="ContactSolver / Friction",
                    root_cause="Kinetic friction alone approaches zero as speed decreases, failing to achieve complete static lock.",
                    fix_instructions=[
                        "1. When tangential velocity magnitude drops below static friction impulse threshold, clamp tangential velocity to exactly ZERO.",
                        "2. For spheres, apply rolling resistance damping (roll_drag = 1.0 - friction * 0.15 * dt * 60.0).",
                        "3. Ensure sustained normal load presses resting bodies into the surface even when vertical velocity is zero."
                    ],
                    priority="MEDIUM"
                ))
            elif an.code == "WARN_GRAVITY_ACCEL_MISMATCH":
                self.recommendations.append(Recommendation(
                    title="Calibrate Screen Scaling (PPM) & Freefall Drag",
                    subsystem="Physics Engine Scaling / Integrator",
                    root_cause="Pixels-per-meter scaling mismatch or excessive linear damping skewing ballistic trajectory.",
                    fix_instructions=[
                        "1. Verify that gravity in physics_engine.dll is set to g_screen / PPM.",
                        "2. Set default body linear_damping to 0.0 for pure Newtonian freefall, applying damping only to ragdolls or air resistance.",
                        "3. Synchronize trajectory prediction formula to match exact physics gravity."
                    ],
                    priority="MEDIUM"
                ))

        if not self.recommendations:
            self.recommendations.append(Recommendation(
                title="Simulation Operating Within Optimal Physical Tolerances",
                subsystem="All Subsystems",
                root_cause="None. Contacts, energy conservation, toppling, and trajectory conform to physical bounds.",
                fix_instructions=["No fixes required. Simulation passed all analytical verifications."],
                priority="INFO"
            ))

        return self.recommendations

    def print_terminal_summary(self):
        print("\n" + "=" * 75)
        print("          PHYSICS ENGINE ANALYSER & DIAGNOSTIC REPORT")
        print("=" * 75)

        grade = "EXCELLENT (A+)" if self.health_score >= 90 else "GOOD (B)" if self.health_score >= 75 else "NEEDS ATTENTION (C)" if self.health_score >= 50 else "CRITICAL ISSUES (F)"
        print(f"  Overall Physical Health Score: {self.health_score}/100 [{grade}]")
        print(f"  Anomalies Detected: {len(self.anomalies)}")
        print("-" * 75)

        if self.anomalies:
            print("[DETECTED ANOMALIES]")
            for an in self.anomalies:
                sev_icon = "[!]" if an.severity == "CRITICAL" else "[*]" if an.severity == "WARNING" else "[i]"
                print(f"  {sev_icon} [{an.severity}] {an.title}")
                print(f"     Details: {an.description}")
                print(f"     Metric Value: {an.metric_value:.2f} (Threshold: {an.threshold:.2f})")
                print()

        print("-" * 75)
        print("[ACTIONABLE ENGINEERING RECOMMENDATIONS]")
        for rec in self.recommendations:
            print(f"  * {rec.title} [Priority: {rec.priority}]")
            print(f"    Target Subsystem: {rec.subsystem}")
            print(f"    Root Cause: {rec.root_cause}")
            print("    Prescribed Actions:")
            for act in rec.fix_instructions:
                print(f"      {act}")
            print()
        print("=" * 75 + "\n")

    def print_summary(self):
        self.print_terminal_summary()
