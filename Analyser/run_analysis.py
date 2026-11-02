#!/usr/bin/env python3
"""
Physics Engine Analyser & Automated Visual Diagnostics Suite
Executes physics simulation scenarios, captures visual keyframe screenshots and replay GIFs,
samples frame-by-frame physical telemetry, detects physical defects, and compiles
interactive diagnostic reports.
"""

import os
import sys
import math
import argparse
import time

# Ensure Analyser and parent folders are in sys.path
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
analyser_dir = os.path.dirname(os.path.abspath(__file__))
gamelib_dir = os.path.abspath(os.path.join(analyser_dir, ".."))
gamesim_dir = os.path.abspath(os.path.join(gamelib_dir, "..", "EngineLib Games", "GameSim"))
sampletest_dir = os.path.join(gamelib_dir, "SampleTest", "python")

for p in (analyser_dir, gamelib_dir, gamesim_dir, sampletest_dir):
    if p not in sys.path:
        sys.path.insert(0, p)

from telemetry import TelemetryRecorder
from physics_inspector import PhysicsInspector
from diagnostics import DiagnosticEngine
from recorder import VisualRecorder
from report_generator import ReportGenerator

def load_module_from_path(module_name, file_path):
    """Safely loads a Python module directly from an absolute file path."""
    if module_name in sys.modules:
        return sys.modules[module_name]
    import importlib.util
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load module '{module_name}' from path: {file_path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = mod
    spec.loader.exec_module(mod)
    return mod

def parse_args():
    parser = argparse.ArgumentParser(description="Physics Engine Automated Visual Analyser")
    parser.add_argument("--target", choices=["gamesim", "ball", "ragdoll", "all"], default="gamesim",
                        help="Simulation scenario to execute and analyze (default: gamesim)")
    parser.add_argument("--backend", choices=["dll", "builtin"], default="dll",
                        help="Physics engine backend (default: dll)")
    parser.add_argument("--frames", type=int, default=180,
                        help="Simulation frames to record (default: 180)")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Custom output directory for reports and visual artifacts")
    parser.add_argument("--headless", action="store_true", default=True,
                        help="Run headlessly using SDL dummy driver (default: True)")
    parser.add_argument("--no-headless", dest="headless", action="store_false",
                        help="Disable headless mode and display Pygame window")
    return parser.parse_args()


# ===========================================================================
# Scenario: GameSim Demolition Castle Impact
# ===========================================================================
def run_gamesim_scenario(backend="dll", total_frames=180, output_dir=None, headless=True):
    if headless:
        os.environ["SDL_VIDEODRIVER"] = "dummy"

    import pygame
    pygame.init()
    if not pygame.font.get_init():
        pygame.font.init()

    try:
        from game import SiegePhysicsGame
    except ImportError:
        game_mod = load_module_from_path("game", os.path.join(gamesim_dir, "game.py"))
        SiegePhysicsGame = game_mod.SiegePhysicsGame

    out_dir = output_dir or os.path.join(analyser_dir, "reports", f"gamesim_{backend}")
    os.makedirs(out_dir, exist_ok=True)

    print(f"\n==================================================================")
    print(f"  EXECUTING ANALYSER SCENARIO: GameSim Demolition ({backend.upper()})")
    print(f"==================================================================")
    print(f"Output Directory: {out_dir}")
    print(f"Target Frames:    {total_frames} (approx {total_frames/60.0:.1f}s at 60 FPS)")

    game = SiegePhysicsGame()
    game.state = "PLAYING"
    game.aim_angle = math.radians(-16.0)
    game.aim_power = 680.0
    game.selected_ammo = 1  # Heavy Cannonball

    recorder = VisualRecorder(output_dir=out_dir, max_gif_frames=total_frames)
    telemetry = TelemetryRecorder(floor_y=610.0)

    # Fire cannon projectile
    game.fire_projectile()

    dt = 1.0 / 60.0
    peak_ke = 0.0
    impact_frame = -1
    collapse_frame = -1

    for frame_idx in range(1, total_frames + 1):
        sim_time = frame_idx * dt
        game.update(dt)

        # Telemetry sample
        ft = telemetry.record_frame(frame_idx, sim_time, game.world.bodies, floor_y=610.0)

        # Detect impact: projectile decelerates or collision occurs
        if impact_frame == -1 and game.active_projectiles:
            p = game.active_projectiles[0]
            # Check proximity to tower base (base_x around 780, tower x ~ 720-840)
            if p.position.x > 710:
                impact_frame = frame_idx

        # Detect peak collapse energy
        if ft.total_kinetic_energy > peak_ke:
            peak_ke = ft.total_kinetic_energy
            if frame_idx > 40:
                collapse_frame = frame_idx

        # Visual Capture with Telemetry HUD overlay
        game.render()
        hud_info = {
            "time": sim_time,
            "frame": frame_idx,
            "ke": ft.total_kinetic_energy,
            "max_speed": ft.max_speed,
            "max_pen": ft.max_penetration,
            "active": ft.active_count,
            "sleep": ft.sleeping_count,
        }
        recorder.capture_frame(game.screen, telemetry_info=hud_info, draw_vectors=True, bodies=game.world.bodies)

        # Save keyframe screenshots
        if frame_idx == 10:
            recorder.save_screenshot(game.screen, "01_cannon_launch.png", hud_info, bodies=game.world.bodies)
        elif frame_idx == (impact_frame if impact_frame > 0 else 45):
            recorder.save_screenshot(game.screen, "02_projectile_impact.png", hud_info, bodies=game.world.bodies)
        elif frame_idx == (collapse_frame if collapse_frame > 0 else 90):
            recorder.save_screenshot(game.screen, "03_structural_collapse.png", hud_info, bodies=game.world.bodies)
        elif frame_idx == total_frames:
            recorder.save_screenshot(game.screen, "04_settled_equilibrium.png", hud_info, bodies=game.world.bodies)

        if frame_idx % 30 == 0 or frame_idx == total_frames:
            print(f"  Frame {frame_idx:3d}/{total_frames} | Time: {sim_time:.2f}s | KE: {ft.total_kinetic_energy:6.0f} J | MaxSpd: {ft.max_speed:5.1f} | Active: {ft.active_count}")

    # Compile animated replay GIF
    print(f"\nCompiling animated simulation replay GIF...")
    gif_file = recorder.compile_gif(filename="simulation_replay.gif", duration_ms=25)
    if gif_file:
        print(f"  Saved Replay GIF: {gif_file}")

    # Inspect physics anomalies
    inspector = PhysicsInspector(telemetry, expected_gravity=640.0, floor_y=610.0)
    diagnostic = DiagnosticEngine(inspector)
    diagnostic.evaluate()

    # Calculate run metrics
    settling_time = telemetry.calculate_settling_time(speed_threshold=25.0)
    max_penetration = max((f.max_penetration for f in telemetry.frames), default=0.0)

    extra_meta = {
        "Test Scenario": "GameSim Demolition Tower Impact",
        "Physics Backend": f"Native DLL ({backend})" if backend == "dll" else "Pure Python",
        "Total Frames Sampled": f"{total_frames} frames ({total_frames*dt:.2f}s)",
        "Peak Kinetic Energy": f"{peak_ke:,.0f} J",
        "Max Floor Penetration": f"{max_penetration:.2f} px",
        "Structural Settling Time": f"{settling_time:.2f} s",
    }

    # Generate Reports
    reporter = ReportGenerator(output_dir=out_dir)
    html_report, md_report = reporter.generate(
        test_name="GameSim Demolition",
        backend_name="Native C++ DLL" if backend == "dll" else "Pure Python",
        diagnostic_engine=diagnostic,
        visual_recorder=recorder,
        extra_meta=extra_meta
    )

    # Print Terminal Diagnostic Summary
    diagnostic.print_summary()
    print(f"\n[REPORT GENERATED]")
    print(f"  HTML Dashboard: file:///{html_report.replace(os.sep, '/')}")
    print(f"  Markdown Doc:   file:///{md_report.replace(os.sep, '/')}")

    # Clean up game world
    if hasattr(game.world, "destroy"):
        game.world.destroy()
    pygame.quit()

    return diagnostic.health_score, html_report


# ===========================================================================
# Scenario: 3D Ball Simulation
# ===========================================================================
def run_ball_scenario(total_frames=180, output_dir=None, headless=True):
    if headless:
        os.environ["SDL_VIDEODRIVER"] = "dummy"

    import pygame
    pygame.init()

    try:
        from sample import load_physics_engine, Vector3D
    except ImportError:
        sample_mod = load_module_from_path("sample", os.path.join(sampletest_dir, "sample.py"))
        load_physics_engine = sample_mod.load_physics_engine
        Vector3D = sample_mod.Vector3D
    from ctypes import byref

    class Camera3D:
        def __init__(self, target=(0.0, 3.5, 0.0), distance=28.0, yaw=0.75, pitch=0.42):
            self.target = list(target)
            self.distance = distance
            self.yaw = yaw
            self.pitch = pitch
            self.fov = math.radians(60)

        def project(self, x, y, z, screen_w, screen_h):
            dx, dy, dz = x - self.target[0], y - self.target[1], z - self.target[2]
            cos_y, sin_y = math.cos(self.yaw), math.sin(self.yaw)
            x1 = dx * cos_y - dz * sin_y
            z1 = dx * sin_y + dz * cos_y
            cos_p, sin_p = math.cos(self.pitch), math.sin(self.pitch)
            y2 = dy * cos_p - z1 * sin_p
            z2 = dy * sin_p + z1 * cos_p
            z_cam = z2 + self.distance
            if z_cam <= 0.2:
                return None, None, z_cam
            focal = (screen_w / 2.0) / math.tan(self.fov / 2.0)
            return int(screen_w / 2.0 + (x1 * focal) / z_cam), int(screen_h / 2.0 - (y2 * focal) / z_cam), z_cam

    def create_arena(lib, arena_size=16.0):
        lib.add_rigid_body(0.0, 0.0, 0.0, arena_size + 4.0, 1.0, arena_size + 4.0, 0.0, 0.65)
        half = arena_size / 2.0
        lib.add_rigid_body(0.0, 5.0, half, arena_size, 10.0, 1.0, 0.0, 0.70)
        lib.add_rigid_body(0.0, 5.0, -half, arena_size, 10.0, 1.0, 0.0, 0.70)
        lib.add_rigid_body(half, 5.0, 0.0, 1.0, 10.0, arena_size, 0.0, 0.70)
        lib.add_rigid_body(-half, 5.0, 0.0, 1.0, 10.0, arena_size, 0.0, 0.70)

    lib = load_physics_engine()

    out_dir = output_dir or os.path.join(analyser_dir, "reports", "ball_dll")
    os.makedirs(out_dir, exist_ok=True)

    print(f"\n==================================================================")
    print(f"  EXECUTING ANALYSER SCENARIO: 3D Ball Simulation (Native DLL)")
    print(f"==================================================================")

    screen_w, screen_h = 1000, 700
    screen = pygame.display.set_mode((screen_w, screen_h))
    camera = Camera3D(distance=28.0, yaw=math.radians(35), pitch=math.radians(24))

    lib.init_physics_world(0.0, -9.81, 0.0)
    lib.set_world_substeps(3)
    lib.set_solver_iterations(5)

    arena_size = 22.0
    obstacles = create_arena(lib, arena_size=arena_size)

    # Helper spawn
    import random
    balls = []
    class BallObj:
        def __init__(self, b_id, x, y, z, r, is_heavy=False):
            self.id = b_id
            self.x, self.y, self.z = x, y, z
            self.vx, self.vy, self.vz = 0.0, 0.0, 0.0
            self.radius = r
            self.mass = 12.0 if is_heavy else 1.5
            self.is_static = False
            self.is_heavy = is_heavy
            self.color = (55, 60, 75) if is_heavy else (235, 75, 60)
            self.stripe_color = (190, 195, 205) if is_heavy else (255, 255, 255)
            self.rot_x, self.rot_y, self.rot_z = 0.0, 0.0, 0.0

    # Spawn 6 balls across heights
    pos_buf = Vector3D()
    vel_buf = Vector3D()
    for i in range(6):
        bx = (i - 2.5) * 2.2
        by = 8.0 + (i % 3) * 2.5
        bz = random.uniform(-3.0, 3.0)
        r = 0.85 if i == 0 else 0.55
        is_h = (i == 0)
        bid = lib.add_sphere_body(
            bx, by, bz, r,
            25.0 if is_h else 2.0,
            0.35 if is_h else 0.70,
            0.60 if is_h else 0.45
        )
        balls.append(BallObj(bid, bx, by, bz, r, is_h))

    recorder = VisualRecorder(output_dir=out_dir, max_gif_frames=total_frames)
    telemetry = TelemetryRecorder(floor_y=0.5)

    dt = 1.0 / 60.0
    for frame_idx in range(1, total_frames + 1):
        sim_time = frame_idx * dt
        lib.step_physics(dt)

        # Read back positions
        for b in balls:
            if lib.get_body_position(b.id, byref(pos_buf)):
                b.x, b.y, b.z = pos_buf.x, pos_buf.y, pos_buf.z
            if lib.get_body_velocity(b.id, byref(vel_buf)):
                b.vx, b.vy, b.vz = vel_buf.x, vel_buf.y, vel_buf.z

        # Telemetry sample
        ft = telemetry.record_frame(frame_idx, sim_time, balls, floor_y=0.5)

        # Clear screen
        screen.fill((16, 22, 34))

        # Project & render
        render_queue = []
        for b in balls:
            if b.y >= 0.4:
                sp = camera.project(b.x, 0.51, b.z, screen_w, screen_h)
                if sp[0]:
                    h_factor = max(0.1, min(1.0, 1.0 - (b.y - 0.5) / 14.0))
                    focal = (screen_w / 2.0) / math.tan(camera.fov / 2.0)
                    sr_x = max(2, int((b.radius * 1.15 * focal) / sp[2] * h_factor))
                    sr_y = max(2, int(sr_x * 0.45))
                    alpha = max(0, min(190, int(135 * h_factor)))
                    render_queue.append((sp[2] + 0.01, "shadow", (sp[0], sp[1], sr_x, sr_y, alpha)))

            bp = camera.project(b.x, b.y, b.z, screen_w, screen_h)
            if bp[0]:
                focal = (screen_w / 2.0) / math.tan(camera.fov / 2.0)
                screen_radius = max(2, int((b.radius * focal) / bp[2]))
                render_queue.append((bp[2], "sphere", (b, bp[0], bp[1], screen_radius)))

        render_queue.sort(key=lambda item: item[0], reverse=True)
        for item in render_queue:
            rtype, data = item[1], item[2]
            if rtype == "shadow":
                sx, sy, rx, ry, alpha = data
                if rx > 0 and ry > 0 and alpha > 0:
                    surf = pygame.Surface((rx * 2 + 2, ry * 2 + 2), pygame.SRCALPHA)
                    pygame.draw.ellipse(surf, (8, 10, 16, int(alpha)), (0, 0, rx * 2, ry * 2))
                    screen.blit(surf, (sx - rx, sy - ry))
            elif rtype == "sphere":
                b_obj, sx, sy, r = data
                pygame.draw.circle(screen, b_obj.color, (sx, sy), r)
                pygame.draw.circle(screen, (255, 255, 255), (sx - r//3, sy - r//3), max(1, r//3))

        hud_info = {
            "time": sim_time, "frame": frame_idx, "ke": ft.total_kinetic_energy,
            "max_speed": ft.max_speed, "max_pen": ft.max_penetration,
            "active": ft.active_count, "sleep": ft.sleeping_count,
        }
        recorder.capture_frame(screen, telemetry_info=hud_info, draw_vectors=False)

        if frame_idx == 15:
            recorder.save_screenshot(screen, "01_ball_drop.png", hud_info)
        elif frame_idx == 50:
            recorder.save_screenshot(screen, "02_ramp_impact.png", hud_info)
        elif frame_idx == 100:
            recorder.save_screenshot(screen, "03_floor_rolling.png", hud_info)
        elif frame_idx == total_frames:
            recorder.save_screenshot(screen, "04_settled_balls.png", hud_info)

    recorder.compile_gif("simulation_replay.gif", duration_ms=25)
    inspector = PhysicsInspector(telemetry, expected_gravity=9.81, floor_y=0.5)
    diagnostic = DiagnosticEngine(inspector)
    diagnostic.evaluate()

    reporter = ReportGenerator(output_dir=out_dir)
    html_report, md_report = reporter.generate(
        test_name="3D Ball Simulation",
        backend_name="Native C++ DLL",
        diagnostic_engine=diagnostic,
        visual_recorder=recorder,
        extra_meta={"Balls Simulated": len(balls), "Substeps": 3}
    )

    diagnostic.print_summary()
    lib.destroy_physics_world()
    pygame.quit()
    return diagnostic.health_score, html_report


# ===========================================================================
# CLI Entry Point
# ===========================================================================
def main():
    args = parse_args()

    if args.target == "gamesim":
        score, report = run_gamesim_scenario(
            backend=args.backend,
            total_frames=args.frames,
            output_dir=args.output_dir,
            headless=args.headless
        )
    elif args.target == "ball":
        score, report = run_ball_scenario(
            total_frames=args.frames,
            output_dir=args.output_dir,
            headless=args.headless
        )
    elif args.target == "all":
        print(">>> Running full verification suite...")
        s1, r1 = run_gamesim_scenario(backend="dll", total_frames=args.frames, headless=args.headless)
        s2, r2 = run_ball_scenario(total_frames=args.frames, headless=args.headless)
        print(f"\n[SUITE COMPLETE] GameSim Score: {s1}/100 | BallSim Score: {s2}/100")
    else:
        score, report = run_gamesim_scenario(backend=args.backend, total_frames=args.frames, headless=args.headless)

if __name__ == "__main__":
    main()
