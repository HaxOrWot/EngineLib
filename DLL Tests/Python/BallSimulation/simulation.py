#!/usr/bin/env python3
"""
Interactive 3D Ball Physics Simulation powered by EngineLib (physics_engine.dll)
Features realistic Coulomb friction, rolling resistance, 3D ball surface rotation,
incline ramps, click-and-fling ball launching, dynamic shadows, and camera orbit.
"""

import os
import sys
import math
import random
import ctypes
from ctypes import c_float, c_uint32, c_bool, Structure, POINTER, byref
import pygame

# ---------------------------------------------------------------------------
# 1. Native FFI Bindings
# ---------------------------------------------------------------------------
class Vector3D(Structure):
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
    ]

def load_physics_dll():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(current_dir, "physics_engine.dll"),
        os.path.join(current_dir, "..", "physics_engine.dll"),
        os.path.join(current_dir, "..", "..", "..", "PhysicsEngine", "target", "release", "physics_engine.dll"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            print(f"[DLL] Loaded native engine: {os.path.abspath(path)}")
            lib = ctypes.CDLL(os.path.abspath(path))
            break
    else:
        raise FileNotFoundError("Could not locate physics_engine.dll!")

    # Configure signatures
    lib.init_physics_world.argtypes = [c_float, c_float, c_float]
    lib.init_physics_world.restype = None

    lib.add_rigid_body.argtypes = [c_float, c_float, c_float, c_float, c_float, c_float, c_float, c_float]
    lib.add_rigid_body.restype = c_uint32

    lib.add_sphere_body.argtypes = [c_float, c_float, c_float, c_float, c_float, c_float, c_float]
    lib.add_sphere_body.restype = c_uint32

    lib.set_world_substeps.argtypes = [c_uint32]
    lib.set_world_substeps.restype = None

    lib.set_solver_iterations.argtypes = [c_uint32]
    lib.set_solver_iterations.restype = None

    lib.get_body_position.argtypes = [c_uint32, POINTER(Vector3D)]
    lib.get_body_position.restype = c_bool

    lib.get_body_velocity.argtypes = [c_uint32, POINTER(Vector3D)]
    lib.get_body_velocity.restype = c_bool

    lib.apply_force.argtypes = [c_uint32, c_float, c_float, c_float]
    lib.apply_force.restype = None

    lib.step_physics.argtypes = [c_float]
    lib.step_physics.restype = None

    lib.destroy_physics_world.argtypes = []
    lib.destroy_physics_world.restype = None

    return lib

# ---------------------------------------------------------------------------
# 2. 3D Camera & Projection
# ---------------------------------------------------------------------------
class Camera3D:
    def __init__(self, target=(0.0, 3.5, 0.0), distance=28.0, yaw=0.75, pitch=0.42):
        self.target = list(target)
        self.distance = distance
        self.yaw = yaw       # Horizontal rotation
        self.pitch = pitch   # Vertical rotation
        self.fov = math.radians(60)

    def project(self, x, y, z, screen_w, screen_h):
        dx = x - self.target[0]
        dy = y - self.target[1]
        dz = z - self.target[2]

        # Rotate around Y (yaw)
        cos_y, sin_y = math.cos(self.yaw), math.sin(self.yaw)
        x1 = dx * cos_y - dz * sin_y
        z1 = dx * sin_y + dz * cos_y
        y1 = dy

        # Rotate around X (pitch)
        cos_p, sin_p = math.cos(self.pitch), math.sin(self.pitch)
        y2 = y1 * cos_p - z1 * sin_p
        z2 = y1 * sin_p + z1 * cos_p
        x2 = x1

        # Camera view plane distance
        z_cam = z2 + self.distance
        if z_cam <= 0.2:
            return None, None, z_cam

        focal = (screen_w / 2.0) / math.tan(self.fov / 2.0)
        sx = screen_w / 2.0 + (x2 * focal) / z_cam
        sy = screen_h / 2.0 - (y2 * focal) / z_cam
        return int(sx), int(sy), z_cam

# ---------------------------------------------------------------------------
# 3. Ball Entity with 3D Orientation
# ---------------------------------------------------------------------------
class Ball:
    def __init__(self, body_id, radius, color, is_heavy=False):
        self.id = body_id
        self.radius = radius
        self.color = color
        self.is_heavy = is_heavy
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.prev_x = 0.0
        self.prev_z = 0.0
        self.rot_x = 0.0
        self.rot_z = 0.0
        self.screen_x = 0
        self.screen_y = 0
        self.screen_r = 10
        self.depth = 10.0

        # Pattern stripe color
        self.stripe_color = (
            max(0, color[0] - 70),
            max(0, color[1] - 70),
            max(0, color[2] - 70),
        )

    def update_rotation(self):
        dx = self.x - self.prev_x
        dz = self.z - self.prev_z
        if abs(dx) > 1e-4 or abs(dz) > 1e-4:
            # Angular displacement = linear displacement / radius
            self.rot_x += dz / self.radius
            self.rot_z -= dx / self.radius
        self.prev_x = self.x
        self.prev_z = self.z

VIBRANT_COLORS = [
    (255, 80, 80),    # Coral Red
    (60, 210, 255),   # Cyan
    (255, 210, 50),   # Gold
    (130, 255, 90),   # Lime
    (215, 100, 255),  # Violet
    (255, 145, 40),   # Orange
    (255, 95, 180),   # Hot Pink
]

class ObstacleBox:
    def __init__(self, x, y, z, sx, sy, sz, color=(50, 60, 80)):
        self.x, self.y, self.z = x, y, z
        self.sx, self.sy, self.sz = sx, sy, sz
        self.color = color

def create_arena(lib, arena_size=16.0, wall_height=10.0):
    half = arena_size / 2.0
    # Floor: static at Y=0.0 (surface at Y=0.5)
    lib.add_rigid_body(0.0, 0.0, 0.0, arena_size + 4.0, 1.0, arena_size + 4.0, 0.0, 0.65)

    # 4 Enclosing Walls to contain balls
    lib.add_rigid_body(0.0, wall_height / 2.0, half, arena_size, wall_height, 1.0, 0.0, 0.70)
    lib.add_rigid_body(0.0, wall_height / 2.0, -half, arena_size, wall_height, 1.0, 0.0, 0.70)
    lib.add_rigid_body(half, wall_height / 2.0, 0.0, 1.0, wall_height, arena_size, 0.0, 0.70)
    lib.add_rigid_body(-half, wall_height / 2.0, 0.0, 1.0, wall_height, arena_size, 0.0, 0.70)

    # Incline ramp structure for rolling dynamics
    obstacles = []
    ramp_steps = 4
    for i in range(ramp_steps):
        rw = 5.0
        rh = 0.5
        rd = 1.6
        rx = -4.5 + i * 0.4
        ry = 0.5 + i * rh
        rz = -4.5 + i * rd
        lib.add_rigid_body(rx, ry, rz, rw, rh, rd, 0.0, 0.50)
        obstacles.append(ObstacleBox(rx, ry, rz, rw, rh, rd, (45, 55, 75)))

    return obstacles

# ---------------------------------------------------------------------------
# 4. Main Simulation Loop
# ---------------------------------------------------------------------------
def main():
    pygame.init()
    pygame.display.set_caption("EngineLib 3D Ball Physics & Rolling Simulation")

    screen_w, screen_h = 1050, 720
    screen = pygame.display.set_mode((screen_w, screen_h), pygame.RESIZABLE)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Segoe UI", 16)
    title_font = pygame.font.SysFont("Segoe UI", 20, bold=True)

    lib = load_physics_dll()
    lib.init_physics_world(0.0, -9.81, 0.0)
    lib.set_world_substeps(3)
    lib.set_solver_iterations(5)

    arena_size = 15.0
    obstacles = create_arena(lib, arena_size=arena_size)

    camera = Camera3D()
    balls = []
    pos_buf = Vector3D()

    def spawn_ball(x=None, y=10.0, z=None, radius=None, is_heavy=False):
        if x is None:
            x = random.uniform(-4.0, 4.0)
        if z is None:
            z = random.uniform(-4.0, 4.0)
        if radius is None:
            radius = random.uniform(0.65, 1.1) if not is_heavy else 1.35

        if is_heavy:
            mass = 25.0
            restitution = 0.35
            friction = 0.60
            color = (80, 85, 95) # Dark iron bowling ball
        else:
            mass = radius * radius * radius * 3.5
            restitution = random.uniform(0.65, 0.78)
            friction = 0.45
            color = random.choice(VIBRANT_COLORS)

        b_id = lib.add_sphere_body(x, y, z, radius, mass, restitution, friction)
        # Gentle scatter impulse
        lib.apply_force(b_id, random.uniform(-20.0, 20.0), random.uniform(-5.0, 5.0), random.uniform(-20.0, 20.0))

        b = Ball(b_id, radius, color, is_heavy=is_heavy)
        b.x, b.y, b.z = x, y, z
        b.prev_x, b.prev_z = x, z
        balls.append(b)
        return b

    # Spawn starting cluster of 12 balls
    for _ in range(12):
        spawn_ball(y=random.uniform(6.0, 12.0))

    mouse_orbiting = False
    mouse_aiming = False
    grabbed_ball = None
    drag_start_pos = (0, 0)
    current_mouse = (0, 0)
    last_orbit_pos = (0, 0)
    gravity_on = True

    running = True
    while running:
        dt = clock.tick(60) / 1000.0
        dt = min(dt, 0.033)

        # -------------------------------------------------------------------
        # Event Handling
        # -------------------------------------------------------------------
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.VIDEORESIZE:
                screen_w, screen_h = event.w, event.h
                screen = pygame.display.set_mode((screen_w, screen_h), pygame.RESIZABLE)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    # Check if clicking on an existing ball to launch/fling it
                    mx, my = event.pos
                    clicked_ball = None
                    closest_dist = 999999
                    for b in balls:
                        dist = math.hypot(mx - b.screen_x, my - b.screen_y)
                        if dist <= b.screen_r + 6 and dist < closest_dist:
                            closest_dist = dist
                            clicked_ball = b

                    if clicked_ball is not None:
                        mouse_aiming = True
                        grabbed_ball = clicked_ball
                        drag_start_pos = event.pos
                    else:
                        mouse_orbiting = True
                        last_orbit_pos = event.pos
                elif event.button == 4:
                    camera.distance = max(10.0, camera.distance - 2.0)
                elif event.button == 5:
                    camera.distance = min(60.0, camera.distance + 2.0)
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    if mouse_aiming and grabbed_ball is not None:
                        # Fling ball with impulse proportional to drag vector
                        mx, my = event.pos
                        drag_dx = drag_start_pos[0] - mx
                        drag_dy = drag_start_pos[1] - my

                        # Convert screen drag vector into 3D world force based on camera yaw
                        cos_y, sin_y = math.cos(camera.yaw), math.sin(camera.yaw)
                        fx = (drag_dx * cos_y + drag_dy * sin_y) * 8.0
                        fz = (-drag_dx * sin_y + drag_dy * cos_y) * 8.0
                        fy = max(0.0, -drag_dy * 4.0)

                        lib.apply_force(grabbed_ball.id, fx, fy + 80.0, fz)

                    mouse_aiming = False
                    grabbed_ball = None
                    mouse_orbiting = False
            elif event.type == pygame.MOUSEMOTION:
                current_mouse = event.pos
                if mouse_orbiting:
                    dx = event.pos[0] - last_orbit_pos[0]
                    dy = event.pos[1] - last_orbit_pos[1]
                    camera.yaw += dx * 0.008
                    camera.pitch = max(-0.2, min(1.4, camera.pitch + dy * 0.008))
                    last_orbit_pos = event.pos
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    spawn_ball(y=14.0)
                elif event.key == pygame.K_b:
                    # Drop heavy iron bowling ball
                    spawn_ball(y=12.0, is_heavy=True)
                elif event.key == pygame.K_f:
                    # Explosive force blast
                    for b in balls:
                        lib.apply_force(
                            b.id,
                            random.uniform(-150.0, 150.0),
                            random.uniform(350.0, 700.0),
                            random.uniform(-150.0, 150.0),
                        )
                elif event.key == pygame.K_r:
                    # Reset arena
                    balls.clear()
                    lib.destroy_physics_world()
                    lib.init_physics_world(0.0, -9.81 if gravity_on else 0.0, 0.0)
                    lib.set_world_substeps(3)
                    lib.set_solver_iterations(5)
                    obstacles = create_arena(lib, arena_size=arena_size)
                    for _ in range(12):
                        spawn_ball(y=random.uniform(6.0, 12.0))
                elif event.key == pygame.K_g:
                    gravity_on = not gravity_on
                    lib.destroy_physics_world()
                    lib.init_physics_world(0.0, -9.81 if gravity_on else 0.0, 0.0)
                    lib.set_world_substeps(3)
                    lib.set_solver_iterations(5)
                    obstacles = create_arena(lib, arena_size=arena_size)
                    for b in balls:
                        b.id = lib.add_sphere_body(
                            b.x, b.y, b.z, b.radius,
                            25.0 if b.is_heavy else b.radius ** 3 * 3.5,
                            0.35 if b.is_heavy else 0.70,
                            0.60 if b.is_heavy else 0.45,
                        )

        # -------------------------------------------------------------------
        # Step Physics Engine DLL
        # -------------------------------------------------------------------
        lib.step_physics(c_float(dt))

        # Query updated positions and compute 3D rotational spin
        for b in balls:
            if lib.get_body_position(b.id, byref(pos_buf)):
                b.x = pos_buf.x
                b.y = pos_buf.y
                b.z = pos_buf.z
                b.update_rotation()

        # -------------------------------------------------------------------
        # Render 3D Scene
        # -------------------------------------------------------------------
        screen.fill((16, 20, 28))
        render_queue = []

        # 1. Floor grid rendering
        half = arena_size / 2.0
        grid_lines = 10
        for i in range(grid_lines + 1):
            val = -half + (i / grid_lines) * arena_size
            p1 = camera.project(-half, 0.5, val, screen_w, screen_h)
            p2 = camera.project(half, 0.5, val, screen_w, screen_h)
            if p1[0] and p2[0]:
                avg_z = (p1[2] + p2[2]) / 2.0
                render_queue.append((avg_z, "line", (p1[0], p1[1], p2[0], p2[1], (36, 46, 62), 1)))

            p3 = camera.project(val, 0.5, -half, screen_w, screen_h)
            p4 = camera.project(val, 0.5, half, screen_w, screen_h)
            if p3[0] and p4[0]:
                avg_z = (p3[2] + p4[2]) / 2.0
                render_queue.append((avg_z, "line", (p3[0], p3[1], p4[0], p4[1], (36, 46, 62), 1)))

        # 2. Obstacle Ramp Blocks
        for ob in obstacles:
            corners = [
                (ob.x - ob.sx/2, ob.y - ob.sy/2, ob.z - ob.sz/2),
                (ob.x + ob.sx/2, ob.y - ob.sy/2, ob.z - ob.sz/2),
                (ob.x + ob.sx/2, ob.y - ob.sy/2, ob.z + ob.sz/2),
                (ob.x - ob.sx/2, ob.y - ob.sy/2, ob.z + ob.sz/2),
                (ob.x - ob.sx/2, ob.y + ob.sy/2, ob.z - ob.sz/2),
                (ob.x + ob.sx/2, ob.y + ob.sy/2, ob.z - ob.sz/2),
                (ob.x + ob.sx/2, ob.y + ob.sy/2, ob.z + ob.sz/2),
                (ob.x - ob.sx/2, ob.y + ob.sy/2, ob.z + ob.sz/2),
            ]
            proj = [camera.project(c[0], c[1], c[2], screen_w, screen_h) for c in corners]
            if all(p[0] is not None for p in proj):
                top_poly = [proj[4], proj[5], proj[6], proj[7]]
                avg_depth = sum(p[2] for p in top_poly) / 4.0
                pts = [(p[0], p[1]) for p in top_poly]
                render_queue.append((avg_depth, "poly", (pts, (56, 68, 92), (75, 90, 120))))

        # 3. Balls & Ground Shadows
        for b in balls:
            # Shadow on floor (Y = 0.51)
            if b.y >= 0.4:
                sp = camera.project(b.x, 0.51, b.z, screen_w, screen_h)
                if sp[0]:
                    h_factor = max(0.1, min(1.0, 1.0 - (b.y - 0.5) / 14.0))
                    focal = (screen_w / 2.0) / math.tan(camera.fov / 2.0)
                    sr_x = max(2, int((b.radius * 1.15 * focal) / sp[2] * h_factor))
                    sr_y = max(2, int(sr_x * 0.45))
                    alpha = max(0, min(190, int(135 * h_factor)))
                    render_queue.append((sp[2] + 0.01, "shadow", (sp[0], sp[1], sr_x, sr_y, alpha)))

            # Ball Sphere
            bp = camera.project(b.x, b.y, b.z, screen_w, screen_h)
            if bp[0]:
                focal = (screen_w / 2.0) / math.tan(camera.fov / 2.0)
                screen_radius = max(2, int((b.radius * focal) / bp[2]))
                b.screen_x = bp[0]
                b.screen_y = bp[1]
                b.screen_r = screen_radius
                b.depth = bp[2]
                render_queue.append((bp[2], "sphere", (b, bp[0], bp[1], screen_radius)))

        # Sort render queue from back to front (Painter's Algorithm)
        render_queue.sort(key=lambda item: item[0], reverse=True)

        # Draw primitives in depth order
        for item in render_queue:
            rtype = item[1]
            data = item[2]

            if rtype == "line":
                pygame.draw.line(screen, data[4], (data[0], data[1]), (data[2], data[3]), data[5])
            elif rtype == "poly":
                pts, fill_col, border_col = data
                pygame.draw.polygon(screen, fill_col, pts)
                pygame.draw.polygon(screen, border_col, pts, 1)
            elif rtype == "shadow":
                sx, sy, rx, ry, alpha = data
                alpha = max(0, min(255, int(alpha)))
                if rx > 0 and ry > 0 and alpha > 0:
                    shadow_surf = pygame.Surface((rx * 2 + 2, ry * 2 + 2), pygame.SRCALPHA)
                    pygame.draw.ellipse(shadow_surf, (8, 10, 16, alpha), (0, 0, rx * 2, ry * 2))
                    screen.blit(shadow_surf, (sx - rx, sy - ry))
            elif rtype == "sphere":
                ball_obj, sx, sy, r = data
                col = ball_obj.color
                r = max(2, int(r))

                # 1. Dark outline
                border_col = (max(0, col[0] - 65), max(0, col[1] - 65), max(0, col[2] - 65))
                pygame.draw.circle(screen, border_col, (sx, sy), r)

                # 2. Base sphere body
                pygame.draw.circle(screen, col, (sx, sy), max(1, r - 1))

                # 3. Rotating 3D equator seam line to visualize true rolling
                seam_offset = math.sin(ball_obj.rot_x) * (r * 0.75)
                seam_r_x = max(1, int(r * 0.9))
                seam_r_y = max(1, int(abs(math.cos(ball_obj.rot_z)) * (r * 0.4)))
                seam_rect = pygame.Rect(sx - seam_r_x, sy + int(seam_offset) - seam_r_y, seam_r_x * 2, seam_r_y * 2)
                pygame.draw.arc(screen, ball_obj.stripe_color, seam_rect, 0, math.pi * 2, max(1, r // 10))

                # 4. Specular highlight
                spec_r = max(1, int(r * 0.32))
                spec_ox = sx - int(r * 0.30)
                spec_oy = sy - int(r * 0.30)
                highlight_col = (min(255, col[0] + 90), min(255, col[1] + 90), min(255, col[2] + 90))
                pygame.draw.circle(screen, highlight_col, (spec_ox, spec_oy), spec_r)
                pygame.draw.circle(screen, (255, 255, 255), (spec_ox - 1, spec_oy - 1), max(1, spec_r // 2))

                # Highlight grabbed ball
                if mouse_aiming and grabbed_ball == ball_obj:
                    pygame.draw.circle(screen, (255, 255, 100), (sx, sy), r + 3, 2)

        # Draw Aiming Drag Line
        if mouse_aiming and grabbed_ball is not None:
            # Draw line from ball screen center towards mouse drag
            start_pt = (grabbed_ball.screen_x, grabbed_ball.screen_y)
            arrow_end = (start_pt[0] + (drag_start_pos[0] - current_mouse[0]),
                         start_pt[1] + (drag_start_pos[1] - current_mouse[1]))
            pygame.draw.line(screen, (255, 235, 60), start_pt, arrow_end, 3)
            pygame.draw.circle(screen, (255, 255, 120), arrow_end, 6)

        # -------------------------------------------------------------------
        # HUD Information Display
        # -------------------------------------------------------------------
        title_surf = title_font.render("EngineLib 3D Ball Physics & Rolling Simulation", True, (240, 246, 252))
        screen.blit(title_surf, (20, 15))

        hud_lines = [
            f"FPS: {int(clock.get_fps())} | Active Balls: {len(balls)}",
            f"Gravity: {'ON (-9.81 m/s²)' if gravity_on else 'OFF (0.0 m/s²)'} | Substeps: 3",
            "Dynamics: Coulomb Friction & Rolling Resistance Active",
            "",
            "[Controls]",
            "• Click & Drag on a Ball: Aim & Fling Ball",
            "• Click & Drag on Arena: Orbit 3D Camera",
            "• Scroll Wheel: Zoom Camera In / Out",
            "• [SPACE]: Drop Colorful Sphere",
            "• [B]: Drop Heavy Iron Bowling Ball",
            "• [F]: Explosive Force Blast",
            "• [G]: Toggle Zero-G",
            "• [R]: Reset Arena",
        ]

        y_offset = 48
        for line in hud_lines:
            if line:
                txt = font.render(line, True, (190, 205, 225) if not line.startswith("[") else (130, 155, 185))
                screen.blit(txt, (20, y_offset))
            y_offset += 20

        pygame.display.flip()

    pygame.quit()
    sys.exit(0)

if __name__ == "__main__":
    main()
