#!/usr/bin/env python3
"""
Interactive 3D Ball Physics Simulation powered by EngineLib (physics_engine.dll)
Renders a 3D arena with bouncing spheres, camera orbit, dynamic shadows, and interactive forces.
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

    lib.get_body_position.argtypes = [c_uint32, POINTER(Vector3D)]
    lib.get_body_position.restype = c_bool

    lib.apply_force.argtypes = [c_uint32, c_float, c_float, c_float]
    lib.apply_force.restype = None

    lib.step_physics.argtypes = [c_float]
    lib.step_physics.restype = None

    return lib

# ---------------------------------------------------------------------------
# 2. 3D Camera & Projection
# ---------------------------------------------------------------------------
class Camera3D:
    def __init__(self, target=(0.0, 4.0, 0.0), distance=28.0, yaw=0.7, pitch=0.4):
        self.target = list(target)
        self.distance = distance
        self.yaw = yaw       # Horizontal rotation
        self.pitch = pitch   # Vertical rotation
        self.fov = math.radians(60)

    def project(self, x, y, z, screen_w, screen_h):
        # 1. Translate relative to target
        dx = x - self.target[0]
        dy = y - self.target[1]
        dz = z - self.target[2]

        # 2. Rotate around Y (yaw)
        cos_y, sin_y = math.cos(self.yaw), math.sin(self.yaw)
        x1 = dx * cos_y - dz * sin_y
        z1 = dx * sin_y + dz * cos_y
        y1 = dy

        # 3. Rotate around X (pitch)
        cos_p, sin_p = math.cos(self.pitch), math.sin(self.pitch)
        y2 = y1 * cos_p - z1 * sin_p
        z2 = y1 * sin_p + z1 * cos_p
        x2 = x1

        # 4. Camera view plane distance
        z_cam = z2 + self.distance
        if z_cam <= 0.2:
            return None, None, z_cam # Behind camera

        # 5. Perspective division
        focal = (screen_w / 2.0) / math.tan(self.fov / 2.0)
        sx = screen_w / 2.0 + (x2 * focal) / z_cam
        sy = screen_h / 2.0 - (y2 * focal) / z_cam
        return int(sx), int(sy), z_cam

# ---------------------------------------------------------------------------
# 3. Simulation World Setup
# ---------------------------------------------------------------------------
class Ball:
    def __init__(self, body_id, radius, color):
        self.id = body_id
        self.radius = radius
        self.color = color
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

VIBRANT_COLORS = [
    (255, 75, 75),   # Coral Red
    (75, 215, 255),  # Cyan
    (255, 205, 50),  # Gold
    (140, 255, 100), # Lime
    (210, 105, 255), # Neon Purple
    (255, 140, 50),  # Orange
    (255, 105, 180), # Hot Pink
]

def create_arena(lib, arena_size=16.0, wall_height=10.0):
    half = arena_size / 2.0
    # Floor: static at Y=0.0 (surface at Y=0.5)
    floor_id = lib.add_rigid_body(0.0, 0.0, 0.0, arena_size + 4.0, 1.0, arena_size + 4.0, 0.0, 0.75)

    # 4 Enclosing Walls to contain balls:
    # North wall (+Z)
    lib.add_rigid_body(0.0, wall_height / 2.0, half, arena_size, wall_height, 1.0, 0.0, 0.8)
    # South wall (-Z)
    lib.add_rigid_body(0.0, wall_height / 2.0, -half, arena_size, wall_height, 1.0, 0.0, 0.8)
    # East wall (+X)
    lib.add_rigid_body(half, wall_height / 2.0, 0.0, 1.0, wall_height, arena_size, 0.0, 0.8)
    # West wall (-X)
    lib.add_rigid_body(-half, wall_height / 2.0, 0.0, 1.0, wall_height, arena_size, 0.0, 0.8)
    return floor_id

# ---------------------------------------------------------------------------
# 4. Main Simulation Loop
# ---------------------------------------------------------------------------
def main():
    pygame.init()
    pygame.display.set_caption("EngineLib 3D Ball Physics Simulation (physics_engine.dll)")
    
    screen_w, screen_h = 1000, 700
    screen = pygame.display.set_mode((screen_w, screen_h), pygame.RESIZABLE)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Segoe UI", 16)
    title_font = pygame.font.SysFont("Segoe UI", 20, bold=True)

    # Initialize Engine
    lib = load_physics_dll()
    lib.init_physics_world(0.0, -9.81, 0.0)
    arena_size = 14.0
    create_arena(lib, arena_size=arena_size, wall_height=12.0)

    # Initialize camera & state
    camera = Camera3D()
    balls = []
    pos_buf = Vector3D()

    # Spawn initial burst of balls
    def spawn_ball(x=None, y=10.0, z=None, radius=None):
        if x is None:
            x = random.uniform(-4.0, 4.0)
        if z is None:
            z = random.uniform(-4.0, 4.0)
        if radius is None:
            radius = random.uniform(0.6, 1.2)
        mass = radius * radius * radius * 4.0 # Mass scales with volume
        restitution = random.uniform(0.75, 0.90)
        friction = 0.35
        color = random.choice(VIBRANT_COLORS)

        b_id = lib.add_sphere_body(x, y, z, radius, mass, restitution, friction)
        # Give initial slight impulse for fun scattering
        lib.apply_force(b_id, random.uniform(-30.0, 30.0), random.uniform(-10.0, 10.0), random.uniform(-30.0, 30.0))
        b = Ball(b_id, radius, color)
        balls.append(b)
        return b

    # Spawn starting cluster of 15 balls
    for _ in range(15):
        spawn_ball(y=random.uniform(6.0, 14.0))

    mouse_dragging = False
    last_mouse = (0, 0)
    gravity_on = True

    running = True
    while running:
        dt = clock.tick(60) / 1000.0
        # Clamp dt to prevent explosion on window drag
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
                if event.button == 1: # Left click drag orbit
                    mouse_dragging = True
                    last_mouse = event.pos
                elif event.button == 4: # Scroll up -> Zoom in
                    camera.distance = max(10.0, camera.distance - 2.0)
                elif event.button == 5: # Scroll down -> Zoom out
                    camera.distance = min(60.0, camera.distance + 2.0)
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    mouse_dragging = False
            elif event.type == pygame.MOUSEMOTION and mouse_dragging:
                dx = event.pos[0] - last_mouse[0]
                dy = event.pos[1] - last_mouse[1]
                camera.yaw += dx * 0.008
                camera.pitch = max(-0.2, min(1.4, camera.pitch + dy * 0.008))
                last_mouse = event.pos
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    # Spawn new ball from above
                    spawn_ball(y=14.0)
                elif event.key == pygame.K_f:
                    # Blast all balls with an explosive upward force
                    for b in balls:
                        lib.apply_force(b.id, random.uniform(-100.0, 100.0), random.uniform(300.0, 600.0), random.uniform(-100.0, 100.0))
                elif event.key == pygame.K_r:
                    # Reset simulation
                    balls.clear()
                    lib.destroy_physics_world()
                    lib.init_physics_world(0.0, -9.81 if gravity_on else 0.0, 0.0)
                    create_arena(lib, arena_size=arena_size)
                    for _ in range(12):
                        spawn_ball(y=random.uniform(6.0, 12.0))
                elif event.key == pygame.K_g:
                    # Toggle gravity
                    gravity_on = not gravity_on
                    lib.init_physics_world(0.0, -9.81 if gravity_on else 0.0, 0.0)
                    create_arena(lib, arena_size=arena_size)

        # -------------------------------------------------------------------
        # Step Physics Engine DLL
        # -------------------------------------------------------------------
        lib.step_physics(c_float(dt))

        # Query updated positions from native memory
        for b in balls:
            if lib.get_body_position(b.id, byref(pos_buf)):
                b.x = pos_buf.x
                b.y = pos_buf.y
                b.z = pos_buf.z

        # -------------------------------------------------------------------
        # Render 3D Scene
        # -------------------------------------------------------------------
        screen.fill((16, 20, 28)) # Dark modern theme

        render_queue = [] # (depth_z, render_type, data)

        # 1. Floor grid rendering
        half = arena_size / 2.0
        grid_lines = 8
        for i in range(grid_lines + 1):
            val = -half + (i / grid_lines) * arena_size
            # Lines along X
            p1 = camera.project(-half, 0.5, val, screen_w, screen_h)
            p2 = camera.project(half, 0.5, val, screen_w, screen_h)
            if p1[0] and p2[0]:
                avg_z = (p1[2] + p2[2]) / 2.0
                render_queue.append((avg_z, "line", (p1[0], p1[1], p2[0], p2[1], (38, 48, 64), 1)))

            # Lines along Z
            p3 = camera.project(val, 0.5, -half, screen_w, screen_h)
            p4 = camera.project(val, 0.5, half, screen_w, screen_h)
            if p3[0] and p4[0]:
                avg_z = (p3[2] + p4[2]) / 2.0
                render_queue.append((avg_z, "line", (p3[0], p3[1], p4[0], p4[1], (38, 48, 64), 1)))

        # 2. Arena bounding box boundary posts
        corners = [(-half, -half), (half, -half), (half, half), (-half, half)]
        for cx, cz in corners:
            b_bot = camera.project(cx, 0.5, cz, screen_w, screen_h)
            b_top = camera.project(cx, 8.0, cz, screen_w, screen_h)
            if b_bot[0] and b_top[0]:
                avg_z = (b_bot[2] + b_top[2]) / 2.0
                render_queue.append((avg_z, "line", (b_bot[0], b_bot[1], b_top[0], b_top[1], (60, 75, 100), 2)))

        # 3. Balls & Ground Shadows
        for b in balls:
            # Shadow on floor (Y = 0.5)
            if b.y >= 0.4:
                sp = camera.project(b.x, 0.51, b.z, screen_w, screen_h)
                if sp[0]:
                    h_factor = max(0.1, min(1.0, 1.0 - (b.y - 0.5) / 15.0))
                    focal = (screen_w / 2.0) / math.tan(camera.fov / 2.0)
                    sr_x = max(2, int((b.radius * 1.1 * focal) / sp[2] * h_factor))
                    sr_y = max(2, int(sr_x * 0.45))
                    alpha = max(0, min(200, int(140 * h_factor)))
                    render_queue.append((sp[2] + 0.01, "shadow", (sp[0], sp[1], sr_x, sr_y, alpha)))

            # Ball Sphere
            bp = camera.project(b.x, b.y, b.z, screen_w, screen_h)
            if bp[0]:
                focal = (screen_w / 2.0) / math.tan(camera.fov / 2.0)
                screen_radius = max(2, int((b.radius * focal) / bp[2]))
                render_queue.append((bp[2], "sphere", (bp[0], bp[1], screen_radius, b.color)))

        # Sort render queue from back to front (Painter's Algorithm)
        render_queue.sort(key=lambda item: item[0], reverse=True)

        # Draw primitives in depth order
        for item in render_queue:
            rtype = item[1]
            data = item[2]

            if rtype == "line":
                pygame.draw.line(screen, data[4], (data[0], data[1]), (data[2], data[3]), data[5])
            elif rtype == "shadow":
                # Draw transparent dark ellipse shadow
                sx, sy, rx, ry, alpha = data
                alpha = max(0, min(255, int(alpha)))
                if rx > 0 and ry > 0 and alpha > 0:
                    shadow_surf = pygame.Surface((rx * 2 + 2, ry * 2 + 2), pygame.SRCALPHA)
                    pygame.draw.ellipse(shadow_surf, (8, 10, 16, alpha), (0, 0, rx * 2, ry * 2))
                    screen.blit(shadow_surf, (sx - rx, sy - ry))
            elif rtype == "sphere":
                sx, sy, r, col = data
                r = max(2, int(r))
                # Outer shadow/border
                border_col = (max(0, col[0] - 60), max(0, col[1] - 60), max(0, col[2] - 60))
                pygame.draw.circle(screen, border_col, (sx, sy), r)
                # Main body
                pygame.draw.circle(screen, col, (sx, sy), max(1, r - 1))
                # Specular light reflection on top-left
                spec_r = max(1, int(r * 0.35))
                spec_offset_x = sx - int(r * 0.3)
                spec_offset_y = sy - int(r * 0.3)
                highlight_col = (min(255, col[0] + 100), min(255, col[1] + 100), min(255, col[2] + 100))
                pygame.draw.circle(screen, highlight_col, (spec_offset_x, spec_offset_y), spec_r)
                pygame.draw.circle(screen, (255, 255, 255), (spec_offset_x - 1, spec_offset_y - 1), max(1, spec_r // 2))

        # -------------------------------------------------------------------
        # Heads-Up Display (HUD)
        # -------------------------------------------------------------------
        title_surf = title_font.render("EngineLib 3D Ball Physics", True, (240, 246, 252))
        screen.blit(title_surf, (20, 15))

        hud_info = [
            f"FPS: {int(clock.get_fps())} | Active Balls: {len(balls)}",
            f"Gravity: {'-9.81 m/s² (ON)' if gravity_on else 'OFF'}",
            "",
            "[Controls]",
            "• Left Click + Drag: Orbit Camera",
            "• Scroll Wheel: Zoom In / Out",
            "• [SPACE]: Drop New Ball",
            "• [F]: Explosive Force Blast",
            "• [G]: Toggle Gravity",
            "• [R]: Reset Arena",
        ]

        y_offset = 48
        for line in hud_info:
            if line.startswith("[Controls]"):
                text_surf = font.render(line, True, (88, 166, 255))
            elif line.startswith("•"):
                text_surf = font.render(line, True, (139, 148, 158))
            else:
                text_surf = font.render(line, True, (201, 209, 217))
            screen.blit(text_surf, (20, y_offset))
            y_offset += 20

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
