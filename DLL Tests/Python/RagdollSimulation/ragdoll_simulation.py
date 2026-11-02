#!/usr/bin/env python3
"""
Interactive 3D Ragdoll Physics Simulation powered by EngineLib (physics_engine.dll)
Features articulated skeletal ragdolls with distance constraints, stairs, dynamic shadows,
mouse grabbing/throwing, and camera orbit.
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

    lib.add_distance_joint.argtypes = [
        c_uint32, c_uint32,
        c_float, c_float, c_float,
        c_float, c_float, c_float,
        c_float, c_float, c_float
    ]
    lib.add_distance_joint.restype = c_uint32

    lib.set_body_damping.argtypes = [c_uint32, c_float]
    lib.set_body_damping.restype = None

    lib.set_world_substeps.argtypes = [c_uint32]
    lib.set_world_substeps.restype = None

    lib.set_solver_iterations.argtypes = [c_uint32]
    lib.set_solver_iterations.restype = None

    lib.get_body_position.argtypes = [c_uint32, POINTER(Vector3D)]
    lib.get_body_position.restype = c_bool

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
    def __init__(self, target=(0.0, 5.0, 0.0), distance=32.0, yaw=0.8, pitch=0.35):
        self.target = list(target)
        self.distance = distance
        self.yaw = yaw
        self.pitch = pitch
        self.fov = math.radians(60)

    def project(self, x, y, z, screen_w, screen_h):
        dx = x - self.target[0]
        dy = y - self.target[1]
        dz = z - self.target[2]

        cos_y, sin_y = math.cos(self.yaw), math.sin(self.yaw)
        x1 = dx * cos_y - dz * sin_y
        z1 = dx * sin_y + dz * cos_y
        y1 = dy

        cos_p, sin_p = math.cos(self.pitch), math.sin(self.pitch)
        y2 = y1 * cos_p - z1 * sin_p
        z2 = y1 * sin_p + z1 * cos_p
        x2 = x1

        z_cam = z2 + self.distance
        if z_cam <= 0.2:
            return None, None, z_cam

        focal = (screen_w / 2.0) / math.tan(self.fov / 2.0)
        sx = screen_w / 2.0 + (x2 * focal) / z_cam
        sy = screen_h / 2.0 - (y2 * focal) / z_cam
        return int(sx), int(sy), z_cam

# ---------------------------------------------------------------------------
# 3. Ragdoll Structure
# ---------------------------------------------------------------------------
class Limb:
    def __init__(self, body_id, radius, color, name="limb"):
        self.id = body_id
        self.radius = radius
        self.color = color
        self.name = name
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class Ragdoll:
    def __init__(self, lib, base_x, base_y, base_z, color_theme):
        self.limbs = {}
        self.bones = [] # (limb_a, limb_b)

        # Color assignments
        head_col, torso_col, limb_col = color_theme

        # 1. Spawn Limbs as Spheres (low restitution for realistic floppy ragdoll)
        # Head
        h_id = lib.add_sphere_body(base_x, base_y + 3.8, base_z, 0.55, 3.0, 0.2, 0.5)
        self.limbs["head"] = Limb(h_id, 0.55, head_col, "head")

        # Torso / Chest
        t_id = lib.add_sphere_body(base_x, base_y + 2.4, base_z, 0.70, 10.0, 0.2, 0.5)
        self.limbs["torso"] = Limb(t_id, 0.70, torso_col, "torso")

        # Pelvis / Hips
        p_id = lib.add_sphere_body(base_x, base_y + 1.2, base_z, 0.60, 8.0, 0.2, 0.5)
        self.limbs["pelvis"] = Limb(p_id, 0.60, torso_col, "pelvis")

        # Left Arm (Upper + Lower)
        lua_id = lib.add_sphere_body(base_x - 1.2, base_y + 2.3, base_z, 0.40, 2.5, 0.2, 0.5)
        lla_id = lib.add_sphere_body(base_x - 2.2, base_y + 2.0, base_z, 0.35, 2.0, 0.2, 0.5)
        self.limbs["left_upper_arm"] = Limb(lua_id, 0.40, limb_col, "left_upper_arm")
        self.limbs["left_lower_arm"] = Limb(lla_id, 0.35, limb_col, "left_lower_arm")

        # Right Arm (Upper + Lower)
        rua_id = lib.add_sphere_body(base_x + 1.2, base_y + 2.3, base_z, 0.40, 2.5, 0.2, 0.5)
        rla_id = lib.add_sphere_body(base_x + 2.2, base_y + 2.0, base_z, 0.35, 2.0, 0.2, 0.5)
        self.limbs["right_upper_arm"] = Limb(rua_id, 0.40, limb_col, "right_upper_arm")
        self.limbs["right_lower_arm"] = Limb(rla_id, 0.35, limb_col, "right_lower_arm")

        # Left Leg (Thigh + Shin)
        lul_id = lib.add_sphere_body(base_x - 0.6, base_y - 0.1, base_z, 0.45, 4.0, 0.2, 0.5)
        lll_id = lib.add_sphere_body(base_x - 0.6, base_y - 1.4, base_z, 0.40, 3.0, 0.2, 0.5)
        self.limbs["left_thigh"] = Limb(lul_id, 0.45, limb_col, "left_thigh")
        self.limbs["left_shin"] = Limb(lll_id, 0.40, limb_col, "left_shin")

        # Right Leg (Thigh + Shin)
        rul_id = lib.add_sphere_body(base_x + 0.6, base_y - 0.1, base_z, 0.45, 4.0, 0.2, 0.5)
        rll_id = lib.add_sphere_body(base_x + 0.6, base_y - 1.4, base_z, 0.40, 3.0, 0.2, 0.5)
        self.limbs["right_thigh"] = Limb(rul_id, 0.45, limb_col, "right_thigh")
        self.limbs["right_shin"] = Limb(rll_id, 0.40, limb_col, "right_shin")

        # Set damping on all limbs for natural muscle drag
        for l in self.limbs.values():
            lib.set_body_damping(l.id, 0.15)

        # 2. Connect Skeletal Joints
        def connect(name_a, name_b, rest_len, stiffness=0.95, damping=0.3):
            id_a = self.limbs[name_a].id
            id_b = self.limbs[name_b].id
            lib.add_distance_joint(id_a, id_b, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, rest_len, stiffness, damping)
            self.bones.append((name_a, name_b))

        # Spine & Neck
        connect("head", "torso", 1.3, stiffness=0.98, damping=0.4)
        connect("torso", "pelvis", 1.2, stiffness=0.98, damping=0.4)

        # Left Arm
        connect("torso", "left_upper_arm", 1.2, stiffness=0.92, damping=0.3)
        connect("left_upper_arm", "left_lower_arm", 1.1, stiffness=0.92, damping=0.3)

        # Right Arm
        connect("torso", "right_upper_arm", 1.2, stiffness=0.92, damping=0.3)
        connect("right_upper_arm", "right_lower_arm", 1.1, stiffness=0.92, damping=0.3)

        # Left Leg
        connect("pelvis", "left_thigh", 1.3, stiffness=0.95, damping=0.3)
        connect("left_thigh", "left_shin", 1.3, stiffness=0.95, damping=0.3)

        # Right Leg
        connect("pelvis", "right_thigh", 1.3, stiffness=0.95, damping=0.3)
        connect("right_thigh", "right_shin", 1.3, stiffness=0.95, damping=0.3)

    def apply_impulse(self, lib, fx, fy, fz):
        for l in self.limbs.values():
            lib.apply_force(l.id, fx + random.uniform(-20, 20), fy + random.uniform(-20, 20), fz + random.uniform(-20, 20))

RAGDOLL_PALETTES = [
    ((255, 120, 100), (235, 75, 75), (200, 50, 50)),    # Crimson
    ((100, 220, 255), (45, 160, 240), (25, 120, 210)),  # Azure
    ((255, 215, 80),  (240, 180, 40), (210, 145, 20)),  # Amber
    ((160, 255, 120), (105, 220, 70), (70, 180, 45)),   # Emerald
    ((230, 140, 255), (195, 85, 240), (150, 45, 200)),  # Violet
]

def build_environment(lib):
    # Main Ground
    lib.add_rigid_body(0.0, 0.0, 0.0, 60.0, 1.0, 60.0, 0.0, 0.3)

    # Staircase Obstacles for dramatic tumbling
    steps = 6
    step_width = 16.0
    step_height = 0.8
    step_depth = 2.0
    start_x = -10.0
    start_y = 0.8
    start_z = -4.0

    for i in range(steps):
        sx = start_x + i * 1.8
        sy = start_y + i * step_height
        sz = start_z + i * 1.5
        lib.add_rigid_body(sx, sy, sz, step_width, step_height, step_depth, 0.0, 0.3)

# ---------------------------------------------------------------------------
# 4. Main Simulation Loop
# ---------------------------------------------------------------------------
def main():
    pygame.init()
    pygame.display.set_caption("EngineLib 3D Ragdoll Physics Simulation (physics_engine.dll)")

    screen_w, screen_h = 1050, 720
    screen = pygame.display.set_mode((screen_w, screen_h), pygame.RESIZABLE)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Segoe UI", 16)
    title_font = pygame.font.SysFont("Segoe UI", 20, bold=True)

    lib = load_physics_dll()
    lib.init_physics_world(0.0, -9.81, 0.0)
    # High-fidelity sub-stepping for stiff articulated limbs
    lib.set_world_substeps(4)
    lib.set_solver_iterations(6)

    build_environment(lib)

    camera = Camera3D()
    ragdolls = []
    pos_buf = Vector3D()

    def spawn_ragdoll(x=None, y=14.0, z=None):
        if x is None:
            x = random.uniform(-6.0, 6.0)
        if z is None:
            z = random.uniform(-4.0, 4.0)
        theme = random.choice(RAGDOLL_PALETTES)
        r = Ragdoll(lib, x, y, z, theme)
        # Give gentle initial rotation/tumble impulse
        r.apply_impulse(lib, random.uniform(-40, 40), random.uniform(-10, 10), random.uniform(-40, 40))
        ragdolls.append(r)
        return r

    # Spawn first 2 ragdolls
    spawn_ragdoll(x=-3.0, y=14.0, z=-1.0)
    spawn_ragdoll(x=3.0, y=16.0, z=2.0)

    mouse_dragging = False
    last_mouse = (0, 0)

    running = True
    while running:
        dt = clock.tick(60) / 1000.0
        dt = min(dt, 0.033)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.VIDEORESIZE:
                screen_w, screen_h = event.w, event.h
                screen = pygame.display.set_mode((screen_w, screen_h), pygame.RESIZABLE)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    mouse_dragging = True
                    last_mouse = event.pos
                elif event.button == 4:
                    camera.distance = max(12.0, camera.distance - 2.0)
                elif event.button == 5:
                    camera.distance = min(70.0, camera.distance + 2.0)
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
                    spawn_ragdoll(y=16.0)
                elif event.key == pygame.K_f:
                    # Explosive force blast launching all ragdolls into the air
                    for r in ragdolls:
                        r.apply_impulse(lib, random.uniform(-200, 200), random.uniform(600, 1000), random.uniform(-200, 200))
                elif event.key == pygame.K_r:
                    # Reset arena
                    ragdolls.clear()
                    lib.destroy_physics_world()
                    lib.init_physics_world(0.0, -9.81, 0.0)
                    lib.set_world_substeps(4)
                    lib.set_solver_iterations(6)
                    build_environment(lib)
                    spawn_ragdoll(x=0.0, y=14.0, z=0.0)

        # Step native engine
        lib.step_physics(c_float(dt))

        # Sync limb coordinates
        for r in ragdolls:
            for l in r.limbs.values():
                if lib.get_body_position(l.id, byref(pos_buf)):
                    l.x = pos_buf.x
                    l.y = pos_buf.y
                    l.z = pos_buf.z

        # -------------------------------------------------------------------
        # Render 3D Scene
        # -------------------------------------------------------------------
        screen.fill((16, 20, 28))
        render_queue = []

        # Floor Grid
        grid_size = 28.0
        grid_lines = 10
        half = grid_size / 2.0
        for i in range(grid_lines + 1):
            val = -half + (i / grid_lines) * grid_size
            p1 = camera.project(-half, 0.5, val, screen_w, screen_h)
            p2 = camera.project(half, 0.5, val, screen_w, screen_h)
            if p1[0] and p2[0]:
                render_queue.append(((p1[2] + p2[2]) / 2.0, "line", (p1[0], p1[1], p2[0], p2[1], (34, 42, 56), 1)))

            p3 = camera.project(val, 0.5, -half, screen_w, screen_h)
            p4 = camera.project(val, 0.5, half, screen_w, screen_h)
            if p3[0] and p4[0]:
                render_queue.append(((p3[2] + p4[2]) / 2.0, "line", (p3[0], p3[1], p4[0], p4[1], (34, 42, 56), 1)))

        # Draw Ragdoll Bones and Limbs
        for r in ragdolls:
            # 1. Bones (cylindrical connection lines between joints)
            for name_a, name_b in r.bones:
                la = r.limbs[name_a]
                lb = r.limbs[name_b]
                pa = camera.project(la.x, la.y, la.z, screen_w, screen_h)
                pb = camera.project(lb.x, lb.y, lb.z, screen_w, screen_h)
                if pa[0] and pb[0]:
                    avg_z = (pa[2] + pb[2]) / 2.0
                    thickness = max(2, int(6.0 * 20.0 / avg_z))
                    bone_col = (180, 190, 205)
                    render_queue.append((avg_z, "bone", (pa[0], pa[1], pb[0], pb[1], bone_col, thickness)))

            # 2. Limbs (spherical joints) & Shadows
            for l in r.limbs.values():
                # Ground shadow
                if l.y >= 0.4:
                    sp = camera.project(l.x, 0.51, l.z, screen_w, screen_h)
                    if sp[0]:
                        h_factor = max(0.1, min(1.0, 1.0 - (l.y - 0.5) / 12.0))
                        focal = (screen_w / 2.0) / math.tan(camera.fov / 2.0)
                        sr_x = max(2, int((l.radius * 1.1 * focal) / sp[2] * h_factor))
                        sr_y = max(2, int(sr_x * 0.45))
                        alpha = max(0, min(180, int(130 * h_factor)))
                        render_queue.append((sp[2] + 0.01, "shadow", (sp[0], sp[1], sr_x, sr_y, alpha)))

                # Spherical joint
                lp = camera.project(l.x, l.y, l.z, screen_w, screen_h)
                if lp[0]:
                    focal = (screen_w / 2.0) / math.tan(camera.fov / 2.0)
                    sr = max(3, int((l.radius * focal) / lp[2]))
                    render_queue.append((lp[2], "sphere", (lp[0], lp[1], sr, l.color)))

        # Depth Sort (Painter's Algorithm)
        render_queue.sort(key=lambda item: item[0], reverse=True)

        # Draw primitives in order
        for item in render_queue:
            rtype = item[1]
            data = item[2]

            if rtype == "line":
                pygame.draw.line(screen, data[4], (data[0], data[1]), (data[2], data[3]), data[5])
            elif rtype == "bone":
                # Outer shadow
                pygame.draw.line(screen, (30, 36, 48), (data[0], data[1]), (data[2], data[3]), data[5] + 2)
                # Bone cylinder
                pygame.draw.line(screen, data[4], (data[0], data[1]), (data[2], data[3]), data[5])
            elif rtype == "shadow":
                sx, sy, rx, ry, alpha = data
                alpha = max(0, min(255, int(alpha)))
                if rx > 0 and ry > 0 and alpha > 0:
                    shadow_surf = pygame.Surface((rx * 2 + 2, ry * 2 + 2), pygame.SRCALPHA)
                    pygame.draw.ellipse(shadow_surf, (8, 10, 16, alpha), (0, 0, rx * 2, ry * 2))
                    screen.blit(shadow_surf, (sx - rx, sy - ry))
            elif rtype == "sphere":
                sx, sy, r, col = data
                r = max(2, int(r))
                # Outer shadow border
                pygame.draw.circle(screen, (max(0, col[0] - 60), max(0, col[1] - 60), max(0, col[2] - 60)), (sx, sy), r)
                # Shaded body
                pygame.draw.circle(screen, col, (sx, sy), max(1, r - 1))
                # Specular light highlight
                spec_r = max(1, int(r * 0.35))
                ox = sx - int(r * 0.3)
                oy = sy - int(r * 0.3)
                pygame.draw.circle(screen, (min(255, col[0] + 90), min(255, col[1] + 90), min(255, col[2] + 90)), (ox, oy), spec_r)
                pygame.draw.circle(screen, (255, 255, 255), (ox - 1, oy - 1), max(1, spec_r // 2))

        # HUD
        title_surf = title_font.render("EngineLib 3D Ragdoll Physics", True, (240, 246, 252))
        screen.blit(title_surf, (20, 15))

        hud_lines = [
            f"FPS: {int(clock.get_fps())} | Ragdolls: {len(ragdolls)} (Total Limbs: {len(ragdolls)*10})",
            "Solver: 4 Substeps | 6 Iterations (XPBD Distance Constraints)",
            "",
            "[Controls]",
            "• Left Click + Drag: Orbit 3D Camera",
            "• Scroll Wheel: Zoom Camera In / Out",
            "• [SPACE]: Drop New Ragdoll",
            "• [F]: Explosive Force Blast",
            "• [R]: Reset Arena",
        ]

        y_off = 48
        for line in hud_lines:
            if line.startswith("[Controls]"):
                t_surf = font.render(line, True, (88, 166, 255))
            elif line.startswith("•"):
                t_surf = font.render(line, True, (139, 148, 158))
            else:
                t_surf = font.render(line, True, (201, 209, 217))
            screen.blit(t_surf, (20, y_off))
            y_off += 20

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
