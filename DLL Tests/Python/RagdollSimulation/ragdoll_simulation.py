#!/usr/bin/env python3
"""
Interactive 3D Ragdoll Physics Simulation powered by EngineLib (physics_engine.dll)
Features full biomechanical ragdolls with anatomical cross-bracing constraints,
XPBD distance joints, interactive mouse picking/grabbing/throwing, tumbling stairs,
dynamic shadows, and 3D camera controls.
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
    def __init__(self, target=(0.0, 4.0, 0.0), distance=30.0, yaw=0.85, pitch=0.38):
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
# 3. Biomechanical Ragdoll Structure
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
        self.screen_x = 0
        self.screen_y = 0
        self.screen_r = 8
        self.depth = 10.0

class Ragdoll:
    def __init__(self, lib, base_x, base_y, base_z, color_theme):
        self.limbs = {}
        self.visual_bones = [] # Bones drawn on screen (limb_a, limb_b)

        head_col, torso_col, limb_col = color_theme

        # 1. Spawn Articulated Limbs
        # Realistic human proportions, low restitution (0.15) to absorb impacts
        # Head
        h_id = lib.add_sphere_body(base_x, base_y + 3.8, base_z, 0.50, 3.2, 0.15, 0.55)
        self.limbs["head"] = Limb(h_id, 0.50, head_col, "head")

        # Upper Torso / Chest
        t_id = lib.add_sphere_body(base_x, base_y + 2.5, base_z, 0.65, 8.5, 0.15, 0.55)
        self.limbs["torso"] = Limb(t_id, 0.65, torso_col, "torso")

        # Pelvis / Hips
        p_id = lib.add_sphere_body(base_x, base_y + 1.3, base_z, 0.55, 7.5, 0.15, 0.55)
        self.limbs["pelvis"] = Limb(p_id, 0.55, torso_col, "pelvis")

        # Left Arm (Shoulder + Forearm)
        lua_id = lib.add_sphere_body(base_x - 1.25, base_y + 2.4, base_z, 0.36, 2.2, 0.15, 0.55)
        lla_id = lib.add_sphere_body(base_x - 2.25, base_y + 2.1, base_z, 0.30, 1.8, 0.15, 0.55)
        self.limbs["left_upper_arm"] = Limb(lua_id, 0.36, limb_col, "left_upper_arm")
        self.limbs["left_lower_arm"] = Limb(lla_id, 0.30, limb_col, "left_lower_arm")

        # Right Arm (Shoulder + Forearm)
        rua_id = lib.add_sphere_body(base_x + 1.25, base_y + 2.4, base_z, 0.36, 2.2, 0.15, 0.55)
        rla_id = lib.add_sphere_body(base_x + 2.25, base_y + 2.1, base_z, 0.30, 1.8, 0.15, 0.55)
        self.limbs["right_upper_arm"] = Limb(rua_id, 0.36, limb_col, "right_upper_arm")
        self.limbs["right_lower_arm"] = Limb(rla_id, 0.30, limb_col, "right_lower_arm")

        # Left Leg (Thigh + Shin)
        lul_id = lib.add_sphere_body(base_x - 0.55, base_y + 0.1, base_z, 0.42, 4.5, 0.15, 0.55)
        lll_id = lib.add_sphere_body(base_x - 0.55, base_y - 1.2, base_z, 0.36, 3.2, 0.15, 0.55)
        self.limbs["left_thigh"] = Limb(lul_id, 0.42, limb_col, "left_thigh")
        self.limbs["left_shin"] = Limb(lll_id, 0.36, limb_col, "left_shin")

        # Right Leg (Thigh + Shin)
        rul_id = lib.add_sphere_body(base_x + 0.55, base_y + 0.1, base_z, 0.42, 4.5, 0.15, 0.55)
        rll_id = lib.add_sphere_body(base_x + 0.55, base_y - 1.2, base_z, 0.36, 3.2, 0.15, 0.55)
        self.limbs["right_thigh"] = Limb(rul_id, 0.42, limb_col, "right_thigh")
        self.limbs["right_shin"] = Limb(rll_id, 0.36, limb_col, "right_shin")

        # Set muscle damping on limbs for natural articulation
        for l in self.limbs.values():
            lib.set_body_damping(l.id, 0.22)

        # 2. Connect Skeletal Joints & Structural Cross-Braces
        def add_joint(name_a, name_b, rest_len, stiffness=0.96, damping=0.35, is_visual=True):
            id_a = self.limbs[name_a].id
            id_b = self.limbs[name_b].id
            lib.add_distance_joint(id_a, id_b, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, rest_len, stiffness, damping)
            if is_visual:
                self.visual_bones.append((name_a, name_b))

        # --- Primary Skeletal Bone Joints ---
        add_joint("head", "torso", 1.30, stiffness=0.98, damping=0.45)
        add_joint("torso", "pelvis", 1.20, stiffness=0.98, damping=0.45)

        add_joint("torso", "left_upper_arm", 1.25, stiffness=0.95, damping=0.35)
        add_joint("left_upper_arm", "left_lower_arm", 1.05, stiffness=0.95, damping=0.35)

        add_joint("torso", "right_upper_arm", 1.25, stiffness=0.95, damping=0.35)
        add_joint("right_upper_arm", "right_lower_arm", 1.05, stiffness=0.95, damping=0.35)

        add_joint("pelvis", "left_thigh", 1.25, stiffness=0.96, damping=0.35)
        add_joint("left_thigh", "left_shin", 1.30, stiffness=0.96, damping=0.35)

        add_joint("pelvis", "right_thigh", 1.25, stiffness=0.96, damping=0.35)
        add_joint("right_thigh", "right_shin", 1.30, stiffness=0.96, damping=0.35)

        # --- Biomechanical Cross-Bracing Constraints ---
        # Shoulder-to-Shoulder (maintains chest width and rigid upper torso)
        add_joint("left_upper_arm", "right_upper_arm", 2.50, stiffness=0.92, damping=0.35, is_visual=False)

        # Hip-to-Hip (maintains pelvis separation and prevents scissors overlap)
        add_joint("left_thigh", "right_thigh", 1.10, stiffness=0.92, damping=0.35, is_visual=False)

        # Neck angle support (prevents head from snapping backward unnaturally)
        add_joint("head", "left_upper_arm", 1.85, stiffness=0.88, damping=0.35, is_visual=False)
        add_joint("head", "right_upper_arm", 1.85, stiffness=0.88, damping=0.35, is_visual=False)

        # Torso upright rectangular bracing (prevents accordion spine folding)
        add_joint("left_upper_arm", "pelvis", 1.70, stiffness=0.88, damping=0.35, is_visual=False)
        add_joint("right_upper_arm", "pelvis", 1.70, stiffness=0.88, damping=0.35, is_visual=False)

    def apply_impulse(self, lib, fx, fy, fz):
        for l in self.limbs.values():
            lib.apply_force(
                l.id,
                fx + random.uniform(-25, 25),
                fy + random.uniform(-15, 15),
                fz + random.uniform(-25, 25),
            )

RAGDOLL_PALETTES = [
    ((255, 110, 95), (235, 65, 65), (195, 45, 45)),     # Crimson
    ((95, 215, 255), (40, 155, 235), (20, 115, 205)),   # Azure
    ((255, 210, 75), (235, 175, 35), (205, 140, 18)),   # Amber
    ((155, 255, 115), (100, 215, 65), (65, 175, 40)),   # Emerald
    ((225, 135, 255), (190, 80, 235), (145, 40, 195)),  # Violet
]

def build_environment(lib):
    # Main Ground Floor
    lib.add_rigid_body(0.0, 0.0, 0.0, 70.0, 1.0, 70.0, 0.0, 0.35)

    # Staircase and Tumbling Platform Obstacles
    steps = 7
    step_width = 16.0
    step_height = 0.90
    step_depth = 2.2
    start_x = -12.0
    start_y = 0.90
    start_z = -5.0

    boxes = []
    for i in range(steps):
        sx = start_x + i * 1.6
        sy = start_y + i * step_height
        sz = start_z + i * 1.4
        lib.add_rigid_body(sx, sy, sz, step_width, step_height, step_depth, 0.0, 0.30)
        boxes.append((sx, sy, sz, step_width, step_height, step_depth))

    # Top Drop Platform
    top_x = start_x + (steps - 1) * 1.6 + 4.0
    top_y = start_y + steps * step_height
    top_z = start_z + (steps - 1) * 1.4
    lib.add_rigid_body(top_x, top_y, top_z, 12.0, 1.0, 10.0, 0.0, 0.30)
    boxes.append((top_x, top_y, top_z, 12.0, 1.0, 10.0))

    return boxes

# ---------------------------------------------------------------------------
# 4. Main Simulation Loop
# ---------------------------------------------------------------------------
def main():
    pygame.init()
    pygame.display.set_caption("EngineLib 3D Ragdoll Physics Simulation")

    screen_w, screen_h = 1050, 720
    screen = pygame.display.set_mode((screen_w, screen_h), pygame.RESIZABLE)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Segoe UI", 16)
    title_font = pygame.font.SysFont("Segoe UI", 20, bold=True)

    lib = load_physics_dll()
    lib.init_physics_world(0.0, -9.81, 0.0)
    lib.set_world_substeps(4)
    lib.set_solver_iterations(6)

    boxes = build_environment(lib)

    camera = Camera3D()
    ragdolls = []
    pos_buf = Vector3D()

    def spawn_ragdoll(x=None, y=14.0, z=None):
        if x is None:
            x = random.uniform(-4.0, 4.0)
        if z is None:
            z = random.uniform(-3.0, 3.0)
        theme = random.choice(RAGDOLL_PALETTES)
        r = Ragdoll(lib, x, y, z, theme)
        # Gentle initial tumble impulse
        r.apply_impulse(lib, random.uniform(-30, 30), random.uniform(-5, 5), random.uniform(-30, 30))
        ragdolls.append(r)
        return r

    # Spawn starting ragdolls
    spawn_ragdoll(x=-2.0, y=14.0, z=-1.0)
    spawn_ragdoll(x=3.0, y=16.0, z=2.0)

    mouse_orbiting = False
    mouse_grabbing = False
    grabbed_limb = None
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
                    # Check if clicking a ragdoll limb to grab it
                    mx, my = event.pos
                    clicked_limb = None
                    closest_dist = 999999
                    for r in ragdolls:
                        for l in r.limbs.values():
                            dist = math.hypot(mx - l.screen_x, my - l.screen_y)
                            if dist <= l.screen_r + 6 and dist < closest_dist:
                                closest_dist = dist
                                clicked_limb = l

                    if clicked_limb is not None:
                        mouse_grabbing = True
                        grabbed_limb = clicked_limb
                        drag_start_pos = event.pos
                    else:
                        mouse_orbiting = True
                        last_orbit_pos = event.pos
                elif event.button == 4:
                    camera.distance = max(10.0, camera.distance - 2.0)
                elif event.button == 5:
                    camera.distance = min(70.0, camera.distance + 2.0)
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    if mouse_grabbing and grabbed_limb is not None:
                        # Release with fling impulse
                        mx, my = event.pos
                        drag_dx = mx - drag_start_pos[0]
                        drag_dy = my - drag_start_pos[1]

                        cos_y, sin_y = math.cos(camera.yaw), math.sin(camera.yaw)
                        fx = (drag_dx * cos_y) * 14.0
                        fz = (-drag_dx * sin_y) * 14.0
                        fy = max(-100.0, -drag_dy * 16.0)

                        lib.apply_force(grabbed_limb.id, fx, fy + 50.0, fz)

                    mouse_grabbing = False
                    grabbed_limb = None
                    mouse_orbiting = False
            elif event.type == pygame.MOUSEMOTION:
                current_mouse = event.pos
                if mouse_orbiting:
                    dx = event.pos[0] - last_orbit_pos[0]
                    dy = event.pos[1] - last_orbit_pos[1]
                    camera.yaw += dx * 0.008
                    camera.pitch = max(-0.2, min(1.4, camera.pitch + dy * 0.008))
                    last_orbit_pos = event.pos
                elif mouse_grabbing and grabbed_limb is not None:
                    # Apply continuous dragging spring force towards mouse cursor
                    mx, my = event.pos
                    drag_dx = mx - grabbed_limb.screen_x
                    drag_dy = my - grabbed_limb.screen_y

                    cos_y, sin_y = math.cos(camera.yaw), math.sin(camera.yaw)
                    fx = (drag_dx * cos_y) * 8.0
                    fz = (-drag_dx * sin_y) * 8.0
                    fy = -drag_dy * 9.0

                    lib.apply_force(grabbed_limb.id, fx, fy, fz)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    spawn_ragdoll(y=16.0)
                elif event.key == pygame.K_f:
                    # Explosive force blast launching all ragdolls into the air
                    for r in ragdolls:
                        r.apply_impulse(
                            lib,
                            random.uniform(-180, 180),
                            random.uniform(500, 900),
                            random.uniform(-180, 180),
                        )
                elif event.key == pygame.K_g:
                    gravity_on = not gravity_on
                    lib.destroy_physics_world()
                    lib.init_physics_world(0.0, -9.81 if gravity_on else 0.0, 0.0)
                    lib.set_world_substeps(4)
                    lib.set_solver_iterations(6)
                    build_environment(lib)
                    ragdolls.clear()
                    spawn_ragdoll(x=0.0, y=14.0, z=0.0)
                elif event.key == pygame.K_r:
                    # Reset arena
                    ragdolls.clear()
                    lib.destroy_physics_world()
                    lib.init_physics_world(0.0, -9.81 if gravity_on else 0.0, 0.0)
                    lib.set_world_substeps(4)
                    lib.set_solver_iterations(6)
                    build_environment(lib)
                    spawn_ragdoll(x=-2.0, y=14.0, z=-1.0)
                    spawn_ragdoll(x=3.0, y=16.0, z=2.0)

        # -------------------------------------------------------------------
        # Step Physics Engine DLL
        # -------------------------------------------------------------------
        lib.step_physics(c_float(dt))

        # Sync limb coordinates from native memory
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
        grid_size = 32.0
        grid_lines = 10
        half = grid_size / 2.0
        for i in range(grid_lines + 1):
            val = -half + (i / grid_lines) * grid_size
            p1 = camera.project(-half, 0.5, val, screen_w, screen_h)
            p2 = camera.project(half, 0.5, val, screen_w, screen_h)
            if p1[0] and p2[0]:
                render_queue.append(((p1[2] + p2[2]) / 2.0, "line", (p1[0], p1[1], p2[0], p2[1], (34, 42, 58), 1)))

            p3 = camera.project(val, 0.5, -half, screen_w, screen_h)
            p4 = camera.project(val, 0.5, half, screen_w, screen_h)
            if p3[0] and p4[0]:
                render_queue.append(((p3[2] + p4[2]) / 2.0, "line", (p3[0], p3[1], p4[0], p4[1], (34, 42, 58), 1)))

        # Staircase and Platform Geometry
        for b_box in boxes:
            bx, by, bz, bsx, bsy, bsz = b_box
            corners = [
                (bx - bsx/2, by - bsy/2, bz - bsz/2),
                (bx + bsx/2, by - bsy/2, bz - bsz/2),
                (bx + bsx/2, by - bsy/2, bz + bsz/2),
                (bx - bsx/2, by - bsy/2, bz + bsz/2),
                (bx - bsx/2, by + bsy/2, bz - bsz/2),
                (bx + bsx/2, by + bsy/2, bz - bsz/2),
                (bx + bsx/2, by + bsy/2, bz + bsz/2),
                (bx - bsx/2, by + bsy/2, bz + bsz/2),
            ]
            proj = [camera.project(c[0], c[1], c[2], screen_w, screen_h) for c in corners]
            if all(p[0] is not None for p in proj):
                top_poly = [proj[4], proj[5], proj[6], proj[7]]
                avg_depth = sum(p[2] for p in top_poly) / 4.0
                pts = [(p[0], p[1]) for p in top_poly]
                render_queue.append((avg_depth, "poly", (pts, (48, 60, 82), (70, 85, 115))))

        # Draw Ragdoll Bones and Limbs
        for r in ragdolls:
            # 1. Bones (cylinders connecting joints)
            for name_a, name_b in r.visual_bones:
                la = r.limbs[name_a]
                lb = r.limbs[name_b]
                pa = camera.project(la.x, la.y, la.z, screen_w, screen_h)
                pb = camera.project(lb.x, lb.y, lb.z, screen_w, screen_h)
                if pa[0] and pb[0]:
                    avg_z = (pa[2] + pb[2]) / 2.0
                    focal = (screen_w / 2.0) / math.tan(camera.fov / 2.0)
                    thickness = max(3, int(0.20 * focal / avg_z))
                    bone_col = (
                        (la.color[0] + lb.color[0]) // 2,
                        (la.color[1] + lb.color[1]) // 2,
                        (la.color[2] + lb.color[2]) // 2,
                    )
                    render_queue.append((avg_z + 0.02, "bone", (pa[0], pa[1], pb[0], pb[1], bone_col, thickness)))

            # 2. Limb Spheres & Shadows
            for l in r.limbs.values():
                # Ground shadow
                if l.y >= 0.4:
                    sp = camera.project(l.x, 0.51, l.z, screen_w, screen_h)
                    if sp[0]:
                        h_factor = max(0.1, min(1.0, 1.0 - (l.y - 0.5) / 14.0))
                        focal = (screen_w / 2.0) / math.tan(camera.fov / 2.0)
                        sr_x = max(2, int((l.radius * 1.15 * focal) / sp[2] * h_factor))
                        sr_y = max(2, int(sr_x * 0.45))
                        alpha = max(0, min(180, int(130 * h_factor)))
                        render_queue.append((sp[2] + 0.01, "shadow", (sp[0], sp[1], sr_x, sr_y, alpha)))

                # Spherical joint
                lp = camera.project(l.x, l.y, l.z, screen_w, screen_h)
                if lp[0]:
                    focal = (screen_w / 2.0) / math.tan(camera.fov / 2.0)
                    sr = max(3, int((l.radius * focal) / lp[2]))
                    l.screen_x = lp[0]
                    l.screen_y = lp[1]
                    l.screen_r = sr
                    l.depth = lp[2]
                    render_queue.append((lp[2], "sphere", (l, lp[0], lp[1], sr, l.color)))

        # Depth Sort (Painter's Algorithm)
        render_queue.sort(key=lambda item: item[0], reverse=True)

        # Draw primitives in order
        for item in render_queue:
            rtype = item[1]
            data = item[2]

            if rtype == "line":
                pygame.draw.line(screen, data[4], (data[0], data[1]), (data[2], data[3]), data[5])
            elif rtype == "poly":
                pts, fill_col, border_col = data
                pygame.draw.polygon(screen, fill_col, pts)
                pygame.draw.polygon(screen, border_col, pts, 1)
            elif rtype == "bone":
                # Outer shadow
                pygame.draw.line(screen, (28, 34, 46), (data[0], data[1]), (data[2], data[3]), data[5] + 2)
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
                limb_obj, sx, sy, r, col = data
                r = max(2, int(r))

                # Outer shadow border
                pygame.draw.circle(screen, (max(0, col[0] - 65), max(0, col[1] - 65), max(0, col[2] - 65)), (sx, sy), r)
                # Shaded body
                pygame.draw.circle(screen, col, (sx, sy), max(1, r - 1))
                # Specular light highlight
                spec_r = max(1, int(r * 0.32))
                ox = sx - int(r * 0.30)
                oy = sy - int(r * 0.30)
                pygame.draw.circle(screen, (min(255, col[0] + 90), min(255, col[1] + 90), min(255, col[2] + 90)), (ox, oy), spec_r)
                pygame.draw.circle(screen, (255, 255, 255), (ox - 1, oy - 1), max(1, spec_r // 2))

                # Highlight grabbed limb
                if mouse_grabbing and grabbed_limb == limb_obj:
                    pygame.draw.circle(screen, (255, 255, 120), (sx, sy), r + 4, 2)

        # Draw Mouse Drag Spring Line
        if mouse_grabbing and grabbed_limb is not None:
            start_pt = (grabbed_limb.screen_x, grabbed_limb.screen_y)
            pygame.draw.line(screen, (255, 235, 70), start_pt, current_mouse, 3)
            pygame.draw.circle(screen, (255, 255, 120), current_mouse, 6)

        # -------------------------------------------------------------------
        # HUD Display
        # -------------------------------------------------------------------
        title_surf = title_font.render("EngineLib 3D Ragdoll Physics Simulation", True, (240, 246, 252))
        screen.blit(title_surf, (20, 15))

        grabbed_text = f"Grabbed: {grabbed_limb.name.replace('_', ' ').title()}" if grabbed_limb else "Grabbed: None"
        hud_lines = [
            f"FPS: {int(clock.get_fps())} | Ragdolls: {len(ragdolls)} (Total Limbs: {len(ragdolls)*11})",
            f"Gravity: {'ON (-9.81 m/s²)' if gravity_on else 'OFF'} | {grabbed_text}",
            "Solver: 4 Substeps | 6 XPBD Joint Iterations with Cross-Braces",
            "",
            "[Controls]",
            "• Click & Drag on Any Limb: Grab & Fling Ragdoll",
            "• Click & Drag on Empty Arena: Orbit 3D Camera",
            "• Scroll Wheel: Zoom Camera In / Out",
            "• [SPACE]: Drop New Ragdoll",
            "• [F]: Explosive Force Blast",
            "• [G]: Toggle Zero-G Mode",
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
