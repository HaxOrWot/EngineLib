#!/usr/bin/env python3
"""
=============================================================================
 EngineLib (physics_engine.dll) - Python Integration Sample
=============================================================================
This sample demonstrates how to bind and consume physics_engine.dll from Python
using standard library `ctypes`. No external dependencies required!

Topics Covered:
  1. Locating and loading the native DLL (64-bit Windows)
  2. Defining C-ABI structs (Vector3D, RaycastHit)
  3. Configuring ctypes function signatures (argtypes, restype)
  4. Initializing the 3D physics world
  5. Spawning static terrain and dynamic rigid bodies (Box, Sphere)
  6. Articulating limbs/joints with distance constraints
  7. Advancing the physics simulation loop (step_physics)
  8. Querying positions, velocities, and rotations
  9. Performing scene raycasts
  10. Proper teardown and cleanup
=============================================================================
"""

import os
import sys
import ctypes
from ctypes import c_float, c_uint32, c_bool, Structure, POINTER, byref

# ---------------------------------------------------------------------------
# 1. C-ABI Structure Definitions
# ---------------------------------------------------------------------------
class Vector3D(Structure):
    """Matches #[repr(C)] struct Vector3D { x: f32, y: f32, z: f32 }"""
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
    ]

    def __repr__(self):
        return f"({self.x:6.2f}, {self.y:6.2f}, {self.z:6.2f})"


class RaycastHitFfi(Structure):
    """Matches #[repr(C)] struct RaycastHitFfi"""
    _fields_ = [
        ("point", Vector3D),
        ("normal", Vector3D),
        ("distance", c_float),
        ("body_id", c_uint32),
    ]


# ---------------------------------------------------------------------------
# 2. DLL Loader & Signature Configuration
# ---------------------------------------------------------------------------
def load_physics_engine():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(current_dir, "physics_engine.dll"),
        os.path.join(current_dir, "..", "physics_engine.dll"),
        os.path.join(current_dir, "..", "..", "PhysicsEngine", "target", "release", "physics_engine.dll"),
    ]

    dll_path = None
    for p in candidates:
        if os.path.isfile(p):
            dll_path = os.path.abspath(p)
            break

    if not dll_path:
        raise FileNotFoundError(f"Could not locate physics_engine.dll! Searched: {candidates}")

    lib = ctypes.CDLL(dll_path)
    print(f"[EngineLib] Successfully loaded: {dll_path}")

    # World Management
    lib.init_physics_world.argtypes = [c_float, c_float, c_float]
    lib.init_physics_world.restype = None

    lib.destroy_physics_world.argtypes = []
    lib.destroy_physics_world.restype = None

    lib.step_physics.argtypes = [c_float]
    lib.step_physics.restype = None

    lib.set_world_substeps.argtypes = [c_uint32]
    lib.set_world_substeps.restype = None

    lib.set_solver_iterations.argtypes = [c_uint32]
    lib.set_solver_iterations.restype = None

    # Body Management
    lib.add_rigid_body.argtypes = [c_float, c_float, c_float, c_float, c_float, c_float, c_float, c_float]
    lib.add_rigid_body.restype = c_uint32

    lib.add_sphere_body.argtypes = [c_float, c_float, c_float, c_float, c_float, c_float, c_float]
    lib.add_sphere_body.restype = c_uint32

    if hasattr(lib, "remove_body"):
        lib.remove_body.argtypes = [c_uint32]
        lib.remove_body.restype = c_bool

    # State Queries & Telemetry
    lib.get_body_position.argtypes = [c_uint32, POINTER(Vector3D)]
    lib.get_body_position.restype = c_bool

    lib.get_body_velocity.argtypes = [c_uint32, POINTER(Vector3D)]
    lib.get_body_velocity.restype = c_bool

    lib.get_body_rotation.argtypes = [c_uint32, POINTER(Vector3D)]
    lib.get_body_rotation.restype = c_bool

    lib.get_body_angular_velocity.argtypes = [c_uint32, POINTER(Vector3D)]
    lib.get_body_angular_velocity.restype = c_bool

    # Dynamics & Impulses
    lib.apply_force.argtypes = [c_uint32, c_float, c_float, c_float]
    lib.apply_force.restype = None

    lib.apply_torque.argtypes = [c_uint32, c_float, c_float, c_float]
    lib.apply_torque.restype = None

    lib.apply_impulse_at_point.argtypes = [
        c_uint32,
        c_float, c_float, c_float,  # impulse x, y, z
        c_float, c_float, c_float   # point x, y, z
    ]
    lib.apply_impulse_at_point.restype = None

    # Joint Constraints
    lib.add_distance_joint.argtypes = [
        c_uint32, c_uint32,         # body_a, body_b
        c_float, c_float, c_float,  # offset_a (x, y, z)
        c_float, c_float, c_float,  # offset_b (x, y, z)
        c_float, c_float, c_float   # rest_length, stiffness, damping
    ]
    lib.add_distance_joint.restype = c_uint32

    # Scene Queries
    lib.raycast.argtypes = [
        c_float, c_float, c_float,  # origin x, y, z
        c_float, c_float, c_float,  # dir x, y, z
        c_float,                    # max_dist
        POINTER(RaycastHitFfi)      # output hit
    ]
    lib.raycast.restype = c_bool

    return lib


# ---------------------------------------------------------------------------
# 3. Execution Sample
# ---------------------------------------------------------------------------
def main():
    print("=" * 70)
    print("   EngineLib Native DLL (physics_engine.dll) - Python Sample")
    print("=" * 70)

    lib = load_physics_engine()

    # Step 1: Initialize the world with standard Earth gravity: g = -9.81 m/s^2 on Y
    print("\n[Step 1] Initializing Physics World (Gravity: Y = -9.81 m/s^2)...")
    lib.init_physics_world(0.0, -9.81, 0.0)
    lib.set_world_substeps(4)       # 4 sub-steps per frame for high stability
    lib.set_solver_iterations(8)    # 8 Gauss-Seidel iterations for rigid contacts

    # Step 2: Spawn static floor (mass = 0.0 means static/immovable)
    # Box at Y=0.0 with size (50m x 1m x 50m) -> Top surface is at Y = 0.5m
    print("[Step 2] Spawning static ground floor...")
    floor_id = lib.add_rigid_body(
        0.0, 0.0, 0.0,      # position (x, y, z)
        50.0, 1.0, 50.0,    # size (width, height, depth)
        0.0,                # mass = 0 (STATIC)
        0.2                 # restitution (rebound bounciness 0.0 - 1.0)
    )
    print(f"  Floor created with ID: {floor_id}")

    # Step 3: Spawn a dynamic bouncy sphere above the floor
    print("[Step 3] Spawning dynamic sphere at Y = 6.0m...")
    sphere_id = lib.add_sphere_body(
        0.0, 6.0, 0.0,      # position (x, y, z)
        0.5,                # radius = 0.5m
        2.5,                # mass = 2.5 kg
        0.65,               # restitution = 0.65 (bouncy)
        0.45                # friction = 0.45
    )
    print(f"  Sphere created with ID: {sphere_id}")

    # Step 4: Spawn a dynamic falling box with off-center impact
    print("[Step 4] Spawning dynamic box at Y = 8.0m...")
    box_id = lib.add_rigid_body(
        1.5, 8.0, 0.0,      # position (x, y, z)
        1.0, 2.0, 1.0,      # size: 1m x 2m x 1m tall pillar
        5.0,                # mass = 5.0 kg
        0.15                # restitution = 0.15 (firm stone)
    )
    print(f"  Box created with ID: {box_id}")

    # Apply off-center impulse to demonstrate rotational torque & toppling
    lib.apply_impulse_at_point(box_id, 15.0, 0.0, 0.0, 1.5, 8.8, 0.0)

    # Step 5: Simulate 90 frames (1.5 seconds at 60 FPS)
    print("\n[Step 5] Simulating 90 physics frames (60 Hz)...")
    print("-" * 70)
    print(f"{'Frame':^6} | {'Time':^6} | {'Sphere Pos (Y)':^15} | {'Sphere Vel (Y)':^15} | {'Box Rot (Z)':^12}")
    print("-" * 70)

    pos_s = Vector3D()
    vel_s = Vector3D()
    rot_b = Vector3D()

    dt = 1.0 / 60.0
    for frame in range(1, 91):
        # Advance physics by delta-time
        lib.step_physics(c_float(dt))

        # Query state from native memory
        lib.get_body_position(sphere_id, byref(pos_s))
        lib.get_body_velocity(sphere_id, byref(vel_s))
        lib.get_body_rotation(box_id, byref(rot_b))

        # Log periodic progress
        if frame % 15 == 0 or frame == 1:
            sim_time = frame * dt
            print(f"{frame:6d} | {sim_time:5.2f}s | Y={pos_s.y:6.2f}m        | Vy={vel_s.y:6.2f} m/s    | Rot={rot_b.z:6.2f} rad")

    print("-" * 70)

    # Step 6: Perform Scene Raycast
    print("\n[Step 6] Performing downward Raycast from Y = 10.0m...")
    hit = RaycastHitFfi()
    has_hit = lib.raycast(
        0.0, 10.0, 0.0,     # origin (x, y, z)
        0.0, -1.0, 0.0,     # direction (downwards)
        20.0,               # max distance
        byref(hit)
    )
    if has_hit:
        print(f"  Raycast HIT! Body ID: {hit.body_id} at Distance: {hit.distance:.2f}m")
        print(f"  Impact Point: {hit.point} | Normal: {hit.normal}")
    else:
        print("  Raycast: No hit detected.")

    # Step 7: Clean teardown
    print("\n[Step 7] Destroying Physics World & Releasing C Memory...")
    lib.destroy_physics_world()
    print("[OK] Sample completed successfully!")
    print("=" * 70)


if __name__ == "__main__":
    main()
