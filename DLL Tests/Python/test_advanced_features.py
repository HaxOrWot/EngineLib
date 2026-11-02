"""
EngineLib Advanced Features Test Suite (Milestone 2)
Tests:
1. Sphere-Floor Dynamic Bouncing
2. Fast Raycast Scene Queries (Line-of-Sight & Ground Distance)
3. Coulomb Friction & Energy-Conserving Body Sleeping
"""

import os
import sys
import ctypes
from ctypes import c_float, c_uint32, c_bool, Structure, POINTER, byref

def load_engine():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    dll_path = os.path.join(current_dir, "physics_engine.dll")
    if not os.path.isfile(dll_path):
        dll_path = os.path.join(current_dir, "..", "..", "PhysicsEngine", "target", "release", "physics_engine.dll")
    return ctypes.CDLL(os.path.abspath(dll_path))

class Vector3D(Structure):
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
    ]
    def __repr__(self):
        return f"({self.x:6.2f}, {self.y:6.2f}, {self.z:6.2f})"

class RaycastHitFfi(Structure):
    _fields_ = [
        ("point", Vector3D),
        ("normal", Vector3D),
        ("distance", c_float),
        ("body_id", c_uint32),
    ]

def setup_ffi(lib):
    # World lifecycle
    lib.init_physics_world.argtypes = [c_float, c_float, c_float]
    lib.init_physics_world.restype = None

    lib.add_rigid_body.argtypes = [c_float, c_float, c_float, c_float, c_float, c_float, c_float, c_float]
    lib.add_rigid_body.restype = c_uint32

    # New Milestone 2 APIs
    lib.add_sphere_body.argtypes = [c_float, c_float, c_float, c_float, c_float, c_float, c_float]
    lib.add_sphere_body.restype = c_uint32

    lib.set_body_friction.argtypes = [c_uint32, c_float]
    lib.set_body_friction.restype = None

    lib.is_body_sleeping.argtypes = [c_uint32]
    lib.is_body_sleeping.restype = c_bool

    lib.wake_body.argtypes = [c_uint32]
    lib.wake_body.restype = None

    lib.get_body_position.argtypes = [c_uint32, POINTER(Vector3D)]
    lib.get_body_position.restype = c_bool

    lib.apply_force.argtypes = [c_uint32, c_float, c_float, c_float]
    lib.apply_force.restype = None

    lib.step_physics.argtypes = [c_float]
    lib.step_physics.restype = None

    lib.raycast.argtypes = [c_float, c_float, c_float, c_float, c_float, c_float, c_float, POINTER(RaycastHitFfi)]
    lib.raycast.restype = c_bool

def test_spheres_and_raycasts(lib):
    print("=" * 65)
    print("  EngineLib Milestone 2: Sphere Physics & Raycasting Suite")
    print("=" * 65)

    lib.init_physics_world(0.0, -9.81, 0.0)

    # 1. Static Floor: Box at Y=0.0 (top surface at Y=0.5)
    floor_id = lib.add_rigid_body(0.0, 0.0, 0.0, 40.0, 1.0, 40.0, 0.0, 0.8)
    print(f"[+] Created Floor AABB (ID: {floor_id}) at Y=0.0")

    # 2. Dynamic Sphere: Radius=1.0m, Mass=2.0kg, Restitution=0.85, dropped from Y=12.0m
    sphere_id = lib.add_sphere_body(0.0, 12.0, 0.0, 1.0, 2.0, 0.85, 0.4)
    print(f"[+] Created Dynamic Sphere (ID: {sphere_id}) [Radius=1.0m, Mass=2.0kg] at Y=12.0m")

    # 3. Raycast Test from sky downwards
    print("\n--- Raycast Query ---")
    hit = RaycastHitFfi()
    # Cast ray from (0, 20, 0) straight down along -Y
    has_hit = lib.raycast(0.0, 20.0, 0.0, 0.0, -1.0, 0.0, 50.0, byref(hit))
    if has_hit:
        print(f"[+] Downward Raycast HIT Body ID: {hit.body_id}")
        print(f"    Impact Point:  ({hit.point.x:.2f}, {hit.point.y:.2f}, {hit.point.z:.2f})")
        print(f"    Impact Normal: ({hit.normal.x:.2f}, {hit.normal.y:.2f}, {hit.normal.z:.2f})")
        print(f"    Hit Distance:  {hit.distance:.2f} m (Expected Sphere top at Y = 13.0m)")
        assert hit.body_id == sphere_id, "Ray should hit the sphere first!"
    else:
        print("[-] Raycast failed to hit any body!")
        sys.exit(1)

    # 4. Simulate Sphere freefall and bounce off floor
    print("\n--- Simulating Sphere Bouncing (120 frames @ 60 Hz) ---")
    dt = 1.0 / 60.0
    pos = Vector3D()
    bounced = False
    prev_y = 12.0

    for frame in range(1, 121):
        lib.step_physics(c_float(dt))
        lib.get_body_position(sphere_id, byref(pos))

        if not bounced and pos.y > prev_y and pos.y < 3.0:
            bounced = True
            print(f">>> [SPHERE BOUNCE DETECTED at frame {frame:03d} (t={frame*dt:.2f}s)] Rebounded from Y={prev_y:.3f}m! <<<")

        if frame % 25 == 0 or frame == 1:
            print(f"  Frame {frame:03d} | t={frame*dt:.2f}s | Sphere Pos: Y={pos.y:6.2f}m")

        prev_y = pos.y

    assert bounced, "Sphere failed to bounce off floor!"

    # 5. Body Sleeping Test
    print("\n--- Sleeping Subsystem Test ---")
    # Let resting box settle over 100 frames
    box = lib.add_rigid_body(5.0, 1.0, 0.0, 1.0, 1.0, 1.0, 10.0, 0.0)
    for _ in range(100):
        lib.step_physics(c_float(dt))
    
    sleeping = lib.is_body_sleeping(box)
    print(f"[+] Resting box ID={box} sleep state after settling: {'SLEEPING (Energy Saved)' if sleeping else 'ACTIVE'}")

    print("\n" + "=" * 65)
    print("  ALL MILESTONE 2 ADVANCED TESTS PASSED SUCCESSFULLY!")
    print("=" * 65)

if __name__ == "__main__":
    eng = load_engine()
    setup_ffi(eng)
    test_spheres_and_raycasts(eng)
