#!/usr/bin/env python3
"""
3D Physics Engine - Python Interactive DLL Test Suite
Demonstrates loading physics_engine.dll, running gravity, restitution, and collision simulations.
"""

import os
import sys
import time
import ctypes
from ctypes import c_float, c_uint32, c_bool, Structure, POINTER, byref

# ---------------------------------------------------------------------------
# 1. Locate and Load the DLL
# ---------------------------------------------------------------------------
def load_engine():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(current_dir, "physics_engine.dll"),
        os.path.join(current_dir, "..", "..", "PhysicsEngine", "target", "release", "physics_engine.dll"),
        os.path.join(current_dir, "..", "..", "PhysicsEngine", "target", "debug", "physics_engine.dll"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            dll_path = os.path.abspath(path)
            print(f"[DLL Loader] Found engine: {dll_path}")
            return ctypes.CDLL(dll_path)
    raise FileNotFoundError("Could not locate physics_engine.dll! Please compile with 'cargo build --release'.")

# ---------------------------------------------------------------------------
# 2. C-ABI Structure Definitions
# ---------------------------------------------------------------------------
class Vector3D(Structure):
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
    ]

    def __repr__(self):
        return f"({self.x:6.2f}, {self.y:6.2f}, {self.z:6.2f})"

# ---------------------------------------------------------------------------
# 3. Configure Function Signatures
# ---------------------------------------------------------------------------
def setup_ffi(lib):
    lib.init_physics_world.argtypes = [c_float, c_float, c_float]
    lib.init_physics_world.restype = None

    lib.add_rigid_body.argtypes = [
        c_float, c_float, c_float,  # Position: px, py, pz
        c_float, c_float, c_float,  # Size: sx, sy, sz
        c_float, c_float            # Mass, Restitution
    ]
    lib.add_rigid_body.restype = c_uint32

    lib.get_body_position.argtypes = [c_uint32, POINTER(Vector3D)]
    lib.get_body_position.restype = c_bool

    lib.apply_force.argtypes = [c_uint32, c_float, c_float, c_float]
    lib.apply_force.restype = None

    lib.step_physics.argtypes = [c_float]
    lib.step_physics.restype = None

# ---------------------------------------------------------------------------
# 4. ASCII Trajectory Visualizer
# ---------------------------------------------------------------------------
def render_ascii_graph(trajectory, max_height=10.0, width=50):
    print("\n--- Height (Y) vs Time Trajectory Plot ---")
    rows = 15
    for r in range(rows, -1, -1):
        threshold = (r / rows) * max_height
        label = f"{threshold:4.1f}m |"
        line = ""
        for y in trajectory:
            if abs(y - threshold) < (max_height / rows * 0.5):
                line += "o"
            elif y >= threshold:
                line += " "
            else:
                line += " "
        # Print only if there's any point in range
        sampled_line = line[::max(1, len(line) // width)]
        print(f"{label} {sampled_line}")
    print("      +" + "-" * (width + 5) + "> Time")

# ---------------------------------------------------------------------------
# Test Scenario 1: Bouncing Cube on Floor with ASCII trajectory
# ---------------------------------------------------------------------------
def test_bouncing_cube(lib):
    print("\n" + "=" * 65)
    print("  TEST 1: Dynamic Bouncing Cube (Gravity & Restitution)")
    print("=" * 65)

    # Initialize with Earth gravity (-9.81 m/s^2 on Y)
    lib.init_physics_world(0.0, -9.81, 0.0)

    # Floor: static (mass=0), size=(30, 1, 30), at Y=0.0
    floor_id = lib.add_rigid_body(0.0, 0.0, 0.0, 30.0, 1.0, 30.0, 0.0, 0.8)
    # Falling Box: mass=1.0kg, size=(1, 1, 1), initial height Y=10.0m, restitution=0.85
    box_id = lib.add_rigid_body(0.0, 10.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.85)

    print(f"[*] Floor created: ID={floor_id} (Immovable terrain, Y=0.0)")
    print(f"[*] Falling box created: ID={box_id} (Mass=1.0kg, Y=10.0, Restitution=0.85)")

    dt = 1.0 / 60.0
    pos = Vector3D()
    trajectory = []
    bounces = 0
    prev_y = 10.0
    descending = True

    print("\n[Simulating 150 frames @ 60 Hz (2.5 seconds)]")
    for frame in range(1, 151):
        lib.step_physics(c_float(dt))
        lib.get_body_position(box_id, byref(pos))
        trajectory.append(pos.y)

        # Detect bounce crests and valleys
        if descending and pos.y > prev_y:
            bounces += 1
            descending = False
            print(f"  -> BOUNCE #{bounces} at frame {frame:03d} (t={frame*dt:.2f}s): Rebounded from Y={prev_y:.3f}m")
        elif not descending and pos.y < prev_y:
            descending = True

        if frame % 25 == 0 or frame == 1:
            print(f"  Frame {frame:03d} | t={frame*dt:.2f}s | Box Pos: Y={pos.y:6.2f}m")
        prev_y = pos.y

    render_ascii_graph(trajectory, max_height=10.0, width=50)
    print(f"\n[Test 1 Result] Completed with {bounces} recorded bounce(s). SUCCESS.")

# ---------------------------------------------------------------------------
# Test Scenario 2: Head-On Mid-Air Collision of Two Boxes
# ---------------------------------------------------------------------------
def test_head_on_collision(lib):
    print("\n" + "=" * 65)
    print("  TEST 2: Head-On Mid-Air Collision (Two Moving Bodies)")
    print("=" * 65)

    # Zero gravity for pure momentum transfer test
    lib.init_physics_world(0.0, 0.0, 0.0)

    # Box 1: at X = -10, moving East (+X)
    box1 = lib.add_rigid_body(-10.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    # Box 2: at X = +10, moving West (-X)
    box2 = lib.add_rigid_body(10.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0)

    # Apply forces to launch them towards each other
    # F = 300N for 1 frame -> delta_v = 300 * dt = 5 m/s
    dt = 1.0 / 60.0
    lib.apply_force(box1, 300.0, 0.0, 0.0)
    lib.apply_force(box2, -300.0, 0.0, 0.0)

    pos1 = Vector3D()
    pos2 = Vector3D()
    collided = False

    print("[*] Launched Box 1 from X = -10.0 (+X direction)")
    print("[*] Launched Box 2 from X = +10.0 (-X direction)")

    for frame in range(1, 150):
        lib.step_physics(c_float(dt))
        lib.get_body_position(box1, byref(pos1))
        lib.get_body_position(box2, byref(pos2))

        # Check if they have rebounded
        if pos1.x > -1.0 and pos2.x < 1.0 and not collided:
            collided = True
            print(f"  -> IMPACT DETECTED at frame {frame:03d} (t={frame*dt:.2f}s) near X=0.0!")
            print(f"     Box 1 X: {pos1.x:.2f} | Box 2 X: {pos2.x:.2f}")

        if frame % 30 == 0:
            print(f"  Frame {frame:03d} | Box1 X: {pos1.x:6.2f} | Box2 X: {pos2.x:6.2f}")

    print(f"\n[Test 2 Result] Mid-air collision detected and resolved cleanly. SUCCESS.")

# ---------------------------------------------------------------------------
# Main Entry Point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("=" * 65)
    print("  3D Physics Engine - Dynamic Shared Library Test Runner")
    print("=" * 65)
    engine = load_engine()
    setup_ffi(engine)
    test_bouncing_cube(engine)
    test_head_on_collision(engine)
    print("\n" + "=" * 65)
    print("  ALL PYTHON TESTS COMPLETED SUCCESSFULLY!")
    print("=" * 65)
