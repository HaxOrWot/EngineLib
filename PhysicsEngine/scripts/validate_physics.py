#!/usr/bin/env python3
"""
validate_physics.py
Validates the compiled Physics Engine DLL via ctypes.
"""
import os
import sys
import ctypes
from ctypes import c_float, c_uint32, c_bool, Structure, POINTER, byref

def load_physics_engine():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(base_dir, ".."))
    
    if sys.platform.startswith("win"):
        dll_name = "physics_engine.dll"
    elif sys.platform.startswith("darwin"):
        dll_name = "libphysics_engine.dylib"
    else:
        dll_name = "libphysics_engine.so"
        
    search_paths = [
        os.path.join(project_root, "target", "release", dll_name),
        os.path.join(project_root, "target", "debug", dll_name),
        os.path.join(project_root, dll_name),
        os.path.join(base_dir, dll_name),
    ]
    
    for path in search_paths:
        if os.path.isfile(path):
            print(f"[+] Loading DLL from: {path}")
            return ctypes.CDLL(path)
            
    raise FileNotFoundError(f"Could not locate {dll_name}. Searched: {search_paths}")

class Vector3D(Structure):
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
    ]

    def __repr__(self):
        return f"Vector3D(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f})"

def configure_bindings(lib):
    # init_physics_world(gx, gy, gz)
    lib.init_physics_world.argtypes = [c_float, c_float, c_float]
    lib.init_physics_world.restype = None

    # add_rigid_body(px, py, pz, sx, sy, sz, mass, restitution) -> uint32
    lib.add_rigid_body.argtypes = [
        c_float, c_float, c_float, # Position
        c_float, c_float, c_float, # Size
        c_float, c_float           # Mass, Restitution
    ]
    lib.add_rigid_body.restype = c_uint32

    # get_body_position(id, out_pos) -> bool
    lib.get_body_position.argtypes = [c_uint32, POINTER(Vector3D)]
    lib.get_body_position.restype = c_bool

    # apply_force(id, fx, fy, fz)
    lib.apply_force.argtypes = [c_uint32, c_float, c_float, c_float]
    lib.apply_force.restype = None

    # step_physics(dt)
    lib.step_physics.argtypes = [c_float]
    lib.step_physics.restype = None

def run_validation():
    print("=" * 65)
    print("  Universal 3D Physics Engine - Python Validation Test Suite")
    print("=" * 65)
    
    lib = load_physics_engine()
    configure_bindings(lib)
    print("[+] Successfully bound native C-ABI symbols.")

    # 1. Initialize world with Earth gravity (Y = -9.81 m/s^2)
    lib.init_physics_world(0.0, -9.81, 0.0)
    print("[+] Initialized Physics World with gravity = (0.0, -9.81, 0.0)")

    # 2. Add static floor: position=(0, 0, 0), size=(20, 1, 20), mass=0, restitution=0.8
    floor_id = lib.add_rigid_body(
        0.0, 0.0, 0.0,      # Pos
        20.0, 1.0, 20.0,    # Size (Floor surface is at Y = 0.5)
        0.0, 0.8            # Mass 0 = immovable static terrain
    )
    print(f"[+] Created Static Floor (ID: {floor_id}) at Y=0.0")

    # 3. Add falling cube: position=(0, 10, 0), size=(1, 1, 1), mass=1.0, restitution=0.8
    cube_id = lib.add_rigid_body(
        0.0, 10.0, 0.0,     # Pos
        1.0, 1.0, 1.0,      # Size
        1.0, 0.8            # Mass 1.0, Restitution 0.8
    )
    print(f"[+] Created Falling Cube (ID: {cube_id}) at Y=10.0")

    # 4. Simulation loop: 120 steps @ 60 FPS (dt = 0.01667s)
    dt = 1.0 / 60.0
    pos = Vector3D()
    
    print("\n--- Starting Simulation Ticks ---")
    bounced = False
    prev_y = 10.0
    lowest_y = 10.0

    for step in range(1, 121):
        lib.step_physics(c_float(dt))
        lib.get_body_position(cube_id, byref(pos))
        
        sim_time = step * dt
        
        if pos.y < lowest_y:
            lowest_y = pos.y
            
        # Check for bounce: when Y starts increasing after approaching floor
        if not bounced and pos.y > prev_y and pos.y < 3.0:
            bounced = True
            print(f">>> [BOUNCE DETECTED at Step {step:03d} (t={sim_time:.2f}s)] Cube rebounded to Y={pos.y:.3f} m (Lowest Y: {lowest_y:.3f} m) <<<")
        
        # Print progress periodically
        if step % 15 == 0 or step == 1:
            print(f"Step {step:03d} | t={sim_time:.3f}s | Cube Pos: ({pos.x:.2f}, {pos.y:.3f}, {pos.z:.2f})")
            
        prev_y = pos.y

    print("\n" + "=" * 65)
    if bounced:
        print("  RESULT: SUCCESS - Collision detection & elastic impulse verified!")
    else:
        print("  RESULT: FAILED - Object did not rebound.")
    print("=" * 65)
    
    if not bounced:
        sys.exit(1)

if __name__ == "__main__":
    run_validation()
