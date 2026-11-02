# 🚀 EngineLib (`physics_engine.dll`) - Sample Integration Suite

Welcome to the **EngineLib SampleTest Suite**! This directory provides clean, complete, standalone code examples demonstrating how to integrate `physics_engine.dll` into any application or game engine.

Samples are provided in **Python (ctypes)**, **C (C99/C11)**, and **Modern C++ (C++17 RAII)**.

---

## 📂 Directory Layout

```
SampleTest/
├── python/
│   ├── sample.py             # Complete Python implementation using ctypes (zero external dependencies)
│   ├── run.bat               # 1-click execution script for Python
│   └── physics_engine.dll    # 64-bit release binary
├── c/
│   ├── physics_engine.h      # Complete C-ABI header declaring all structs and functions
│   ├── sample.c              # Idiomatic C99 simulation sample
│   ├── build.bat             # Compiles with GCC and runs sample_c.exe
│   └── physics_engine.dll    # 64-bit release binary
├── cpp/
│   ├── physics_engine.hpp    # Modern C++17 RAII wrapper class (EngineLib::PhysicsWorld)
│   ├── sample.cpp            # Object-oriented C++17 simulation sample
│   ├── build.bat             # Compiles with G++ and runs sample_cpp.exe
│   └── physics_engine.dll    # 64-bit release binary
├── physics_engine.dll        # Bundled root release dynamic library
├── run_all.bat               # 1-click test suite runner (runs Python, C, and C++ sequentially)
└── README.md                 # This integration guide
```

---

## ⚡ Quick Start

### 1. Run Everything at Once
Double-click `run_all.bat` or run from PowerShell:
```powershell
cd Z:\VSCode\GameLib\SampleTest
.\run_all.bat
```

### 2. Python Integration (`python/sample.py`)
No `pip` packages required—runs directly with standard Python 3.8+:
```powershell
cd Z:\VSCode\GameLib\SampleTest\python
python sample.py
```

### 3. C Integration (`c/sample.c`)
Requires GCC (MinGW-w64):
```powershell
cd Z:\VSCode\GameLib\SampleTest\c
gcc -O2 sample.c -L. -lphysics_engine -o sample_c.exe
.\sample_c.exe
```

### 4. C++ Integration (`cpp/sample.cpp`)
Requires G++ with C++17 support:
```powershell
cd Z:\VSCode\GameLib\SampleTest\cpp
g++ -std=c++17 -O2 sample.cpp -L. -lphysics_engine -o sample_cpp.exe
.\sample_cpp.exe
```

---

## 📐 Units & Coordinate System

| Property | Unit | Notes |
|---|---|---|
| **Coordinates** | Right-Handed 3D | $+X$ = Right, $+Y$ = Up, $+Z$ = Forward |
| **Position** | Meters ($\text{m}$) | $1.0\text{ unit} = 1.0\text{ meter}$ |
| **Mass** | Kilograms ($\text{kg}$) | `mass = 0.0` marks a body as **static / immovable** terrain |
| **Linear Velocity** | Meters / Second ($\text{m/s}$) | Standard SI velocity vector |
| **Angular Velocity** | Radians / Second ($\text{rad/s}$) | Rotational spin around X, Y, Z axes |
| **Force** | Newtons ($\text{N}$) | Applied for single frame ($\Delta v = \frac{F}{m} \Delta t$) |
| **Impulse** | Newton-Seconds ($\text{N}\cdot\text{s}$) | Instantaneous velocity change ($\Delta v = \frac{J}{m}$) |
| **Restitution** | Dimensionless $[0.0, 1.0]$ | $0.0$ = completely inelastic (mud/rock), $1.0$ = elastic bounce |
| **Friction** | Coulomb coefficient $\ge 0.0$ | $0.1$ = ice, $0.5$ = wood, $0.8$ = rubber/stone |

---

## 📖 C-ABI API Reference

### World Management
```c
void init_physics_world(float gx, float gy, float gz);
void destroy_physics_world(void);
void step_physics(float dt);
void set_world_substeps(uint32_t substeps);      // Recommended: 3 - 5
void set_solver_iterations(uint32_t iterations); // Recommended: 6 - 10
```

### Body Creation & Deletion
```c
// Creates a 3D Box (AABB)
uint32_t add_rigid_body(float px, float py, float pz, float sx, float sy, float sz, float mass, float restitution);

// Creates a 3D Sphere with Coulomb friction
uint32_t add_sphere_body(float px, float py, float pz, float radius, float mass, float restitution, float friction);

// Removes a body from the simulation
bool remove_body(uint32_t id);
```

### State Queries & Telemetry
```c
bool get_body_position(uint32_t id, Vector3D* out_pos);
bool set_body_position(uint32_t id, float x, float y, float z);

bool get_body_velocity(uint32_t id, Vector3D* out_vel);
bool set_body_velocity(uint32_t id, float vx, float vy, float vz);

bool get_body_rotation(uint32_t id, Vector3D* out_rot);
bool set_body_rotation(uint32_t id, float rx, float ry, float rz);

bool get_body_angular_velocity(uint32_t id, Vector3D* out_ang_vel);
bool set_body_angular_velocity(uint32_t id, float wx, float wy, float wz);
```

### Dynamic Impulses & Torques
```c
void apply_force(uint32_t id, float fx, float fy, float fz);
void apply_torque(uint32_t id, float tx, float ty, float tz);
void apply_impulse_at_point(uint32_t id, float ix, float iy, float iz, float px, float py, float pz);
```

### Distance Joint Constraints (Ragdolls & Hinges)
```c
uint32_t add_distance_joint(
    uint32_t body_a, uint32_t body_b,
    float off_ax, float off_ay, float off_az,
    float off_bx, float off_by, float off_bz,
    float rest_length, float stiffness, float damping
);
bool remove_joint(uint32_t id);
```

### Scene Queries (Raycasting)
```c
bool raycast(
    float ox, float oy, float oz,
    float dx, float dy, float dz,
    float max_dist,
    RaycastHitFfi* out_hit
);
```
