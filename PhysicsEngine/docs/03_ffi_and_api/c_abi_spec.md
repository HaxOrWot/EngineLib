# C-ABI Specification & Function Signatures

This document defines the complete binary interface, exported symbol names, calling conventions, parameter types, and memory contracts for the dynamic library.

---

## 1. Calling Convention & Export Conventions

- **Calling Convention**: Standard `cdecl` (`extern "C"`) on x86, standard platform ABI on x86-64 (System V on Linux/macOS, Microsoft x64 on Windows).
- **Symbol Mangling**: Disabled via `#[no_mangle]`. Function names appear verbatim in the exported DLL export table.
- **Data Types**: Restricted strictly to C-compatible scalar primitives (`f32`, `u32`, `bool`, and `#[repr(C)]` structs / pointers).

---

## 2. Shared Data Structures

### `Vector3D`
```c
typedef struct {
    float x;
    float y;
    float z;
} Vector3D;
```
- **Size**: 12 bytes
- **Alignment**: 4 bytes

### `RigidBody3D`
```c
typedef struct {
    uint32_t id;
    Vector3D position;
    Vector3D velocity;
    Vector3D acceleration;
    Vector3D size;
    float mass;
    float inverse_mass;
    float restitution;
} RigidBody3D;
```
- **Size**: 64 bytes
- **Alignment**: 4 bytes

---

## 3. Core Exported Function Signatures

### A. `init_physics_world`
Initializes or resets the global physics world with the specified constant gravity vector.

```c
void init_physics_world(float gx, float gy, float gz);
```
- **Parameters**:
  - `gx`: Gravity acceleration along the X-axis ($m/s^2$).
  - `gy`: Gravity acceleration along the Y-axis ($m/s^2$, typically `-9.81` for earth gravity).
  - `gz`: Gravity acceleration along the Z-axis ($m/s^2$).
- **Behavior**: Drops any existing world, allocates an empty body registry, resets the entity ID counter to 1, and stores the gravity vector.

---

### B. `add_rigid_body`
Allocates and adds a new rigid body into the simulation world.

```c
uint32_t add_rigid_body(
    float px, float py, float pz,
    float sx, float sy, float sz,
    float mass, float restitution
);
```
- **Parameters**:
  - `px, py, pz`: Initial 3D world position coordinates.
  - `sx, sy, sz`: Extents (width, height, depth). Must be positive non-zero values.
  - `mass`: Inertial mass in kilograms. If `mass <= 0.0`, the body is treated as static/immovable (`inverse_mass = 0.0`).
  - `restitution`: Elasticity coefficient between `0.0` (inelastic) and `1.0` (fully elastic).
- **Returns**: A unique non-zero 32-bit entity ID (`uint32_t`). Returns `0` if initialization fails.

---

### C. `get_body_position`
Reads the current 3D position coordinates of a specified body into a host-provided memory pointer.

```c
bool get_body_position(uint32_t id, Vector3D* out_pos);
```
- **Parameters**:
  - `id`: Unique body ID returned by `add_rigid_body`.
  - `out_pos`: Non-null pointer to a caller-allocated `Vector3D` memory buffer.
- **Returns**:
  - `true`: Body found and coordinates written into `*out_pos`.
  - `false`: Body ID does not exist, or `out_pos` is a null pointer.

---

### D. `apply_force`
Applies an instantaneous force vector to a dynamic body for the current simulation frame.

```c
void apply_force(uint32_t id, float fx, float fy, float fz);
```
- **Parameters**:
  - `id`: Unique body ID.
  - `fx, fy, fz`: Force vector components in Newtons ($N = kg \cdot m / s^2$).
- **Behavior**: Accumulates linear acceleration into the body: $\Delta \mathbf{a} = \mathbf{F} \cdot m^{-1}$. If `inverse_mass == 0.0`, the call is a no-op.

---

### E. `step_physics`
Advances the physical world simulation by a discrete timestep $\Delta t$.

```c
void step_physics(float dt);
```
- **Parameters**:
  - `dt`: Delta time in seconds (e.g. `0.016667f` for 60 Hz).
- **Pipeline Execution**:
  1. Integrates linear acceleration into velocity and velocity into position.
  2. Resets accumulated per-frame forces to zero.
  3. Detects pairwise AABB overlaps.
  4. Resolves penetration via Baumgarte positional stabilization.
  5. Computes and applies Newtonian elastic bounce impulses along the contact normal.

---

## 4. Extended Utility Signatures (Recommended Additions)

To support full lifecycle management, the following extended signatures are planned for the production library:

```c
// Remove an individual body from simulation
bool remove_rigid_body(uint32_t id);

// Set or reset body velocity directly
bool set_body_velocity(uint32_t id, float vx, float vy, float vz);

// Query full rigid body state (position, velocity, acceleration)
bool get_body_state(uint32_t id, RigidBody3D* out_body);

// Cleanly tear down the world and release all native heap allocations
void destroy_physics_world(void);
```
