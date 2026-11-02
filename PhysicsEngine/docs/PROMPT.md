# Master Autonomous Implementation Prompt

> **Instructions for the User**:
> Copy and paste the prompt below directly into any AI coding assistant to instruct it to implement, compile, and validate this entire physics engine autonomously.

---

```markdown
You are an expert Systems and Game Engine Architect specializing in high-performance Rust, physical simulation mechanics, and Foreign Function Interface (FFI) development.

### Objective
Your mission is to construct, compile, and validate the **Universal Cross-Language 3D Kinematic & Rigid Body Physics Engine** inside `Z:\VSCode\GameLib\PhysicsEngine`. The engine must compile into a native dynamic shared library (`.dll` on Windows, `.so` on Linux, `.dylib` on macOS) exposing a universal, thread-safe C-ABI interface for host environments like Python, C#, C++, and Java.

---

### Step 1: Ingest the Architecture & Design Specifications
Before writing any code, thoroughly read and strictly adhere to all architectural specifications located in `Z:\VSCode\GameLib\PhysicsEngine\docs/`:
1. `docs/README.md` - Master topology, execution pipeline, and design tenets.
2. `docs/01_architecture/` - Memory layout (64-byte alignment), FFI safety boundaries, thread synchronization, and edge cases (degenerate normals, resting contact jitter).
3. `docs/02_math_and_physics/` - Symplectic Euler integration, AABB overlap tests, minimum penetration normal derivation, Baumgarte positional stabilization, and Coulomb friction.
4. `docs/03_ffi_and_api/` - Exact exported function signatures, pointer validation, and `std::panic::catch_unwind` wrappers.
5. `docs/05_engine_implementation_plan/complete_code_templates.md` - Direct source code blueprints for every module.

---

### Step 2: Establish the Production Cargo Configuration
In `Z:\VSCode\GameLib\PhysicsEngine/Cargo.toml`, configure the project as follows:
- Package name: `physics_engine`
- Edition: `2021`
- `crate-type = ["cdylib", "rlib"]`
- Zero external runtime dependencies.
- Release profile tuned for maximum throughput:
  - `opt-level = 3`
  - `lto = "fat"`
  - `codegen-units = 1`
  - `panic = "abort"`
  - `strip = true`

---

### Step 3: Implement the Modular Engine Architecture
Organize the source code cleanly into modular files inside `src/`:

1. `src/math/vec3.rs`:
   - `Vector3D` struct decorated with `#[repr(C)]`.
   - Implement vector arithmetic (`Add`, `Sub`, `Mul`, `Div`, `Neg`), dot product, cross product, length, and normalization.

2. `src/core/body.rs`:
   - `RigidBody3D` struct decorated with `#[repr(C)]` (exactly 64 bytes).
   - Fields: `id`, `position`, `velocity`, `acceleration`, `size`, `mass`, `inverse_mass`, `restitution`.
   - Helper methods: `is_static()`, `apply_force()`.

3. `src/core/world.rs`:
   - `PhysicsWorld` container managing a `HashMap<u32, RigidBody3D>`, entity ID generator, and global `gravity: Vector3D`.
   - Methods: `new()`, `add_body()`, `remove_body()`, `step(dt)`.

4. `src/dynamics/integrator.rs`:
   - Symplectic (Semi-Implicit) Euler numerical integration:
     - $v_{n+1} = v_n + (a_n + g) \cdot dt$
     - $x_{n+1} = x_n + v_{n+1} \cdot dt$
     - $a_{n+1} = 0$

5. `src/collision/narrowphase.rs`:
   - 3D AABB overlap evaluation across X, Y, and Z axes.
   - Contact normal derivation along the axis of minimum penetration depth.
   - Return structured `ContactManifold` (`normal`, `penetration`).

6. `src/solver/contact.rs`:
   - Baumgarte positional stabilization: Separates overlapping geometry based on inverse mass with penetration slop (5mm) and correction factor (60%).
   - Relative velocity calculation along the contact normal.
   - Elastic restitution impulse with resting velocity threshold clamping (< 0.30 m/s) to prevent high-frequency surface jitter.

7. `src/ffi/exports.rs` and `src/lib.rs`:
   - Thread-safe global world instance wrapped in `Mutex<Option<PhysicsWorld>>`.
   - Expose the required C-ABI functions using `#[no_mangle] pub extern "C"`:
     - `init_physics_world(gx: f32, gy: f32, gz: f32)`
     - `add_rigid_body(px, py, pz, sx, sy, sz, mass, restitution) -> u32`
     - `get_body_position(id: u32, out_pos: *mut Vector3D) -> bool`
     - `apply_force(id: u32, fx: f32, fy: f32, fz: f32)`
     - `step_physics(dt: f32)`
   - Every FFI function must guard against panics with `std::panic::catch_unwind` and validate null pointers.

---

### Step 4: Build and Compile
Execute the native build:
```powershell
cargo build --release
```
Confirm that `target/release/physics_engine.dll` is emitted without any compiler warnings or errors.

---

### Step 5: Native Verification Tests
Create and execute unit tests in `tests/` covering:
1. `test_gravitational_freefall`: Asserts analytical velocity and position under constant gravity.
2. `test_elastic_restitution`: Asserts correct velocity reflection upon wall collision according to coefficient $e$.
3. `test_floor_penetration_prevention`: Asserts that sustained resting contact does not sink through the floor.

Run:
```powershell
cargo test
```

---

### Step 6: Host Cross-Language Validation (Python ctypes)
Create `scripts/validate_physics.py` implementing the test harness documented in `docs/04_language_bindings/python_ctypes.md`:
- Load the compiled `target/release/physics_engine.dll` via `ctypes`.
- Initialize world with Earth gravity (`0.0, -9.81, 0.0`).
- Add a static floor (`mass = 0.0`) at $Y = 0.0$.
- Add a falling box (`mass = 1.0`, `restitution = 0.8`) at $Y = 10.0$.
- Run 120 simulation steps at $dt = 1/60s$.
- Print frame coordinates and programmatically verify that the cube bounces cleanly off the floor.

---

### Acceptance Criteria
- [ ] Codebase compiles cleanly with zero errors and zero warnings.
- [ ] All native `cargo test` suites pass.
- [ ] `validate_physics.py` executes successfully, detecting and confirming a clean physical rebound.
```
