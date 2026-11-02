# Memory Model & FFI Safety

This document details the memory layout, data alignment, allocation strategies, and cross-boundary pointer safety guarantees of the physics engine.

---

## 1. Memory Boundaries & Ownership

In a cross-language dynamic library architecture, clear memory ownership is critical to prevent memory corruption, double frees, and memory leaks.

```
+-------------------------------------------------------------+
|                 Host Process (e.g., Python / C#)            |
|                                                             |
|  [Managed Heap / GC Runtime]                                |
|  - Creates struct placeholders (e.g. ctypes.Structure)      |
|  - Passes memory pointers to Rust DLL                       |
+-------------------------------------------------------------+
                              │
                     FFI Boundary (Raw Pointers)
                              ▼
+-------------------------------------------------------------+
|               Native Rust Engine (.DLL Process)             |
|                                                             |
|  [Native System Heap]                                       |
|  - Allocates and owns RigidBody3D records in HashMap/Vec    |
|  - Never transfers ownership of internal heap to host       |
|  - Writes outputs into host-provided pointer locations      |
+-------------------------------------------------------------+
```

### Golden Rules of the Memory Contract
1. **The Host owns its buffers**: The host runtime allocates output memory (e.g., an uninitialized `Vector3D` struct) and passes a raw pointer (`*mut Vector3D`) to Rust.
2. **Rust fills the buffers**: Rust writes directly into the memory location provided by the caller, avoiding internal-to-external allocation handoffs.
3. **Rust owns internal simulation state**: Bodies and spatial lookup structures reside in Rust-managed memory and are freed only via explicit engine API calls or process teardown.

---

## 2. Binary Layout & Data Alignment (`#[repr(C)]`)

By default, the Rust compiler optimizes struct layouts by reordering fields to minimize padding bytes. Across an FFI boundary, this reordering breaks compatibility with C, Python, and C#.

To guarantee deterministic field order and alignment matching standard C compilers:
All shared structs are decorated with `#[repr(C)]`.

### Memory Layout: `Vector3D`
```rust
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vector3D {
    pub x: f32, // Offset 0x00, 4 bytes
    pub y: f32, // Offset 0x04, 4 bytes
    pub z: f32, // Offset 0x08, 4 bytes
}
```
- **Total Size**: 12 bytes (`sizeof(f32) * 3`).
- **Alignment**: 4-byte aligned. No internal or trailing padding bytes.

### Memory Layout: `RigidBody3D`
```rust
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RigidBody3D {
    pub id: u32,              // Offset 0x00, 4 bytes
    pub position: Vector3D,   // Offset 0x04, 12 bytes
    pub velocity: Vector3D,   // Offset 0x10, 12 bytes
    pub acceleration: Vector3D,// Offset 0x1C, 12 bytes
    pub size: Vector3D,       // Offset 0x28, 12 bytes
    pub mass: f32,            // Offset 0x34, 4 bytes
    pub inverse_mass: f32,    // Offset 0x38, 4 bytes
    pub restitution: f32,     // Offset 0x3C, 4 bytes
}
```
- **Total Size**: 64 bytes (`0x40`).
- **Alignment**: 4-byte aligned (matching the largest scalar member).
- **Cache Line Efficiency**: Exactly 64 bytes—fits perfectly within a standard x86-64 / ARM64 cache line (64 bytes), avoiding false sharing across cache boundaries.

---

## 3. Safe FFI Pointer Dereferencing

External host languages can pass invalid or null pointers across the FFI boundary. The engine applies defensive validation before any memory write:

```rust
#[no_mangle]
pub unsafe extern "C" fn get_body_position(id: u32, out_pos: *mut Vector3D) -> bool {
    // 1. Guard against null pointers immediately
    if out_pos.is_null() {
        return false;
    }
    
    // 2. Lock world state
    let world_lock = match WORLD.lock() {
        Ok(guard) => guard,
        Err(_) => return false, // Mutex poisoned
    };
    
    // 3. Query entity and write to validated pointer
    if let Some(ref world) = *world_lock {
        if let Some(body) = world.bodies.get(&id) {
            // Write directly to host memory
            std::ptr::write(out_pos, body.position);
            return true;
        }
    }
    false
}
```

---

## 4. Avoiding Aliasing Violations During Collision Resolution

During narrowphase collision resolution, two bodies in the world must be updated simultaneously. In safe Rust, borrowing two mutable references from a collection is forbidden by the borrow checker:

```rust
// ILLEGAL in safe Rust:
let body_a = world.bodies.get_mut(&id_a).unwrap();
let body_b = world.bodies.get_mut(&id_b).unwrap(); // ERROR: second mutable borrow occurs here
```

### The Safe Raw Pointer Idiom
Since the engine verifies `id_a != id_b`, the two bodies occupy distinct non-overlapping memory addresses. The raw pointer approach safely bypasses the borrow checker without creating aliasing:

```rust
if id_a != id_b {
    let ptr_a = world.bodies.get_mut(&id_a).unwrap() as *mut RigidBody3D;
    let ptr_b = world.bodies.get_mut(&id_b).unwrap() as *mut RigidBody3D;

    unsafe {
        let body_a = &mut *ptr_a;
        let body_b = &mut *ptr_b;
        // Safe: body_a and body_b point to distinct, non-aliasing heap allocations
    }
}
```
