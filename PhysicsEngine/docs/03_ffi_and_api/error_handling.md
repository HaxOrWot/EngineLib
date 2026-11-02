# FFI Error Handling & Panic Boundary Safety

This document describes defensive programming practices across the Foreign Function Interface (FFI), panic containment, and error reporting.

---

## 1. The Peril of Rust Panics Crossing the FFI Boundary

In Rust, a `panic!` initiates stack unwinding. 

> [!CAUTION]
> **Stack unwinding across an `extern "C"` boundary is Undefined Behavior (UB)**.
> If a panic unwinds past an `extern "C"` function, the program will immediately abort or produce catastrophic memory corruption in the host process (Python, C#, etc.).

To ensure 100% host stability, **all exported FFI functions must wrap their logic in `std::panic::catch_unwind`**.

---

## 2. Implementing the Panic Boundary Wrapper

```rust
use std::panic::{catch_unwind, AssertUnwindSafe};

#[no_mangle]
pub extern "C" fn step_physics(dt: f32) {
    // Intercept any potential panics before they hit the C boundary
    let result = catch_unwind(AssertUnwindSafe(|| {
        if dt.is_nan() || dt <= 0.0 {
            return;
        }
        internal_step_physics(dt);
    }));

    if result.is_err() {
        eprintln!("[PHYSICS_ENGINE_FATAL] Caught panic inside step_physics. Unwinding halted.");
    }
}
```

---

## 3. Explicit Error Code Enumeration (`PhysicsResult`)

Rather than returning unstructured booleans or relying on exceptions, advanced FFI functions return a standardized 32-bit status code:

```rust
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhysicsResult {
    Success = 0,
    WorldNotInitialized = 1,
    EntityNotFound = 2,
    NullPointerPassed = 3,
    InvalidParameter = 4, // e.g. NaN, negative mass, zero size
    LockPoisoned = 5,
    InternalPanic = 6,
}
```

### Usage Pattern in Host Bindings (C# Example)
```csharp
PhysicsResult res = NativeMethods.get_body_position(id, out Vector3D pos);
if (res != PhysicsResult.Success) {
    Debug.LogError($"Physics query failed with status: {res}");
}
```

---

## 4. Input Sanitization & Defensive Checks

Before any computation begins, the engine sanitizes all incoming floating-point inputs against:
1. **NaN (Not a Number)**: Propagates through arithmetic and poisons spatial coordinates.
2. **Infinite Values ($\pm \infty$)**: Corrupts bounding box calculations and impulse scalars.
3. **Negative Dimensions**: Size coordinates must be strictly greater than zero ($\min(sx, sy, sz) > 0.0001$).
4. **Negative Restitution**: Restitution must be clamped between $0.0$ and $1.0$.

```rust
fn validate_dimensions(sx: f32, sy: f32, sz: f32) -> bool {
    sx.is_finite() && sy.is_finite() && sz.is_finite() &&
    sx > 0.0001 && sy > 0.0001 && sz > 0.0001
}
```
