# Edge Cases, Numerical Stability & Robustness Guide

This document catalogs critical mathematical hazards, edge cases, numerical instabilities, and practical mitigations required for an industrial-strength physics simulation.

---

## 1. Zero-Overlap & Degenerate Normal Resolution

### The Identical Position Hazard
When two bodies are spawned at the exact same spatial coordinates ($P_{A} = P_{B}$):
- $\Delta x = 0.0, \Delta y = 0.0, \Delta z = 0.0$.
- A naive `sign` check (`if body_a.position.x > body_b.position.x { 1.0 } else { -1.0 }`) will default to $-1.0$ even though neither body is to the left or right.
- Worse, if $P_A == P_B$ and size is identical, overlapping penetration is maximal along all three axes, producing an indeterminate normal.

### Mitigation: Epsilon Perturbation & Default Normal
```rust
const EPSILON: f32 = 1e-6;

let delta_x = body_a.position.x - body_b.position.x;
let sign_x = if delta_x.abs() > EPSILON {
    delta_x.signum()
} else {
    1.0 // Arbitrary deterministic push along positive X
};
```
If all three deltas are below `EPSILON`, the engine assigns a default upward ejection vector along the positive Y axis: $\mathbf{n} = (0.0, 1.0, 0.0)$.

---

## 2. Restitution Micro-Jitter (Infinite Bouncing Trap)

### The Problem
When a body rests on a floor under constant gravity, gravity accelerates it downward by $g \cdot \Delta t$ every frame. The contact solver resolves this with an elastic rebound impulse proportional to $(1 + e)$.
If $e > 0$, the body endlessly bounces by fractions of a millimeter every frame, producing visible high-frequency surface vibration and numerical jitter.

### Mitigation: Resting Velocity Threshold ($v_{\text{threshold}}$)
If the relative velocity along the contact normal is below a designated velocity cutoff threshold (typically between $0.25\text{ m/s}$ and $0.5\text{ m/s}$), the coefficient of restitution is forced to **zero**:

```rust
const RESTING_VELOCITY_THRESHOLD: f32 = 0.35; // m/s

let e = if vel_along_normal.abs() < RESTING_VELOCITY_THRESHOLD {
    0.0 // Suppress bounce for resting contact
} else {
    body_a.restitution.min(body_b.restitution)
};
```

---

## 3. Baumgarte Over-Correction Oscillation & Penetration Slop

### The Problem
Correcting 100% of the penetration depth ($penetration \times 1.0$) every frame can induce elastic instability in stacked bodies:
1. Body A penetrates Body B by $1\text{ mm}$.
2. The solver pushes Body A upward by $1\text{ mm}$.
3. In the next frame, Body A is propelled slightly above Body B, then falls again.
4. The bodies begin to oscillate violently.

### Mitigation: Linear Slop and Percentage Damping
1. **Penetration Slop**: Ignore microscopic penetrations (e.g., $< 0.005\text{ m} = 5\text{ mm}$) to avoid resolving sub-millimeter floating point noise.
2. **Baumgarte Factor ($\beta$)**: Only resolve $20\%$ to $80\%$ of the allowed penetration in a single frame.

```rust
const PENETRATION_SLOP: f32 = 0.005; // 5 mm allowance
const BAUMGARTE_FACTOR: f32 = 0.40;   // Resolve 40% of penetration per tick

let effective_penetration = (penetration - PENETRATION_SLOP).max(0.0);
let correction_scalar = (effective_penetration / total_inv_mass) * BAUMGARTE_FACTOR;
```

---

## 4. Division by Zero Protection: Static Bodies

If two immovable static bodies collide ($m_A = \infty, m_B = \infty \implies m_A^{-1} = 0.0, m_B^{-1} = 0.0$):
$$\text{Total Inverse Mass} = 0.0 + 0.0 = 0.0$$
Any division by $\text{Total Inverse Mass}$ generates a float `NaN` or `+inf`, instantly destroying the simulation state.

### Mitigation: Static Pair Early-Out
```rust
let total_inv_mass = body_a.inverse_mass + body_b.inverse_mass;

// Both bodies are static/immovable: do not resolve collision
if total_inv_mass <= 0.0 {
    continue;
}
```

---

## 5. Tunneling Hazard (Bullet-Through-Paper Effect)

### The Problem
Under discrete integration, a body moves by $\Delta \mathbf{x} = \mathbf{v} \cdot \Delta t$ instantaneously between frames.
If a fast-moving projectile travels faster than its bounding box size within a single timestep ($\|\mathbf{v}\| \cdot \Delta t > \text{size}$), it will completely jump across an obstacle without triggering an AABB intersection test.

```
Frame N:          [ Box A ]                 | Wall |
Frame N+1:                                  | Wall |  [ Box A ]  (Tunneled!)
```

### Mitigations:
1. **Sub-stepping**: When velocities exceed a threshold, split `step_physics(dt)` into $M$ smaller sub-steps (e.g. 4 sub-steps of $\Delta t / 4$).
2. **Velocity Clamping**: Clamp maximum linear speed to $\|\mathbf{v}\|_{\max} \le \frac{\min(\text{size})}{2 \cdot \Delta t}$.
3. **Continuous Collision Detection (CCD)**: Raycast the swept volume of the AABB along its velocity vector during broadphase.

---

## 6. Mutex Poisoning Recovery

In standard Rust, if a thread holding a `Mutex` panics, the mutex becomes "poisoned" to protect against corrupted invariants. A subsequent naive `.lock().unwrap()` will panic in all caller threads.

### Safe Mutex Unwrapping:
```rust
let mut world_lock = match WORLD.lock() {
    Ok(guard) => guard,
    Err(poisoned) => {
        eprintln!("[PHYSICS WARNING] Global world mutex was poisoned. Recovering state guard.");
        poisoned.into_inner() // Safely retrieve the inner guard anyway
    }
};
```
