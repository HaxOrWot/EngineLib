# Numerical Integration: Timestep & Kinematics

This document details the mathematical formulations for time integration, numerical stability comparisons, and accumulator loop strategies.

---

## 1. Classical Kinematic Equations

In a continuous classical Newtonian physical system, linear motion is governed by Newton's Second Law:

$$\mathbf{F} = m \cdot \mathbf{a} \implies \mathbf{a} = \frac{\mathbf{F}}{m} = \mathbf{F} \cdot m^{-1}$$

Where:
- $\mathbf{F} \in \mathbb{R}^3$ is the sum of net external forces applied to the body.
- $m \in \mathbb{R}^+$ is the inertial mass of the body.
- $m^{-1}$ is the precalculated **inverse mass** (`inverse_mass = 1.0 / mass`).
- For static, immovable geometry (such as terrain or boundaries), $m = \infty \implies m^{-1} = 0.0$, naturally neutralizing all acceleration.

The equations of motion are first-order differential systems:

$$\frac{d\mathbf{v}}{dt} = \mathbf{a} + \mathbf{g}$$

$$\frac{d\mathbf{x}}{dt} = \mathbf{v}$$

---

## 2. Integration Schemes Comparison

In real-time computational game engines, continuous differential equations must be approximated at discrete intervals ($\Delta t$).

### A. Explicit (Forward) Euler
Updates position using the **previous** velocity:

$$\mathbf{v}_{n+1} = \mathbf{v}_n + \mathbf{a}_n \Delta t$$

$$\mathbf{x}_{n+1} = \mathbf{x}_n + \mathbf{v}_n \Delta t$$

> [!WARNING]
> Explicit Euler is unconditionally unstable for oscillatory and gravitational systems. It continuously adds artificial kinetic energy into the simulation, causing orbits to spiral outward and resting stacks of boxes to violently explode.

---

### B. Semi-Implicit (Symplectic) Euler *(Implemented Scheme)*
Updates velocity **first**, and then advances position using the **new** velocity:

$$\mathbf{v}_{n+1} = \mathbf{v}_n + (\mathbf{a}_n + \mathbf{g}) \Delta t$$

$$\mathbf{x}_{n+1} = \mathbf{x}_n + \mathbf{v}_{n+1} \Delta t$$

$$\mathbf{a}_{n+1} = \mathbf{0}$$

#### Why Symplectic Euler is Preferred:
1. **Symplectic Structure**: Conserves phase space area, ensuring near-constant total energy over time without explosive energy drift.
2. **Computational Cost**: Zero extra memory allocations or function evaluations (unlike 4th-order Runge-Kutta).
3. **Rigid Body Stability**: Perfectly handles restitution collisions and contact velocity damping.

---

### C. Velocity Verlet Integration (Future Upgrade)
Calculates position with second-order accuracy ($O(\Delta t^2)$):

$$\mathbf{x}_{n+1} = \mathbf{x}_n + \mathbf{v}_n \Delta t + \frac{1}{2}\mathbf{a}_n \Delta t^2$$

$$\mathbf{v}_{n+1} = \mathbf{v}_n + \frac{\mathbf{a}_n + \mathbf{a}_{n+1}}{2}\Delta t$$

---

## 3. Fixed vs. Variable Timestep (The Accumulator Loop)

Passing variable $\Delta t$ directly from the rendering frame rate (e.g. `1/30s` during lag spikes, `1/144s` on high-refresh monitors) leads to non-deterministic physics, clipping bugs, and tunneling.

To ensure absolute determinism across all platforms, client applications should drive `step_physics` via a **Fixed Timestep Accumulator**:

```python
# Recommended Host Runtime Loop (Python Example)
FIXED_DT = 1.0 / 60.0  # 60 Hz physics tick (0.016667s)
accumulator = 0.0
last_time = time.perf_counter()

while game_running:
    current_time = time.perf_counter()
    frame_time = current_time - last_time
    last_time = current_time
    
    # Clamp frame_time to avoid "spiral of death" during major pauses
    if frame_time > 0.25:
        frame_time = 0.25
        
    accumulator += frame_time
    
    # Consume accumulated real time in discrete, identical physical slices
    while accumulator >= FIXED_DT:
        physics_dll.step_physics(c_float(FIXED_DT))
        accumulator -= FIXED_DT
        
    render_scene()
```
