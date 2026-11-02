# Collision Detection: Broadphase & Narrowphase

This document details the collision detection pipeline, mathematical formulation of 3D Axis-Aligned Bounding Box (AABB) intersection tests, penetration depth calculation, and broadphase acceleration.

---

## 1. The Collision Pipeline Overview

Collision detection operates in two distinct stages to optimize computational throughput:

```
               +--------------------------------------+
               |      All Bodies in Simulation        |
               +--------------------------------------+
                                  │
                                  ▼
               +--------------------------------------+
               |     Phase 1: Broadphase Culling      |
               |  (Discards distant, non-colliding    |
               |   pairs via spatial partitioning)    |
               +--------------------------------------+
                                  │ Potential Candidate Pairs
                                  ▼
               +--------------------------------------+
               |    Phase 2: Narrowphase Evaluation   |
               |  (Exact geometric AABB intersection, |
               |   contact normal & penetration depth)|
               +--------------------------------------+
                                  │
                                  ▼
                         Contact Manifold List
```

---

## 2. Mathematical Definition of 3D AABB Overlap

An Axis-Aligned Bounding Box (AABB) is defined by its center position $\mathbf{P} = (p_x, p_y, p_z)$ and extents $\mathbf{S} = (s_x, s_y, s_z)$ representing width, height, and depth.

The half-extents (radii along each coordinate axis) are:

$$\mathbf{H} = \frac{1}{2} \mathbf{S} = \left(\frac{s_x}{2}, \frac{s_y}{2}, \frac{s_z}{2}\right)$$

### Overlap Condition
Two bodies $A$ and $B$ intersect in 3D Euclidean space if and only if their intervals overlap simultaneously across all three Cartesian axes:

$$\text{Overlap}_x = (H_{A,x} + H_{B,x}) - |P_{A,x} - P_{B,x}| > 0$$

$$\text{Overlap}_y = (H_{A,y} + H_{B,y}) - |P_{A,y} - P_{B,y}| > 0$$

$$\text{Overlap}_z = (H_{A,z} + H_{B,z}) - |P_{A,z} - P_{B,z}| > 0$$

If any axis yields an overlap $\le 0.0$, the bodies are separated by that Cartesian axis (the Separating Axis), and no collision has occurred.

```
       +--------------+
       |   Body A     |
       |       +------+-------+
       |       | OVERLAP |    |
       +-------+------+       |
               |    Body B    |
               +--------------+
```

---

## 3. Contact Normal & Minimum Penetration Depth

When all three overlaps are positive ($\text{Overlap}_x > 0 \land \text{Overlap}_y > 0 \land \text{Overlap}_z > 0$), a collision is confirmed.

To resolve the collision with the least energetic distortion, the collision normal $\mathbf{n}$ is chosen along the axis with the **minimum penetration depth** (the shallowest overlap):

$$\text{penetration} = \min(\text{Overlap}_x, \text{Overlap}_y, \text{Overlap}_z)$$

### Normal Derivation
The unit normal vector $\mathbf{n}$ points from Body B to Body A:

- **If Overlap X is minimal**:
  $$\mathbf{n} = \begin{pmatrix} \text{sgn}(P_{A,x} - P_{B,x}) \\ 0 \\ 0 \end{pmatrix}$$
- **If Overlap Y is minimal**:
  $$\mathbf{n} = \begin{pmatrix} 0 \\ \text{sgn}(P_{A,y} - P_{B,y}) \\ 0 \end{pmatrix}$$
- **If Overlap Z is minimal**:
  $$\mathbf{n} = \begin{pmatrix} 0 \\ 0 \\ \text{sgn}(P_{A,z} - P_{B,z}) \end{pmatrix}$$

Where:
$$\text{sgn}(u) = \begin{cases} +1.0 & \text{if } u \ge 0 \\ -1.0 & \text{if } u < 0 \end{cases}$$

---

## 4. Broadphase Strategies & Scaling

The initial implementation evaluates all unique object pairs using an $O(N^2)$ nested loop:

```rust
// Naive Pair Generation: O(N^2)
for i in 0..keys.len() {
    for j in (i + 1)..keys.len() {
        // Evaluate pair (keys[i], keys[j])
    }
}
```

While suitable for $N \le 200$ bodies, $N = 5,000$ requires $12.5$ million intersection tests per frame.

### Production Broadphase Solutions

```
+-------------------------------------------------------------------------+
| Broadphase Algorithm  | Time Complexity | Best Suited For               |
+-----------------------+-----------------+-------------------------------+
| Brute Force O(N^2)    | O(N^2)          | < 200 bodies (Simple, minimal)|
| Spatial Hash Grid     | O(N) average    | Uniformly sized dynamic bodies|
| Sweep and Prune (SAP) | O(N log N)      | Coherent scenes (low velocity)|
| Dynamic BVH (AABB Tree| O(N log N)      | Complex worlds, raycasts      |
+-------------------------------------------------------------------------+
```

### Spatial Hashing Pipeline (Architecture Roadmap)
A spatial hash maps 3D coordinate space into fixed cubic cells of size $D_{\text{cell}}$. Each body registers into every cell its AABB touches:

$$\text{CellIndex}(x, y, z) = \left( \left\lfloor\frac{x}{D}\right\rfloor, \left\lfloor\frac{y}{D}\right\rfloor, \left\lfloor\frac{z}{D}\right\rfloor \right)$$

Bodies sharing identical cell keys are dispatched to narrowphase testing, cutting average comparison complexity from $O(N^2)$ to $O(N)$.
