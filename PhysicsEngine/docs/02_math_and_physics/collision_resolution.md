# Collision Resolution: Impulse Dynamics & Positional Correction

This document presents the mathematical derivations for Newtonian linear impulse resolution, restitution elasticity, and Baumgarte positional stabilization.

---

## 1. Physical Impulse Formulation

When two rigid bodies collide, an instantaneous force acts over an infinitesimal duration $\Delta t$, generating an **impulse** $\mathbf{J}$:

$$\mathbf{J} = \int \mathbf{F} \, dt = \Delta \mathbf{p} = m \Delta \mathbf{v}$$

For bodies $A$ and $B$, Newton's Third Law demands equal and opposite reaction:

$$\Delta \mathbf{v}_A = \mathbf{J} \cdot m_A^{-1}$$

$$\Delta \mathbf{v}_B = -\mathbf{J} \cdot m_B^{-1}$$

Since friction is omitted in this basic AABB model, the impulse acts purely parallel to the contact normal $\mathbf{n}$:

$$\mathbf{J} = j \cdot \mathbf{n}$$

Where $j \in \mathbb{R}$ is the scalar impulse magnitude.

---

## 2. Relative Velocity & Restitution Derivation

Let the relative velocity between Body A and Body B before collision be:

$$\mathbf{v}_{\text{rel}} = \mathbf{v}_A - \mathbf{v}_B$$

The approach velocity along the collision normal is:

$$v_n = \mathbf{v}_{\text{rel}} \cdot \mathbf{n}$$

### Separation Check
If $v_n \ge 0.0$, the bodies are already moving away from each other or stationary. No impulse should be applied:

$$\text{if } v_n \ge 0.0 \implies j = 0$$

### Coefficient of Restitution ($e$)
The coefficient of restitution defines elasticity:
- $e = 0.0$: Inelastic collision (objects stick together, kinetic energy dissipated).
- $e = 1.0$: Perfectly elastic collision (kinetic energy conserved).

When two bodies with restitution $e_A$ and $e_B$ collide, their combined elasticity is:

$$e = \min(e_A, e_B)$$

### Solving for the Impulse Scalar $j$
By Newton's Law of Restitution, the post-collision relative normal velocity $v'_n$ satisfies:

$$v'_n = -e \cdot v_n$$

Substituting post-collision velocities:

$$(\mathbf{v}'_A - \mathbf{v}'_B) \cdot \mathbf{n} = -e (\mathbf{v}_A - \mathbf{v}_B) \cdot \mathbf{n}$$

$$\left( (\mathbf{v}_A + j m_A^{-1} \mathbf{n}) - (\mathbf{v}_B - j m_B^{-1} \mathbf{n}) \right) \cdot \mathbf{n} = -e \, v_n$$

$$v_n + j (m_A^{-1} + m_B^{-1}) (\mathbf{n} \cdot \mathbf{n}) = -e \, v_n$$

Since $\mathbf{n}$ is a normalized unit vector ($\mathbf{n} \cdot \mathbf{n} = 1$):

$$j = \frac{-(1 + e) \, v_n}{m_A^{-1} + m_B^{-1}}$$

---

## 3. Applying the Velocity Updates

With the impulse scalar $j$ computed, velocity vectors are updated directly:

$$\mathbf{v}_A \leftarrow \mathbf{v}_A + j \cdot m_A^{-1} \cdot \mathbf{n}$$

$$\mathbf{v}_B \leftarrow \mathbf{v}_B - j \cdot m_B^{-1} \cdot \mathbf{n}$$

### Static Terrain Invariance
If Body B is immovable terrain ($m_B = \infty \implies m_B^{-1} = 0.0$):
- $m_A^{-1} + m_B^{-1} = m_A^{-1}$
- Body B experiences zero velocity change ($0.0 \cdot j = 0.0$).
- Body A absorbs the entire rebound impulse.

---

## 4. Baumgarte Positional Stabilization (Anti-Sinking)

Due to discrete time stepping ($\Delta t$), bodies will inevitably penetrate each other by a non-zero depth $d$ before an impulse can be computed. Resolving velocity alone does not undo existing spatial overlap, causing stacked objects to sink or "jitter" through floors.

To resolve penetration without introducing artificial kinetic energy, a **Positional Correction** factor is applied directly to coordinates:

$$\text{Total Inverse Mass} = m_A^{-1} + m_B^{-1}$$

$$\text{Correction Scalar} = \frac{\text{penetration}}{\text{Total Inverse Mass}} \cdot \beta$$

Where:
- $\beta \in [0.2, 0.8]$ is the **Baumgarte correction factor** (commonly $0.8$ or $80\%$). Using less than $100\%$ prevents over-correction oscillations between closely packed objects.

### Coordinate Displacements:

$$\mathbf{P}_A \leftarrow \mathbf{P}_A + \mathbf{n} \cdot m_A^{-1} \cdot \text{Correction Scalar}$$

$$\mathbf{P}_B \leftarrow \mathbf{P}_B - \mathbf{n} \cdot m_B^{-1} \cdot \text{Correction Scalar}$$

This immediately separates intersecting geometry in proportion to their mass distribution.
