# Friction Dynamics: Coulomb Friction Formulation

This document details the mathematical derivation and algorithmic implementation of dry friction (Coulomb friction) along contact tangent planes.

---

## 1. Classical Coulomb Friction Law

While normal impulses prevent objects from passing through each other, **friction impulses** resist tangential sliding across contact interfaces.

Coulomb's Law states that the maximum tangential frictional force $F_t$ before sliding occurs is proportional to the normal force $F_n$:

$$F_t \le \mu \cdot F_n$$

Where:
- $\mu_s$: Static friction coefficient (threshold required to initiate motion from rest).
- $\mu_k$: Kinetic / Dynamic friction coefficient (resistance opposing ongoing sliding, typically $\mu_k \le \mu_s$).

In discrete impulse dynamics, forces are integrated over timestep $\Delta t$, yielding an impulse relationship:

$$|j_t| \le \mu \cdot j_n$$

Where:
- $j_n$: Normal impulse magnitude computed during collision resolution.
- $j_t$: Tangential impulse magnitude.

---

## 2. Derivation of Tangential Contact Vectors

Given contact normal unit vector $\mathbf{n}$, the relative velocity between Body A and Body B is:

$$\mathbf{v}_{\text{rel}} = \mathbf{v}_A - \mathbf{v}_B$$

The normal velocity component is:

$$\mathbf{v}_n = (\mathbf{v}_{\text{rel}} \cdot \mathbf{n}) \mathbf{n}$$

The tangential sliding velocity vector $\mathbf{v}_t$ is obtained by subtracting the normal component:

$$\mathbf{v}_t = \mathbf{v}_{\text{rel}} - \mathbf{v}_n = \mathbf{v}_{\text{rel}} - (\mathbf{v}_{\text{rel}} \cdot \mathbf{n}) \mathbf{n}$$

### Friction Tangent Unit Vector ($\mathbf{t}$)
If $\|\mathbf{v}_t\| > \epsilon$ (where $\epsilon \approx 10^{-5}$), the unit tangent vector opposing relative motion is:

$$\mathbf{t} = \frac{\mathbf{v}_t}{\|\mathbf{v}_t\|}$$

If $\|\mathbf{v}_t\| \le \epsilon$, the bodies are not sliding tangentially, so tangential impulse is zero.

---

## 3. Tangential Impulse Computation

The unbounded impulse required to completely cancel relative sliding velocity is:

$$j_t^* = \frac{-(\mathbf{v}_{\text{rel}} \cdot \mathbf{t})}{m_A^{-1} + m_B^{-1}}$$

We determine the combined friction coefficient between the materials of both bodies:

$$\mu = \sqrt{\mu_A \cdot \mu_B} \quad \text{or} \quad \mu = \frac{\mu_A + \mu_B}{2}$$

### Clamping via Coulomb's Law:

$$j_t = \begin{cases}
j_t^* & \text{if } |j_t^*| \le \mu_s \cdot j_n \quad \text{(Static Friction: stops completely)} \\
-\text{sgn}(\mathbf{v}_{\text{rel}} \cdot \mathbf{t}) \cdot \mu_k \cdot j_n & \text{if } |j_t^*| > \mu_s \cdot j_n \quad \text{(Kinetic Friction: dynamic drag)}
\end{cases}$$

---

## 4. Applying Tangential Impulses

The frictional impulse vector $\mathbf{J}_t = j_t \cdot \mathbf{t}$ is applied to both bodies in equal and opposite directions:

$$\mathbf{v}_A \leftarrow \mathbf{v}_A + \mathbf{J}_t \cdot m_A^{-1}$$

$$\mathbf{v}_B \leftarrow \mathbf{v}_B - \mathbf{J}_t \cdot m_B^{-1}$$

This realistic drag prevents objects from sliding indefinitely on flat surfaces as if on frictionless ice.
