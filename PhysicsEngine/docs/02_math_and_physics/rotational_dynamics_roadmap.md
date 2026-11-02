# Rotational Dynamics Roadmap: 6-DOF Rigid Body Mechanics

This document outlines the theoretical roadmap to elevate the physics engine from translational AABB kinematics to a full **6 Degrees of Freedom (6-DOF)** rigid body engine featuring angular velocity, inertia tensors, and orientation quaternions.

---

## 1. Mathematical Transition: Linear to Angular

| Property | Linear Kinematics | Angular / Rotational Dynamics |
| :--- | :--- | :--- |
| **State** | Position $\mathbf{x} \in \mathbb{R}^3$ | Orientation Quaternion $\mathbf{q} \in \mathbb{H}$ |
| **Rate of Change** | Velocity $\mathbf{v} \in \mathbb{R}^3$ | Angular Velocity $\boldsymbol{\omega} \in \mathbb{R}^3$ |
| **Resistance** | Scalar Mass $m \in \mathbb{R}$ | Moment of Inertia Tensor $\mathbf{I} \in \mathbb{R}^{3 \times 3}$ |
| **Effort** | Force $\mathbf{F} \in \mathbb{R}^3$ | Torque $\boldsymbol{\tau} = \mathbf{r} \times \mathbf{F}$ |
| **Momentum** | Linear Momentum $\mathbf{p} = m \mathbf{v}$ | Angular Momentum $\mathbf{L} = \mathbf{I} \boldsymbol{\omega}$ |

---

## 2. Representation of Orientation: Unit Quaternions

Euler angles (pitch, yaw, roll) suffer from gimbal lock and non-commutative matrix composition. Modern physics engines represent 3D orientation using unit quaternions:

$$\mathbf{q} = (w, x, y, z) = \cos\left(\frac{\theta}{2}\right) + \mathbf{u} \sin\left(\frac{\theta}{2}\right)$$

Where $\mathbf{u}$ is the unit rotation axis and $\theta$ is the angle of rotation.

### Quaternion Time Derivative
Given angular velocity $\boldsymbol{\omega} = (\omega_x, \omega_y, \omega_z)$, the quaternion orientation evolves according to:

$$\frac{d\mathbf{q}}{dt} = \frac{1}{2} \boldsymbol{\omega}^* \otimes \mathbf{q}$$

Where $\boldsymbol{\omega}^* = (0, \omega_x, \omega_y, \omega_z)$ is the pure quaternion form.

During integration:

$$\mathbf{q}_{n+1} = \text{normalize}\left(\mathbf{q}_n + \frac{\Delta t}{2} \boldsymbol{\omega}^* \otimes \mathbf{q}_n\right)$$

---

## 3. Moment of Inertia Tensor for 3D Boxes

For a cuboid of mass $m$ with dimensions $(w, h, d)$ (width along X, height along Y, depth along Z), the local diagonal inertia tensor is:

$$\mathbf{I}_{\text{body}} = \begin{pmatrix}
\frac{1}{12}m(h^2 + d^2) & 0 & 0 \\
0 & \frac{1}{12}m(w^2 + d^2) & 0 \\
0 & 0 & \frac{1}{12}m(w^2 + h^2)
\end{pmatrix}$$

Its inverse $\mathbf{I}_{\text{body}}^{-1}$ is precomputed once at creation.

### World-Space Inverse Inertia Tensor
As the body rotates into orientation matrix $\mathbf{R}(\mathbf{q})$, the effective moment of inertia in world space transforms dynamically:

$$\mathbf{I}_{\text{world}}^{-1} = \mathbf{R} \, \mathbf{I}_{\text{body}}^{-1} \, \mathbf{R}^T$$

---

## 4. Angular Impulse at a Contact Point

When a contact occurs at point $\mathbf{p}_{\text{contact}}$, the leverage vectors from center-of-mass are:

$$\mathbf{r}_A = \mathbf{p}_{\text{contact}} - \mathbf{P}_A$$

$$\mathbf{r}_B = \mathbf{p}_{\text{contact}} - \mathbf{P}_B$$

The total velocity at the contact point includes rotational velocity:

$$\mathbf{v}_{\text{contact}, A} = \mathbf{v}_A + \boldsymbol{\omega}_A \times \mathbf{r}_A$$

$$\mathbf{v}_{\text{contact}, B} = \mathbf{v}_B + \boldsymbol{\omega}_B \times \mathbf{r}_B$$

The denominator for the impulse scalar $j$ expands to include angular mass terms:

$$j = \frac{-(1 + e) \, (\mathbf{v}_{\text{rel}} \cdot \mathbf{n})}{m_A^{-1} + m_B^{-1} + \left( \mathbf{I}_{A}^{-1}(\mathbf{r}_A \times \mathbf{n}) \times \mathbf{r}_A + \mathbf{I}_{B}^{-1}(\mathbf{r}_B \times \mathbf{n}) \times \mathbf{r}_B \right) \cdot \mathbf{n}}$$

### Applying Linear and Angular Updates:

$$\mathbf{v} \leftarrow \mathbf{v} \pm j \cdot m^{-1} \cdot \mathbf{n}$$

$$\boldsymbol{\omega} \leftarrow \boldsymbol{\omega} \pm \mathbf{I}^{-1}(\mathbf{r} \times (j \cdot \mathbf{n}))$$
