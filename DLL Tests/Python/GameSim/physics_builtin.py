"""
Pure Python 2D Rigid Body Physics Engine for SiegePhysics
Provides full simulation capabilities WITHOUT external DLL dependencies:
- Circle and Box (AABB/OBB) shapes
- Mass, inertia, velocity, restitution, Coulomb friction
- Collision detection (Circle-Circle, Circle-Box, Box-Box)
- Impulse-based contact resolution with anti-sinking positional projection
- Radial explosion shockwaves and damage calculation
"""

import math

class Vector2D:
    __slots__ = ('x', 'y')

    def __init__(self, x=0.0, y=0.0):
        self.x = float(x)
        self.y = float(y)

    def __repr__(self):
        return f"Vec2({self.x:.2f}, {self.y:.2f})"

    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return Vector2D(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar):
        return Vector2D(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar):
        if scalar == 0.0:
            return Vector2D(0.0, 0.0)
        return Vector2D(self.x / scalar, self.y / scalar)

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def cross(self, other):
        return self.x * other.y - self.y * other.x

    def length_squared(self):
        return self.x * self.x + self.y * self.y

    def length(self):
        return math.hypot(self.x, self.y)

    def normalized(self):
        l = self.length()
        if l > 1e-6:
            return Vector2D(self.x / l, self.y / l)
        return Vector2D(0.0, 0.0)

    def distance_to(self, other):
        return math.hypot(self.x - other.x, self.y - other.y)


class RigidBody2D:
    def __init__(
        self,
        body_id,
        shape_type,
        x,
        y,
        mass=1.0,
        restitution=0.3,
        friction=0.5,
        radius=1.0,
        width=1.0,
        height=1.0,
        material="wood",
    ):
        self.id = body_id
        self.shape_type = shape_type  # 'circle' or 'box'
        self.position = Vector2D(x, y)
        self.velocity = Vector2D(0.0, 0.0)
        self.acceleration = Vector2D(0.0, 0.0)

        self.angle = 0.0
        self.angular_velocity = 0.0

        self.mass = float(mass)
        self.inv_mass = 1.0 / self.mass if self.mass > 0.0 else 0.0

        self.restitution = float(restitution)
        self.friction = float(friction)
        self.radius = float(radius)
        self.width = float(width)
        self.height = float(height)
        self.material = material  # 'wood', 'stone', 'tnt', 'glass', 'enemy', 'cannonball', 'terrain'

        # Moment of Inertia
        if self.inv_mass == 0.0:
            self.inv_inertia = 0.0
        elif self.shape_type == "circle":
            # I = 0.5 * m * r^2
            inertia = 0.5 * self.mass * (self.radius ** 2)
            self.inv_inertia = 1.0 / inertia if inertia > 0 else 0.0
        else:
            # I = (1/12) * m * (w^2 + h^2)
            inertia = (1.0 / 12.0) * self.mass * (self.width ** 2 + self.height ** 2)
            self.inv_inertia = 1.0 / inertia if inertia > 0 else 0.0

        # Health & State
        self.max_health = 100.0
        if material == "wood":
            self.max_health = 120.0
        elif material == "stone":
            self.max_health = 250.0
        elif material == "tnt":
            self.max_health = 40.0
        elif material == "glass":
            self.max_health = 30.0
        elif material == "enemy":
            self.max_health = 50.0
        elif material == "cannonball":
            self.max_health = 9999.0

        self.health = self.max_health
        self.is_dead = False
        self.is_sleeping = False
        self.sleep_timer = 0.0
        self.color = (200, 200, 200)

    @property
    def is_static(self):
        return self.inv_mass == 0.0

    def apply_impulse(self, impulse, contact_vector=None):
        if self.is_static:
            return
        self.velocity += impulse * self.inv_mass
        if contact_vector is not None and self.inv_inertia > 0.0:
            torque = contact_vector.cross(impulse)
            self.angular_velocity += torque * self.inv_inertia
        self.is_sleeping = False
        self.sleep_timer = 0.0

    def take_damage(self, amount):
        if self.material == "terrain" or self.material == "cannonball":
            return False
        self.health -= amount
        if self.health <= 0.0:
            self.is_dead = True
            return True
        return False


class ContactManifold:
    __slots__ = ('body_a', 'body_b', 'normal', 'penetration', 'contact_point')

    def __init__(self, body_a, body_b, normal, penetration, contact_point=None):
        self.body_a = body_a
        self.body_b = body_b
        self.normal = normal  # Points from B to A
        self.penetration = penetration
        self.contact_point = contact_point or body_a.position


class BuiltinPhysicsWorld:
    def __init__(self, gravity_x=0.0, gravity_y=980.0):
        # Default 2D gravity (downward in Pygame screen coordinates)
        self.gravity = Vector2D(gravity_x, gravity_y)
        self.bodies = []
        self.next_id = 1
        self.substeps = 4
        self.position_iterations = 3

    def add_body(self, body):
        self.bodies.append(body)
        return body

    def remove_body(self, body):
        if body in self.bodies:
            self.bodies.remove(body)

    def create_circle(self, x, y, radius, mass=1.0, restitution=0.4, friction=0.4, material="wood"):
        body = RigidBody2D(
            self.next_id,
            "circle",
            x,
            y,
            mass=mass,
            restitution=restitution,
            friction=friction,
            radius=radius,
            material=material,
        )
        self.next_id += 1
        return self.add_body(body)

    def create_box(self, x, y, width, height, mass=1.0, restitution=0.2, friction=0.6, material="wood"):
        body = RigidBody2D(
            self.next_id,
            "box",
            x,
            y,
            mass=mass,
            restitution=restitution,
            friction=friction,
            width=width,
            height=height,
            material=material,
        )
        self.next_id += 1
        return self.add_body(body)

    def apply_radial_impulse(self, center_x, center_y, radius, max_force, max_damage=150.0):
        """Applies explosive force and blast damage in a radial sphere"""
        center = Vector2D(center_x, center_y)
        hit_bodies = []

        for b in self.bodies:
            if b.is_static:
                continue
            dist = center.distance_to(b.position)
            if dist < radius:
                factor = 1.0 - (dist / radius)
                dir_vec = (b.position - center).normalized()
                if dir_vec.length_squared() < 0.01:
                    dir_vec = Vector2D(0.0, -1.0)  # Push upward if dead center

                impulse = dir_vec * (max_force * factor)
                b.apply_impulse(impulse)
                dmg = max_damage * factor
                died = b.take_damage(dmg)
                hit_bodies.append((b, died, dmg))

        return hit_bodies

    def step(self, dt):
        if dt <= 0.0:
            return

        dt = min(dt, 0.033)
        sub_dt = dt / self.substeps

        for _ in range(self.substeps):
            self._substep(sub_dt)

    def _substep(self, dt):
        # 1. Integrate forces and velocities
        for b in self.bodies:
            if b.is_static or b.is_sleeping:
                continue

            # Gravity
            b.velocity += self.gravity * dt

            # Air drag / angular damping
            b.velocity *= 1.0 / (1.0 + 0.15 * dt)
            b.angular_velocity *= 1.0 / (1.0 + 0.35 * dt)

            # Integrate position
            b.position += b.velocity * dt
            b.angle += b.angular_velocity * dt

        # 2. Collision Detection
        manifolds = []
        n = len(self.bodies)
        for i in range(n):
            b_a = self.bodies[i]
            for j in range(i + 1, n):
                b_b = self.bodies[j]

                # Skip static vs static or sleeping vs sleeping
                if (b_a.is_static or b_a.is_sleeping) and (b_b.is_static or b_b.is_sleeping):
                    continue

                manifold = self._detect_collision(b_a, b_b)
                if manifold is not None:
                    manifolds.append(manifold)

        # 3. Resolve Contacts
        for m in manifolds:
            self._resolve_contact(m, dt)

    def _detect_collision(self, a, b):
        if a.shape_type == "circle" and b.shape_type == "circle":
            return self._circle_vs_circle(a, b)
        elif a.shape_type == "circle" and b.shape_type == "box":
            return self._circle_vs_box(a, b)
        elif a.shape_type == "box" and b.shape_type == "circle":
            m = self._circle_vs_box(b, a)
            if m is not None:
                m.body_a = a
                m.body_b = b
                m.normal = m.normal * -1.0
            return m
        elif a.shape_type == "box" and b.shape_type == "box":
            return self._box_vs_box(a, b)
        return None

    def _circle_vs_circle(self, a, b):
        delta = a.position - b.position
        dist_sq = delta.length_squared()
        radius_sum = a.radius + b.radius

        if dist_sq >= radius_sum * radius_sum:
            return None

        dist = math.sqrt(dist_sq)
        if dist > 1e-5:
            normal = delta * (1.0 / dist)
            penetration = radius_sum - dist
        else:
            normal = Vector2D(0.0, -1.0)
            penetration = radius_sum

        contact_point = b.position + normal * (b.radius - penetration * 0.5)
        return ContactManifold(a, b, normal, penetration, contact_point)

    def _circle_vs_box(self, c, b):
        # AABB clamped closest point
        half_w = b.width * 0.5
        half_h = b.height * 0.5

        closest_x = max(b.position.x - half_w, min(c.position.x, b.position.x + half_w))
        closest_y = max(b.position.y - half_h, min(c.position.y, b.position.y + half_h))

        closest = Vector2D(closest_x, closest_y)
        delta = c.position - closest
        dist_sq = delta.length_squared()

        if dist_sq > c.radius * c.radius:
            return None

        dist = math.sqrt(dist_sq)
        if dist > 1e-5:
            normal = delta * (1.0 / dist)
            penetration = c.radius - dist
        else:
            # Circle center is inside box: push out along shallowest face
            dx = c.position.x - b.position.x
            dy = c.position.y - b.position.y
            overlap_x = half_w - abs(dx)
            overlap_y = half_h - abs(dy)

            if overlap_x < overlap_y:
                sign = 1.0 if dx >= 0 else -1.0
                normal = Vector2D(sign, 0.0)
                penetration = overlap_x + c.radius
            else:
                sign = 1.0 if dy >= 0 else -1.0
                normal = Vector2D(0.0, sign)
                penetration = overlap_y + c.radius

        return ContactManifold(c, b, normal, penetration, closest)

    def _box_vs_box(self, a, b):
        # Axis-Aligned Bounding Box overlap
        half_a_w = a.width * 0.5
        half_a_h = a.height * 0.5
        half_b_w = b.width * 0.5
        half_b_h = b.height * 0.5

        delta = a.position - b.position
        overlap_x = (half_a_w + half_b_w) - abs(delta.x)
        if overlap_x <= 0.0:
            return None

        overlap_y = (half_a_h + half_b_h) - abs(delta.y)
        if overlap_y <= 0.0:
            return None

        # Shallowest axis is contact normal
        if overlap_x < overlap_y:
            sign = 1.0 if delta.x >= 0 else -1.0
            normal = Vector2D(sign, 0.0)
            penetration = overlap_x
            contact_point = Vector2D(
                b.position.x + sign * half_b_w,
                (a.position.y + b.position.y) * 0.5
            )
        else:
            sign = 1.0 if delta.y >= 0 else -1.0
            normal = Vector2D(0.0, sign)
            penetration = overlap_y
            contact_point = Vector2D(
                (a.position.x + b.position.x) * 0.5,
                b.position.y + sign * half_b_h
            )

        return ContactManifold(a, b, normal, penetration, contact_point)

    def _resolve_contact(self, m, dt):
        a = m.body_a
        b = m.body_b
        total_inv_mass = a.inv_mass + b.inv_mass
        if total_inv_mass <= 0.0:
            return

        normal = m.normal
        penetration = m.penetration

        # 1. Positional Anti-Sinking Projection (Baumgarte)
        slop = 0.5
        effective_pen = max(0.0, penetration - slop)
        if effective_pen > 0.0:
            correction = (effective_pen / total_inv_mass) * 0.45
            a.position += normal * (a.inv_mass * correction)
            b.position -= normal * (b.inv_mass * correction)

        # 2. Velocity resolution
        rel_vel = a.velocity - b.velocity
        vel_along_normal = rel_vel.dot(normal)

        # Relative impact speed for structural damage
        impact_speed = abs(vel_along_normal)
        if impact_speed > 180.0:
            # High-velocity impact inflicts damage
            dmg = (impact_speed - 180.0) * 0.35
            a.take_damage(dmg)
            b.take_damage(dmg)

        # Elastic restitution
        restitution = min(a.restitution, b.restitution)
        if impact_speed < 30.0:
            restitution = 0.0  # Kill jitter for resting contacts

        impulse_scalar = 0.0
        if vel_along_normal < 0.0:
            impulse_scalar = -(1.0 + restitution) * vel_along_normal / total_inv_mass
            impulse = normal * impulse_scalar
            a.velocity += impulse * a.inv_mass
            b.velocity -= impulse * b.inv_mass

        # 3. Tangential Friction
        updated_rel_vel = a.velocity - b.velocity
        tangent_vel = updated_rel_vel - normal * updated_rel_vel.dot(normal)
        tangent_speed = tangent_vel.length()

        if tangent_speed > 1e-4:
            tangent = tangent_vel * (1.0 / tangent_speed)
            friction_coeff = math.sqrt(a.friction * b.friction)

            # Normal sustained load + impact impulse
            sustained_load = max(0.0, (self.gravity.y * dt) / total_inv_mass)
            normal_load = max(impulse_scalar, sustained_load)

            friction_ideal = -tangent_speed / total_inv_mass
            max_friction = friction_coeff * normal_load
            friction_clamped = max(friction_ideal, -max_friction)

            friction_vec = tangent * friction_clamped
            a.velocity += friction_vec * a.inv_mass
            b.velocity -= friction_vec * b.inv_mass

        # Wake bodies up
        a.is_sleeping = False
        b.is_sleeping = False
