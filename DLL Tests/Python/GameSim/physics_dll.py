"""
Native DLL Physics Engine Backend for SiegePhysics
Hooks into EngineLib (physics_engine.dll) via ctypes to provide high-performance
C-accelerated rigid-body dynamics, torque, angular velocity, and toppling physics.
"""

import os
import math
import ctypes
from ctypes import c_float, c_uint32, c_bool, Structure, POINTER, byref

# ---------------------------------------------------------------------------
# C-ABI Structures & DLL Loader
# ---------------------------------------------------------------------------
class Vector3D(Structure):
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
    ]

def load_physics_dll():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(current_dir, "physics_engine.dll"),
        os.path.join(current_dir, "..", "physics_engine.dll"),
        os.path.join(current_dir, "..", "..", "..", "PhysicsEngine", "target", "release", "physics_engine.dll"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            lib = ctypes.CDLL(os.path.abspath(path))
            return lib
    raise FileNotFoundError("Could not locate physics_engine.dll!")

def configure_dll_signatures(lib):
    lib.init_physics_world.argtypes = [c_float, c_float, c_float]
    lib.init_physics_world.restype = None

    lib.destroy_physics_world.argtypes = []
    lib.destroy_physics_world.restype = None

    lib.add_rigid_body.argtypes = [c_float, c_float, c_float, c_float, c_float, c_float, c_float, c_float]
    lib.add_rigid_body.restype = c_uint32

    lib.add_sphere_body.argtypes = [c_float, c_float, c_float, c_float, c_float, c_float, c_float]
    lib.add_sphere_body.restype = c_uint32

    if hasattr(lib, "remove_rigid_body"):
        lib.remove_rigid_body.argtypes = [c_uint32]
        lib.remove_rigid_body.restype = c_bool
    if hasattr(lib, "remove_body"):
        lib.remove_body.argtypes = [c_uint32]
        lib.remove_body.restype = c_bool

    lib.get_body_position.argtypes = [c_uint32, POINTER(Vector3D)]
    lib.get_body_position.restype = c_bool

    lib.get_body_velocity.argtypes = [c_uint32, POINTER(Vector3D)]
    lib.get_body_velocity.restype = c_bool

    lib.set_body_velocity.argtypes = [c_uint32, c_float, c_float, c_float]
    lib.set_body_velocity.restype = c_bool

    # Rotational FFI
    if hasattr(lib, "get_body_rotation"):
        lib.get_body_rotation.argtypes = [c_uint32, POINTER(Vector3D)]
        lib.get_body_rotation.restype = c_bool

    if hasattr(lib, "set_body_rotation"):
        lib.set_body_rotation.argtypes = [c_uint32, c_float, c_float, c_float]
        lib.set_body_rotation.restype = c_bool

    if hasattr(lib, "get_body_angular_velocity"):
        lib.get_body_angular_velocity.argtypes = [c_uint32, POINTER(Vector3D)]
        lib.get_body_angular_velocity.restype = c_bool

    if hasattr(lib, "set_body_angular_velocity"):
        lib.set_body_angular_velocity.argtypes = [c_uint32, c_float, c_float, c_float]
        lib.set_body_angular_velocity.restype = c_bool

    if hasattr(lib, "apply_torque"):
        lib.apply_torque.argtypes = [c_uint32, c_float, c_float, c_float]
        lib.apply_torque.restype = None

    if hasattr(lib, "apply_impulse_at_point"):
        lib.apply_impulse_at_point.argtypes = [
            c_uint32,
            c_float, c_float, c_float,
            c_float, c_float, c_float
        ]
        lib.apply_impulse_at_point.restype = None

    lib.apply_force.argtypes = [c_uint32, c_float, c_float, c_float]
    lib.apply_force.restype = None

    lib.set_body_damping.argtypes = [c_uint32, c_float]
    lib.set_body_damping.restype = None

    lib.set_world_substeps.argtypes = [c_uint32]
    lib.set_world_substeps.restype = None

    lib.set_solver_iterations.argtypes = [c_uint32]
    lib.set_solver_iterations.restype = None

    lib.step_physics.argtypes = [c_float]
    lib.step_physics.restype = None

    return lib


# ---------------------------------------------------------------------------
# Python Wrapper Types
# ---------------------------------------------------------------------------
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

    def dot(self, other):
        return self.x * other.x + self.y * other.y

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


class DLLRigidBody2D:
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
        self.shape_type = shape_type
        self.position = Vector2D(x, y)
        self.velocity = Vector2D(0.0, 0.0)
        self.angle = 0.0
        self.angular_velocity = 0.0

        self.mass = float(mass)
        self.inv_mass = 1.0 / self.mass if self.mass > 0.0 else 0.0
        self.restitution = float(restitution)
        self.friction = float(friction)
        self.radius = float(radius)
        self.width = float(width)
        self.height = float(height)
        self.material = material

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
        self.color = (200, 200, 200)

    @property
    def is_static(self):
        return self.inv_mass == 0.0

    def take_damage(self, amount):
        if self.material == "terrain" or self.material == "cannonball":
            return False
        self.health -= amount
        if self.health <= 0.0:
            self.is_dead = True
            return True
        return False


# ---------------------------------------------------------------------------
# DLL-Backed Physics World
# ---------------------------------------------------------------------------
class DLLPhysicsWorld:
    PPM = 25.0  # Pixels Per Meter scaling factor

    def __init__(self, gravity_x=0.0, gravity_y=680.0):
        self.raw_lib = load_physics_dll()
        self.lib = configure_dll_signatures(self.raw_lib)

        self.gravity = Vector2D(gravity_x, gravity_y)
        self.bodies = []
        self.bodies_by_id = {}

        # Re-initialize native world in C memory
        gx_m = gravity_x / self.PPM
        gy_m = gravity_y / self.PPM
        self.lib.destroy_physics_world()
        self.lib.init_physics_world(gx_m, gy_m, 0.0)

        # High-precision substeps and iterations for stable stacks
        self.lib.set_world_substeps(4)
        self.lib.set_solver_iterations(8)

        # Ctypes buffers
        self._pos_buf = Vector3D()
        self._vel_buf = Vector3D()
        self._rot_buf = Vector3D()
        self._ang_buf = Vector3D()

    def destroy(self):
        self.lib.destroy_physics_world()
        self.bodies.clear()
        self.bodies_by_id.clear()

    def create_circle(self, x, y, radius, mass=1.0, restitution=0.4, friction=0.4, material="wood"):
        mx = x / self.PPM
        my = y / self.PPM
        mr = radius / self.PPM

        c_id = self.lib.add_sphere_body(
            c_float(mx), c_float(my), c_float(0.0),
            c_float(mr), c_float(mass), c_float(restitution), c_float(friction)
        )

        body = DLLRigidBody2D(
            c_id, "circle", x, y, mass=mass, restitution=restitution,
            friction=friction, radius=radius, material=material
        )
        self.bodies.append(body)
        self.bodies_by_id[c_id] = body
        return body

    def create_box(self, x, y, width, height, mass=1.0, restitution=0.2, friction=0.6, material="wood"):
        mx = x / self.PPM
        my = y / self.PPM
        mw = width / self.PPM
        mh = height / self.PPM
        mz = 10.0  # Deep Z slab to lock 2D planar collisions

        c_id = self.lib.add_rigid_body(
            c_float(mx), c_float(my), c_float(0.0),
            c_float(mw), c_float(mh), c_float(mz),
            c_float(mass), c_float(restitution)
        )

        body = DLLRigidBody2D(
            c_id, "box", x, y, mass=mass, restitution=restitution,
            friction=friction, width=width, height=height, material=material
        )
        self.bodies.append(body)
        self.bodies_by_id[c_id] = body
        return body

    def remove_body(self, body):
        if body in self.bodies:
            self.bodies.remove(body)
        if body.id in self.bodies_by_id:
            del self.bodies_by_id[body.id]
        if hasattr(self.lib, "remove_rigid_body"):
            self.lib.remove_rigid_body(body.id)
        elif hasattr(self.lib, "remove_body"):
            self.lib.remove_body(body.id)

    def set_body_velocity(self, body, vx, vy):
        body.velocity.x = vx
        body.velocity.y = vy
        vx_m = vx / self.PPM
        vy_m = vy / self.PPM
        self.lib.set_body_velocity(body.id, c_float(vx_m), c_float(vy_m), c_float(0.0))

    def apply_impulse(self, body, impulse_x, impulse_y, at_point_x=None, at_point_y=None):
        if body.is_static or body.inv_mass <= 0.0:
            return

        if at_point_x is not None and at_point_y is not None and hasattr(self.lib, "apply_impulse_at_point"):
            ix_m = impulse_x / self.PPM
            iy_m = impulse_y / self.PPM
            px_m = at_point_x / self.PPM
            py_m = at_point_y / self.PPM
            self.lib.apply_impulse_at_point(
                body.id,
                c_float(ix_m), c_float(iy_m), c_float(0.0),
                c_float(px_m), c_float(py_m), c_float(0.0),
            )
        else:
            dvx = (impulse_x / body.mass)
            dvy = (impulse_y / body.mass)
            new_vx = body.velocity.x + dvx
            new_vy = body.velocity.y + dvy
            self.set_body_velocity(body, new_vx, new_vy)

    def apply_radial_impulse(self, center_x, center_y, radius, max_force, max_damage=150.0):
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
                    dir_vec = Vector2D(0.0, -1.0)

                impulse = dir_vec * (max_force * factor * 0.045)
                # Apply at offset point for rotational blast tumbling
                contact_pt_x = b.position.x - dir_vec.x * (b.width * 0.4 if b.shape_type == "box" else b.radius)
                contact_pt_y = b.position.y - dir_vec.y * (b.height * 0.4 if b.shape_type == "box" else b.radius)

                self.apply_impulse(b, impulse.x, impulse.y, contact_pt_x, contact_pt_y)

                dmg = max_damage * factor
                died = b.take_damage(dmg)
                hit_bodies.append((b, died, dmg))

        return hit_bodies

    def step(self, dt):
        if dt <= 0.0:
            return

        dt = min(dt, 0.033)

        # 1. Step the native C++ physics engine
        self.lib.step_physics(c_float(dt))

        # 2. Synchronize linear & rotational state from native DLL memory
        has_rot = hasattr(self.lib, "get_body_rotation")
        has_ang = hasattr(self.lib, "get_body_angular_velocity")

        for b in self.bodies:
            if not b.is_static:
                # Position
                if self.lib.get_body_position(b.id, byref(self._pos_buf)):
                    b.position.x = self._pos_buf.x * self.PPM
                    b.position.y = self._pos_buf.y * self.PPM

                # Linear Velocity
                if self.lib.get_body_velocity(b.id, byref(self._vel_buf)):
                    prev_vx = b.velocity.x
                    prev_vy = b.velocity.y
                    b.velocity.x = self._vel_buf.x * self.PPM
                    b.velocity.y = self._vel_buf.y * self.PPM

                    # Impact damage detection based on deceleration
                    dv = math.hypot(b.velocity.x - prev_vx, b.velocity.y - prev_vy)
                    if dv > 180.0:
                        dmg = (dv - 180.0) * 0.35
                        b.take_damage(dmg)

                # Rotation & Angular Velocity (Z-axis in 2D plane)
                if has_rot and self.lib.get_body_rotation(b.id, byref(self._rot_buf)):
                    b.angle = self._rot_buf.z

                if has_ang and self.lib.get_body_angular_velocity(b.id, byref(self._ang_buf)):
                    b.angular_velocity = self._ang_buf.z
