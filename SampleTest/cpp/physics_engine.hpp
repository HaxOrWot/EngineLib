/**
 * @file physics_engine.hpp
 * @brief Modern C++17 RAII Wrapper for EngineLib (physics_engine.dll)
 */

#ifndef PHYSICS_ENGINE_HPP
#define PHYSICS_ENGINE_HPP

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <iostream>

extern "C" {

typedef struct {
    float x, y, z;
} Vector3D;

typedef struct {
    Vector3D point;
    Vector3D normal;
    float distance;
    uint32_t body_id;
} RaycastHitFfi;

void init_physics_world(float gx, float gy, float gz);
void destroy_physics_world(void);
void step_physics(float dt);
void set_world_substeps(uint32_t substeps);
void set_solver_iterations(uint32_t iterations);

uint32_t add_rigid_body(
    float px, float py, float pz,
    float sx, float sy, float sz,
    float mass, float restitution
);

uint32_t add_sphere_body(
    float px, float py, float pz,
    float radius,
    float mass,
    float restitution,
    float friction
);

bool remove_rigid_body(uint32_t id);
bool remove_body(uint32_t id);

bool get_body_position(uint32_t id, Vector3D* out_pos);
bool set_body_position(uint32_t id, float x, float y, float z);

bool get_body_velocity(uint32_t id, Vector3D* out_vel);
bool set_body_velocity(uint32_t id, float vx, float vy, float vz);

bool get_body_rotation(uint32_t id, Vector3D* out_rot);
bool set_body_rotation(uint32_t id, float rx, float ry, float rz);

bool get_body_angular_velocity(uint32_t id, Vector3D* out_ang_vel);
bool set_body_angular_velocity(uint32_t id, float wx, float wy, float wz);

void apply_force(uint32_t id, float fx, float fy, float fz);
void apply_torque(uint32_t id, float tx, float ty, float tz);
void apply_impulse_at_point(
    uint32_t id,
    float ix, float iy, float iz,
    float px, float py, float pz
);

uint32_t add_distance_joint(
    uint32_t body_a, uint32_t body_b,
    float off_ax, float off_ay, float off_az,
    float off_bx, float off_by, float off_bz,
    float rest_length, float stiffness, float damping
);

bool raycast(
    float ox, float oy, float oz,
    float dx, float dy, float dz,
    float max_dist,
    RaycastHitFfi* out_hit
);

} // extern "C"

namespace EngineLib {

struct Vec3 {
    float x{0.0f}, y{0.0f}, z{0.0f};

    Vec3() = default;
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3(const Vector3D& c_vec) : x(c_vec.x), y(c_vec.y), z(c_vec.z) {}

    Vector3D toC() const { return Vector3D{x, y, z}; }
};

struct RaycastHit {
    Vec3 point;
    Vec3 normal;
    float distance{0.0f};
    uint32_t body_id{0};
};

class PhysicsWorld {
public:
    PhysicsWorld(float gx = 0.0f, float gy = -9.81f, float gz = 0.0f, uint32_t substeps = 4, uint32_t iterations = 8) {
        init_physics_world(gx, gy, gz);
        set_world_substeps(substeps);
        set_solver_iterations(iterations);
    }

    ~PhysicsWorld() {
        destroy_physics_world();
    }

    // Disable copy semantics to manage native state cleanly
    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    uint32_t createBox(const Vec3& pos, const Vec3& size, float mass = 1.0f, float restitution = 0.2f) {
        uint32_t id = add_rigid_body(pos.x, pos.y, pos.z, size.x, size.y, size.z, mass, restitution);
        if (id == 0) throw std::runtime_error("Failed to allocate box in native world");
        return id;
    }

    uint32_t createSphere(const Vec3& pos, float radius, float mass = 1.0f, float restitution = 0.6f, float friction = 0.45f) {
        uint32_t id = add_sphere_body(pos.x, pos.y, pos.z, radius, mass, restitution, friction);
        if (id == 0) throw std::runtime_error("Failed to allocate sphere in native world");
        return id;
    }

    void remove(uint32_t id) {
        remove_body(id);
    }

    void step(float dt) {
        step_physics(dt);
    }

    std::optional<Vec3> getPosition(uint32_t id) const {
        Vector3D cPos{};
        if (get_body_position(id, &cPos)) return Vec3(cPos);
        return std::nullopt;
    }

    std::optional<Vec3> getVelocity(uint32_t id) const {
        Vector3D cVel{};
        if (get_body_velocity(id, &cVel)) return Vec3(cVel);
        return std::nullopt;
    }

    std::optional<Vec3> getRotation(uint32_t id) const {
        Vector3D cRot{};
        if (get_body_rotation(id, &cRot)) return Vec3(cRot);
        return std::nullopt;
    }

    void applyForce(uint32_t id, const Vec3& force) {
        ::apply_force(id, force.x, force.y, force.z);
    }

    void applyTorque(uint32_t id, const Vec3& torque) {
        ::apply_torque(id, torque.x, torque.y, torque.z);
    }

    void applyImpulseAtPoint(uint32_t id, const Vec3& impulse, const Vec3& point) {
        ::apply_impulse_at_point(id, impulse.x, impulse.y, impulse.z, point.x, point.y, point.z);
    }

    uint32_t createDistanceJoint(
        uint32_t body_a, uint32_t body_b,
        const Vec3& offset_a, const Vec3& offset_b,
        float rest_length, float stiffness = 0.95f, float damping = 0.35f
    ) {
        return add_distance_joint(
            body_a, body_b,
            offset_a.x, offset_a.y, offset_a.z,
            offset_b.x, offset_b.y, offset_b.z,
            rest_length, stiffness, damping
        );
    }

    std::optional<RaycastHit> castRay(const Vec3& origin, const Vec3& dir, float max_dist = 100.0f) const {
        RaycastHitFfi hit{};
        if (raycast(origin.x, origin.y, origin.z, dir.x, dir.y, dir.z, max_dist, &hit)) {
            return RaycastHit{Vec3(hit.point), Vec3(hit.normal), hit.distance, hit.body_id};
        }
        return std::nullopt;
    }
};

} // namespace EngineLib

#endif // PHYSICS_ENGINE_HPP
