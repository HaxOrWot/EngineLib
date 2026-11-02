#ifndef PHYSICS_ENGINE_HPP
#define PHYSICS_ENGINE_HPP

#include <cstdint>
#include <optional>
#include <stdexcept>

extern "C" {

typedef struct {
    float x;
    float y;
    float z;
} Vector3D;

typedef struct {
    uint32_t id;
    Vector3D position;
    Vector3D velocity;
    Vector3D acceleration;
    Vector3D size;
    float mass;
    float inverse_mass;
    float restitution;
} RigidBody3D;

void init_physics_world(float gx, float gy, float gz);

uint32_t add_rigid_body(
    float px, float py, float pz,
    float sx, float sy, float sz,
    float mass, float restitution
);

bool get_body_position(uint32_t id, Vector3D* out_pos);
void apply_force(uint32_t id, float fx, float fy, float fz);
void step_physics(float dt);
bool remove_rigid_body(uint32_t id);
void destroy_physics_world(void);

} // extern "C"

namespace Physics {

struct Vec3 {
    float x, y, z;
    Vec3(float x = 0.0f, float y = 0.0f, float z = 0.0f) : x(x), y(y), z(z) {}
    Vec3(const Vector3D& v) : x(v.x), y(v.y), z(v.z) {}

    Vector3D toC() const { return Vector3D{x, y, z}; }
};

class World {
public:
    World(float gx = 0.0f, float gy = -9.81f, float gz = 0.0f) {
        init_physics_world(gx, gy, gz);
    }

    ~World() {
        destroy_physics_world();
    }

    uint32_t addBody(const Vec3& pos, const Vec3& size, float mass, float restitution = 0.8f) {
        uint32_t id = add_rigid_body(
            pos.x, pos.y, pos.z,
            size.x, size.y, size.z,
            mass, restitution
        );
        if (id == 0) {
            throw std::runtime_error("Failed to allocate rigid body in native world!");
        }
        return id;
    }

    std::optional<Vec3> getPosition(uint32_t id) const {
        Vector3D cPos{};
        if (get_body_position(id, &cPos)) {
            return Vec3(cPos);
        }
        return std::nullopt;
    }

    void applyForce(uint32_t id, const Vec3& force) {
        ::apply_force(id, force.x, force.y, force.z);
    }

    void step(float dt) {
        step_physics(dt);
    }
};

} // namespace Physics

#endif // PHYSICS_ENGINE_HPP
