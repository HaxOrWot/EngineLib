# C & C++ Interop: Header & RAII Wrapper

This document provides the standard C header file (`physics_engine.h`) and a modern C++20 RAII wrapper class.

---

## 1. Universal C Header (`physics_engine.h`)

Save this file as `include/physics_engine.h` for consumption in C and C++ projects:

```c
#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
    #define PHYSICS_API __declspec(dllimport)
#else
    #define PHYSICS_API
#endif

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

typedef struct {
    uint32_t id;
    Vector3D position;
} BodyTransform;

PHYSICS_API void init_physics_world(float gx, float gy, float gz);
PHYSICS_API uint32_t add_rigid_body(
    float px, float py, float pz,
    float sx, float sy, float sz,
    float mass, float restitution
);
PHYSICS_API bool get_body_position(uint32_t id, Vector3D* out_pos);
PHYSICS_API void apply_force(uint32_t id, float fx, float fy, float fz);
PHYSICS_API void step_physics(float dt);

#ifdef __cplusplus
}
#endif

#endif // PHYSICS_ENGINE_H
```

---

## 2. Modern C++ RAII Wrapper Class

Provides idiomatic C++ semantics with operator overloading and type safety:

```cpp
#include "physics_engine.h"
#include <optional>
#include <stdexcept>

namespace Physics {

class World {
public:
    World(float gx = 0.0f, float gy = -9.81f, float gz = 0.0f) {
        init_physics_world(gx, gy, gz);
    }

    uint32_t addBody(const Vector3D& pos, const Vector3D& size, float mass, float restitution) {
        uint32_t id = add_rigid_body(
            pos.x, pos.y, pos.z,
            size.x, size.y, size.z,
            mass, restitution
        );
        if (id == 0) {
            throw std::runtime_error("Failed to allocate rigid body in native physics world");
        }
        return id;
    }

    std::optional<Vector3D> getPosition(uint32_t id) const {
        Vector3D pos{};
        if (get_body_position(id, &pos)) {
            return pos;
        }
        return std::nullopt;
    }

    void applyForce(uint32_t id, const Vector3D& force) {
        apply_force(id, force.x, force.y, force.z);
    }

    void step(float dt) {
        step_physics(dt);
    }
};

} // namespace Physics
```
