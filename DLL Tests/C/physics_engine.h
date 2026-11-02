#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
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

#ifdef __cplusplus
}
#endif

#endif // PHYSICS_ENGINE_H
