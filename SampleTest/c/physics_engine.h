/**
 * @file physics_engine.h
 * @brief EngineLib Native 3D Rigid Body Physics Engine C-ABI Header
 * @copyright MIT or Apache-2.0
 */

#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
    #if defined(PHYSICS_ENGINE_EXPORTS)
        #define PHYSICS_API __declspec(dllexport)
    #else
        #define PHYSICS_API __declspec(dllimport)
    #endif
#else
    #define PHYSICS_API __attribute__((visibility("default")))
#endif

/**
 * @brief 3D Single-Precision Vector (12 bytes, 4-byte aligned)
 */
typedef struct {
    float x;
    float y;
    float z;
} Vector3D;

/**
 * @brief Batch Transform Stream Structure (16 bytes)
 */
typedef struct {
    uint32_t id;
    Vector3D position;
} BodyTransform;

/**
 * @brief Raycast Hit Result Structure (32 bytes)
 */
typedef struct {
    Vector3D point;
    Vector3D normal;
    float distance;
    uint32_t body_id;
} RaycastHitFfi;

/* ========================================================================= */
/* World Management & Stepping                                               */
/* ========================================================================= */

PHYSICS_API void init_physics_world(float gx, float gy, float gz);
PHYSICS_API void destroy_physics_world(void);
PHYSICS_API void step_physics(float dt);
PHYSICS_API void set_world_substeps(uint32_t substeps);
PHYSICS_API void set_solver_iterations(uint32_t iterations);

/* ========================================================================= */
/* Rigid Body Creation & Deletion                                            */
/* ========================================================================= */

PHYSICS_API uint32_t add_rigid_body(
    float px, float py, float pz,
    float sx, float sy, float sz,
    float mass, float restitution
);

PHYSICS_API uint32_t add_sphere_body(
    float px, float py, float pz,
    float radius,
    float mass,
    float restitution,
    float friction
);

PHYSICS_API bool remove_rigid_body(uint32_t id);
PHYSICS_API bool remove_body(uint32_t id);

/* ========================================================================= */
/* State Queries & Telemetry                                                 */
/* ========================================================================= */

PHYSICS_API bool get_body_position(uint32_t id, Vector3D* out_pos);
PHYSICS_API bool set_body_position(uint32_t id, float x, float y, float z);

PHYSICS_API bool get_body_velocity(uint32_t id, Vector3D* out_vel);
PHYSICS_API bool set_body_velocity(uint32_t id, float vx, float vy, float vz);

PHYSICS_API bool get_body_rotation(uint32_t id, Vector3D* out_rot);
PHYSICS_API bool set_body_rotation(uint32_t id, float rx, float ry, float rz);

PHYSICS_API bool get_body_angular_velocity(uint32_t id, Vector3D* out_ang_vel);
PHYSICS_API bool set_body_angular_velocity(uint32_t id, float wx, float wy, float wz);

PHYSICS_API bool is_body_sleeping(uint32_t id);
PHYSICS_API void wake_body(uint32_t id);

/* ========================================================================= */
/* Forces, Impulses & Torques                                                */
/* ========================================================================= */

PHYSICS_API void apply_force(uint32_t id, float fx, float fy, float fz);
PHYSICS_API void apply_torque(uint32_t id, float tx, float ty, float tz);
PHYSICS_API void apply_impulse_at_point(
    uint32_t id,
    float ix, float iy, float iz,
    float px, float py, float pz
);

PHYSICS_API void set_body_friction(uint32_t id, float friction);
PHYSICS_API void set_body_damping(uint32_t id, float damping);

/* ========================================================================= */
/* Articulated Joints & Constraints                                          */
/* ========================================================================= */

PHYSICS_API uint32_t add_distance_joint(
    uint32_t body_a, uint32_t body_b,
    float off_ax, float off_ay, float off_az,
    float off_bx, float off_by, float off_bz,
    float rest_length, float stiffness, float damping
);

PHYSICS_API bool remove_joint(uint32_t id);

/* ========================================================================= */
/* Scene Queries & Batch Sync                                                */
/* ========================================================================= */

PHYSICS_API bool raycast(
    float ox, float oy, float oz,
    float dx, float dy, float dz,
    float max_dist,
    RaycastHitFfi* out_hit
);

PHYSICS_API bool sync_all_transforms(
    BodyTransform* out_buffer,
    uint32_t buffer_capacity,
    uint32_t* out_written_count
);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_ENGINE_H */
