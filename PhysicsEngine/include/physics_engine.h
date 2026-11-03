/**
 * @file physics_engine.h
 * @brief Universal High-Performance 3D Rigid Body Physics Engine C-ABI Header
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
 * @brief Rigid body collision shape identifier
 */
typedef enum {
    SHAPE_AABB = 0,
    SHAPE_SPHERE = 1,
    SHAPE_CAPSULE = 2
} ShapeType;

/**
 * @brief 3D Single-Precision Cartesian Vector (12 bytes, 4-byte aligned)
 */
typedef struct {
    float x;
    float y;
    float z;
} Vector3D;

/**
 * @brief 3D Single-Precision Unit Quaternion for drift-free orientation (16 bytes, 4-byte aligned)
 */
typedef struct {
    float x;
    float y;
    float z;
    float w;
} Quaternion;

/**
 * @brief Batch Transform Stream Structure (16 bytes)
 */
typedef struct {
    uint32_t id;
    Vector3D position;
} BodyTransform;

/**
 * @brief Extended Batch Transform Stream with full Quaternion orientation (32 bytes)
 */
typedef struct {
    uint32_t id;
    Vector3D position;
    Quaternion orientation;
} BodyTransformEx;

/**
 * @brief Raycast Hit Result Structure (32 bytes)
 */
typedef struct {
    Vector3D point;
    Vector3D normal;
    float distance;
    uint32_t body_id;
} RaycastHitFfi;

/**
 * @brief High-precision collision telemetry and trigger event (40 bytes)
 */
typedef struct {
    uint32_t body_a;
    uint32_t body_b;
    Vector3D contact_point;
    Vector3D normal;
    float impulse;
    uint32_t is_sensor;
} CollisionEvent;

/* ========================================================================= */
/*                          WORLD MANAGEMENT                                 */
/* ========================================================================= */

/**
 * @brief Initializes or resets the global singleton physics world.
 * @param gx Gravity along X axis (m/s^2)
 * @param gy Gravity along Y axis (m/s^2, e.g. -9.81)
 * @param gz Gravity along Z axis (m/s^2)
 */
PHYSICS_API void init_physics_world(float gx, float gy, float gz);

/**
 * @brief Advances the simulation by a discrete timestep dt.
 * @param dt Timestep in seconds (e.g. 0.016667 for 60 Hz)
 */
PHYSICS_API void step_physics(float dt);

/**
 * @brief Tears down the global physics world and releases all memory.
 */
PHYSICS_API void destroy_physics_world(void);

/**
 * @brief Configures sub-stepping for enhanced numerical stability (default: 2, range 1..16).
 */
PHYSICS_API void set_world_substeps(uint32_t substeps);

/**
 * @brief Configures iterative constraint solver pass count (default: 4, range 1..20).
 */
PHYSICS_API void set_solver_iterations(uint32_t iters);

/* ========================================================================= */
/*                       RIGID BODY ALLOCATION & REMOVAL                     */
/* ========================================================================= */

/**
 * @brief Allocates and inserts a new AABB rigid body.
 * @param px, py, pz Center position in world coordinates
 * @param sx, sy, sz Bounding box dimensions (width, height, depth)
 * @param mass Mass in kg (0.0 creates a static, immovable object)
 * @param restitution Restitution coefficient (0.0 = inelastic, 1.0 = fully elastic)
 * @return Unique entity ID (> 0), or 0 on failure
 */
PHYSICS_API uint32_t add_rigid_body(
    float px, float py, float pz,
    float sx, float sy, float sz,
    float mass, float restitution
);

/**
 * @brief Allocates and inserts a new spherical rigid body.
 * @param px, py, pz Center position
 * @param radius Sphere radius
 * @param mass Mass in kg
 * @param restitution Restitution coefficient (0.0 to 1.0)
 * @param friction Coulomb friction coefficient
 * @return Unique entity ID (> 0), or 0 on failure
 */
PHYSICS_API uint32_t add_sphere_body(
    float px, float py, float pz,
    float radius,
    float mass,
    float restitution,
    float friction
);

/**
 * @brief Allocates and inserts a new capsule rigid body (ideal for character controllers).
 * @param px, py, pz Center position
 * @param radius Hemisphere cap radius
 * @param height Height of the central cylinder segment
 * @param mass Mass in kg
 * @param restitution Restitution coefficient (0.0 to 1.0)
 * @param friction Coulomb friction coefficient
 * @return Unique entity ID (> 0), or 0 on failure
 */
PHYSICS_API uint32_t add_capsule_body(
    float px, float py, float pz,
    float radius,
    float height,
    float mass,
    float restitution,
    float friction
);

/**
 * @brief Removes an individual body from simulation.
 */
PHYSICS_API bool remove_rigid_body(uint32_t id);
PHYSICS_API bool remove_body(uint32_t id);

/* ========================================================================= */
/*                          BODY STATE ACCESSORS                             */
/* ========================================================================= */

PHYSICS_API bool get_body_position(uint32_t id, Vector3D* out_pos);
PHYSICS_API bool get_body_velocity(uint32_t id, Vector3D* out_vel);
PHYSICS_API bool set_body_velocity(uint32_t id, float vx, float vy, float vz);

PHYSICS_API bool get_body_rotation(uint32_t id, Vector3D* out_rot);
PHYSICS_API bool set_body_rotation(uint32_t id, float rx, float ry, float rz);

PHYSICS_API bool get_body_orientation(uint32_t id, Quaternion* out_quat);
PHYSICS_API bool set_body_orientation(uint32_t id, float qx, float qy, float qz, float qw);

PHYSICS_API bool get_body_angular_velocity(uint32_t id, Vector3D* out_ang_vel);
PHYSICS_API bool set_body_angular_velocity(uint32_t id, float wx, float wy, float wz);

PHYSICS_API void set_body_friction(uint32_t id, float friction);
PHYSICS_API void set_body_damping(uint32_t id, float damping);

PHYSICS_API bool is_body_sleeping(uint32_t id);
PHYSICS_API void wake_body(uint32_t id);

PHYSICS_API bool set_body_sensor(uint32_t id, uint32_t is_sensor);
PHYSICS_API bool is_body_sensor(uint32_t id);

PHYSICS_API bool set_body_ccd(uint32_t id, uint32_t enable_ccd);
PHYSICS_API bool is_body_ccd(uint32_t id);

/* ========================================================================= */
/*                           FORCES & IMPULSES                               */
/* ========================================================================= */

PHYSICS_API void apply_force(uint32_t id, float fx, float fy, float fz);
PHYSICS_API void apply_torque(uint32_t id, float tx, float ty, float tz);
PHYSICS_API void apply_impulse_at_point(
    uint32_t id,
    float ix, float iy, float iz,
    float px, float py, float pz
);

/* ========================================================================= */
/*                         CONSTRAINTS & JOINTS                              */
/* ========================================================================= */

PHYSICS_API uint32_t add_distance_joint(
    uint32_t body_a,
    uint32_t body_b,
    float ax, float ay, float az,
    float bx, float by, float bz,
    float rest_len,
    float stiffness,
    float damping
);

PHYSICS_API bool remove_joint(uint32_t id);
PHYSICS_API void disable_collision_pair(uint32_t body_a, uint32_t body_b);
PHYSICS_API void enable_collision_pair(uint32_t body_a, uint32_t body_b);

/* ========================================================================= */
/*                          SCENE QUERIES & STREAMING                        */
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

PHYSICS_API bool sync_all_transforms_ex(
    BodyTransformEx* out_buffer,
    uint32_t buffer_capacity,
    uint32_t* out_written_count
);

PHYSICS_API uint32_t get_collision_events(
    CollisionEvent* out_events,
    uint32_t max_count
);

PHYSICS_API uint32_t get_collision_event_count(void);
PHYSICS_API void clear_collision_events(void);

/* ========================================================================= */
/*                      MULTI-WORLD INSTANCE API                             */
/* ========================================================================= */

typedef struct PhysicsWorld PhysicsWorld;

PHYSICS_API PhysicsWorld* create_physics_world(float gx, float gy, float gz);
PHYSICS_API void destroy_physics_world_instance(PhysicsWorld* world);
PHYSICS_API void step_physics_world(PhysicsWorld* world, float dt);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_ENGINE_H */
