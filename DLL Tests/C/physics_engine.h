/**
 * @file physics_engine.h
 * @brief Universal Cross-Language 3D Kinematic & Rigid Body Physics Engine C-ABI Header
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
 * @brief 3D Single-Precision Cartesian Vector (12 bytes, 4-byte aligned)
 */
typedef struct {
    float x;
    float y;
    float z;
} Vector3D;

/**
 * @brief 3D Rigid Body Data Structure (64 bytes, 4-byte aligned)
 */
typedef struct {
    uint32_t id;
    Vector3D position;
    Vector3D velocity;
    Vector3D acceleration;
    Vector3D size;
    float mass;
    float inverse_mass;
    float restitution;
    float friction;
    float radius;
    uint32_t shape_type; // 0 = AABB, 1 = Sphere
    uint32_t is_sleeping;
    float sleep_timer;
} RigidBody3D;

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

/**
 * @brief Initializes or resets the global physics world.
 * @param gx Gravity along X axis (m/s^2)
 * @param gy Gravity along Y axis (m/s^2, e.g. -9.81)
 * @param gz Gravity along Z axis (m/s^2)
 */
PHYSICS_API void init_physics_world(float gx, float gy, float gz);

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
 * @brief Sets the Coulomb friction coefficient of a body.
 */
PHYSICS_API void set_body_friction(uint32_t id, float friction);

/**
 * @brief Checks if a body is currently in an energy-conserving sleep state.
 */
PHYSICS_API bool is_body_sleeping(uint32_t id);

/**
 * @brief Forces a sleeping body awake.
 */
PHYSICS_API void wake_body(uint32_t id);

/**
 * @brief Reads the current world position of a body.
 * @param id Body entity ID
 * @param out_pos Pointer to destination Vector3D buffer
 * @return true if successful, false if body not found or pointer is null
 */
PHYSICS_API bool get_body_position(uint32_t id, Vector3D* out_pos);

/**
 * @brief Applies an instantaneous force vector for the current simulation frame.
 * @param id Body entity ID
 * @param fx, fy, fz Force components in Newtons
 */
PHYSICS_API void apply_force(uint32_t id, float fx, float fy, float fz);

/**
 * @brief Advances the simulation by a discrete timestep dt.
 * @param dt Timestep in seconds (e.g. 0.016667 for 60 Hz)
 */
PHYSICS_API void step_physics(float dt);

/**
 * @brief Casts a ray through the scene and finds the nearest hit.
 */
PHYSICS_API bool raycast(
    float ox, float oy, float oz,
    float dx, float dy, float dz,
    float max_dist,
    RaycastHitFfi* out_hit
);

/**
 * @brief Streams all body transforms in a single zero-copy batch call.
 */
PHYSICS_API bool sync_all_transforms(
    BodyTransform* out_buffer,
    uint32_t buffer_capacity,
    uint32_t* out_written_count
);

/**
 * @brief Removes an individual body from simulation.
 */
PHYSICS_API bool remove_rigid_body(uint32_t id);

/**
 * @brief Tears down the physics world and releases all memory.
 */
PHYSICS_API void destroy_physics_world(void);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_ENGINE_H */
