/**
 * ============================================================================
 *  EngineLib (physics_engine.dll) - C99 Integration Sample
 * ============================================================================
 *  Demonstrates direct C integration by including physics_engine.h and linking
 *  against physics_engine.dll.
 *
 *  Compilation:
 *    gcc -O2 sample.c -L. -lphysics_engine -o sample_c.exe
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include "physics_engine.h"

int main(void) {
    printf("======================================================================\n");
    printf("   EngineLib Native DLL (physics_engine.dll) - C99 Sample\n");
    printf("======================================================================\n\n");

    // 1. Initialize World with standard gravity
    printf("[Step 1] Initializing Physics World (g = -9.81 m/s^2)...\n");
    init_physics_world(0.0f, -9.81f, 0.0f);
    set_world_substeps(4);
    set_solver_iterations(8);

    // 2. Add Static Floor (mass = 0.0f)
    printf("[Step 2] Spawning static ground floor...\n");
    uint32_t floor_id = add_rigid_body(
        0.0f, 0.0f, 0.0f,     // pos (x, y, z)
        40.0f, 1.0f, 40.0f,   // size (w, h, d)
        0.0f,                 // mass = 0 (STATIC)
        0.2f                  // restitution
    );
    printf("  Floor ID: %u\n", floor_id);

    // 3. Add Dynamic Sphere
    printf("[Step 3] Spawning dynamic sphere at Y = 5.0m...\n");
    uint32_t sphere_id = add_sphere_body(
        0.0f, 5.0f, 0.0f,     // pos (x, y, z)
        0.5f,                 // radius = 0.5m
        2.0f,                 // mass = 2.0 kg
        0.7f,                 // restitution = 0.70 (elastic)
        0.4f                  // friction = 0.40
    );
    printf("  Sphere ID: %u\n", sphere_id);

    // 4. Simulate 60 frames (1.0 second at 60 FPS)
    printf("\n[Step 4] Stepping simulation for 60 frames (dt = 1/60s)...\n");
    printf("----------------------------------------------------------------------\n");
    printf(" Frame |  Time  |    Position (X, Y, Z)     |    Velocity (X, Y, Z)\n");
    printf("----------------------------------------------------------------------\n");

    Vector3D pos = {0};
    Vector3D vel = {0};
    const float dt = 1.0f / 60.0f;

    for (int frame = 1; frame <= 60; ++frame) {
        step_physics(dt);

        get_body_position(sphere_id, &pos);
        get_body_velocity(sphere_id, &vel);

        if (frame % 10 == 0 || frame == 1) {
            float sim_time = frame * dt;
            printf(" %5d | %5.2fs | (%6.2f, %6.2f, %6.2f) | (%6.2f, %6.2f, %6.2f)\n",
                   frame, sim_time, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z);
        }
    }
    printf("----------------------------------------------------------------------\n");

    // 5. Scene Raycast Query
    printf("\n[Step 5] Performing Raycast Query from Y = 10.0m downward...\n");
    RaycastHitFfi hit = {0};
    bool has_hit = raycast(
        0.0f, 10.0f, 0.0f,    // origin
        0.0f, -1.0f, 0.0f,    // direction
        20.0f,                // max distance
        &hit
    );

    if (has_hit) {
        printf("  Raycast HIT! Body ID: %u at Distance: %.2fm\n", hit.body_id, hit.distance);
        printf("  Contact Point: (%.2f, %.2f, %.2f) | Normal: (%.2f, %.2f, %.2f)\n",
               hit.point.x, hit.point.y, hit.point.z,
               hit.normal.x, hit.normal.y, hit.normal.z);
    } else {
        printf("  Raycast: No hit.\n");
    }

    // 6. Cleanup
    printf("\n[Step 6] Releasing World & Cleaning Up...\n");
    destroy_physics_world();
    printf("[OK] C Sample executed successfully!\n");
    printf("======================================================================\n");

    return 0;
}
