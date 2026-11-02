#include <stdio.h>
#include <stdbool.h>
#include "physics_engine.h"

int main(void) {
    printf("=================================================================\n");
    printf("  Native C Test Harness for Physics Engine Dynamic Library (.DLL)\n");
    printf("=================================================================\n");

    // 1. Initialize world with Earth gravity (-9.81 m/s^2 on Y)
    printf("[*] Initializing physics world with gravity (0.0, -9.81, 0.0)...\n");
    init_physics_world(0.0f, -9.81f, 0.0f);

    // 2. Add static floor at Y=0.0 (mass = 0.0)
    uint32_t floor_id = add_rigid_body(
        0.0f, 0.0f, 0.0f,   // Position
        20.0f, 1.0f, 20.0f, // Size (Floor surface is at Y = 0.5)
        0.0f,               // Mass = 0.0 (static)
        0.80f               // Restitution
    );
    printf("[+] Static floor created with ID: %u at Y = 0.0\n", floor_id);

    // 3. Add falling box at Y=10.0 (mass = 1.0, restitution = 0.85)
    uint32_t box_id = add_rigid_body(
        0.0f, 10.0f, 0.0f,  // Position
        1.0f, 1.0f, 1.0f,   // Size
        1.0f,               // Mass = 1.0 kg
        0.85f               // Restitution
    );
    printf("[+] Dynamic falling box created with ID: %u at Y = 10.0\n", box_id);

    // 4. Run simulation loop for 120 steps @ 60 FPS (dt = 1/60s)
    const float dt = 1.0f / 60.0f;
    Vector3D pos = {0.0f, 0.0f, 0.0f};
    bool bounced = false;
    float prev_y = 10.0f;
    float lowest_y = 10.0f;

    printf("\n--- Starting Simulation (120 Steps @ 60 Hz) ---\n");
    for (int step = 1; step <= 120; step++) {
        step_physics(dt);
        
        if (!get_body_position(box_id, &pos)) {
            fprintf(stderr, "[-] Error: Failed to retrieve body position!\n");
            return 1;
        }

        if (pos.y < lowest_y) {
            lowest_y = pos.y;
        }

        // Detect bounce: when Y velocity reverses from floor impact
        if (!bounced && pos.y > prev_y && pos.y < 3.0f) {
            bounced = true;
            printf(">>> [BOUNCE DETECTED at Step %03d (t=%.2fs)] Rebounded to Y = %.3f m (Lowest: %.3f m) <<<\n",
                   step, step * dt, pos.y, lowest_y);
        }

        if (step % 15 == 0 || step == 1) {
            printf("  Step %03d | Time: %.3fs | Box Pos: (%.2f, %.3f, %.2f)\n",
                   step, step * dt, pos.x, pos.y, pos.z);
        }

        prev_y = pos.y;
    }

    printf("\n=================================================================\n");
    if (bounced) {
        printf("  RESULT: SUCCESS - Native C compilation & FFI linking verified!\n");
        printf("=================================================================\n");
        return 0;
    } else {
        printf("  RESULT: FAILED - Object did not rebound from the floor!\n");
        printf("=================================================================\n");
        return 1;
    }
}
