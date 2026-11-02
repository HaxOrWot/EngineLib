/**
 * ============================================================================
 *  EngineLib (physics_engine.dll) - C++17 Sample
 * ============================================================================
 *  Demonstrates modern object-oriented RAII physics integration using
 *  EngineLib::PhysicsWorld.
 *
 *  Compilation:
 *    g++ -std=c++17 -O2 sample.cpp -L. -lphysics_engine -o sample_cpp.exe
 * ============================================================================
 */

#include <iostream>
#include <iomanip>
#include "physics_engine.hpp"

int main() {
    std::cout << "======================================================================\n";
    std::cout << "   EngineLib Native DLL (physics_engine.dll) - C++17 Sample\n";
    std::cout << "======================================================================\n\n";

    try {
        // 1. Initialize World via RAII
        std::cout << "[Step 1] Constructing EngineLib::PhysicsWorld (g = -9.81 m/s^2)...\n";
        EngineLib::PhysicsWorld world(0.0f, -9.81f, 0.0f);

        // 2. Add Static Floor (mass = 0.0f)
        std::cout << "[Step 2] Spawning static ground floor...\n";
        uint32_t floorId = world.createBox(
            EngineLib::Vec3(0.0f, 0.0f, 0.0f),
            EngineLib::Vec3(50.0f, 1.0f, 50.0f),
            0.0f, // Static
            0.2f  // Restitution
        );
        std::cout << "  Floor ID: " << floorId << "\n";

        // 3. Add Dynamic Sphere
        std::cout << "[Step 3] Spawning dynamic sphere at Y = 7.0m...\n";
        uint32_t sphereId = world.createSphere(
            EngineLib::Vec3(0.0f, 7.0f, 0.0f),
            0.5f,  // radius = 0.5m
            2.5f,  // mass = 2.5 kg
            0.75f, // restitution = 0.75 (elastic bounce)
            0.45f  // friction = 0.45
        );
        std::cout << "  Sphere ID: " << sphereId << "\n";

        // 4. Add Dynamic Box with Rotational Impulse
        std::cout << "[Step 4] Spawning dynamic box at Y = 9.0m with off-center torque...\n";
        uint32_t boxId = world.createBox(
            EngineLib::Vec3(2.0f, 9.0f, 0.0f),
            EngineLib::Vec3(1.0f, 2.0f, 1.0f),
            4.0f,  // mass = 4.0 kg
            0.2f
        );
        std::cout << "  Box ID: " << boxId << "\n";

        world.applyImpulseAtPoint(boxId, EngineLib::Vec3(12.0f, 0.0f, 0.0f), EngineLib::Vec3(2.0f, 9.8f, 0.0f));

        // 5. Simulate 90 frames (1.5 seconds at 60 Hz)
        std::cout << "\n[Step 5] Simulating 90 frames (60 FPS)...\n";
        std::cout << "----------------------------------------------------------------------\n";
        std::cout << " Frame |  Time  |    Sphere Pos Y    |    Sphere Vel Y    |   Box Rot Z   \n";
        std::cout << "----------------------------------------------------------------------\n";

        const float dt = 1.0f / 60.0f;
        for (int frame = 1; frame <= 90; ++frame) {
            world.step(dt);

            if (frame % 15 == 0 || frame == 1) {
                float time = frame * dt;
                auto sPos = world.getPosition(sphereId);
                auto sVel = world.getVelocity(sphereId);
                auto bRot = world.getRotation(boxId);

                std::cout << std::setw(6) << frame << " | "
                          << std::fixed << std::setprecision(2) << std::setw(5) << time << "s | "
                          << "   Y = " << std::setw(6) << (sPos ? sPos->y : 0.0f) << "m     | "
                          << "  Vy = " << std::setw(6) << (sVel ? sVel->y : 0.0f) << " m/s  | "
                          << " Rot = " << std::setw(6) << (bRot ? bRot->z : 0.0f) << " rad\n";
            }
        }
        std::cout << "----------------------------------------------------------------------\n";

        // 6. Raycast Query
        std::cout << "\n[Step 6] Performing downward Raycast Query from Y = 12.0m...\n";
        auto hit = world.castRay(EngineLib::Vec3(0.0f, 12.0f, 0.0f), EngineLib::Vec3(0.0f, -1.0f, 0.0f), 20.0f);
        if (hit) {
            std::cout << "  Raycast HIT! Body ID: " << hit->body_id << " at Distance: " << hit->distance << "m\n";
            std::cout << "  Impact Point: (" << hit->point.x << ", " << hit->point.y << ", " << hit->point.z << ")\n";
        } else {
            std::cout << "  Raycast: No hit.\n";
        }

        std::cout << "\n[Step 7] Destructor will automatically release native C memory.\n";
        std::cout << "[OK] C++ Sample executed successfully!\n";
        std::cout << "======================================================================\n";

    } catch (const std::exception& ex) {
        std::cerr << "[ERROR] Exception caught: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
