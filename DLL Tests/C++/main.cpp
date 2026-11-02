#include <iostream>
#include <iomanip>
#include "physics_engine.hpp"

int main() {
    std::cout << "=================================================================\n";
    std::cout << "  Modern C++ (RAII) Test Harness for Physics Engine DLL\n";
    std::cout << "=================================================================\n";

    try {
        // Instantiate Physics World via RAII wrapper
        Physics::World world(0.0f, -9.81f, 0.0f);
        std::cout << "[+] Initialized C++ Physics::World (Earth Gravity: -9.81 m/s^2 on Y)\n";

        // Add static floor at Y = 0.0
        uint32_t floorId = world.addBody(
            Physics::Vec3(0.0f, 0.0f, 0.0f),
            Physics::Vec3(30.0f, 1.0f, 30.0f),
            0.0f, // Mass 0 = static terrain
            0.75f // Restitution
        );
        std::cout << "[+] Created Static Floor (ID: " << floorId << ") at Y = 0.0\n";

        // Add Box 1 (High drop: Y = 12.0)
        uint32_t box1 = world.addBody(
            Physics::Vec3(-2.0f, 12.0f, 0.0f),
            Physics::Vec3(1.0f, 1.0f, 1.0f),
            1.0f, 0.85f
        );

        // Add Box 2 (Low drop: Y = 6.0)
        uint32_t box2 = world.addBody(
            Physics::Vec3(2.0f, 6.0f, 0.0f),
            Physics::Vec3(1.0f, 1.0f, 1.0f),
            2.0f, 0.70f
        );

        std::cout << "[+] Created Box 1 (ID: " << box1 << ") at (-2.0, 12.0, 0.0) [Mass: 1.0kg, Restitution: 0.85]\n";
        std::cout << "[+] Created Box 2 (ID: " << box2 << ") at ( 2.0,  6.0, 0.0) [Mass: 2.0kg, Restitution: 0.70]\n";

        const float dt = 1.0f / 60.0f;
        bool box1Bounced = false;
        bool box2Bounced = false;
        float prevY1 = 12.0f, prevY2 = 6.0f;

        std::cout << "\n--- Stepping Simulation (140 Frames @ 60 Hz) ---\n";
        for (int frame = 1; frame <= 140; ++frame) {
            world.step(dt);

            auto pos1 = world.getPosition(box1);
            auto pos2 = world.getPosition(box2);

            if (!pos1 || !pos2) {
                std::cerr << "[-] Error: Failed to retrieve positions!\n";
                return 1;
            }

            // Check bounce conditions
            if (!box2Bounced && pos2->y > prevY2 && pos2->y < 2.0f) {
                box2Bounced = true;
                std::cout << ">>> [BOX 2 BOUNCE] Frame " << std::setw(3) << frame 
                          << " (t=" << std::fixed << std::setprecision(2) << frame * dt << "s)"
                          << " rebounded from Y = " << std::setprecision(3) << prevY2 << " m <<<\n";
            }

            if (!box1Bounced && pos1->y > prevY1 && pos1->y < 2.0f) {
                box1Bounced = true;
                std::cout << ">>> [BOX 1 BOUNCE] Frame " << std::setw(3) << frame 
                          << " (t=" << std::fixed << std::setprecision(2) << frame * dt << "s)"
                          << " rebounded from Y = " << std::setprecision(3) << prevY1 << " m <<<\n";
            }

            if (frame % 25 == 0 || frame == 1) {
                std::cout << "  Frame " << std::setw(3) << frame 
                          << " | t=" << std::fixed << std::setprecision(2) << frame * dt << "s"
                          << " | Box1 Y: " << std::setw(6) << std::setprecision(2) << pos1->y
                          << " | Box2 Y: " << std::setw(6) << std::setprecision(2) << pos2->y << "\n";
            }

            prevY1 = pos1->y;
            prevY2 = pos2->y;
        }

        std::cout << "\n=================================================================\n";
        if (box1Bounced && box2Bounced) {
            std::cout << "  RESULT: SUCCESS - Multi-body C++ simulation verified!\n";
            std::cout << "=================================================================\n";
            return 0;
        } else {
            std::cout << "  RESULT: FAILED - One or more bodies failed to bounce!\n";
            std::cout << "=================================================================\n";
            return 1;
        }

    } catch (const std::exception& ex) {
        std::cerr << "[-] Exception caught: " << ex.what() << "\n";
        return 1;
    }
}
