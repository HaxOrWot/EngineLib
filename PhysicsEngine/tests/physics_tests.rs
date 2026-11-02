use physics_engine::{PhysicsWorld, Vector3D};

#[test]
fn test_gravitational_freefall() {
    let mut world = PhysicsWorld::new(0.0, -9.81, 0.0);
    let id = world.add_body(
        Vector3D::new(0.0, 100.0, 0.0),
        Vector3D::new(1.0, 1.0, 1.0),
        10.0,
        0.5,
    );

    let dt = 1.0 / 60.0;
    let total_steps = 60; // 1.0 second of simulation

    for _ in 0..total_steps {
        world.step(dt);
    }

    let body = world.get_body(id).expect("Body must exist");

    // Theoretical velocity after 1.0s: -9.81 m/s
    assert!(
        (body.velocity.y - (-9.81)).abs() < 0.05,
        "Velocity deviation too high: actual={}, expected=-9.81",
        body.velocity.y
    );

    // Theoretical position after 1.0s: 100 - 0.5 * 9.81 * 1.0^2 = 95.095 m
    assert!(
        (body.position.y - 95.095).abs() < 0.20,
        "Position deviation too high: actual={}, expected=95.095",
        body.position.y
    );
}

#[test]
fn test_elastic_restitution() {
    let mut world = PhysicsWorld::new(0.0, 0.0, 0.0); // Zero gravity

    // Static immovable wall at X = 0 (size 2 x 20 x 20)
    world.add_body(
        Vector3D::new(0.0, 0.0, 0.0),
        Vector3D::new(2.0, 20.0, 20.0),
        0.0, // mass 0 = immovable
        0.8, // restitution 0.8
    );

    // Dynamic body moving towards wall at -10 m/s with restitution 0.8
    let mover = world.add_body(
        Vector3D::new(5.0, 0.0, 0.0),
        Vector3D::new(1.0, 1.0, 1.0),
        1.0,
        0.8,
    );
    world.get_body_mut(mover).unwrap().velocity = Vector3D::new(-10.0, 0.0, 0.0);

    let dt = 0.01;
    let mut rebounded = false;

    for _ in 0..100 {
        world.step(dt);
        let b = world.get_body(mover).unwrap();
        if b.velocity.x > 0.0 {
            rebounded = true;
            // Expected rebound velocity: ~8.0 m/s (0.8 * 10.0)
            assert!(
                (b.velocity.x - 8.0).abs() < 0.5,
                "Rebound velocity outside tolerance: actual={}, expected=8.0",
                b.velocity.x
            );
            break;
        }
    }

    assert!(rebounded, "Body failed to rebound from wall");
}

#[test]
fn test_floor_penetration_prevention() {
    let mut world = PhysicsWorld::new(0.0, -9.81, 0.0);

    // Floor top surface is at Y = 0.5 (center at 0.0, size Y = 1.0)
    world.add_body(
        Vector3D::new(0.0, 0.0, 0.0),
        Vector3D::new(20.0, 1.0, 20.0),
        0.0,
        0.0,
    );

    // Box resting on floor (bottom at Y = 0.5, center at Y = 1.0, size Y = 1.0)
    let box_id = world.add_body(
        Vector3D::new(0.0, 1.0, 0.0),
        Vector3D::new(1.0, 1.0, 1.0),
        100.0,
        0.0,
    );

    // Run 300 physics frames (5 seconds of simulated time)
    for _ in 0..300 {
        world.step(1.0 / 60.0);
    }

    let b = world.get_body(box_id).unwrap();
    let box_bottom = b.position.y - 0.5;

    // Box bottom must not sink significantly below floor surface (Y = 0.5)
    assert!(
        box_bottom >= 0.45,
        "Box sank through floor: bottom at {}, expected >= 0.45",
        box_bottom
    );
}
