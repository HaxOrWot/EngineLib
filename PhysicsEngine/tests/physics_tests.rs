use physics_engine::{PhysicsWorld, Ray, Vector3D};

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

#[test]
fn test_sphere_floor_bounce() {
    let mut world = PhysicsWorld::new(0.0, -9.81, 0.0);

    // Floor at Y = 0.0 (top surface at Y = 0.5)
    world.add_body(
        Vector3D::new(0.0, 0.0, 0.0),
        Vector3D::new(20.0, 1.0, 20.0),
        0.0,
        0.85,
    );

    // Sphere dropped from Y = 8.0 with radius = 0.5
    let sphere_id = world.add_sphere(
        Vector3D::new(0.0, 8.0, 0.0),
        0.5,
        1.0,
        0.85,
        0.3,
    );

    let dt = 1.0 / 60.0;
    let mut bounced = false;
    let mut prev_y = 8.0;

    for _ in 0..120 {
        world.step(dt);
        let pos = world.get_body(sphere_id).unwrap().position;
        if pos.y > prev_y && pos.y < 3.0 {
            bounced = true;
            break;
        }
        prev_y = pos.y;
    }

    assert!(bounced, "Sphere failed to bounce off floor");
}

#[test]
fn test_raycast_scene_queries() {
    let mut world = PhysicsWorld::new(0.0, 0.0, 0.0);

    // Box at (0, 0, 10) of size (2, 2, 2)
    let box_id = world.add_body(
        Vector3D::new(0.0, 0.0, 10.0),
        Vector3D::new(2.0, 2.0, 2.0),
        0.0,
        0.5,
    );

    // Sphere at (5, 0, 10) of radius 1.0
    let sphere_id = world.add_sphere(
        Vector3D::new(5.0, 0.0, 10.0),
        1.0,
        0.0,
        0.5,
        0.3,
    );

    // Raycast towards the box: from origin (0, 0, 0) along +Z
    let ray1 = Ray::new(Vector3D::ZERO, Vector3D::new(0.0, 0.0, 1.0));
    let hit1 = world.raycast(&ray1, 50.0).expect("Ray should hit box");
    assert_eq!(hit1.body_id, box_id);
    // Front face of box is at Z = 10 - 1 = 9.0
    assert!((hit1.distance - 9.0).abs() < 0.05);

    // Raycast towards the sphere: from (5, 0, 0) along +Z
    let ray2 = Ray::new(Vector3D::new(5.0, 0.0, 0.0), Vector3D::new(0.0, 0.0, 1.0));
    let hit2 = world.raycast(&ray2, 50.0).expect("Ray should hit sphere");
    assert_eq!(hit2.body_id, sphere_id);
    // Front point of sphere is at Z = 10 - 1 = 9.0
    assert!((hit2.distance - 9.0).abs() < 0.05);
}

#[test]
fn test_spatial_hash_many_bodies() {
    let mut world = PhysicsWorld::new(0.0, -9.81, 0.0);

    // Spawn 50 dynamic bodies in a spatial grid
    for x in 0..5 {
        for z in 0..10 {
            world.add_body(
                Vector3D::new(x as f32 * 3.0, 10.0, z as f32 * 3.0),
                Vector3D::new(1.0, 1.0, 1.0),
                1.0,
                0.5,
            );
        }
    }

    assert_eq!(world.bodies.len(), 50);

    // Step world multiple times; spatial hash culls distant pairs
    for _ in 0..30 {
        world.step(1.0 / 60.0);
    }
}

#[test]
fn test_distance_joint_constraint() {
    let mut world = PhysicsWorld::new(0.0, 0.0, 0.0); // Zero gravity for pure constraint test
    world.substeps = 4;
    world.solver_iterations = 8;

    // Body A at (0, 0, 0)
    let body_a = world.add_body(
        Vector3D::new(0.0, 0.0, 0.0),
        Vector3D::new(1.0, 1.0, 1.0),
        1.0,
        0.5,
    );

    // Body B placed far away at (5.0, 0, 0)
    let body_b = world.add_body(
        Vector3D::new(5.0, 0.0, 0.0),
        Vector3D::new(1.0, 1.0, 1.0),
        1.0,
        0.5,
    );

    // Target joint distance = 2.0 with high stiffness
    world.add_distance_joint(
        body_a,
        body_b,
        Vector3D::ZERO,
        Vector3D::ZERO,
        2.0,
        0.95,
        0.2,
    );

    // Step physics for 30 frames
    for _ in 0..30 {
        world.step(1.0 / 60.0);
    }

    let pos_a = world.get_body(body_a).unwrap().position;
    let pos_b = world.get_body(body_b).unwrap().position;
    let final_distance = pos_a.distance(pos_b);

    // Assert that distance converged to 2.0 within tolerance
    assert!(
        (final_distance - 2.0).abs() < 0.15,
        "Joint failed to converge: distance={}, expected 2.0",
        final_distance
    );
}
