use physics_engine::{
    add_capsule_body, add_rigid_body, add_sphere_body, destroy_physics_world,
    init_physics_world, step_physics, sync_all_transforms_ex, BodyTransformEx,
    PhysicsWorld, Quaternion, Ray, Vector3D,
};

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

#[test]
fn test_sliding_friction_deceleration() {
    let mut world = PhysicsWorld::new(0.0, -9.81, 0.0);

    // Floor at Y = 0.0 (top surface at Y = 0.5)
    world.add_body(
        Vector3D::new(0.0, 0.0, 0.0),
        Vector3D::new(50.0, 1.0, 50.0),
        0.0,
        0.0,
    );

    // Sphere placed on the floor (Y = 1.0, radius = 0.5) with initial horizontal velocity
    let sphere_id = world.add_sphere(
        Vector3D::new(0.0, 1.0, 0.0),
        0.5,
        1.0,
        0.0,
        0.5, // friction 0.5
    );

    world.get_body_mut(sphere_id).unwrap().velocity = Vector3D::new(5.0, 0.0, 0.0);

    // Step physics for 120 frames (2.0 seconds)
    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }

    let b = world.get_body(sphere_id).unwrap();
    // Due to ground friction and rolling resistance, velocity.x should be significantly reduced (< 1.0 m/s)
    assert!(
        b.velocity.x < 1.0,
        "Friction failed to decelerate sliding sphere: velocity.x={}",
        b.velocity.x
    );
}

#[test]
fn test_capsule_floor_bounce() {
    let mut world = PhysicsWorld::new(0.0, -9.81, 0.0);

    // Immovable Floor at Y = 0 (top surface at Y = 0.5)
    world.add_body(
        Vector3D::new(0.0, 0.0, 0.0),
        Vector3D::new(20.0, 1.0, 20.0),
        0.0,
        0.7,
    );

    // Upright capsule dropped from Y = 6.0: radius = 0.5, height = 2.0
    let capsule_id = world.add_capsule(
        Vector3D::new(0.0, 6.0, 0.0),
        0.5,
        2.0,
        5.0,
        0.7,
        0.3,
    );

    let dt = 1.0 / 60.0;
    let mut bounced = false;
    let mut min_y = 6.0;

    for _ in 0..120 {
        world.step(dt);
        let b = world.get_body(capsule_id).unwrap();
        if b.position.y < min_y {
            min_y = b.position.y;
        } else if b.position.y > min_y + 0.1 && min_y < 3.0 {
            bounced = true;
            break;
        }
    }

    assert!(bounced, "Capsule failed to rebound from floor; min_y={}", min_y);
}

#[test]
fn test_capsule_capsule_collision() {
    let mut world = PhysicsWorld::new(0.0, 0.0, 0.0); // Zero-G

    // Capsule A at (-3, 0, 0) moving right at +5 m/s
    let cap_a = world.add_capsule(
        Vector3D::new(-3.0, 0.0, 0.0),
        0.5,
        2.0,
        2.0,
        0.8,
        0.2,
    );
    world.get_body_mut(cap_a).unwrap().velocity = Vector3D::new(5.0, 0.0, 0.0);

    // Capsule B at (+3, 0, 0) moving left at -5 m/s
    let cap_b = world.add_capsule(
        Vector3D::new(3.0, 0.0, 0.0),
        0.5,
        2.0,
        2.0,
        0.8,
        0.2,
    );
    world.get_body_mut(cap_b).unwrap().velocity = Vector3D::new(-5.0, 0.0, 0.0);

    let dt = 1.0 / 60.0;
    let mut rebounded = false;

    for _ in 0..60 {
        world.step(dt);
        let va = world.get_body(cap_a).unwrap().velocity.x;
        let vb = world.get_body(cap_b).unwrap().velocity.x;
        if va < 0.0 && vb > 0.0 {
            rebounded = true;
            break;
        }
    }

    assert!(rebounded, "Capsules failed to collide and rebound from each other");
}

#[test]
fn test_ccd_anti_tunneling() {
    let mut world = PhysicsWorld::new(0.0, 0.0, 0.0);

    // Thin immovable barrier at Z = 5.0 (thickness 0.2: min Z = 4.9, max Z = 5.1)
    world.add_body(
        Vector3D::new(0.0, 0.0, 5.0),
        Vector3D::new(20.0, 20.0, 0.2),
        0.0,
        0.5,
    );

    // High-speed projectile at Z = 0.0 traveling at 250 m/s along +Z
    let bullet = world.add_sphere(
        Vector3D::new(0.0, 0.0, 0.0),
        0.1,
        0.5,
        0.5,
        0.0,
    );
    world.get_body_mut(bullet).unwrap().velocity = Vector3D::new(0.0, 0.0, 250.0);
    world.get_body_mut(bullet).unwrap().enable_ccd = 1; // CCD active

    let dt = 1.0 / 60.0;
    for _ in 0..10 {
        world.step(dt);
    }

    let b = world.get_body(bullet).unwrap();
    // Projectile must be stopped/rebounded before or at the barrier face (Z <= 5.1)
    assert!(
        b.position.z <= 5.1,
        "Tunneling detected! Bullet tunneled through thin barrier to Z={}",
        b.position.z
    );
    assert!(
        b.velocity.z <= 0.0,
        "Bullet did not rebound from barrier: velocity.z={}",
        b.velocity.z
    );
}

#[test]
fn test_sensor_trigger_detection() {
    let mut world = PhysicsWorld::new(0.0, 0.0, 0.0);

    // Trigger volume at (0, 0, 5) with is_sensor = 1
    let trigger_id = world.add_body(
        Vector3D::new(0.0, 0.0, 5.0),
        Vector3D::new(4.0, 4.0, 4.0),
        0.0,
        0.0,
    );
    world.get_body_mut(trigger_id).unwrap().is_sensor = 1;

    // Moving sphere passes right through the trigger at 10 m/s
    let mover_id = world.add_sphere(
        Vector3D::new(0.0, 0.0, 0.0),
        0.5,
        1.0,
        0.0,
        0.0,
    );
    world.get_body_mut(mover_id).unwrap().velocity = Vector3D::new(0.0, 0.0, 10.0);

    let mut event_detected = false;
    let dt = 1.0 / 60.0;

    for _ in 0..60 {
        world.step(dt);
        for ev in &world.collision_events {
            if (ev.body_a == mover_id && ev.body_b == trigger_id)
                || (ev.body_a == trigger_id && ev.body_b == mover_id)
            {
                event_detected = true;
                assert_eq!(ev.is_sensor, 1, "Event should be marked as sensor trigger");
                assert_eq!(ev.impulse, 0.0, "Sensor must not impart physical impulse");
            }
        }
    }

    assert!(event_detected, "Trigger zone failed to detect passing body");

    // Ensure mover passed completely through without being slowed down
    let final_pos = world.get_body(mover_id).unwrap().position;
    assert!(
        final_pos.z > 8.0,
        "Body was impeded by sensor: final_z={}, expected > 8.0",
        final_pos.z
    );
}

#[test]
fn test_quaternion_rotational_dynamics() {
    let mut world = PhysicsWorld::new(0.0, 0.0, 0.0);

    let id = world.add_body(
        Vector3D::ZERO,
        Vector3D::new(1.0, 1.0, 1.0),
        1.0,
        0.5,
    );

    // Apply high angular velocity around an arbitrary 3D axis
    let ang_vel = Vector3D::new(10.0, 20.0, 15.0);
    world.get_body_mut(id).unwrap().angular_velocity = ang_vel;

    let dt = 1.0 / 60.0;
    for _ in 0..120 {
        world.step(dt);
        let b = world.get_body(id).unwrap();
        // Quaternion must preserve exact unit length (drift-free)
        let q_len = b.orientation.length();
        assert!(
            (q_len - 1.0).abs() < 1e-4,
            "Quaternion normalization drifted: length={}",
            q_len
        );
    }
}

#[test]
fn test_batch_transforms_ex_ffi() {
    init_physics_world(0.0, -9.81, 0.0);

    let id1 = add_rigid_body(0.0, 10.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.5);
    let id2 = add_sphere_body(5.0, 20.0, 0.0, 0.5, 2.0, 0.5, 0.3);
    let id3 = add_capsule_body(10.0, 15.0, 0.0, 0.5, 2.0, 3.0, 0.5, 0.3);

    step_physics(1.0 / 60.0);

    let mut buffer = [BodyTransformEx {
        id: 0,
        position: Vector3D::ZERO,
        orientation: Quaternion::IDENTITY,
    }; 16];
    let mut written_count = 0u32;

    let success = unsafe {
        sync_all_transforms_ex(buffer.as_mut_ptr(), 16, &mut written_count)
    };

    assert!(success);
    assert_eq!(written_count, 3);

    let ids: Vec<u32> = (0..written_count as usize).map(|i| buffer[i].id).collect();
    assert!(ids.contains(&id1));
    assert!(ids.contains(&id2));
    assert!(ids.contains(&id3));

    // Verify orientations are normalized
    for i in 0..written_count as usize {
        let q = buffer[i].orientation;
        assert!((q.length() - 1.0).abs() < 1e-4);
    }

    destroy_physics_world();
}

#[test]
fn test_collision_event_telemetry() {
    let mut world = PhysicsWorld::new(0.0, 0.0, 0.0);

    let a = world.add_sphere(Vector3D::new(-2.0, 0.0, 0.0), 1.0, 2.0, 1.0, 0.0);
    let b = world.add_sphere(Vector3D::new(2.0, 0.0, 0.0), 1.0, 2.0, 1.0, 0.0);

    world.get_body_mut(a).unwrap().velocity = Vector3D::new(10.0, 0.0, 0.0);
    world.get_body_mut(b).unwrap().velocity = Vector3D::new(-10.0, 0.0, 0.0);

    let mut event_found = false;
    for _ in 0..30 {
        world.step(1.0 / 60.0);
        if !world.collision_events.is_empty() {
            let ev = &world.collision_events[0];
            assert!(ev.body_a == a || ev.body_b == a);
            assert!(ev.body_a == b || ev.body_b == b);
            assert!(ev.impulse > 0.0, "Impulse should be positive: {}", ev.impulse);
            assert!((ev.contact_point.x.abs()) < 0.5, "Contact point near origin");
            event_found = true;
            break;
        }
    }

    assert!(event_found, "Collision event was not recorded");
}

