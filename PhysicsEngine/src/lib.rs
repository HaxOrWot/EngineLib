pub mod math;
pub mod core;
pub mod dynamics;
pub mod collision;
pub mod solver;
pub mod ffi;

// Re-export core types for ergonomic Rust usage
pub use math::vec3::Vector3D;
pub use core::body::RigidBody3D;
pub use core::world::PhysicsWorld;
pub use dynamics::integrator::Integrator;
pub use collision::narrowphase::{ContactManifold, Narrowphase};
pub use solver::contact::ContactSolver;

// Re-export FFI entry points
pub use ffi::exports::*;
pub use ffi::types::*;
