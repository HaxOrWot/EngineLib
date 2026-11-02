use std::collections::{HashMap, HashSet};
use crate::core::body::RigidBody3D;

pub struct SpatialHash {
    pub cell_size: f32,
    grid: HashMap<(i32, i32, i32), Vec<u32>>,
}

impl SpatialHash {
    pub fn new(cell_size: f32) -> Self {
        Self {
            cell_size: if cell_size > 0.1 { cell_size } else { 4.0 },
            grid: HashMap::new(),
        }
    }

    /// Clears and inserts all active bodies, returning unique candidate collision pairs (id_a < id_b)
    pub fn generate_candidate_pairs(
        &mut self,
        bodies: &HashMap<u32, RigidBody3D>,
    ) -> Vec<(u32, u32)> {
        self.grid.clear();
        let inv_cell = 1.0 / self.cell_size;

        // Register each body into all touched grid cells
        for (id, body) in bodies.iter() {
            let (min, max) = body.aabb_bounds();

            let min_cx = (min.x * inv_cell).floor() as i32;
            let max_cx = (max.x * inv_cell).floor() as i32;
            let min_cy = (min.y * inv_cell).floor() as i32;
            let max_cy = (max.y * inv_cell).floor() as i32;
            let min_cz = (min.z * inv_cell).floor() as i32;
            let max_cz = (max.z * inv_cell).floor() as i32;

            for cx in min_cx..=max_cx {
                for cy in min_cy..=max_cy {
                    for cz in min_cz..=max_cz {
                        self.grid.entry((cx, cy, cz)).or_default().push(*id);
                    }
                }
            }
        }

        // Collect unique candidate pairs
        let mut candidate_set = HashSet::new();
        for cell_bodies in self.grid.values() {
            let count = cell_bodies.len();
            if count < 2 {
                continue;
            }
            for i in 0..count {
                for j in (i + 1)..count {
                    let a = cell_bodies[i];
                    let b = cell_bodies[j];
                    if a != b {
                        let pair = if a < b { (a, b) } else { (b, a) };
                        candidate_set.insert(pair);
                    }
                }
            }
        }

        candidate_set.into_iter().collect()
    }
}
