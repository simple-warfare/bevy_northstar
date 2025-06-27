use bevy::math::UVec3;

use crate::prelude::ORDINAL_3D_OFFSETS;

/// [`Point`] represents a single position on the grid.
#[derive(Debug, Default, Clone)]
pub struct Point {
    /// The movement cost associated with this point.
    pub cost: u32,
    /// Solid will block all movement, aka wall.
    pub solid: bool,
    /// Ramp will allow movement up or down.
    pub ramp: bool,
    // Cached neighbors to the point
    pub(crate) neighbor_bits: u32,
}

impl Point {
    pub fn new(cost: u32, solid: bool) -> Self {
        Point {
            cost,
            solid,
            ramp: false,
            neighbor_bits: 0,
        }
    }

    pub fn neighbor_iter(&self, pos: UVec3) -> impl Iterator<Item = UVec3> + '_ {
        let origin = pos.as_ivec3();
        ORDINAL_3D_OFFSETS
            .iter()
            .enumerate()
            .filter_map(move |(i, offset)| {
                if (self.neighbor_bits >> i) & 1 != 0 {
                    Some((origin + *offset).as_uvec3())
                } else {
                    None
                }
            })
    }
}