//! This module defines the `Neighborhood` trait and its implementations for different types of neighborhoods.
use bevy::math::{IVec3, UVec3};
use ndarray::ArrayView3;
use std::{fmt::Debug, sync::Arc};

use crate::{grid::NeighborhoodSettings, point::Point};

/// The `Neighborhood` trait defines the interface for different neighborhood types.
/// You can implement this trait to define custom neighborhoods and hueristics.
pub trait Neighborhood: Clone + Default + Sync + Send {
    /// Returns the possible directions at a position by the neighborhood.
    fn directions(&self) -> &'static [(i32, i32, i32)];

    /// Returns a u32 bitmask representing the neighbors of a given position in the grid.
    /// If you implement this method, you will need to ensure that the returned bit mask is in the correct order.
    /// The default implementation should handle almost all directional movement cases, you can likely use `filter_neighbors` instead
    /// if you want customer neighbor logic. It should handle most custom cases.
    /// 
    /// You will also need to ensure the neighbors are within the bounds of the grid.
    /// 
    /// Bit Mask Format:
    /// Index: Offset (x, y, z)
    /// ------ -----------------
    ///   0     (-1, -1, -1) ← North-West-Down
    ///   1     ( 0, -1, -1) ← North-Down
    ///   2     ( 1, -1, -1) ← South-West-Down
    ///   3     (-1,  0, -1) ← West-Down
    ///   4     ( 0,  0, -1) ← Down
    ///   5     ( 1,  0, -1) ← East-Down
    ///   6     (-1,  1, -1) ← North-East-Down
    ///   7     ( 0,  1, -1) ← North-Down
    ///   8     ( 1,  1, -1) ← South-East-Down
    ///   9     (-1, -1,  0) ← North-West
    ///  10     ( 0, -1,  0) ← North
    ///  11     ( 1, -1,  0) ← South-West
    ///  12     (-1,  0,  0) ← West
    ///  13     ( 1,  0,  0) ← East
    ///  14     (-1,  1,  0) ← North-East
    ///  15     ( 0,  1,  0) ← South
    ///  16     ( 1,  1,  0) ← South-East
    ///  17     (-1, -1,  1) ← North-West-Up
    ///  18     ( 0, -1,  1) ← North-Up
    ///  19     ( 1, -1,  1) ← South-West-Up
    ///  20     (-1,  0,  1) ← West-Up
    ///  21     ( 0,  0,  1) ← Up
    ///  22     ( 1,  0,  1) ← East-Up
    ///  23     (-1,  1,  1) ← North-East-Up
    ///  24     ( 0,  1,  1) ← South-Up
    ///  25     ( 1,  1,  1) ← South-East-Up
    fn neighbors(&self, grid: &ArrayView3<Point>, pos: UVec3) -> u32 {
        let shape = grid.shape();
        let mut bits = 0;
        let origin = pos.as_ivec3();

        for &dir in self.directions() {
            let offset = IVec3::new(dir.0, dir.1, dir.2);
            if let Some(i) = ORDINAL_3D_OFFSETS.iter().position(|&o| o == offset) {
                let neighbor = origin + offset;

                if neighbor.cmplt(IVec3::ZERO).any() {
                    continue;
                }

                let nx = neighbor.x as usize;
                let ny = neighbor.y as usize;
                let nz = neighbor.z as usize;

                if nx < shape[0] && ny < shape[1] && nz < shape[2] {
                    if !grid[[nx, ny, nz]].solid {
                        bits |= 1 << i;
                    }
                }
            } else {
                panic!("Direction {:?} not in ORDINAL_3D_OFFSETS!", offset);
            }
        }

        // Apply filters
        for filter in self.filters() {
            bits = filter.filter(pos, bits, grid);
        }

        bits
    }

    /// Returns the filters applied to the neighborhood.
    fn filters(&self) -> &[Arc<dyn NeighborFilter + Send + Sync + 'static>];

    /// Returns the heuristic cost from a position to a target.
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32;

    /// Returns true if the neighborhood allows ordinal movement.
    fn is_ordinal(&self) -> bool {
        false
    }
    /// Returns any settings for the neighborhood.
    fn settings(&self) -> Option<NeighborhoodSettings> {
        None
    }

    /// Optional constructor override for settings-based neighborhood
    fn from_settings(_settings: &NeighborhoodSettings) -> Self {
        Self::default()
    }
}

/// `NeighborFilter` is used to add custom filtering logic to the neighbors returned by the `Neighborhood::neighbors` method.
pub trait NeighborFilter {
    fn filter(&self, pos: UVec3, mask: u32, grid: &ArrayView3<Point>) -> u32;
}

/// Use `CardinalNeighborhood` for standard 2D pathfinding with no diagonal movement.
#[derive(Clone, Default)]
pub struct CardinalNeighborhood {
    filters: Vec<Arc<dyn NeighborFilter + Send + Sync + 'static>>,
}

impl Neighborhood for CardinalNeighborhood {
    #[inline(always)]
    fn directions(&self) -> &'static [(i32, i32, i32)] {
        // The third coordinate is always 0 for 2D neighborhoods.
        static DIRECTIONS: [(i32, i32, i32); 4] = [
            (-1, 0, 0), // North
            (1, 0, 0),  // South
            (0, -1, 0), // West
            (0, 1, 0),  // East
        ];
        &DIRECTIONS
    }

    /*#[inline(always)]
    fn neighbors(&self, grid: &ArrayView3<Point>, pos: UVec3) -> u32 {
        let x = pos.x as i32;
        let y = pos.y as i32;
        let z = pos.z as i32;

        let mut bits: u32 = 0;

        for (i, &(dx, dy, dz)) in self.directions().iter().enumerate() {
            let nx = x + dx;
            let ny = y + dy;
            let nz = z + dz;

            if nx >= 0 && ny >= 0 && nz >= 0 {
                let nx = nx as usize;
                let ny = ny as usize;
                let nz = nz as usize;

                let shape = grid.shape();
                if nx < shape[0] && ny < shape[1] && nz < shape[2] {
                    if !grid[[nx, ny, nz]].solid {
                        bits |= 1 << i;
                    }
                }
            }
        }

        bits
    }*/

    #[inline(always)]
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        ((pos.x as i32 - target.x as i32).abs() + (pos.y as i32 - target.y as i32).abs()) as u32
    }

    fn filters(&self) -> &[Arc<dyn NeighborFilter + Send + Sync + 'static>] {
        &self.filters
    }
}

/// Use `CardinalNeighborhood3d` for 3D pathfinding with no diagonal movement.
/// This neighborhood allows movement in the cardinal directions in 3D space only in UP or DOWN directions.
#[derive(Clone, Copy, Debug, Default)]
pub struct CardinalNeighborhood3d;

impl Neighborhood for CardinalNeighborhood3d {
    #[inline(always)]
    fn directions(&self) -> &'static [(i32, i32, i32)] {
        // Cardinal directions in 3D: N, S, E, W, UP, DOWN
        // The third coordinate is always 0 for 2D neighborhoods.
        static DIRECTIONS: [(i32, i32, i32); 6] = [
            (-1, 0, 0), // North
            (1, 0, 0),  // South
            (0, -1, 0), // West
            (0, 1, 0),  // East
            (0, 0, -1), // Down
            (0, 0, 1),  // Up
        ];
        &DIRECTIONS
    }

    /*#[inline(always)]
    fn neighbors(&self, grid: &ArrayView3<Point>, pos: UVec3) -> u32 {
        let x = pos.x as i32;
        let y = pos.y as i32;
        let z = pos.z as i32;

        let mut bits: u32 = 0;

        for (i, &(dx, dy, dz)) in self.directions().iter().enumerate() {
            let nx = x + dx;
            let ny = y + dy;
            let nz = z + dz;

            if nx >= 0 && ny >= 0 && nz >= 0 {
                let nx = nx as usize;
                let ny = ny as usize;
                let nz = nz as usize;

                let shape = grid.shape();
                if nx < shape[0] && ny < shape[1] && nz < shape[2] {
                    if !grid[[nx, ny, nz]].solid {
                        bits |= 1 << i;
                    }
                }
            }
        }

        bits
    }*/

    #[inline(always)]
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        let dx = pos.x.max(target.x) - pos.x.min(target.x);
        let dy = pos.y.max(target.y) - pos.y.min(target.y);
        let dz = pos.z.max(target.z) - pos.z.min(target.z);
        dx + dy + dz
    }

    fn filters(&self) -> &[Arc<dyn NeighborFilter + Send + Sync + 'static>] {
        &[]
    }
}

/// Use `OrdinalNeighborhood` for 2D pathfinding with diagonal movement.
/// This neighborhood allows movement in all 8 directions.
#[derive(Clone, Default)]
pub struct OrdinalNeighborhood {
    filters: Vec<Arc<dyn NeighborFilter + Send + Sync + 'static>>,
}

impl Neighborhood for OrdinalNeighborhood {
    #[inline(always)]
    fn directions(&self) -> &'static [(i32, i32, i32)] {
        // Ordinal directions in 2D: N, S, E, W, NE, SE, SW, NW
        // The third coordinate is always 0 for 2D neighborhoods.
        static DIRECTIONS: [(i32, i32, i32); 8] = [
            (-1, 0, 0),  // North
            (1, 0, 0),   // South
            (0, -1, 0),  // West
            (0, 1, 0),   // East
            (-1, -1, 0), // North-East
            (1, -1, 0),  // South-East
            (1, 1, 0),   // South-West
            (-1, 1, 0),  // North-West
        ];
        &DIRECTIONS
    }

    /*#[inline(always)]
    fn neighbors(&self, grid: &ArrayView3<Point>, pos: UVec3) -> u32 {
        let x = pos.x as i32;
        let y = pos.y as i32;
        let z = pos.z as usize;

        let shape = grid.shape();
        let (max_x, max_y) = (shape[0] as u32, shape[1] as u32);

        let mut mask: u32 = 0;
        let mut cardinal_mask: u32 = 0;

        for (i, &(dx, dy, _)) in self.directions().iter().enumerate() {
            let nx = x + dx;
            let ny = y + dy;

            if nx < 0 || ny < 0 {
                continue;
            }

            let nxu = nx as u32;
            let nyu = ny as u32;

            if nxu >= max_x || nyu >= max_y {
                continue;
            }

            let solid = grid[[nxu as usize, nyu as usize, z]].solid;
            if solid {
                continue;
            }

            let bit = 1u32 << i;
            if i < 4 {
                cardinal_mask |= bit;
            } else {
                mask |= bit;
            }
        }

        // Filter diagonals using cardinal_mask
        if !self.allow_corner_clipping {
            // Diagonal 0 (NE): needs N (bit 0) and E (bit 1)
            if (cardinal_mask & (1 << 0 | 1 << 1)) != (1 << 0 | 1 << 1) {
                mask &= !(1 << 4);
            }
            // Diagonal 1 (SE): S and E
            if (cardinal_mask & (1 << 2 | 1 << 1)) != (1 << 2 | 1 << 1) {
                mask &= !(1 << 5);
            }
            // Diagonal 2 (SW): S and W
            if (cardinal_mask & (1 << 2 | 1 << 3)) != (1 << 2 | 1 << 3) {
                mask &= !(1 << 6);
            }
            // Diagonal 3 (NW): N and W
            if (cardinal_mask & (1 << 0 | 1 << 3)) != (1 << 0 | 1 << 3) {
                mask &= !(1 << 7);
            }
        }

        cardinal_mask | mask
    }*/

    #[inline(always)]
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        let dx = pos.x.abs_diff(target.x);
        let dy = pos.y.abs_diff(target.y);
        dx.max(dy)
    }

    #[inline(always)]
    fn is_ordinal(&self) -> bool {
        true
    }

    fn from_settings(settings: &NeighborhoodSettings) -> Self {
        Self {
            filters: settings.filters.clone(),
        }
    }

    #[inline(always)]
    fn settings(&self) -> Option<NeighborhoodSettings> {
        Some(NeighborhoodSettings {
            filters: self.filters.clone(),
        })
    }

    fn filters(&self) -> &[Arc<dyn NeighborFilter + Send + Sync + 'static>] {
        &self.filters
    }
}

/// Use `OrdinalNeighborhood3d` for 3D pathfinding with diagonal movement.
/// This neighborhood allows movement in all 26 directions.
/// It's the 3D version of `OrdinalNeighborhood`.
#[derive(Clone, Copy, Debug, Default)]
pub struct OrdinalNeighborhood3d;

impl Neighborhood for OrdinalNeighborhood3d {
    #[inline(always)]
    fn directions(&self) -> &'static [(i32, i32, i32)] {
        // Ordinal directions in 3D: N, S, E, W, NE, SE, SW, NW, UP, DOWN
        // The third coordinate is always 0 for 2D neighborhoods.
        static DIRECTIONS: [(i32, i32, i32); 26] = [
            (-1, -1, -1), // North-West-Down
            (-1, -1, 0),  // North-West
            (-1, -1, 1),  // North-West-Up
            (-1, 0, -1),  // North-Down
            (-1, 0, 0),   // North
            (-1, 0, 1),   // North-Up
            (-1, 1, -1),  // North-East-Down
            (-1, 1, 0),   // North-East
            (-1, 1, 1),   // North-East-Up
            (0, -1, -1),  // West-Down
            (0, -1, 0),   // West
            (0, -1, 1),   // West-Up
            (0, 0, -1),   // Down
            (0, 0, 1),    // Up
            (0, 1, -1),   // East-Down
            (0, 1, 0),    // East
            (0, 1, 1),    // East-Up
            (1, -1, -1),  // South-West-Down
            (1, -1, 0),   // South-West
            (1, -1, 1),   // South-West-Up
            (1, 0, -1),   // South-Down
            (1, 0, 0),    // South
            (1, 0, 1),    // South-Up
            (1, 1, -1),   // South-East-Down
            (1, 1, 0),    // South-East
            (1, 1, 1),    // South-East-Up
        ];
        &DIRECTIONS

        /*vec![
            (-1, -1, -1),
            (-1, -1, 0),
            (-1, -1, 1),

            (-1, 0, -1),
            (-1, 0, 0),
            (-1, 0, 1),

            (-1, 1, -1),
            (-1, 1, 0),
            (-1, 1, 1),

            (0, -1, -1),
            (0, -1, 0),
            (0, -1, 1),

            (0, 0, -1),
            (0, 0, 1),
            (0, 1, -1),

            (0, 1, 0),
            (0, 1, 1),
            (1, -1, -1),

            (1, -1, 0),
            (1, -1, 1),
            (1, 0, -1),

            (1, 0, 0),
            (1, 0, 1),
            (1, 1, -1),

            (1, 1, 0),
            (1, 1, 1),
        ]*/
    }

    /*#[inline(always)]
    fn neighbors(&self, grid: &ArrayView3<Point>, pos: UVec3) -> u32 {
        let x = pos.x as i32;
        let y = pos.y as i32;
        let z = pos.z as i32;

        let shape = grid.shape();
        let max_x = shape[0] as u32;
        let max_y = shape[1] as u32;
        let max_z = shape[2] as u32;

        let mut bits: u32 = 0;
        let mut index = 0;

        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue; // Skip center
                    }

                    let nx = x + dx;
                    let ny = y + dy;
                    let nz = z + dz;

                    if nx >= 0 && ny >= 0 && nz >= 0 {
                        let (nx, ny, nz) = (nx as u32, ny as u32, nz as u32);

                        if nx < max_x && ny < max_y && nz < max_z {
                            let neighbor = &grid[[nx as usize, ny as usize, nz as usize]];
                            if !neighbor.solid {
                                bits |= 1 << index;
                            }
                        }
                    }

                    index += 1;
                }
            }
        }

        bits
    }*/

    #[inline(always)]
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        let dx = pos.x.abs_diff(target.x);
        let dy = pos.y.abs_diff(target.y);
        let dz = pos.z.abs_diff(target.z);
        dx.max(dy).max(dz)
    }

    #[inline(always)]
    fn is_ordinal(&self) -> bool {
        true
    }

    fn filters(&self) -> &[Arc<dyn NeighborFilter + Send + Sync + 'static>] {
        &[]
    }
}

pub(crate) const ORDINAL_3D_OFFSETS: [IVec3; 26] = {
    let mut offsets = [IVec3::ZERO; 26];
    let mut index = 0;
    let mut dz = -1;
    while dz <= 1 {
        let mut dy = -1;
        while dy <= 1 {
            let mut dx = -1;
            while dx <= 1 {
                if dx != 0 || dy != 0 || dz != 0 {
                    offsets[index] = IVec3::new(dx, dy, dz);
                    index += 1;
                }
                dx += 1;
            }
            dy += 1;
        }
        dz += 1;
    }
    offsets
};

#[derive(Copy, Clone, Debug, Default)]
pub struct NoCornerClippingFilter;

impl NeighborFilter for NoCornerClippingFilter {
    fn filter(&self, pos: UVec3, mut mask: u32, grid: &ArrayView3<Point>) -> u32 {
        let shape = grid.shape();
        let origin = pos.as_ivec3();

        for (i, &offset) in ORDINAL_3D_OFFSETS.iter().enumerate() {
            // Skip if this neighbor isn't currently included
            if (mask >> i) & 1 == 0 {
                continue;
            }

            // Only apply filter for 2D diagonal moves
            if offset.z == 0 && offset.x != 0 && offset.y != 0 {
                // Get the adjacent cardinals
                let cardinal1 = origin + IVec3::new(offset.x, 0, 0);
                let cardinal2 = origin + IVec3::new(0, offset.y, 0);

                let solid_cardinals = [cardinal1, cardinal2]
                    .iter()
                    .filter(|&&c| {
                        c.cmplt(IVec3::ZERO).any()
                            || c.x >= shape[0] as i32
                            || c.y >= shape[1] as i32
                            || c.z >= shape[2] as i32
                            || grid[[c.x as usize, c.y as usize, c.z as usize]].solid
                    })
                    .count();

                // Disallow diagonal if both cardinals are solid
                if solid_cardinals == 2 {
                    mask &= !(1 << i); // Clear the bit
                }
            }
        }

        mask
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cardinal_neighbors() {
        let neighborhood = CardinalNeighborhood {
            filters: Vec::new(),
        };
        let points: [Point; 9] = std::array::from_fn(|_| Point::default());
        let grid = ArrayView3::from_shape((3, 3, 1), &points).unwrap();

        let neighbors = neighborhood.neighbors(&grid, UVec3::new(1, 1, 0));

        assert_eq!(neighbors.count_ones(), 4);
    }

    #[test]
    fn test_cardinal_neighbors_3d() {
        let neighborhood = CardinalNeighborhood3d;
        let points: [Point; 27] = std::array::from_fn(|_| Point::default());
        let grid = ArrayView3::from_shape((3, 3, 3), &points).unwrap();

        let neighbors = neighborhood.neighbors(&grid, UVec3::new(1, 1, 1));

        assert_eq!(neighbors.count_ones(), 6);
    }

    #[test]
    fn test_ordinal_neighbors_3d() {
        let neighborhood = OrdinalNeighborhood3d;
        let points: [Point; 27] = std::array::from_fn(|_| Point::default());
        let grid = ArrayView3::from_shape((3, 3, 3), &points).unwrap();

        let neighbors_mask = neighborhood.neighbors(&grid, UVec3::new(1, 1, 1));

        // All 26 neighbors should be available (all bits set)
        assert_eq!(neighbors_mask.count_ones(), 26);

        // Check that each bit corresponds to a valid neighbor position
        let mut expected_positions = Vec::new();
        let center = UVec3::new(1, 1, 1);
        let mut bit = 0;
        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue; // skip center
                    }
                    let pos = UVec3::new(
                        (center.x as i32 + dx) as u32,
                        (center.y as i32 + dy) as u32,
                        (center.z as i32 + dz) as u32,
                    );
                    // The bit at position `bit` should be set
                    assert!(
                        (neighbors_mask & (1 << bit)) != 0,
                        "Neighbor bit {} (pos {:?}) not set",
                        bit,
                        pos
                    );
                    expected_positions.push(pos);
                    bit += 1;
                }
            }
        }
        assert_eq!(bit, 26);
    }

    #[test]
    fn test_ordinal_neighors_at_0() {
        let neighborhood = OrdinalNeighborhood3d;
        let points: [Point; 27] = std::array::from_fn(|_| Point::default());
        let grid = ArrayView3::from_shape((3, 3, 3), &points).unwrap();

        let neighbors = neighborhood.neighbors(&grid, UVec3::new(0, 0, 0));

        // Count the number of set bits (neighbors)
        assert_eq!(neighbors.count_ones(), 7);

        // Check that the correct neighbor bits are set
        let expected_positions = [
            (0, 0, 1),
            (0, 1, 0),
            (0, 1, 1),
            (1, 0, 0),
            (1, 0, 1),
            (1, 1, 0),
            (1, 1, 1),
        ];

        let mut bit = 0;
        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    let pos = (
                        (0_i32 + dx) as u32,
                        (0_i32 + dy) as u32,
                        (0_i32 + dz) as u32,
                    );
                    if expected_positions.contains(&pos) {
                        assert!(
                            (neighbors & (1 << bit)) != 0,
                            "Expected neighbor at {:?} (bit {}) to be set",
                            pos,
                            bit
                        );
                    } else {
                        assert!(
                            (neighbors & (1 << bit)) == 0,
                            "Unexpected neighbor at {:?} (bit {}) set",
                            pos,
                            bit
                        );
                    }
                    bit += 1;
                }
            }
        }
    }

    #[test]
    fn test_ordinal_neighbors_no_depth() {
        let neighborhood = OrdinalNeighborhood3d;
        let points: [Point; 9] = std::array::from_fn(|_| Point::default());

        let grid = ArrayView3::from_shape((3, 3, 1), &points).unwrap();

        let neighbors = neighborhood.neighbors(&grid, UVec3::new(1, 1, 0));

        // There should be 8 neighbors (all 2D surrounding positions)
        assert_eq!(neighbors.count_ones(), 8);

        let expected_positions = [
            (0, 0, 0),
            (0, 1, 0),
            (0, 2, 0),
            (1, 0, 0),
            (1, 2, 0),
            (2, 0, 0),
            (2, 1, 0),
            (2, 2, 0),
        ];

        let mut bit = 0;
        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    let pos = (
                        (1_i32 + dx) as u32,
                        (1_i32 + dy) as u32,
                        (0_i32 + dz) as u32,
                    );
                    if expected_positions.contains(&pos) {
                        assert!(
                            (neighbors & (1 << bit)) != 0,
                            "Expected neighbor at {:?} (bit {}) to be set",
                            pos,
                            bit
                        );
                    } else {
                        assert!(
                            (neighbors & (1 << bit)) == 0,
                            "Unexpected neighbor at {:?} (bit {}) set",
                            pos,
                            bit
                        );
                    }
                    bit += 1;
                }
            }
        }

        assert_eq!(bit, 26);
    }

    #[test]
    fn test_ordinal_heuristic() {
        let neighborhood = OrdinalNeighborhood3d;

        assert_eq!(
            neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(1, 1, 1)),
            1
        );
        assert_eq!(
            neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(1, 0, 0)),
            1
        );
        assert_eq!(
            neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(0, 0, 0)),
            0
        );
        assert_eq!(
            neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(2, 2, 2)),
            2
        );
        assert_eq!(
            neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(7, 7, 7)),
            7
        );
    }
}
