use bevy::math::UVec3;
use ndarray::ArrayView3;

use std::fmt::Debug;

use crate::Point;

pub trait Neighborhood: Clone + Debug + Default + Sync + Send {
    fn directions(&self) -> Vec<(i32, i32, i32)>;
    fn neighbors(&self, grid: &ArrayView3<Point>, pos: UVec3, target: &mut Vec<UVec3>);
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32;
    fn is_ordinal(&self) -> bool {
        false
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CardinalNeighborhood;

impl Neighborhood for CardinalNeighborhood {
    #[inline(always)]
    fn directions(&self) -> Vec<(i32, i32, i32)> {
        vec![
            (-1, 0, 0),
            (1, 0, 0),
            (0, -1, 0),
            (0, 1, 0),
        ]
    }

    #[inline(always)]
    fn neighbors(&self, grid: &ArrayView3<Point>, pos: UVec3, target: &mut Vec<UVec3>) {
        let x = pos.x as i32;
        let y = pos.y as i32;

        let directions = self.directions();

        for &(dx, dy, _) in &directions {
            let nx = x + dx;
            let ny = y + dy;

            if nx >= 0 && ny >= 0 {
                let nx = nx as u32;
                let ny = ny as u32;

                if nx < grid.shape()[0] as u32 && ny < grid.shape()[1] as u32 {
                    let neighbor = UVec3::new(nx, ny, pos.z);
                    if !grid[[nx as usize, ny as usize, pos.z as usize]].wall {
                        target.push(neighbor);
                    }
                }
            }
        }
    }

    #[inline(always)]
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        ((pos.x as i32 - target.x as i32).abs() + (pos.y as i32 - target.y as i32).abs()) as u32
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CardinalNeighborhood3d;

impl Neighborhood for CardinalNeighborhood3d {
    #[inline(always)]
    fn directions(&self) -> Vec<(i32, i32, i32)> {
        vec![
            (-1, 0, 0),
            (1, 0, 0),
            (0, -1, 0),
            (0, 1, 0),
            (0, 0, -1),
            (0, 0, 1),
        ]
    }
    #[inline(always)]
    fn neighbors(&self, grid: &ArrayView3<Point>, pos: UVec3, target: &mut Vec<UVec3>) {
        let x = pos.x as i32;
        let y = pos.y as i32;
        let z = pos.z as i32;

        let directions = [
            (-1, 0, 0),
            (1, 0, 0),
            (0, -1, 0),
            (0, 1, 0),
            (0, 0, -1),
            (0, 0, 1),
        ];

        for &(dx, dy, dz) in &directions {
            let nx = x + dx;
            let ny = y + dy;
            let nz = z + dz;

            if nx >= 0 && ny >= 0 && nz >= 0 {
                let nx = nx as u32;
                let ny = ny as u32;
                let nz = nz as u32;

                if nx < grid.shape()[0] as u32 && ny < grid.shape()[1] as u32 && nz < grid.shape()[2] as u32 {
                    let neighbor = UVec3::new(nx, ny, nz);
                    if !grid[[nx as usize, ny as usize, nz as usize]].wall {
                        target.push(neighbor);
                    }
                }
            }
        }
    }

    #[inline(always)]
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        let dx = pos.x.max(target.x) - pos.x.min(target.x);
        let dy = pos.y.max(target.y) - pos.y.min(target.y);
        let dz = pos.z.max(target.z) - pos.z.min(target.z);
        dx + dy + dz
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct OrdinalNeighborhood;

impl Neighborhood for OrdinalNeighborhood {
    #[inline(always)]
    fn directions(&self) -> Vec<(i32, i32, i32)> {
        vec![
            (-1, -1, 0),
            (-1, 0, 0),
            (-1, 1, 0),
            (0, -1, 0),
            (0, 1, 0),
            (1, -1, 0),
            (1, 0, 0),
            (1, 1, 0),
        ]
    }

    #[inline(always)]
    fn neighbors(&self, grid: &ArrayView3<Point>, pos: UVec3, target: &mut Vec<UVec3>) {
        let x = pos.x as i32;
        let y = pos.y as i32;

        for &(dx, dy, _) in &self.directions() {
            let nx = x + dx;
            let ny = y + dy;

            if nx >= 0 && ny >= 0 {
                let nx = nx as u32;
                let ny = ny as u32;

                if nx < grid.shape()[0] as u32 && ny < grid.shape()[1] as u32 {
                    let neighbor = UVec3::new(nx, ny, pos.z);
                    if !grid[[nx as usize, ny as usize, pos.z as usize]].wall {
                        target.push(neighbor);
                    }
                }
            }
        }
    }

    #[inline(always)]
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        let dx = pos.x.max(target.x) - pos.x.min(target.x);
        let dy = pos.y.max(target.y) - pos.y.min(target.y);
        dx + dy
    }

    #[inline(always)]
    fn is_ordinal(&self) -> bool {
        true
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct OrdinalNeighborhood3d;

impl Neighborhood for OrdinalNeighborhood3d {
    #[inline(always)]
    fn directions(&self) -> Vec<(i32, i32, i32)> {
        vec![
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
        ]
    }

    #[inline(always)]
    fn neighbors(&self, grid: &ArrayView3<Point>, pos: UVec3, target: &mut Vec<UVec3>) {
        let x = pos.x as i32;
        let y = pos.y as i32;
        let z = pos.z as i32;

        for i in -1..=1 {
            for j in -1..=1 {
                for k in -1..=1 {
                    if i == 0 && j == 0 && k == 0 {
                        continue;
                    }

                    let nx = x + i;
                    let ny = y + j;
                    let nz = z + k;

                    if nx >= 0 && ny >= 0 && nz >= 0 {
                        let nx = nx as u32;
                        let ny = ny as u32;
                        let nz = nz as u32;

                        if nx < grid.shape()[0] as u32 && ny < grid.shape()[1] as u32 && nz < grid.shape()[2] as u32 {
                            let neighbor = UVec3::new(nx, ny, nz);
                            if !grid[[nx as usize, ny as usize, nz as usize]].wall {
                                target.push(neighbor);
                            }
                        }
                    }
                }
            }
        }
    }

    #[inline(always)]
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        let dx = (pos.x as i32 - target.x as i32).abs() as u32;
        let dy = (pos.y as i32 - target.y as i32).abs() as u32;
        let dz = (pos.z as i32 - target.z as i32).abs() as u32;
        dx.max(dy).max(dz)
    }

    #[inline(always)]
    fn is_ordinal(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cardinal_neighbors() {
        let neighborhood = CardinalNeighborhood;
        let points = [Point::default(); 9];
        let grid = ArrayView3::from_shape((3, 3, 1), &points).unwrap();
        let mut target = Vec::new();

        neighborhood.neighbors(&grid, UVec3::new(1, 1, 0), &mut target);

        assert_eq!(target.len(), 4);
    }

    #[test]
    fn test_cardinal_neighbors_3d() {
        let neighborhood = CardinalNeighborhood3d;
        let points = [Point::default(); 27];
        let grid = ArrayView3::from_shape((3, 3, 3), &points).unwrap();
        let mut target = Vec::new();

        neighborhood.neighbors(&grid, UVec3::new(1, 1, 1), &mut target);

        assert_eq!(target.len(), 6);
    }

    #[test]
    fn test_ordinal_neighbors_3d() {
        let neighborhood = OrdinalNeighborhood3d;
        let points = [Point::default(); 27];
        let grid = ArrayView3::from_shape((3, 3, 3), &points).unwrap();
        let mut target = Vec::new();

        neighborhood.neighbors(&grid, UVec3::new(1, 1, 1), &mut target);

        assert_eq!(target.len(), 26);
        assert_eq!(target[0], UVec3::new(0, 0, 0));
        assert_eq!(target[1], UVec3::new(0, 0, 1));
        assert_eq!(target[2], UVec3::new(0, 0, 2));
        assert_eq!(target[3], UVec3::new(0, 1, 0));
        assert_eq!(target[4], UVec3::new(0, 1, 1));
        assert_eq!(target[5], UVec3::new(0, 1, 2));
        assert_eq!(target[6], UVec3::new(0, 2, 0));
        assert_eq!(target[7], UVec3::new(0, 2, 1));
        assert_eq!(target[8], UVec3::new(0, 2, 2));
        assert_eq!(target[9], UVec3::new(1, 0, 0));
        assert_eq!(target[10], UVec3::new(1, 0, 1));
        assert_eq!(target[11], UVec3::new(1, 0, 2));
        assert_eq!(target[12], UVec3::new(1, 1, 0));
        assert_eq!(target[13], UVec3::new(1, 1, 2));
        assert_eq!(target[14], UVec3::new(1, 2, 0));
        assert_eq!(target[15], UVec3::new(1, 2, 1));
        assert_eq!(target[16], UVec3::new(1, 2, 2));
        assert_eq!(target[17], UVec3::new(2, 0, 0));
        assert_eq!(target[18], UVec3::new(2, 0, 1));
        assert_eq!(target[19], UVec3::new(2, 0, 2));
        assert_eq!(target[20], UVec3::new(2, 1, 0));
        assert_eq!(target[21], UVec3::new(2, 1, 1));
        assert_eq!(target[22], UVec3::new(2, 1, 2));
        assert_eq!(target[23], UVec3::new(2, 2, 0));
        assert_eq!(target[24], UVec3::new(2, 2, 1));
        assert_eq!(target[25], UVec3::new(2, 2, 2));
    }

    #[test]
    fn test_ordinal_neighors_at_0() {
        let neighborhood = OrdinalNeighborhood3d;
        let points = [Point::default(); 27];
        let grid = ArrayView3::from_shape((3, 3, 3), &points).unwrap();
        let mut target = Vec::new();

        neighborhood.neighbors(&grid, UVec3::new(0, 0, 0), &mut target);

        assert_eq!(target.len(), 7);
        assert_eq!(target[0], UVec3::new(0, 0, 1));
        assert_eq!(target[1], UVec3::new(0, 1, 0));
        assert_eq!(target[2], UVec3::new(0, 1, 1));
        assert_eq!(target[3], UVec3::new(1, 0, 0));
        assert_eq!(target[4], UVec3::new(1, 0, 1));
        assert_eq!(target[5], UVec3::new(1, 1, 0));
        assert_eq!(target[6], UVec3::new(1, 1, 1));
    }

    #[test]
    fn test_ordinal_neighbors_no_depth() {
        let neighborhood = OrdinalNeighborhood3d;
        let points = [Point::default(); 9];
        let grid = ArrayView3::from_shape((3, 3, 1), &points).unwrap();

        let mut target = Vec::new();

        neighborhood.neighbors(&grid, UVec3::new(1, 1, 0), &mut target);

        assert_eq!(target.len(), 8);
        assert_eq!(target[0], UVec3::new(0, 0, 0));
        assert_eq!(target[1], UVec3::new(0, 1, 0));
        assert_eq!(target[2], UVec3::new(0, 2, 0));
        assert_eq!(target[3], UVec3::new(1, 0, 0));
        assert_eq!(target[4], UVec3::new(1, 2, 0));
        assert_eq!(target[5], UVec3::new(2, 0, 0));
        assert_eq!(target[6], UVec3::new(2, 1, 0));
        assert_eq!(target[7], UVec3::new(2, 2, 0));
    }

    #[test]
    fn test_ordinal_heuristic() {
        let neighborhood = OrdinalNeighborhood3d;

        assert_eq!(neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(1, 1, 1)), 1);
        assert_eq!(neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(1, 0, 0)), 1);
        assert_eq!(neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(0, 0, 0)), 0);
        assert_eq!(neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(2, 2, 2)), 2);
        assert_eq!(neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(7, 7, 7)), 7);
    }
}