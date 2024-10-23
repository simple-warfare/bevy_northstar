use bevy::math::UVec3;
use ndarray::ArrayView3;

use std::fmt::Debug;

use crate::Point;

pub trait Neighborhood: Clone + Debug {
    fn neighbors(&self, grid: &ArrayView3<Point>, pos: UVec3, target: &mut Vec<UVec3>);
    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32;
}

#[derive(Clone, Copy, Debug)]
pub struct CardinalNeighborhood;

impl Neighborhood for CardinalNeighborhood {
    fn neighbors(&self, grid: &ArrayView3<Point>, pos: UVec3, target: &mut Vec<UVec3>) {
        let x = pos.x;
        let y = pos.y;

        let directions = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
        ];

        for (dx, dy) in directions.iter() {
            let x = x as i32 + dx;
            let y = y as i32 + dy;

            if x < 0 || y < 0 {
                continue;
            }

            let x = x as u32;
            let y = y as u32;

            if x >= grid.shape()[0] as u32 || y >= grid.shape()[1] as u32 {
                continue;
            }

            let neighbor = UVec3::new(x, y, 0);
            let point = &grid[[x as usize, y as usize, 0]];

            if point.wall {
                continue;
            }

            target.push(neighbor);
        }
    }

    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        let dx = (pos.x as i32 - target.x as i32).abs() as u32;
        let dy = (pos.y as i32 - target.y as i32).abs() as u32;

        dx + dy
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CardinalNeighboorhood3d;

impl Neighborhood for CardinalNeighboorhood3d {
    fn neighbors(&self, grid: &ArrayView3<Point>, pos: UVec3, target: &mut Vec<UVec3>) {
        let x = pos.x;
        let y = pos.y;
        let z = pos.z;

        let directions = [
            (-1, 0, 0),
            (1, 0, 0),
            (0, -1, 0),
            (0, 1, 0),
            (0, 0, -1),
            (0, 0, 1),
        ];

        for (dx, dy, dz) in directions.iter() {
            let x = x as i32 + dx;
            let y = y as i32 + dy;
            let z = z as i32 + dz;

            if x < 0 || y < 0 || z < 0 {
                continue;
            }

            let x = x as u32;
            let y = y as u32;
            let z = z as u32;

            if x >= grid.shape()[0] as u32 || y >= grid.shape()[1] as u32 || z >= grid.shape()[2] as u32 {
                continue;
            }

            let neighbor = UVec3::new(x, y, z);
            let point = &grid[[x as usize, y as usize, z as usize]];

            if point.wall {
                continue;
            }

            target.push(neighbor);
        }
    }

    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        let dx = (pos.x as i32 - target.x as i32).abs() as u32;
        let dy = (pos.y as i32 - target.y as i32).abs() as u32;
        let dz = (pos.z as i32 - target.z as i32).abs() as u32;

        dx + dy + dz
    }
}

#[derive(Clone, Copy, Debug)]
pub struct OrdinalNeighborhood;

#[derive(Clone, Copy, Debug)]
pub struct OrdinalNeighborhood3d;

impl Neighborhood for OrdinalNeighborhood3d {
    fn neighbors(&self, grid: &ArrayView3<Point>, pos: UVec3, target: &mut Vec<UVec3>) {
        let x = pos.x;
        let y = pos.y;
        let z = pos.z;

        for i in -1..=1 {
            for j in -1..=1 {
                for k in -1..=1 {
                    if i == 0 && j == 0 && k == 0 {
                        continue;
                    }

                    let x = x as i32 + i;
                    let y = y as i32 + j;
                    let z = z as i32 + k;

                    if x < 0 || y < 0 || z < 0 {
                        continue;
                    }

                    let x = x as u32;
                    let y = y as u32;
                    let z = z as u32;

                    if x >= grid.shape()[0] as u32 || y >= grid.shape()[1] as u32 || z >= grid.shape()[2] as u32 {
                        continue;
                    }

                    let neighbor = UVec3::new(x, y, z);
                    let point = &grid[[x as usize, y as usize, z as usize]];

                    if point.wall {
                        continue;
                    }

                    target.push(neighbor);
                }
            }
        }
    }

    fn heuristic(&self, pos: UVec3, target: UVec3) -> u32 {
        let dx = (pos.x as i32 - target.x as i32).abs() as u32;
        let dy = (pos.y as i32 - target.y as i32).abs() as u32;
        let dz = (pos.z as i32 - target.z as i32).abs() as u32;

        dx + dy + dz
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
        let neighborhood = CardinalNeighboorhood3d;
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

        assert_eq!(neighborhood.heuristic(UVec3::new(0, 0, 0), UVec3::new(7, 7, 1)), 15);
        assert_eq!(neighborhood.heuristic(UVec3::new(1, 0, 0), UVec3::new(7, 7, 1)), 14);
        assert_eq!(neighborhood.heuristic(UVec3::new(0, 1, 0), UVec3::new(7, 7, 1)), 14);
        assert_eq!(neighborhood.heuristic(UVec3::new(1, 1, 0), UVec3::new(7, 7, 1)), 13);
        assert_eq!(neighborhood.heuristic(UVec3::new(1, 1, 1), UVec3::new(7, 7, 1)), 12);
    }
}