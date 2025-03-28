use bevy::math::UVec3;
use ndarray::{s, Array3, ArrayView2, ArrayView3};

use crate::{dir::Dir, Point};

/// A chunk is a 3D region of the grid.
#[derive(Debug, Clone)]
pub struct Chunk {
    pub min: UVec3,
    pub max: UVec3,
    //pub nodes: Vec<Node>,
}

impl PartialEq for Chunk {
    fn eq(&self, other: &Self) -> bool {
        self.min == other.min && self.max == other.max
    }
}

impl Eq for Chunk {}

impl Chunk {
    pub fn new(min: UVec3, max: UVec3) -> Self {
        Chunk { min, max }
    }

    /// Returns a 3D `ArrayView3`` of `Point`s of the chunk from the grid.
    pub fn view<'a>(&self, grid: &'a Array3<Point>) -> ArrayView3<'a, Point> {
        grid.slice(s![
            self.min.x as usize..self.max.x as usize + 1,
            self.min.y as usize..self.max.y as usize + 1,
            self.min.z as usize..self.max.z as usize + 1
        ])
    }

    /// Returns a 2D `ArrayView2`` of the edge of the chunk in the given direction.
    pub fn edge<'a>(&self, grid: &'a Array3<Point>, dir: Dir) -> ArrayView2<'a, Point> {
        match dir {
            Dir::NORTH => grid.slice(s![
                self.min.x as usize..self.max.x as usize + 1,
                self.max.y as usize,
                self.min.z as usize..self.max.z as usize + 1,
            ]),
            Dir::EAST => grid.slice(s![
                self.max.x as usize,
                self.min.y as usize..self.max.y as usize + 1,
                self.min.z as usize..self.max.z as usize + 1,
            ]),
            Dir::SOUTH => grid.slice(s![
                self.min.x as usize..self.max.x as usize + 1,
                self.min.y as usize,
                self.min.z as usize..self.max.z as usize + 1,
            ]),
            Dir::WEST => grid.slice(s![
                self.min.x as usize,
                self.min.y as usize..self.max.y as usize + 1,
                self.min.z as usize..self.max.z as usize + 1,
            ]),
            Dir::UP => grid.slice(s![
                self.min.x as usize..self.max.x as usize + 1,
                self.min.y as usize..self.max.y as usize + 1,
                self.max.z as usize + 1,
            ]),
            Dir::DOWN => grid.slice(s![
                self.min.x as usize..self.max.x as usize + 1,
                self.min.y as usize..self.max.y as usize + 1,
                self.min.z as usize,
            ]),
            _ => panic!("Ordinal directions are not an edge"),
        }
    }

    /// Returns the chunk corner `Point` for the given ordinal direction.
    pub fn corner<'a>(&self, grid: &'a Array3<Point>, dir: Dir) -> Point {
        match dir {
            Dir::NORTHEAST => {
                grid[[
                    self.max.x as usize,
                    self.max.y as usize,
                    self.min.z as usize,
                ]]
            }
            Dir::SOUTHEAST => {
                grid[[
                    self.max.x as usize,
                    self.min.y as usize,
                    self.min.z as usize,
                ]]
            }
            Dir::SOUTHWEST => {
                grid[[
                    self.min.x as usize,
                    self.min.y as usize,
                    self.min.z as usize,
                ]]
            }
            Dir::NORTHWEST => {
                grid[[
                    self.min.x as usize,
                    self.max.y as usize,
                    self.min.z as usize,
                ]]
            }
            Dir::NORTHEASTUP => {
                grid[[
                    self.max.x as usize,
                    self.max.y as usize,
                    self.max.z as usize,
                ]]
            }
            Dir::SOUTHEASTUP => {
                grid[[
                    self.max.x as usize,
                    self.min.y as usize,
                    self.max.z as usize,
                ]]
            }
            Dir::SOUTHWESTUP => {
                grid[[
                    self.min.x as usize,
                    self.min.y as usize,
                    self.max.z as usize,
                ]]
            }
            Dir::NORTHWESTUP => {
                grid[[
                    self.min.x as usize,
                    self.max.y as usize,
                    self.max.z as usize,
                ]]
            }
            Dir::NORTHEASTDOWN => {
                grid[[
                    self.max.x as usize,
                    self.max.y as usize,
                    self.min.z as usize,
                ]]
            }
            Dir::SOUTHEASTDOWN => {
                grid[[
                    self.max.x as usize,
                    self.min.y as usize,
                    self.min.z as usize,
                ]]
            }
            Dir::SOUTHWESTDOWN => {
                grid[[
                    self.min.x as usize,
                    self.min.y as usize,
                    self.min.z as usize,
                ]]
            }
            Dir::NORTHWESTDOWN => {
                grid[[
                    self.min.x as usize,
                    self.max.y as usize,
                    self.min.z as usize,
                ]]
            }
            _ => panic!("Diagonal directions are not a corner"),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_chunk_view() {
        let grid = Array3::from_elem((10, 10, 10), Point::default());
        let chunk = Chunk::new(UVec3::new(0, 0, 0), UVec3::new(4, 4, 4));
        let view = chunk.view(&grid);

        assert_eq!(view.shape(), [5, 5, 5]);
    }

    #[test]
    fn test_chunk_edge() {
        let grid = Array3::from_elem((10, 10, 10), Point::default());
        let chunk = Chunk::new(UVec3::new(0, 0, 0), UVec3::new(4, 4, 4));
        let edge = chunk.edge(&grid, Dir::NORTH);

        assert_eq!(edge.shape(), [5, 5]);
    }
}
