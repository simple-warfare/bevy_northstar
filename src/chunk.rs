use bevy::math::UVec3;
use ndarray::{s, Array3, ArrayView2, ArrayView3};

use crate::{dir::Dir, Point};

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

    pub fn view<'a>(&self, grid: &'a Array3<Point>) -> ArrayView3<'a, Point> {
        grid.slice(s![
            self.min.x as usize..self.max.x as usize,
            self.min.y as usize..self.max.y as usize,
            self.min.z as usize..self.max.z as usize
        ])
    }

    pub fn edge<'a>(&self, grid: &'a Array3<Point>, dir: Dir) -> ArrayView2<'a, Point> {
        match dir {
            Dir::NORTH => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.max.y as usize,
                self.min.z as usize..self.max.z as usize,
            ]),
            Dir::EAST => grid.slice(s![
                self.max.x as usize,
                self.min.y as usize..self.max.y as usize,
                self.min.z as usize..self.max.z as usize,
            ]),
            Dir::SOUTH => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.min.y as usize,
                self.min.z as usize..self.max.z as usize,
            ]),
            Dir::WEST => grid.slice(s![
                self.min.x as usize,
                self.min.y as usize..self.max.y as usize,
                self.min.z as usize..self.max.z as usize,
            ]),
            Dir::UP => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.min.y as usize..self.max.y as usize,
                self.max.z as usize,
            ]),
            Dir::DOWN => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.min.y as usize..self.max.y as usize,
                self.min.z as usize,
            ]),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_chunk_view() {
        let grid = Array3::from_elem((10, 10, 10), Point::default());
        let chunk = Chunk::new(UVec3::new(0, 0, 0), UVec3::new(5, 5, 5));
        let view = chunk.view(&grid);

        assert_eq!(view.shape(), [5, 5, 5]);
    }

    #[test]
    fn test_chunk_edge() {
        let grid = Array3::from_elem((10, 10, 10), Point::default());
        let chunk = Chunk::new(UVec3::new(0, 0, 0), UVec3::new(5, 5, 5));
        let edge = chunk.edge(&grid, Dir::NORTH);

        assert_eq!(edge.shape(), [5, 5]);
    }
}
