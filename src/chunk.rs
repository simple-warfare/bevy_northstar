//! This module defines the `Chunk` struct, which represents a 3D region of the grid.
use bevy::math::UVec3;
use ndarray::{s, Array3, ArrayView2, ArrayView3};

use crate::{dir::Dir, nav::NavCell};

/// A chunk is a 3D region of the grid.
#[derive(Debug, Clone)]
pub(crate) struct Chunk {
    /// The minimum coordinates of the chunk.
    min: UVec3,
    /// The maximum coordinates of the chunk.
    max: UVec3,
    dirty_cells: bool,
    dirty_edges: [bool; 18],
}

impl PartialEq for Chunk {
    fn eq(&self, other: &Self) -> bool {
        self.min == other.min && self.max == other.max
    }
}

impl Eq for Chunk {}

impl Chunk {
    /// Creates a new chunk from minimum and maximum coordinates.
    pub(crate) fn new(min: UVec3, max: UVec3) -> Self {
        Chunk { min, max, dirty_cells: true, dirty_edges: [true; 18] }
    }

    pub(crate) fn min(&self) -> UVec3 {
        self.min
    }

    pub(crate) fn max(&self) -> UVec3 {
        self.max
    }

    pub(crate) fn bounds(&self) -> impl Iterator<Item = UVec3> {
        let min = self.min();
        let max = self.max();

        (min.x..max.x).flat_map(move |x| {
            (min.y..max.y).flat_map(move |y| {
                (min.z..max.z).map(move |z| UVec3::new(x, y, z))
            })
        })
    }

    pub(crate) fn has_dirty_cells(&self) -> bool {
        self.dirty_cells
    }

    pub(crate) fn has_dirty_edges(&self) -> bool {
        self.dirty_edges.iter().any(|&edge| edge)
    }

    pub(crate) fn is_edge_dirty(&self, dir: Dir) -> bool {
        self.dirty_edges[dir as usize]
    }

    pub(crate) fn set_dirty_cells(&mut self, dirty: bool) {
        self.dirty_cells = dirty;
    }

    pub(crate) fn set_dirty_edge(&mut self, dir: Dir, dirty: bool) {
        self.dirty_edges[dir as usize] = dirty;
    }

    pub(crate) fn set_all_edges_dirty(&mut self, dirty: bool) {
        for edge in self.dirty_edges.iter_mut() {
            *edge = dirty;
        }
    }

    pub(crate) fn clean(&mut self) {
        self.dirty_cells = false;
        self.dirty_edges.fill(false);
    }

    // Adjusts a position to the local coordinates of the chunk.
    pub(crate) fn to_local(&self, pos: &UVec3) -> UVec3 {
        UVec3::new(
            pos.x.saturating_sub(self.min().x),
            pos.y.saturating_sub(self.min().y),
            pos.z.saturating_sub(self.min().z),
        )
    }

    /// Returns a 3D `ArrayView3`` of `NavCell`s of the chunk from the grid.
    pub(crate) fn view<'a>(&self, grid: &'a Array3<NavCell>) -> ArrayView3<'a, NavCell> {
        grid.slice(s![
            self.min.x as usize..self.max.x as usize,
            self.min.y as usize..self.max.y as usize,
            self.min.z as usize..self.max.z as usize
        ])
    }

    /// Returns a 2D `ArrayView2`` of the edge of the chunk in the given direction.
    pub(crate) fn edge<'a>(&self, grid: &'a Array3<NavCell>, dir: Dir) -> ArrayView2<'a, NavCell> {
        match dir {
            Dir::NORTH => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.max.y as usize - 1,
                self.min.z as usize..self.max.z as usize,
            ]),
            Dir::EAST => grid.slice(s![
                self.max.x as usize - 1,
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
                self.max.z as usize - 1,
            ]),
            Dir::DOWN => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.min.y as usize..self.max.y as usize,
                self.min.z as usize,
            ]),
            _ => panic!("Ordinal directions do not have an edge."),
        }
    }

    /// Returns the chunk corner `NavCell` for the given ordinal direction.
    pub(crate) fn corner(&self, grid: &Array3<NavCell>, dir: Dir) -> NavCell {
        match dir {
            Dir::NORTHEAST => grid[[
                self.max.x as usize - 1,
                self.max.y as usize - 1,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::SOUTHEAST => grid[[
                self.max.x as usize - 1,
                self.min.y as usize,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::SOUTHWEST => grid[[
                self.min.x as usize,
                self.min.y as usize,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::NORTHWEST => grid[[
                self.min.x as usize,
                self.max.y as usize - 1,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::NORTHEASTUP => grid[[
                self.max.x as usize - 1,
                self.max.y as usize - 1,
                self.max.z as usize - 1,
            ]]
            .clone(),
            Dir::SOUTHEASTUP => grid[[
                self.max.x as usize - 1,
                self.min.y as usize,
                self.max.z as usize - 1,
            ]]
            .clone(),
            Dir::SOUTHWESTUP => grid[[
                self.min.x as usize,
                self.min.y as usize,
                self.max.z as usize - 1,
            ]]
            .clone(),
            Dir::NORTHWESTUP => grid[[
                self.min.x as usize,
                self.max.y as usize - 1,
                self.max.z as usize - 1,
            ]]
            .clone(),
            Dir::NORTHEASTDOWN => grid[[
                self.max.x as usize - 1,
                self.max.y as usize - 1,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::SOUTHEASTDOWN => grid[[
                self.max.x as usize - 1,
                self.min.y as usize,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::SOUTHWESTDOWN => grid[[
                self.min.x as usize,
                self.min.y as usize,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::NORTHWESTDOWN => grid[[
                self.min.x as usize,
                self.max.y as usize - 1,
                self.min.z as usize,
            ]]
            .clone(),
            _ => panic!("Cardinal directions have no corner."),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_chunk_view() {
        let grid = Array3::from_elem((10, 10, 10), NavCell::default());
        let chunk = Chunk::new(UVec3::new(0, 0, 0), UVec3::new(4, 4, 4));
        let view = chunk.view(&grid);

        assert_eq!(view.shape(), [4, 4, 4]);
    }

    #[test]
    fn test_chunk_edge() {
        let grid = Array3::from_elem((10, 10, 10), NavCell::default());
        let chunk = Chunk::new(UVec3::new(0, 0, 0), UVec3::new(4, 4, 4));
        let edge = chunk.edge(&grid, Dir::NORTH);

        assert_eq!(edge.shape(), [4, 4]);
    }
}
