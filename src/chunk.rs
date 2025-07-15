//! This module defines the `Chunk` struct, which represents a 3D region of the grid.
use bevy::math::UVec3;
use ndarray::{s, Array3, ArrayView1, ArrayView2, ArrayView3};

use crate::{dir::Dir, nav::NavCell};

/// A chunk is a 3D region of the grid.
#[derive(Debug, Clone)]
pub(crate) struct Chunk {
    /// The index of the chunk in the grid.
    index: (usize, usize, usize),
    /// The minimum coordinates of the chunk.
    min: UVec3,
    /// The maximum coordinates of the chunk.
    max: UVec3,
    /// Flags which indicate which edges are dirty.
    dirty_edges: [bool; 26],
}

impl PartialEq for Chunk {
    fn eq(&self, other: &Self) -> bool {
        self.min == other.min && self.max == other.max
    }
}

impl Eq for Chunk {}

impl Chunk {
    /// Creates a new chunk from minimum and maximum coordinates.
    pub(crate) fn new(index: (usize, usize, usize), min: UVec3, max: UVec3) -> Self {
        Chunk {
            index,
            min,
            max,
            dirty_edges: [true; 26],
        }
    }

    pub(crate) fn index(&self) -> (usize, usize, usize) {
        self.index
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
            (min.y..max.y).flat_map(move |y| (min.z..max.z).map(move |z| UVec3::new(x, y, z)))
        })
    }

    pub(crate) fn has_dirty_edges(&self) -> bool {
        self.dirty_edges.iter().any(|&edge| edge)
    }

    pub(crate) fn is_edge_dirty(&self, dir: Dir) -> bool {
        self.dirty_edges[dir as usize]
    }

    pub(crate) fn set_dirty_edge(&mut self, dir: Dir, dirty: bool) {
        self.dirty_edges[dir as usize] = dirty;
    }

    #[allow(dead_code)]
    pub(crate) fn set_all_edges_dirty(&mut self, dirty: bool) {
        for edge in self.dirty_edges.iter_mut() {
            *edge = dirty;
        }
    }

    pub(crate) fn clean(&mut self) {
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

    // Returns a 2D `ArrayView2`` of the edge of the chunk in the given direction.
    /*pub(crate) fn edge<'a>(&self, grid: &'a Array3<NavCell>, dir: Dir) -> ArrayView2<'a, NavCell> {
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
    }*/

    /*pub(crate) fn edge<'a>(&self, grid: &'a Array3<NavCell>, dir: Dir) -> ArrayView2<'a, NavCell> {
        match dir {
            Dir::North => {
                let y = self.max.y as usize - 1;
                grid.slice(s![
                    self.min.x as usize..self.max.x as usize,
                    y,
                    self.min.z as usize..self.max.z as usize
                ])
                .into_dimensionality::<Ix2>()
                .expect("Failed to cast NORTH edge to 2D")
            }
            Dir::East => {
                // fixed X = max.x - 1, so slice [Y, Z]
                let slice = grid.slice(s![
                    self.max.x as usize - 1,
                    self.min.y as usize..self.max.y as usize,
                    self.min.z as usize..self.max.z as usize,
                ]);
                slice
                    .into_dimensionality::<Ix2>()
                    .expect("Failed to cast EAST edge to 2D")
            }
            Dir::South => {
                // fixed Y = min.y, so slice [X, Z]
                let slice = grid.slice(s![
                    self.min.x as usize..self.max.x as usize,
                    self.min.y as usize,
                    self.min.z as usize..self.max.z as usize,
                ]);
                slice
                    .into_dimensionality::<Ix2>()
                    .expect("Failed to cast SOUTH edge to 2D")
            }
            Dir::West => {
                // fixed X = min.x, so slice [Y, Z]
                let slice = grid.slice(s![
                    self.min.x as usize,
                    self.min.y as usize..self.max.y as usize,
                    self.min.z as usize..self.max.z as usize,
                ]);
                slice
                    .into_dimensionality::<Ix2>()
                    .expect("Failed to cast WEST edge to 2D")
            }
            Dir::Up => {
                // fixed Z = max.z - 1, so slice [X, Y]
                let slice = grid.slice(s![
                    self.min.x as usize..self.max.x as usize,
                    self.min.y as usize..self.max.y as usize,
                    self.max.z as usize - 1,
                ]);
                slice
                    .into_dimensionality::<Ix2>()
                    .expect("Failed to cast UP edge to 2D")
            }
            Dir::Down => {
                // fixed Z = min.z, so slice [X, Y]
                let slice = grid.slice(s![
                    self.min.x as usize..self.max.x as usize,
                    self.min.y as usize..self.max.y as usize,
                    self.min.z as usize,
                ]);
                slice
                    .into_dimensionality::<Ix2>()
                    .expect("Failed to cast DOWN edge to 2D")
            },
            Dir::NorthUp => {
                let y = self.max.y as usize - 1;
                grid.slice(s![
                    self.min.x as usize..self.max.x as usize,
                    y,
                    self.max.z as usize - 1
                ])
                .into_dimensionality::<Ix2>()
                .expect("Failed to cast NORTH_UP edge to 2D")
            },
            Dir::EastUp => {
                // fixed X = max.x - 1, so slice [Y, Z]
                let slice = grid.slice(s![
                    self.max.x as usize - 1,
                    self.min.y as usize..self.max.y as usize,
                    self.max.z as usize - 1,
                ]);
                slice
                    .into_dimensionality::<Ix2>()
                    .expect("Failed to cast EAST_UP edge to 2D")
            },
            Dir::SouthUp => {
                // fixed Y = min.y, so slice [X, Z]
                let slice = grid.slice(s![
                    self.min.x as usize..self.max.x as usize,
                    self.min.y as usize,
                    self.max.z as usize - 1,
                ]);
                slice
                    .into_dimensionality::<Ix2>()
                    .expect("Failed to cast SOUTH_UP edge to 2D")
            },
            Dir::WestUp => {
                // fixed X = min.x, so slice [Y, Z]
                let slice = grid.slice(s![
                    self.min.x as usize,
                    self.min.y as usize..self.max.y as usize,
                    self.max.z as usize - 1,
                ]);
                slice
                    .into_dimensionality::<Ix2>()
                    .expect("Failed to cast WEST_UP edge to 2D")
            },
            Dir::NorthDown => {
                let y = self.max.y as usize - 1;
                grid.slice(s![
                    self.min.x as usize..self.max.x as usize,
                    y,
                    self.min.z as usize
                ])
                .into_dimensionality::<Ix2>()
                .expect("Failed to cast NORTH_DOWN edge to 2D")
            },
            Dir::EastDown => {
                // fixed X = max.x - 1, so slice [Y, Z]
                let slice = grid.slice(s![
                    self.max.x as usize - 1,
                    self.min.y as usize..self.max.y as usize,
                    self.min.z as usize,
                ]);
                slice
                    .into_dimensionality::<Ix2>()
                    .expect("Failed to cast EAST_DOWN edge to 2D")
            },
            Dir::SouthDown => {
                // fixed Y = min.y, so slice [X, Z]
                let slice = grid.slice(s![
                    self.min.x as usize..self.max.x as usize,
                    self.min.y as usize,
                    self.min.z as usize,
                ]);
                slice
                    .into_dimensionality::<Ix2>()
                    .expect("Failed to cast SOUTH_DOWN edge to 2D")
            },
            Dir::WestDown => {
                // fixed X = min.x, so slice [Y, Z]
                let slice = grid.slice(s![
                    self.min.x as usize,
                    self.min.y as usize..self.max.y as usize,
                    self.min.z as usize,
                ]);
                slice
                    .into_dimensionality::<Ix2>()
                    .expect("Failed to cast WEST_DOWN edge to 2D")
            },
            _ => panic!("Diagonal directions do not correspond to a single edge"),
        }
    }*/

    pub(crate) fn face<'a>(&self, grid: &'a Array3<NavCell>, dir: Dir) -> ArrayView2<'a, NavCell> {
        match dir {
            Dir::North => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.max.y as usize - 1,
                self.min.z as usize..self.max.z as usize,
            ]),
            Dir::East => grid.slice(s![
                self.max.x as usize - 1,
                self.min.y as usize..self.max.y as usize,
                self.min.z as usize..self.max.z as usize,
            ]),
            Dir::South => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.min.y as usize,
                self.min.z as usize..self.max.z as usize,
            ]),
            Dir::West => grid.slice(s![
                self.min.x as usize,
                self.min.y as usize..self.max.y as usize,
                self.min.z as usize..self.max.z as usize,
            ]),
            Dir::Up => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.min.y as usize..self.max.y as usize,
                self.max.z as usize - 1,
            ]),
            Dir::Down => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.min.y as usize..self.max.y as usize,
                self.min.z as usize,
            ]),
            _ => panic!("{dir:?} does not correspond to a face."),
        }
    }

    pub(crate) fn edge<'a>(&self, grid: &'a Array3<NavCell>, dir: Dir) -> ArrayView1<'a, NavCell> {
        match dir {
            Dir::NorthUp => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.max.y as usize - 1,
                self.max.z as usize - 1
            ]),
            Dir::EastUp => grid.slice(s![
                self.max.x as usize - 1,
                self.min.y as usize..self.max.y as usize,
                self.max.z as usize - 1
            ]),
            Dir::SouthUp => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.min.y as usize,
                self.max.z as usize - 1
            ]),
            Dir::WestUp => grid.slice(s![
                self.min.x as usize,
                self.min.y as usize..self.max.y as usize,
                self.max.z as usize - 1
            ]),
            Dir::NorthDown => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.max.y as usize - 1,
                self.min.z as usize
            ]),
            Dir::EastDown => grid.slice(s![
                self.max.x as usize - 1,
                self.min.y as usize..self.max.y as usize,
                self.min.z as usize
            ]),
            Dir::SouthDown => grid.slice(s![
                self.min.x as usize..self.max.x as usize,
                self.min.y as usize,
                self.min.z as usize
            ]),
            Dir::WestDown => grid.slice(s![
                self.min.x as usize,
                self.min.y as usize..self.max.y as usize,
                self.min.z as usize
            ]),
            _ => panic!("{dir:?} does not correspond to an edge."),
        }
    }

    /// Returns the chunk corner `NavCell` for the given ordinal direction.
    pub(crate) fn corner(&self, grid: &Array3<NavCell>, dir: Dir) -> NavCell {
        match dir {
            Dir::NorthEast => grid[[
                self.max.x as usize - 1,
                self.max.y as usize - 1,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::SouthEast => grid[[
                self.max.x as usize - 1,
                self.min.y as usize,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::SouthWest => grid[[
                self.min.x as usize,
                self.min.y as usize,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::NorthWest => grid[[
                self.min.x as usize,
                self.max.y as usize - 1,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::NorthEastUp => grid[[
                self.max.x as usize - 1,
                self.max.y as usize - 1,
                self.max.z as usize - 1,
            ]]
            .clone(),
            Dir::SouthEastUp => grid[[
                self.max.x as usize - 1,
                self.min.y as usize,
                self.max.z as usize - 1,
            ]]
            .clone(),
            Dir::SouthWestUp => grid[[
                self.min.x as usize,
                self.min.y as usize,
                self.max.z as usize - 1,
            ]]
            .clone(),
            Dir::NorthWestUp => grid[[
                self.min.x as usize,
                self.max.y as usize - 1,
                self.max.z as usize - 1,
            ]]
            .clone(),
            Dir::NorthEastDown => grid[[
                self.max.x as usize - 1,
                self.max.y as usize - 1,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::SouthEastDown => grid[[
                self.max.x as usize - 1,
                self.min.y as usize,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::SouthWestDown => grid[[
                self.min.x as usize,
                self.min.y as usize,
                self.min.z as usize,
            ]]
            .clone(),
            Dir::NorthWestDown => grid[[
                self.min.x as usize,
                self.max.y as usize - 1,
                self.min.z as usize,
            ]]
            .clone(),
            _ => panic!("{dir:?} does not correspond to a corner."),
        }
    }

    pub(crate) fn touching_edges(&self, pos: UVec3) -> impl Iterator<Item = Dir> + '_ {
        Dir::all().filter(move |&dir| {
            let offset = dir.offset();
            let (dx, dy, dz) = (offset.x, offset.y, offset.z);

            let x_match = match dx {
                -1 => pos.x == self.min().x,
                1 => pos.x + 1 == self.max().x,
                _ => true, // direction doesn't check x
            };

            let y_match = match dy {
                -1 => pos.y == self.min().y,
                1 => pos.y + 1 == self.max().y,
                _ => true,
            };

            let z_match = match dz {
                -1 => pos.z == self.min().z,
                1 => pos.z + 1 == self.max().z,
                _ => true,
            };

            x_match && y_match && z_match
        })
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_chunk_view() {
        let grid = Array3::from_elem((10, 10, 10), NavCell::default());
        let chunk = Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(4, 4, 4));
        let view = chunk.view(&grid);

        assert_eq!(view.shape(), [4, 4, 4]);
    }

    #[test]
    fn test_chunk_edge() {
        let grid = Array3::from_elem((10, 10, 10), NavCell::default());
        let chunk = Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(4, 4, 4));
        let edge = chunk.face(&grid, Dir::North);

        assert_eq!(edge.shape(), [4, 4]);
    }

    #[test]
    fn test_touching_edges() {
        let chunk = Chunk::new((0, 0, 0), UVec3::new(0, 0, 0), UVec3::new(4, 4, 4));
        let pos = UVec3::new(3, 2, 0);

        let edges: Vec<Dir> = chunk.touching_edges(pos).collect();
        assert!(!edges.contains(&Dir::West));
        assert!(!edges.contains(&Dir::North));
        assert!(edges.contains(&Dir::East));
        assert!(!edges.contains(&Dir::South));
    }
}
