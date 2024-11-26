use std::cmp::Ordering;
use std::hash::BuildHasherDefault;

use bevy::math::UVec3;
use dir::Dir;
use indexmap::IndexMap;
use rustc_hash::FxHasher;

mod astar;
mod chunk;
mod dijkstra;
mod dir;
mod graph;
pub mod grid;
mod neighbor;
mod node;
pub mod path;
pub mod plugin;
pub mod debug;

pub mod prelude {
    pub use crate::grid::{Grid, GridSettings};
    pub use crate::Point;
    pub use crate::plugin::NorthstarPlugin;
    pub use crate::debug::{MapType, NorthstarDebugConfig, NorthstarDebugPlugin};
    pub use crate::path::Path;
    pub use crate::neighbor::*;
}

pub type NodeId = usize;

type FxIndexMap<K, V> = IndexMap<K, V, BuildHasherDefault<FxHasher>>;

#[derive(Debug, Clone)]
pub struct GridCoords {
    pub x: usize,
    pub y: usize,
    pub z: usize,
}

impl GridCoords {
    pub fn new(x: usize, y: usize, z: usize) -> Self {
        GridCoords { x, y, z }
    }

    pub fn from_uvec3(uvec3: UVec3) -> Self {
        GridCoords {
            x: uvec3.x as usize,
            y: uvec3.y as usize,
            z: uvec3.z as usize,
        }
    }

    pub fn to_uvec3(&self) -> UVec3 {
        UVec3::new(self.x as u32, self.y as u32, self.z as u32)
    }
}


#[derive(Debug, Default, Clone, Copy)]
pub struct Point {
    pub cost: u32,
    pub wall: bool,
    pub ramp: bool,
}

impl Point {
    pub fn new(cost: u32, wall: bool) -> Self {
        Point {
            cost,
            wall,
            ramp: false,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Edge {
    pub start: UVec3,
    pub end: UVec3,
    pub dir: Dir,
    pub cost: u32,
    pub walkable: bool,
}

impl Edge {
    pub fn new(start: UVec3, end: UVec3, dir: Dir, cost: u32, walkable: bool) -> Self {
        Edge {
            start,
            end,
            dir,
            cost,
            walkable,
        }
    }
}

pub(crate) struct SmallestCostHolder<Id> {
    estimated_cost: Id,
    cost: Id,
    index: usize,
}

impl<Id: PartialEq> PartialEq for SmallestCostHolder<Id> {
    fn eq(&self, other: &Self) -> bool {
        self.estimated_cost.eq(&other.estimated_cost) && self.cost.eq(&other.cost)
    }
}

impl<Id: Eq> Eq for SmallestCostHolder<Id> {}

impl<Id: Ord> PartialOrd for SmallestCostHolder<Id> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<Id: Ord> Ord for SmallestCostHolder<Id> {
    fn cmp(&self, other: &Self) -> Ordering {
        match other.estimated_cost.cmp(&self.estimated_cost) {
            Ordering::Equal => self.cost.cmp(&other.cost),
            s => s,
        }
    }
}
