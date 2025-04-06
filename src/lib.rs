#![doc = include_str!("../README.md")]

use indexmap::IndexMap;
use rustc_hash::FxHasher;
use std::cmp::Ordering;
use std::hash::BuildHasherDefault;

mod astar;
mod chunk;
pub mod components;
pub mod debug;
mod dijkstra;
pub mod dir;
mod graph;
pub mod grid;
pub mod neighbor;
mod node;
pub mod path;
pub mod pathfind;
pub mod plugin;
pub mod raycast;

pub mod prelude {
    pub use crate::components::*;
    pub use crate::debug::{DebugMapType, NorthstarDebugPlugin};
    pub use crate::dir::Dir;
    pub use crate::grid::{Grid, GridSettings, Point};
    pub use crate::neighbor::*;
    pub use crate::path::Path;
    pub use crate::plugin::NorthstarPlugin;
    pub use crate::plugin::NorthstarSettings;
    pub use crate::plugin::PathingSet;
    pub use crate::plugin::Stats;
}

pub(crate) type NodeId = usize;

type FxIndexMap<K, V> = IndexMap<K, V, BuildHasherDefault<FxHasher>>;

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
