use std::collections::VecDeque;

use bevy::math::UVec3;
use bevy::prelude::Component;

#[derive(Debug, Clone, Component)]
pub struct Path {
    pub(crate) path: VecDeque<UVec3>,
    pub(crate) graph_path: VecDeque<UVec3>,
    cost: u32,
    is_reversed: bool,
}

impl Path {
    pub fn new(path: Vec<UVec3>, cost: u32) -> Self {
        let path = path.into_iter().collect();

        Path {
            path,
            graph_path: VecDeque::new(),
            cost,
            is_reversed: false,
        }
    }

    pub fn from_slice(path: &[UVec3], cost: u32) -> Self {
        let path = path.iter().cloned().collect();

        Path {
            path: path,
            graph_path: VecDeque::new(),
            cost,
            is_reversed: false,
        }
    }

    pub fn is_position_in_path(&self, pos: UVec3) -> bool {
        self.path.contains(&pos)
    }

    pub fn path(&self) -> &[UVec3] {
        &self.path.as_slices().0
    }

    pub fn cost(&self) -> u32 {
        self.cost
    }

    pub fn len(&self) -> usize {
        self.path.len()
    }

    pub fn is_empty(&self) -> bool {
        self.path.is_empty()
    }

    pub fn reverse(&mut self) {
        self.path.make_contiguous().reverse();
        self.is_reversed = !self.is_reversed;
    }

    pub fn pop(&mut self) -> Option<UVec3> {
        // Remove the first element of the path
        self.path.pop_front()
    }
}

impl PartialEq for Path {
    fn eq(&self, other: &Self) -> bool {
        self.path == other.path
    }
}

impl Eq for Path {}

// Implement iter for Path
impl IntoIterator for Path {
    type Item = UVec3;
    type IntoIter = std::collections::vec_deque::IntoIter<UVec3>;

    fn into_iter(self) -> Self::IntoIter {
        self.path.into_iter()
    }
}
