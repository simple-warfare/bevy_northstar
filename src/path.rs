//! This module defines the important `Path` component.
use bevy::math::UVec3;
use bevy::prelude::Component;
use std::collections::VecDeque;

/// The path component. This is inserted to an entity after the plugin
/// systems have pathfound to the goal position.
///
/// `Path` will also be returned if you manually call pathfinding functions.
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

    /// Create a new path from a slice of `UVec3` positions
    pub fn from_slice(path: &[UVec3], cost: u32) -> Self {
        let path = path.iter().cloned().collect();

        Path {
            path,
            graph_path: VecDeque::new(),
            cost,
            is_reversed: false,
        }
    }

    /// Returns true if the path contains the given position
    pub fn is_position_in_path(&self, pos: UVec3) -> bool {
        self.path.contains(&pos)
    }

    /// Returns the path as a slice of `UVec3` positions.
    /// Useful to represent the path for UI etc.
    ///
    /// # Example
    ///
    /// ```rust
    /// use bevy::prelude::*;
    /// use bevy_northstar::prelude::*;
    ///
    /// let path = Path::new(vec![UVec3::new(1, 2, 3), UVec3::new(4, 5, 6)], 10);
    /// assert_eq!(path.path(), &[UVec3::new(1, 2, 3), UVec3::new(4, 5, 6)]);
    /// ```
    pub fn path(&self) -> &[UVec3] {
        self.path.as_slices().0
    }

    /// Returns the cost of the path
    pub fn cost(&self) -> u32 {
        self.cost
    }

    /// Returns the length of the path
    pub fn len(&self) -> usize {
        self.path.len()
    }

    /// Returns true if the path is empty
    pub fn is_empty(&self) -> bool {
        self.path.is_empty()
    }

    /// Reverse the path in place.
    pub fn reverse(&mut self) {
        self.path.make_contiguous().reverse();
        self.is_reversed = !self.is_reversed;
    }

    /// Pops the first position of the path.
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
