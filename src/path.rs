use bevy::math::UVec3;
use bevy::prelude::Component;

#[derive(Debug, Clone, Component)]
pub struct Path {
    path: Vec<UVec3>,
    cost: u32,
    is_reversed: bool,
}

impl Path {
    pub fn new(path: Vec<UVec3>, cost: u32) -> Self {
        Path {
            path,
            cost,
            is_reversed: false,
        }
    }

    pub fn from_slice(path: &[UVec3], cost: u32) -> Self {
        Path {
            path: path.into(),
            cost,
            is_reversed: false,
        }
    }

    pub fn is_position_in_path(&self, pos: UVec3) -> bool {
        self.path.contains(&pos)
    }

    pub fn path(&self) -> &[UVec3] {
        &self.path
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
        self.path.reverse();
        self.is_reversed = !self.is_reversed;
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
    type IntoIter = std::vec::IntoIter<UVec3>;

    fn into_iter(self) -> Self::IntoIter {
        self.path.into_iter()
    }
}