use bevy::{math::UVec3, utils::hashbrown::HashMap};
use std::hash::{Hash, Hasher};

use crate::chunk::Chunk;
use crate::dir::Dir;
use crate::path::Path;

#[derive(Debug, Clone)]
pub struct Node {
    pub pos: UVec3,
    pub chunk: Chunk,
    pub edges: HashMap<UVec3, Path>,
    pub dir: Option<Dir>,
}

impl Node {
    pub fn new(pos: UVec3, chunk: Chunk, dir: Option<Dir>) -> Self {
        Node {
            pos,
            chunk,
            dir,
            edges: HashMap::new(),
        }
    }

    pub fn get_edges(&self) -> Vec<UVec3> {
        self.edges.keys().cloned().collect()
    }
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.pos == other.pos
    }
}

impl Eq for Node {}

impl Hash for Node {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.pos.hash(state);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_eq() {
        let node1 = Node::new(
            UVec3::new(1, 2, 3),
            Chunk::new(UVec3::new(0, 0, 0), UVec3::new(16, 16, 16)),
            None,
        );
        let node2 = Node::new(
            UVec3::new(1, 2, 3),
            Chunk::new(UVec3::new(0, 0, 0), UVec3::new(16, 16, 16)),
            None,
        );

        assert_eq!(node1, node2);
    }

    #[test]
    fn test_node_hash() {
        let node1 = Node::new(
            UVec3::new(1, 2, 3),
            Chunk::new(UVec3::new(0, 0, 0), UVec3::new(16, 16, 16)),
            None,
        );
        let node2 = Node::new(
            UVec3::new(1, 2, 3),
            Chunk::new(UVec3::new(0, 0, 0), UVec3::new(16, 16, 16)),
            None,
        );

        let mut hasher1 = std::collections::hash_map::DefaultHasher::new();
        let mut hasher2 = std::collections::hash_map::DefaultHasher::new();

        node1.hash(&mut hasher1);
        node2.hash(&mut hasher2);

        assert_eq!(hasher1.finish(), hasher2.finish());
    }
}
