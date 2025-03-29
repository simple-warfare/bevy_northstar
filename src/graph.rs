use bevy::{math::UVec3, utils::hashbrown::HashMap};

use crate::chunk::Chunk;
use crate::dir::Dir;
use crate::node::Node;
use crate::path::Path;
use crate::NodeId;

pub struct Graph {
    nodes: slab::Slab<Node>,
    node_ids: HashMap<UVec3, NodeId>,
}

impl Graph {
    pub fn new() -> Self {
        Graph {
            nodes: slab::Slab::new(),
            node_ids: HashMap::new(),
        }
    }

    pub fn get_node(&self, pos: UVec3) -> Option<&Node> {
        self.node_ids.get(&pos).and_then(|&id| self.nodes.get(id))
    }

    pub fn get_nodes(&self) -> Vec<&Node> {
        self.nodes.iter().map(|(_, node)| node).collect()
    }

    pub fn get_nodes_in_chunk(&self, chunk: Chunk) -> Vec<&Node> {
        let nodes = self
            .nodes
            .iter()
            .filter(|(_, node)| node.chunk == chunk)
            .map(|(_, node)| node)
            .collect();

        nodes
    }

    pub fn get_closest_node_in_chunk(&self, pos: UVec3, chunk: Chunk) -> Option<&Node> {
        let node = self
            .get_nodes_in_chunk(chunk)
            .iter()
            .min_by_key(|node| {
                let dx = (node.pos.x as i32 - pos.x as i32).abs();
                let dy = (node.pos.y as i32 - pos.y as i32).abs();
                let dz = (node.pos.z as i32 - pos.z as i32).abs();
                (dx + dy + dz) as u32
            })
            .cloned();

        node
    }

    pub fn get_nodes_in_range(&self, range: Vec<UVec3>) -> Vec<&Node> {
        let nodes = self
            .nodes
            .iter()
            .filter(|(_, node)| range.contains(&node.pos))
            .map(|(_, node)| node)
            .collect();

        nodes
    }

    pub fn add_node(&mut self, pos: UVec3, chunk: Chunk, dir: Option<Dir>) -> usize {
        if let Some(&id) = self.node_ids.get(&pos) {
            return id;
        }

        let node = Node::new(pos, chunk, dir);
        let id = self.nodes.insert(node.clone());
        self.node_ids.insert(pos, id);

        id
    }

    pub fn add_nodes(&mut self, nodes: &Vec<Node>) {
        for node in nodes {
            self.node_ids
                .insert(node.pos, self.nodes.insert(node.clone()));
        }
    }

    pub fn remove_node(&mut self, pos: UVec3) {
        if let Some(id) = self.node_ids.remove(&pos) {
            self.nodes.remove(id);
        }
    }

    pub fn connect_node(&mut self, from: UVec3, to: UVec3, path: Path) {
        if let Some(&from_id) = self.node_ids.get(&from) {
            self.nodes.get_mut(from_id).unwrap().edges.insert(to, path);
        }
    }

    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    pub fn edge_count(&self) -> usize {
        self.nodes.iter().map(|(_, node)| node.edges.len()).sum()
    }

    pub fn clear_duplicates(&mut self) {
        let mut to_remove = Vec::new();
        for (id, node) in self.nodes.iter() {
            if let Some(&other_id) = self.node_ids.get(&node.pos) {
                if id != other_id {
                    to_remove.push(id);
                }
            }
        }

        for id in to_remove {
            self.nodes.remove(id);
        }
    }

    pub fn get_edge_cost(&self, from: UVec3, to: UVec3) -> Option<u32> {
        self.node_ids
            .get(&from)
            .and_then(|&from_id| self.nodes.get(from_id))
            .and_then(|node| node.edges.get(&to).map(|path| path.cost()))
    }

    pub fn get_all_nodes_in_chunk(&self, chunk: Chunk) -> Vec<&Node> {
        let nodes = self
            .nodes
            .iter()
            .filter(|(_, node)| node.chunk == chunk)
            .map(|(_, node)| node)
            .collect();

        nodes
    }

    pub fn get_all_paths(&self) -> Vec<Path> {
        let mut paths = Vec::new();
        for node in self.nodes.iter().map(|(_, node)| node) {
            for path in node.edges.values() {
                paths.push(path.clone());
            }
        }

        paths
    }

    pub fn get_node_at(&self, pos: UVec3) -> Option<&Node> {
        self.node_ids.get(&pos).and_then(|&id| self.nodes.get(id))
    }
}

#[cfg(test)]

mod tests {
    use super::*;

    #[test]
    fn test_add_node() {
        let mut graph = Graph::new();
        let pos = UVec3::new(0, 0, 0);
        let chunk = Chunk::new(UVec3::new(0, 0, 0), UVec3::new(16, 16, 16));
        let id = graph.add_node(pos, chunk.clone(), None);

        let node = graph.get_node(pos).unwrap();
        assert_eq!(node.pos, pos);
        assert_eq!(node.chunk, chunk);
        assert_eq!(node.dir, None);
        assert_eq!(id, 0);
    }

    #[test]
    fn test_get_all_nodes_in_chunk() {
        let mut graph = Graph::new();
        let pos1 = UVec3::new(0, 0, 0);
        let pos2 = UVec3::new(1, 1, 1);
        let chunk = Chunk::new(UVec3::new(0, 0, 0), UVec3::new(16, 16, 16));
        graph.add_node(pos1, chunk.clone(), None);
        graph.add_node(pos2, chunk.clone(), None);

        let nodes = graph.get_all_nodes_in_chunk(chunk);
        assert_eq!(nodes.len(), 2);
        assert_eq!(nodes[0].pos, pos1);
        assert_eq!(nodes[1].pos, pos2);
    }

    #[test]
    fn test_get_closest_node_in_chunk() {
        let mut graph = Graph::new();
        let pos1 = UVec3::new(0, 0, 0);
        let pos2 = UVec3::new(1, 1, 1);
        let chunk = Chunk::new(UVec3::new(0, 0, 0), UVec3::new(16, 16, 16));
        graph.add_node(pos1, chunk.clone(), None);
        graph.add_node(pos2, chunk.clone(), None);

        let pos = UVec3::new(0, 1, 0);
        let node = graph.get_closest_node_in_chunk(pos, chunk).unwrap();
        assert_eq!(node.pos, pos1);
    }

    #[test]
    fn test_get_nodes_in_range() {
        let mut graph = Graph::new();
        let pos1 = UVec3::new(0, 0, 0);
        let pos2 = UVec3::new(1, 1, 1);
        let pos3 = UVec3::new(2, 2, 2);
        let chunk = Chunk::new(UVec3::new(0, 0, 0), UVec3::new(16, 16, 16));
        graph.add_node(pos1, chunk.clone(), None);
        graph.add_node(pos2, chunk.clone(), None);
        graph.add_node(pos3, chunk.clone(), None);

        let range = vec![pos1, pos2];
        let nodes = graph.get_nodes_in_range(range);
        assert_eq!(nodes.len(), 2);
        assert_eq!(nodes[0].pos, pos1);
        assert_eq!(nodes[1].pos, pos2);
    }
}
