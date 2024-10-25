use std::{collections::BinaryHeap, hash::BuildHasherDefault};

use bevy::math::UVec3;
use indexmap::IndexMap;
use indexmap::map::Entry::{Occupied, Vacant};
use rustc_hash::FxHasher;
use ndarray::ArrayView3;

use crate::graph::Graph;
use crate::{neighbor, FxIndexMap};
use crate::{neighbor::Neighborhood, path::Path, Point, SmallestCostHolder};

pub fn astar_grid<N: Neighborhood>(
    neighborhood: N,
    grid: &ArrayView3<Point>,
    start: UVec3,
    goal: UVec3,
    size_hint: usize,
) -> Option<Path> {
    let mut to_visit = BinaryHeap::with_capacity(size_hint / 2);
    to_visit.push(SmallestCostHolder {
        estimated_cost: 0,
        cost: 0,
        index: 0,
    });

    let mut visited: FxIndexMap<UVec3, (usize, u32)> = FxIndexMap::default();
    visited.insert(start, (usize::MAX, 0));

    while let Some(SmallestCostHolder {cost, index, .. }) = to_visit.pop() {
        let neighbors = {
            let (current_pos, &(_, current_cost)) = visited.get_index(index).unwrap();
            if *current_pos == goal {
                let mut current = index;
                let mut steps = vec![];

                while current != usize::MAX {
                    let (pos, _) = visited.get_index(current).unwrap();
                    steps.push(*pos);
                    current = visited.get(pos).unwrap().0;
                }

                steps.reverse();
                return Some(Path::new(steps, current_cost));
            }

            if cost > current_cost {
                continue;
            }

            let mut neighbors = vec![];
            neighborhood.neighbors(grid, *current_pos, &mut neighbors);
            neighbors
        };

        for &neighbor in neighbors.iter() {
            let neighbor_point = &grid[[neighbor.x as usize, neighbor.y as usize, neighbor.z as usize]];

            if neighbor_point.wall || neighbor_point.cost == 0 {
                continue;
            }
            let new_cost = cost + neighbor_point.cost;
            let h;
            let n;
            match visited.entry(neighbor) {
                Vacant(e) => {
                    h = neighborhood.heuristic(neighbor, goal);
                    n = e.index();
                    e.insert((index, new_cost));
                }
                Occupied(mut e) => {
                    if e.get().1 > new_cost {
                        h = neighborhood.heuristic(neighbor, goal);
                        n = e.index();
                        e.insert((index, new_cost));             
                    } else {
                        continue;
                    }
                }
            }

            to_visit.push(SmallestCostHolder {
                estimated_cost: h,
                cost: new_cost,
                index: n,
            });
        }
    }

    None
}

pub fn astar_graph<N: Neighborhood>(
    neighborhood: N,
    graph: &Graph,
    start: UVec3,
    goal: UVec3,
    size_hint: usize,
) -> Option<Path> {
    let mut to_visit = BinaryHeap::with_capacity(size_hint / 2);
    to_visit.push(SmallestCostHolder {
        estimated_cost: 0,
        cost: 0,
        index: 0,
    });

    let mut visited: FxIndexMap<UVec3, (usize, u32)> = FxIndexMap::default();
    visited.insert(start, (usize::MAX, 0));

    while let Some(SmallestCostHolder {cost, index, .. }) = to_visit.pop() {
        let neighbors = {
            let (current_pos, &(_, current_cost)) = visited.get_index(index).unwrap();
            if *current_pos == goal {
                let mut current = index;
                let mut steps = vec![];

                while current != usize::MAX {
                    let (pos, _) = visited.get_index(current).unwrap();
                    steps.push(*pos);
                    current = visited.get(pos).unwrap().0;
                }

                steps.reverse();
                return Some(Path::new(steps, current_cost));
            }

            if cost > current_cost {
                continue;
            }

            let node = graph.get_node(*current_pos).unwrap();
            node.get_edges()
        };

        for neighbor in neighbors.iter() {
            let neighbor_node = graph.get_node(*neighbor).unwrap();
            if neighbor_node.edges.is_empty() {
                continue;
            }

            let new_cost = cost + 1;
            let h;
            let n;
            match visited.entry(neighbor_node.pos) {
                Vacant(e) => {
                    h = neighborhood.heuristic(neighbor_node.pos, goal);
                    n = e.index();
                    e.insert((index, new_cost));
                }
                Occupied(mut e) => {
                    if e.get().1 > new_cost {
                        h = neighborhood.heuristic(neighbor_node.pos, goal);
                        n = e.index();
                        e.insert((index, new_cost));             
                    } else {
                        continue;
                    }
                }
            }

            to_visit.push(SmallestCostHolder {
                estimated_cost: h,
                cost: new_cost,
                index: n,
            });
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::neighbor::OrdinalNeighborhood3d;
    use crate::chunk::Chunk;

    #[test]
    fn test_astar_grid() {
        let mut grid = ndarray::Array3::from_elem((3, 3, 3), Point::new(1, false));
        grid[[1, 1, 1]].cost = 1;

        let start = UVec3::new(0, 0, 0);
        let goal = UVec3::new(2, 2, 2);

        let path = astar_grid(OrdinalNeighborhood3d, &grid.view(), start, goal, 64).unwrap();

        assert_eq!(path.cost(), 2);
        assert_eq!(path.len(), 3);
        assert_eq!(path.path()[0], start);
        assert_eq!(path.path()[2], goal);
    }

    #[test]
    fn test_astar_grid_with_wall() {
        let mut grid = ndarray::Array3::from_elem((3, 3, 3), Point::new(1, false));
        grid[[1, 1, 1]].cost = 1;
        grid[[1, 1, 1]].wall = true;

        let start = UVec3::new(0, 0, 0);
        let goal = UVec3::new(2, 2, 2);

        let path = astar_grid(OrdinalNeighborhood3d, &grid.view(), start, goal, 64).unwrap();

        assert_eq!(path.cost(), 3);
        assert_eq!(path.len(), 4);
        assert_eq!(path.path()[0], start);
        assert_eq!(path.path()[3], goal);
        assert_eq!(path.is_position_in_path(UVec3::new(1, 1, 1)), false);
    }

    #[test]
    fn test_astar_grid_8x8() {
        let grid = ndarray::Array3::from_elem((8, 8, 8), Point::new(1, false));

        let start = UVec3::new(0, 0, 0);
        let goal = UVec3::new(7, 7, 7);

        let path = astar_grid(OrdinalNeighborhood3d, &grid.view(), start, goal, 16).unwrap();

        assert_eq!(path.len(), 8);
    }

    #[test]
    fn test_astar_graph() {
        let mut graph = Graph::new();

        let _ = graph.add_node(UVec3::new(0, 0, 0), Chunk::new(UVec3::new(0, 0, 0), UVec3::new(16, 16, 16)), None);
        let _ = graph.add_node(UVec3::new(1, 1, 1), Chunk::new(UVec3::new(0, 0, 0), UVec3::new(16, 16, 16)), None);
        let _ = graph.add_node(UVec3::new(2, 2, 2), Chunk::new(UVec3::new(0, 0, 0), UVec3::new(16, 16, 16)), None);

        graph.connect_node(UVec3::new(0, 0, 0), UVec3::new(1, 1, 1), Path::new(vec![UVec3::new(0, 0, 0), UVec3::new(1, 1, 1)], 1));
        graph.connect_node(UVec3::new(1, 1, 1), UVec3::new(0, 0, 0), Path::new(vec![UVec3::new(1, 1, 1), UVec3::new(0, 0, 0)], 1));
        graph.connect_node(UVec3::new(1, 1, 1), UVec3::new(2, 2, 2), Path::new(vec![UVec3::new(1, 1, 1), UVec3::new(2, 2, 2)], 1));
        graph.connect_node(UVec3::new(2, 2, 2), UVec3::new(1, 1, 1), Path::new(vec![UVec3::new(2, 2, 2), UVec3::new(1, 1, 1)], 1));

        let path = astar_graph(OrdinalNeighborhood3d, &graph, UVec3::new(0, 0, 0), UVec3::new(2, 2, 2), 64).unwrap();

        assert_eq!(path.cost(), 2);
        assert_eq!(path.len(), 3);
        assert_eq!(path.path()[0], UVec3::new(0, 0, 0));
        assert_eq!(path.path()[2], UVec3::new(2, 2, 2));
    }
}