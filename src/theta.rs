use std::collections::BinaryHeap;

use bevy::math::UVec3;
use indexmap::map::Entry::{Occupied, Vacant};
use ndarray::ArrayView3;

use crate::graph::Graph;
use crate::los::line_of_sight;
use crate::{neighbor::Neighborhood, path::Path, FxIndexMap, Point, SmallestCostHolder};

pub fn theta_grid<N: Neighborhood>(
    neighborhood: &N,
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

    while let Some(SmallestCostHolder { cost, index, .. }) = to_visit.pop() {
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
            let neighbor_point = &grid[[
                neighbor.x as usize,
                neighbor.y as usize,
                neighbor.z as usize,
            ]];

            if neighbor_point.wall || neighbor_point.cost == 0 {
                continue;
            }

            let (parent_pos, (parent_index, parent_cost)) = visited.get_index(index).unwrap();

            let (new_cost, parent) = if line_of_sight(grid, *parent_pos, neighbor) {
                // Line of sight exists between grandparent and neighbor
                (
                    parent_cost + neighbor_point.cost,
                    *parent_index,
                )
            } else {
                // No line of sight; use current node as parent
                (
                    cost + neighbor_point.cost,
                    index,
                )
            };

            let h;
            let n;
            match visited.entry(neighbor) {
                Vacant(e) => {
                    h = neighborhood.heuristic(neighbor, goal);
                    n = e.index();
                    e.insert((parent, new_cost));
                }
                Occupied(mut e) => {
                    if e.get().1 > new_cost {
                        h = neighborhood.heuristic(neighbor, goal);
                        n = e.index();
                        e.insert((parent, new_cost));
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

pub fn theta_graph<N: Neighborhood>(
    neighborhood: &N,
    graph: &Graph,
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

    while let Some(SmallestCostHolder { cost, index, .. }) = to_visit.pop() {
        let (neighbors, current_pos) = {
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
            let neighbors = node.get_edges();

            (neighbors, current_pos.clone())
        };

        for &neighbor in neighbors.iter() {
            let neighbor_node = graph.get_node(neighbor).unwrap();

            if neighbor_node.edges.is_empty() {
                continue;
            }

            let (parent_pos, (parent_index, parent_cost)) = visited.get_index(index).unwrap();

            let neighbor_cost = neighborhood.heuristic(neighbor, goal);

            let (new_cost, parent) = if line_of_sight(grid, *parent_pos, neighbor) {
                (
                    parent_cost + neighbor_cost,
                    *parent_index,
                )
            } else {
                (
                    cost + neighbor_cost,
                    index,
                )
            };

            let h;
            let n;
            match visited.entry(neighbor) {
                Vacant(e) => {
                    h = neighborhood.heuristic(neighbor, goal);
                    n = e.index();
                    e.insert((parent, new_cost));
                }
                Occupied(mut e) => {
                    if e.get().1 > new_cost {
                        h = neighborhood.heuristic(neighbor, goal);
                        n = e.index();
                        e.insert((parent, new_cost));
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
    use crate::{grid::{Grid, GridSettings}, neighbor::OrdinalNeighborhood3d};

    #[test]
    fn test_theta_grid() {
        let mut grid = ndarray::Array3::from_elem((3, 3, 3), Point::new(1, false));
        grid[[1, 1, 1]].cost = 1;

        let start = UVec3::new(0, 0, 0);
        let goal = UVec3::new(2, 2, 2);

        let path = theta_grid(&OrdinalNeighborhood3d, &grid.view(), start, goal, 64).unwrap();

        assert_eq!(path.cost(), 2);
        assert_eq!(path.len(), 3);
        assert_eq!(path.path()[0], start);
        assert_eq!(path.path()[2], goal);
    }

    #[test]
    fn test_theta_grid_with_wall() {
        let mut grid = ndarray::Array3::from_elem((3, 3, 3), Point::new(1, false));
        grid[[1, 1, 1]].cost = 1;
        grid[[1, 1, 1]].wall = true;

        let start = UVec3::new(0, 0, 0);
        let goal = UVec3::new(2, 2, 2);

        let path = theta_grid(&OrdinalNeighborhood3d, &grid.view(), start, goal, 64).unwrap();
        assert_eq!(path.cost(), 3);
        assert_eq!(path.len(), 3);
    }

    #[test]
    fn test_astar_grid_large() {
        let mut grid = ndarray::Array3::from_elem((100, 100, 100), Point::new(1, false));
        grid[[1, 1, 1]].cost = 1;

        let start = UVec3::new(0, 0, 0);
        let goal = UVec3::new(99, 99, 99);

        let path = theta_grid(&OrdinalNeighborhood3d, &grid.view(), start, goal, 64);

        assert!(path.is_some());
        assert!(path.unwrap().len() > 0)
    }

    #[test]
    fn test_theta_graph() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GridSettings {
            width: 12,
            height: 12,
            depth: 1,
            chunk_depth: 1,
            chunk_size: 4,
            chunk_ordinal: true,
            default_cost: 1,
            default_wall: false,
            jump_height: 1,
        });

        grid.build();

        let start = UVec3::new(2, 3, 0);
        let goal = UVec3::new(8, 10, 0);

        let path = theta_graph(&OrdinalNeighborhood3d, &grid.graph, &grid.get_view(), start, goal, 64).unwrap();
    
        assert_eq!(path.cost(), 21);
        assert_eq!(path.len(), 6);
    }
}