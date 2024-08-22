use std::{cmp::Ordering, collections::BinaryHeap, hash::BuildHasherDefault};

use std::hash::Hash;

use bevy::prelude::Resource;
use indexmap::{IndexMap, map::Entry::{Occupied, Vacant}};
use rustc_hash::FxHasher;
use thiserror::Error;


#[derive(Error, Debug, PartialEq, Eq)]
pub enum PathfindingError {
    #[error("No path to goal found")]
    NoPathToGoal,
}

type FxIndexMap<K, V> = IndexMap<K, V, BuildHasherDefault<FxHasher>>;

#[allow(clippy::needless_collect)]
fn reverse_path<N, V, F>(parents: &FxIndexMap<N, V>, mut parent: F, start: usize) -> Vec<N>
where
    N: Eq + Hash + Clone,
    F: FnMut(&V) -> usize,
{
    let mut i = start;
    let path = std::iter::from_fn(|| {
        parents.get_index(i).map(|(node, value)| {
            i = parent(value);
            node
        })
    })
    .collect::<Vec<&N>>();
    // Collecting the going through the vector is needed to revert the path because the
    // unfold iterator is not double-ended due to its iterative nature.
    path.into_iter().rev().cloned().collect()
}

#[derive(Resource)]
pub struct Pathfinding {
    grid: Vec<Point>,
    width: usize,
    height: usize,
    depth: usize,
    heuristic: fn(GridPosition, GridPosition) -> usize,
}

impl Pathfinding {
    pub fn new(width: usize, height: usize, depth: usize, default_cost: usize, default_enabled: bool) -> Self {
        Pathfinding {
            grid: vec![Point::new(default_cost, default_enabled); width * height * depth],
            width,
            height,
            depth,
            heuristic: Pathfinding::default_heuristic
        }
    }

    pub fn init_grid(&mut self, default_cost: usize, default_enabled: bool) {
        self.grid = vec![Point::new(default_cost, default_enabled); self.width * self.height * self.depth];
    }

    fn default_heuristic(position: GridPosition, goal: GridPosition) -> usize {
        (
            (position.x - goal.x).abs() 
            + (position.y - goal.y).abs() 
            + (position.z - goal.z).abs()
        ) as usize
    }

    pub fn get_index(&self, x: usize, y: usize, z: usize) -> usize {
        x + y * self.width + z * self.width * self.height
    }

    fn succesors(&self, position: &GridPosition) -> Vec<(GridPosition, usize)> {
        let mut ret = Vec::with_capacity(32);
    
        for dx in -1i32..=1 {
            for dy in -1i32..=1 {
                for dz in -1i32..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }

                    let x = dx + position.x as i32;
                    if x < 0 || x > self.width as i32 - 1 {
                        continue;
                    }

                    let y = dy + position.y as i32;
                    if y < 0 || y > self.height as i32 - 1 {
                        continue;
                    }

                    let z = dz + position.z as i32;
                    if z < 0 || z > self.depth as i32 - 1 {
                        continue;
                    }

                    let point = self.grid[self.get_index(x as usize, y as usize, z as usize)].clone();
    
                    if !point.enabled {
                        continue;
                    }
    
                    ret.push(( 
                        GridPosition::new(x, y, z),
                        point.cost
                    ));
                }
            }
        }

        ret
    }

    pub fn get_path(&self, start: &GridPosition, goal: &GridPosition) 
    -> Result<
            (Vec<GridPosition>, usize), 
            PathfindingError
        > 
    {
        let mut open = BinaryHeap::new();
        open.push(SmallestCostHolder {
            estimated_cost: 0,
            cost: 0,
            index: 0,
        });

        let mut closed: FxIndexMap<GridPosition, (usize, usize)> = FxIndexMap::default();
        closed.insert(start.clone(), (usize::MAX, 0));

        while let Some(SmallestCostHolder { cost, index, .. }) = open.pop() {
            let successors = {
                let (node, &(_, closed_cost)) = closed.get_index(index).unwrap();
                if node == goal {
                    /*let mut i = index;
                    let path = std::iter::from_fn(|| {
                        closed.get_index(i).map(|(node, value)| {
                            i = value.0;
                            node
                        })
                    })
                    .collect::<Vec<&GridPosition>>();
                    // Collecting the going through the vector is needed to revert the path because the
                    // unfold iterator is not double-ended due to its iterative nature.
                    path.into_iter().rev().cloned().collect();*/
                    let path = reverse_path(&closed, |&(p, _)| p, index);
                    return Ok((path, cost));
                }

                if cost > closed_cost {
                    continue;
                }

                self.succesors(node)
            };
            for (successor, move_cost) in successors {
                let new_cost = cost + move_cost;
                let h; // Need to fix this
                let new_index;

                match closed.entry(successor) {
                    Vacant(e) => {
                        h = (self.heuristic)(e.key().clone(), goal.clone());
                        new_index = e.index();
                        e.insert((index, new_cost));
                    },
                    Occupied(mut e) => {
                        if e.get().1 > new_cost {
                            h = (self.heuristic)(e.key().clone(), goal.clone());
                            new_index = e.index();
                            e.insert((index, new_cost));
                        } else {
                            continue;
                        }
                    }
                }

                open.push(SmallestCostHolder {
                    estimated_cost: cost + h,
                    cost: new_cost,
                    index: new_index
                });
            }
        }

        Err(PathfindingError::NoPathToGoal)
    }
}

struct SmallestCostHolder {
    estimated_cost: usize,
    cost: usize,
    index: usize,
}

impl PartialEq for SmallestCostHolder {
    fn eq(&self, other: &Self) -> bool {
        self.estimated_cost.eq(&other.estimated_cost) && self.cost.eq(&other.cost)
    }
}

impl Eq for SmallestCostHolder {}

impl PartialOrd for SmallestCostHolder {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for SmallestCostHolder {
    fn cmp(&self, other: &Self) -> Ordering {
        match other.estimated_cost.cmp(&self.estimated_cost) {
            Ordering::Equal => self.cost.cmp(&other.cost),
            s => s,
        }
    }
}

#[derive(Default, Clone)]
struct Point {
    cost: usize,
    enabled: bool
}

impl Point {
    fn new(cost: usize, enabled: bool) -> Self {
        Point { cost, enabled }
    }
}

#[derive(Default, Clone, PartialEq, PartialOrd, Ord, Eq, Hash, Debug)]
pub struct GridPosition {
    x: i32,
    y: i32,
    z: i32,
}

impl GridPosition {
    pub fn new(x: i32, y: i32, z: i32) -> Self {
        GridPosition {x, y, z}
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        GridPosition,
        Pathfinding
    };

    #[test]
    fn test_basic_pathfinding() {
        let pathfinding = Pathfinding::new(4, 4, 1, 1, true);

        let desired = vec![
            GridPosition { x: 0, y: 0, z: 0 },
            GridPosition { x: 1, y: 1, z: 0 },
            GridPosition { x: 2, y: 2, z: 0 },
            GridPosition { x: 3, y: 3, z: 0 },
        ];

        let path = pathfinding.get_path(&GridPosition::new(0, 0, 0), &GridPosition::new(3, 3, 0));

        assert_eq!(path.unwrap(), (desired, 0));
    }
}