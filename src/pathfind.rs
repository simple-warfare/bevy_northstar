//! This module defines pathfinding functions which can be called directly.

use bevy::{ecs::entity::Entity, math::UVec3, platform::collections::HashMap};
use ndarray::ArrayView3;

use crate::{
    astar::{astar_graph, astar_grid},
    dijkstra::dijkstra_grid,
    dir::get_movement_type,
    grid::{Grid, Point},
    path::Path,
    prelude::Neighborhood,
    raycast::{
        bresenham_path, generate_path_segment_cardinal, generate_path_segment_ordinal,
        line_of_sight,
    },
};

/// AStar pathfinding
///
/// This function is provided if you want to supply your own grid.
/// If you're using the built in [`Grid`] you can use the pathfinding helper functions
/// provided in the [`Grid`] struct.
///
/// # Arguments
/// * `neighborhood` - The [`Neighborhood`] to use for the pathfinding.
/// * `grid` - The [`ArrayView3`] of [`Point`]s to use for the pathfinding.
/// * `start` - The starting position.
/// * `goal` - The goal position.
/// * `blocking` - A hashmap of blocked positions for dynamic obstacles.
/// * `partial` - If true, the pathfinding will return a partial path if the goal is blocked.
#[inline(always)]
pub fn pathfind_astar<N: Neighborhood>(
    neighborhood: &N,
    grid: &ArrayView3<Point>,
    start: UVec3,
    goal: UVec3,
    blocking: &HashMap<UVec3, Entity>,
    partial: bool,
) -> Option<Path> {
    if grid[[start.x as usize, start.y as usize, start.z as usize]].solid
        || grid[[goal.x as usize, goal.y as usize, goal.z as usize]].solid
    {
        return None;
    }

    // if goal is in the blocking map, return None
    if blocking.contains_key(&goal) && !partial {
        //log::error!("Goal is in the blocking map");
        return None;
    }

    let path = astar_grid(neighborhood, grid, start, goal, 1024, partial, blocking);

    if let Some(mut path) = path {
        path.path.pop_front();
        Some(path)
    } else {
        None
    }
}

/// HPA* pathfinding.
// Keeping this internal for now since Grid has it's own helper function to call this
// and [`Grid`] is required for it.
#[inline(always)]
pub(crate) fn pathfind<N: Neighborhood>(
    grid: &Grid<N>,
    start: UVec3,
    goal: UVec3,
    blocking: &HashMap<UVec3, Entity>,
    partial: bool,
) -> Option<Path> {
    if grid.view()[[start.x as usize, start.y as usize, start.z as usize]].solid
        || grid.view()[[goal.x as usize, goal.y as usize, goal.z as usize]].solid
    {
        return None;
    }

    let start_chunk = grid.chunk_at_position(start)?;
    let goal_chunk = grid.chunk_at_position(goal)?;

    if start_chunk == goal_chunk {
        let path = astar_grid(
            &grid.neighborhood,
            &grid.view(),
            start,
            goal,
            100,
            partial,
            blocking,
        );

        if let Some(mut path) = path {
            // We should still refine this path since AStar wasn't requested
            path = optimize_path(
                &grid.neighborhood,
                &grid.view(),
                &path,
                grid.neighborhood.is_ordinal(),
            );

            path.path.pop_front();
            return Some(path);
        } else {
            return None;
        }
    }

    // Get all nodes in the start chunk
    let start_nodes = grid.graph().nodes_in_chunk(start_chunk.clone());

    // Build a new blocking map that's adjusted to the start chunk
    let start_blocking = blocking
        .iter()
        .map(|(pos, entity)| {
            let adjusted_pos = UVec3::new(
                pos.x.saturating_sub(start_chunk.min().x),
                pos.y.saturating_sub(start_chunk.min().y),
                pos.z.saturating_sub(start_chunk.min().z),
            );
            (adjusted_pos, *entity)
        })
        .collect::<HashMap<_, _>>();

    // Get the djikstra paths to all starting nodes, ruling any that don't have paths
    let start_paths = dijkstra_grid(
        &grid.chunk_view(start_chunk),
        start - start_chunk.min(),
        &start_nodes
            .iter()
            .map(|node| node.pos - start_chunk.min())
            .collect::<Vec<_>>(),
        false,
        100,
        &start_blocking,
    );

    // Rule out any start_nodes that don't have paths
    let start_nodes = start_nodes
        .iter()
        .filter(|node| start_paths.contains_key(&(node.pos - start_chunk.min())))
        .collect::<Vec<_>>();

    if start_nodes.is_empty() {
        return None;
    }

    // Sort start nodes by distance to the goal and the starting point
    let mut start_nodes = start_nodes
        .iter()
        .map(|node| {
            let distance_to_goal = (node.pos.x as i32 - goal.x as i32).abs()
                + (node.pos.y as i32 - goal.y as i32).abs()
                + (node.pos.z as i32 - goal.z as i32).abs();

            let distance_to_start = (node.pos.x as i32 - start.x as i32).abs()
                + (node.pos.y as i32 - start.y as i32).abs()
                + (node.pos.z as i32 - start.z as i32).abs();

            (node, distance_to_goal + distance_to_start)
        })
        .collect::<Vec<_>>();

    // Starting with the node shortest to the goal, find a path to the goal
    start_nodes.sort_by_key(|(_, distance)| *distance);

    // Get all nodes in the goal chunk
    let goal_nodes = grid.graph().nodes_in_chunk(goal_chunk.clone());

    // Build a new blocking map that's adjusted for the goal chunk
    /*let goal_blocking = blocking.clone()
    .iter()
    .map(|(pos, entity)| {
        let adjusted_pos = UVec3::new(
            pos.x.saturating_sub(goal_chunk.min.x),
            pos.y.saturating_sub(goal_chunk.min.y),
            pos.z.saturating_sub(goal_chunk.min.z),
        );
        (adjusted_pos, *entity)
    })
    .collect::<HashMap<_, _>>();*/

    let goal_paths = dijkstra_grid(
        &grid.chunk_view(goal_chunk),
        goal - goal_chunk.min(),
        &goal_nodes
            .iter()
            .map(|node| node.pos - goal_chunk.min())
            .collect::<Vec<_>>(),
        false,
        100,
        &HashMap::new(), //&goal_blocking,
    );

    let goal_nodes = goal_nodes
        .iter()
        .filter(|node| goal_paths.contains_key(&(node.pos - goal_chunk.min())))
        .collect::<Vec<_>>();

    let mut goal_nodes = goal_nodes
        .iter()
        .map(|node| {
            // Calculate the distance from the start to the node and from the node to the goal
            let distance_to_start = (node.pos.x as i32 - start.x as i32).abs()
                + (node.pos.y as i32 - start.y as i32).abs()
                + (node.pos.z as i32 - start.z as i32).abs();
            let distance_to_goal = (node.pos.x as i32 - goal.x as i32).abs()
                + (node.pos.y as i32 - goal.y as i32).abs()
                + (node.pos.z as i32 - goal.z as i32).abs();

            // Calculate the total distance as the sum of the distances to the start and goal
            let total_distance = distance_to_start + distance_to_goal;

            (node, total_distance)
        })
        .collect::<Vec<_>>();

    goal_nodes.sort_by_key(|(_, distance)| *distance);

    // Calculate the distance from the start to the goal
    let start_distance = (start.x as i32 - goal.x as i32).abs()
        + (start.y as i32 - goal.y as i32).abs()
        + (start.z as i32 - goal.z as i32).abs();

    // Move goal_nodes that are farther from the goal in the direction of the starting point to the end of the list
    goal_nodes.sort_by_key(|(_, distance)| {
        let distance = *distance - start_distance;
        if distance < 0 {
            distance.abs()
        } else {
            distance
        }
    });

    let mut path: Vec<UVec3> = Vec::new();
    let mut cost = 0;

    // Here is the problem. If the starting position can't get to the start node then this whole thing fails. Need to fix it.

    for (start_node, _) in start_nodes {
        for (goal_node, _) in goal_nodes.clone() {
            let node_path = astar_graph(
                &grid.neighborhood,
                grid.graph(),
                start_node.pos,
                goal_node.pos,
                100,
            );

            if let Some(node_path) = node_path {
                // Add start_path to the node_path
                let start_path = start_paths
                    .get(&(start_node.pos - start_chunk.min()))
                    .unwrap();
                path.extend(start_path.path().iter().map(|pos| *pos + start_chunk.min()));
                cost += start_path.cost();

                // Add node_path paths to path
                for (node, next_node) in
                    node_path.path().iter().zip(node_path.path().iter().skip(1))
                {
                    // Get the cached edge path between node and next node
                    let cached_path = grid.graph().node_at(*node).unwrap().edges[next_node].clone();
                    path.extend(cached_path.path().iter().skip(1));
                    cost += cached_path.cost();
                }

                // Add end path to path
                let end_path = goal_paths.get(&(goal_node.pos - goal_chunk.min())).unwrap();
                path.extend(
                    end_path
                        .path()
                        .iter()
                        .rev()
                        .map(|pos| *pos + goal_chunk.min()),
                );
                cost += end_path.cost();

                if path.is_empty() {
                    return None;
                }

                let mut refined_path = optimize_path(
                    &grid.neighborhood,
                    &grid.view(),
                    &Path::from_slice(&path, cost),
                    grid.neighborhood.is_ordinal(),
                );

                // remove the start point from the refined path
                refined_path.path.pop_front();

                // add the graph path to the refined path
                refined_path.graph_path = node_path.path;

                return Some(refined_path);
            }
        }
    }

    None
}

/// Optimize a path by using line of sight checks to skip waypoints.
///
/// This is used to optimize paths generated by the HPA* algorithms.
/// [`Grid::pathfind`] uses this internally so this is only needed if you want to
/// optimize a path that was generated by a different method.
///
/// # Arguments
///
/// * `neighborhood` - The [`Neighborhood`] to use for the pathfinding.
/// * `grid` - The [`ArrayView3`] of the grid.
/// * `path` - The [`Path`] to optimize.
/// * `ordinal` - If true, use ordinal movement. If false, use cardinal movement.
///
#[inline(always)]
pub fn optimize_path<N: Neighborhood>(
    neighborhood: &N,
    grid: &ArrayView3<Point>,
    path: &Path,
    ordinal: bool,
) -> Path {
    if path.is_empty() {
        return path.clone();
    }

    let goal = *path.path.back().unwrap();

    let mut refined_path = Vec::new();
    let mut i = 0;
    let mut prev_movement_type = None;

    refined_path.push(path.path[i]); // Always keep the first node

    let allow_corner_clipping = neighborhood
        .settings()
        .map(|settings| settings.allow_corner_clipping)
        .unwrap_or(false);

    while i < path.len() {
        // Check if we can go directly to the goal
        let path_to_goal = bresenham_path(grid, path.path[i], goal, neighborhood.is_ordinal(), allow_corner_clipping);

        if let Some(path_to_goal) = path_to_goal {
            refined_path.extend(path_to_goal.into_iter().skip(1)); // Add intermediate points
            break; // We're done
        }

        let mut best_farthest = i + 1;
        let mut best_farthest_path = None;
        let mut best_score = f32::NEG_INFINITY;

        for farthest in (i + 1)..path.len() {
            let path_to_farthest = bresenham_path(
                grid,
                path.path[i],
                path.path[farthest],
                neighborhood.is_ordinal(),
                allow_corner_clipping
            );

            if let Some(path_to_farthest) = path_to_farthest {
                let shortcut_length = farthest - i; // Reward longer shortcuts
                let goal_proximity = neighborhood.heuristic(path.path[farthest], goal) as f32; // Reward shortcuts closer to the goal

                // Base score: encourage longer shortcuts
                let mut score = shortcut_length as f32 - goal_proximity * 4.0;

                // Apply penalty if switching movement types
                if ordinal {
                    let movement_type =
                        get_movement_type(path.path[i].as_vec3(), path.path[farthest].as_vec3());

                    if let Some(prev) = prev_movement_type {
                        if prev != movement_type {
                            score -= 4.0; // Penalize changing movement types
                        }
                    }
                }

                // Pick the best shortcut that maximizes the score
                if score > best_score {
                    best_farthest = farthest;
                    best_farthest_path = Some(path_to_farthest);
                    best_score = score;
                }
            }
        }

        if let Some(path_to_farthest) = best_farthest_path {
            refined_path.extend(path_to_farthest.into_iter().skip(1)); // Avoid duplicate start points
        }
        prev_movement_type = Some(get_movement_type(
            path.path[i].as_vec3(),
            path.path[best_farthest].as_vec3(),
        ));
        i = best_farthest; // Move to the best shortcut
    }

    // Calculate the cost of the refined path
    let mut cost = 0;
    for pos in refined_path.iter() {
        cost += grid[[pos.x as usize, pos.y as usize, pos.z as usize]].cost;
    }

    let mut path = Path::new(refined_path.to_vec(), cost);
    path.graph_path = path.path.clone();
    path
}

/// Optimize a path by using line of sight checks to skip waypoints.
///
/// This is used to optimize paths generated by the HPA* algorithms.
/// [`Grid::pathfind`] uses this internally so this is only needed if you want to
/// optimize a path that was generated by a different method.
///
/// # Arguments
///
/// * `neighborhood` - The [`Neighborhood`] to use for the pathfinding.
/// * `grid` - The [`ArrayView3`] of the grid.
/// * `path` - The [`Path`] to optimize.
/// * `ordinal` - If true, use ordinal movement. If false, use cardinal movement.
///
#[inline(always)]
pub fn optimize_path_old<N: Neighborhood>(
    neighborhood: &N,
    grid: &ArrayView3<Point>,
    path: &Path,
    ordinal: bool,
) -> Path {
    if path.is_empty() {
        return path.clone();
    }

    let goal = *path.path.back().unwrap();

    let mut refined_path = Vec::new();
    let mut i = 0;
    let mut prev_movement_type = None;

    refined_path.push(path.path[i]); // Always keep the first node

    while i < path.len() {
        // Check if we can go directly to the goal
        if line_of_sight(grid, path.path[i], goal) {
            let segment = if ordinal {
                generate_path_segment_ordinal(path.path[i], goal)
            } else {
                generate_path_segment_cardinal(path.path[i], goal)
            };

            refined_path.extend(segment.into_iter().skip(1)); // Add intermediate points
            break; // We're done
        }

        let mut best_farthest = i + 1;
        let mut best_score = f32::NEG_INFINITY;

        for farthest in (i + 1)..path.len() {
            if line_of_sight(grid, path.path[i], path.path[farthest]) {
                let shortcut_length = farthest - i; // Reward longer shortcuts
                let goal_proximity = neighborhood.heuristic(path.path[farthest], goal) as f32; // Reward shortcuts closer to the goal

                // Base score: encourage longer shortcuts
                let mut score = shortcut_length as f32 - goal_proximity * 4.0;

                // Apply penalty if switching movement types
                if ordinal {
                    let movement_type =
                        get_movement_type(path.path[i].as_vec3(), path.path[farthest].as_vec3());

                    if let Some(prev) = prev_movement_type {
                        if prev != movement_type {
                            score -= 4.0; // Penalize changing movement types
                        }
                    }
                }

                // Pick the best shortcut that maximizes the score
                if score > best_score {
                    best_farthest = farthest;
                    best_score = score;
                }
            }
        }

        let segment = if ordinal {
            generate_path_segment_ordinal(path.path[i], path.path[best_farthest])
        } else {
            generate_path_segment_cardinal(path.path[i], path.path[best_farthest])
        };

        refined_path.extend(segment.into_iter().skip(1)); // Avoid duplicate start points

        prev_movement_type = Some(get_movement_type(
            path.path[i].as_vec3(),
            path.path[best_farthest].as_vec3(),
        ));

        i = best_farthest; // Move to the best shortcut
    }

    // Calculate the cost of the refined path
    let mut cost = 0;

    for pos in refined_path.iter() {
        cost += grid[[pos.x as usize, pos.y as usize, pos.z as usize]].cost;
    }

    let mut path = Path::new(refined_path.to_vec(), cost);
    path.graph_path = path.path.clone();

    path
}

/// Recursively reroutes a path by astar pathing to further chunks until a path can be found.
///
/// Useful if local collision avoidance is failing.
///
/// If you're using the plugin pathing systems, you shouldn't need to call this directly.
///
/// # Arguments
#[inline(always)]
pub(crate) fn reroute_path<N: Neighborhood>(
    grid: &Grid<N>,
    path: &Path,
    start: UVec3,
    goal: UVec3,
    blocking: &HashMap<UVec3, Entity>,
) -> Option<Path> {
    // When the starting chunks entrances are all blocked, this will try astar path to the NEXT chunk in the graph path
    // recursively until it can find a path out.
    // If it can't find a path out, it will return None.

    if path.graph_path.is_empty() {
        // Our only option here is to astar path to the goal
        return pathfind_astar(
            &grid.neighborhood,
            &grid.view(),
            start,
            goal,
            blocking,
            false,
        );
    }

    let new_path = path.graph_path.iter().find_map(|pos| {
        let new_path = pathfind_astar(
            &grid.neighborhood,
            &grid.view(),
            start,
            *pos,
            blocking,
            false,
        );
        if new_path.is_some() && !new_path.as_ref().unwrap().is_empty() {
            new_path
        } else {
            None
        }
    });

    // HPA the rest of the way to the goal using get_path from the last position in the new path to the goal
    if let Some(new_path) = new_path {
        let mut hpa_path = Vec::new();

        for pos in new_path.path() {
            hpa_path.push(*pos);
        }

        let last_pos = *new_path.path().last().unwrap();

        let hpa = pathfind(grid, last_pos, goal, blocking, false);

        if let Some(hpa) = hpa {
            for pos in hpa.path() {
                hpa_path.push(*pos);
            }

            let mut path = Path::new(hpa_path, new_path.cost() + hpa.cost());
            path.graph_path = hpa.graph_path.clone();
            return Some(path);
        }
    }

    None
}
