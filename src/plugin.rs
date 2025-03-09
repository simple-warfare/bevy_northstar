use std::time::Instant;

use crate::prelude::*;
use bevy::{log, prelude::*, utils::hashbrown::HashMap};

#[derive(Default)]
pub struct NorthstarPlugin<N: Neighborhood> {
    _neighborhood: std::marker::PhantomData<N>,
}

#[derive(Default)]
pub struct PathfindingStats {
    pub average_time: f64,
    pub average_length: f64,
    pub pathfind_time: Vec<f64>,
    pub pathfind_length: Vec<f64>,
}

#[derive(Default)]
pub struct CollisionStats {
    pub average_time: f64,
    pub average_length: f64,
    pub avoidance_time: Vec<f64>,
    pub avoidance_length: Vec<f64>,
}

#[derive(Resource, Default)]
pub struct Stats {
    pub pathfinding: PathfindingStats,
    pub collision: CollisionStats,
}

impl Stats {
    pub fn add_pathfinding(&mut self, time: f64, length: f64) {
        self.pathfinding.pathfind_time.push(time);
        self.pathfinding.pathfind_length.push(length);

        self.pathfinding.average_time =
            self.pathfinding.pathfind_time.iter().sum::<f64>() / self.pathfinding.pathfind_time.len() as f64;
        self.pathfinding.average_length =
            self.pathfinding.pathfind_length.iter().sum::<f64>() / self.pathfinding.pathfind_length.len() as f64;
    }

    pub fn reset_pathfinding(&mut self) {
        self.pathfinding.average_time = 0.0;
        self.pathfinding.average_length = 0.0;
        self.pathfinding.pathfind_time.clear();
        self.pathfinding.pathfind_length.clear();
    }

    pub fn add_collision(&mut self, time: f64, length: f64) {
        self.collision.avoidance_time.push(time);
        self.collision.avoidance_length.push(length);

        self.collision.average_time =
            self.collision.avoidance_time.iter().sum::<f64>() / self.collision.avoidance_time.len() as f64;
        self.collision.average_length =
            self.collision.avoidance_length.iter().sum::<f64>() / self.collision.avoidance_length.len() as f64;
    }

    pub fn reset_collision(&mut self) {
        self.collision.average_time = 0.0;
        self.collision.average_length = 0.0;
        self.collision.avoidance_time.clear();
        self.collision.avoidance_length.clear();
    }
}

#[derive(Resource, Default)]
pub struct NorthstarSettings {
    pub collision: bool,
    pub avoidance_distance: usize,
}

impl<N: 'static + Neighborhood> Plugin for NorthstarPlugin<N> {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (
                update_blocking_map,
                pathfind::<N>,
                next_position::<N>,
            )
            .chain()
            .in_set(PathingSet),
        )
        .insert_resource(BlockingMap::default());

        app.insert_resource(Stats::default());
    }
}

#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct PathingSet;

#[derive(Resource, Default)]
pub struct BlockingMap(pub HashMap<UVec3, Entity>);


fn pathfind<N: Neighborhood>(
    grid: Option<Res<Grid<N>>>,
    mut commands: Commands,
    mut query: Query<(Entity, &Position, &Pathfind), Changed<Pathfind>>,
    blocking: Res<BlockingMap>,
    mut stats: ResMut<Stats>,
    settings: Res<NorthstarSettings>,
) where
    N: 'static + Neighborhood,
{
    let grid = match grid {
        Some(grid) => grid,
        None => return,
    };

    query.iter_mut().for_each(|(entity, start, pathfind)| {
        if start.0 == pathfind.goal {
            return;
        }

        #[cfg(feature = "stats")]
        let start_time = Instant::now();

        let blocking = if settings.collision {
            &blocking.0
        } else {
            &HashMap::new()
        };

        let path = if pathfind.use_astar {
            grid.get_astar_path(start.0, pathfind.goal, blocking, false)
        } else { 
            grid.get_path(start.0, pathfind.goal, blocking, false)
        };

        #[cfg(feature = "stats")]
        let elapsed_time = start_time.elapsed().as_secs_f64();

        if let Some(path) = path {
            #[cfg(feature = "stats")]
            stats.add_pathfinding(elapsed_time, path.cost() as f64);
            
            commands.entity(entity).insert(path);
        } else {
            #[cfg(feature = "stats")]
            stats.add_pathfinding(elapsed_time, 0.0);

            commands.entity(entity).remove::<Next>(); // Just to be safe
            commands.entity(entity).insert(Pathfind {
                goal: pathfind.goal,
                use_astar: false,
            }); // Don't let anyone get stuck try again next frame
        }
    });
}

fn next_position<N: Neighborhood>(
    mut query: Query<(Entity, &mut Path, &Position, &Pathfind), Without<Next>>,
    grid: Option<Res<Grid<N>>>,
    mut blocking: ResMut<BlockingMap>,
    settings: Res<NorthstarSettings>,
    mut stats: ResMut<Stats>,
    mut commands: Commands,
) where
    N: 'static + Neighborhood,
{
    let grid = match grid {
        Some(grid) => grid,
        None => return,
    };

    for (entity, mut path, position, pathfind) in &mut query {
        if position.0 == pathfind.goal {
            commands.entity(entity).remove::<Path>();
            commands.entity(entity).remove::<Pathfind>();
            continue;
        }

        let next = if settings.collision {
            #[cfg(feature = "stats")]
            let start = Instant::now();

            avoidance(&grid, entity, &mut path, pathfind, position.0, &blocking.0, settings.avoidance_distance);

            #[cfg(feature = "stats")]
            let elapsed = start.elapsed().as_secs_f64();
            #[cfg(feature = "stats")]
            stats.add_collision(elapsed, path.cost() as f64);

            path.pop()
        } else {
            path.pop()
        };
        
        if let Some(next) = next {
            blocking.0.remove(&position.0);
            blocking.0.insert(next, entity);
            commands.entity(entity).insert(Next(next));
        } else {
            log::error!("No next position found for entity: {:?}", entity);
        }
    }
}

fn avoidance<N: Neighborhood>(
    grid: &Grid<N>,
    entity: Entity,
    path: &mut Path,
    pathfind: &Pathfind,
    position: UVec3,
    blocking: &HashMap<UVec3, Entity>,
    avoidance_distance: usize,
) 
where
    N: 'static + Neighborhood,
{
    // Check if the next few positions are blocked
    let count = if path.path().len() > avoidance_distance {
        avoidance_distance
    } else {
        path.path().len()
    };

    let unblocked_pos: Vec<UVec3> = path
        .path
        .iter()
        .take(count)
        .filter(|pos| blocking.contains_key(&**pos) == false)
        .cloned()
        .collect();

    // If we have a blocked position in the path, repath
    if unblocked_pos.len() < count {
        // Get the first unlocked position AFTER skipping the count
        let avoidance_goal = path
            .path
            .iter()
            .skip(count)
            .find(|pos| blocking.contains_key(&**pos) == false)
            .cloned();
    
        // If we have an avoidance goal, astar path to that
        if let Some(avoidance_goal) = avoidance_goal {
            let new_path = grid.get_astar_path(position, avoidance_goal, &blocking, false);

            // Replace the first few positions of path until the avoidance goal
            if let Some(new_path) = new_path {
                // Get every position AFTER the avoidance goal in the old path
                let old_path = path
                    .path
                    .iter()
                    .skip_while(|pos| *pos != &avoidance_goal)
                    .cloned()
                    .collect::<Vec<UVec3>>();

                // Combine the new path with the old path
                let mut combined_path = new_path.path().to_vec();
                combined_path.extend(old_path);

                if combined_path.len() == 0 {
                    log::error!("Combined path is empty for entity: {:?}", entity);
                    return;
                }

                // Replace the path with the combined path
                *path = Path::from_slice(&combined_path, new_path.cost());
            } else {
                // If we can't avoid locally we need to repath the whole thing
                let new_path = grid.get_astar_path(position, pathfind.goal, &blocking, true);
                if let Some(new_path) = new_path {
                    *path = new_path;
                }  else {
                    log::error!("SECOND repathing failed for {:?}: no path found, avoidance_goal: {:?}", entity, avoidance_goal);
                    return;
                }
            }
        } else {
            // ummm.. try to astar path to goal?
            let new_path = grid.get_astar_path(position, pathfind.goal, &blocking, true);
            if let Some(new_path) = new_path {
                *path = new_path;
            } else {
                log::error!("Full ASTAR Repathing failed for {:?}: no path found, goal: {:?}", entity, pathfind.goal);
                return;
            }
        }
    }

    // We must have gotten to this point because of partial paths
    if path.path.is_empty() {
        // if goal is in blocking
        if blocking.contains_key(&pathfind.goal) {
            return;
        }

        let new_path = grid.get_path(position, pathfind.goal, &blocking, false);

        if let Some(new_path) = new_path {
            *path = new_path;
        } else {
            return;
        }
    }
}


/*fn next_position_call<N: Neighborhood>(
    mut commands: Commands,
    grid: Option<Res<Grid<N>>>,
    mut query: Query<(Entity, &mut Path, &Position, &Pathfind), Without<Next>>,
    mut blocking: ResMut<BlockingMap>,
    settings: Res<NorthstarSettings>,
    mut stats: ResMut<Stats>,
) where
    N: 'static + Neighborhood,
{
    let grid = match grid {
        Some(grid) => grid,
        None => return,
    };

    for (entity, mut path, position, pathfind) in query.iter_mut() {
        // If we're at the goal, we're done
        if position.0 == pathfind.goal {
            commands.entity(entity).remove::<Path>();
            commands.entity(entity).remove::<Pathfind>();
            continue;
        }

        let start_time = Instant::now();

        // Handle avoidance if we have collision enabled
        if settings.collision {
            // Check if the next few positions are blocked
            let count = if path.path().len() > settings.avoidance_distance {
                settings.avoidance_distance
            } else {
                path.path().len()
            };

            let unblocked_pos: Vec<UVec3> = path
                .path
                .iter()
                .take(count)
                .filter(|pos| blocking.0.contains_key(&**pos) == false)
                .cloned()
                .collect();

            // If we have a blocked position in the path, repath
            if unblocked_pos.len() < count {
                // Get the first unlocked position AFTER skipping the count
                let avoidance_goal = path
                    .path
                    .iter()
                    .skip(count)
                    .find(|pos| blocking.0.contains_key(&**pos) == false)
                    .cloned();
            
                // If we have an avoidance goal, astar path to that
                if let Some(avoidance_goal) = avoidance_goal {
                    let new_path = grid.get_astar_path(position.0, avoidance_goal, &blocking.0, false);

                    // Replace the first few positions of path until the avoidance goal
                    if let Some(new_path) = new_path {
                        // Debug, check if any positions in the new path are blocked
                        /*let blocked_pos: Vec<UVec3> = new_path
                            .path
                            .iter()
                            .filter(|pos| blocking.0.contains_key(&**pos))
                            .cloned()
                            .collect();

                        if blocked_pos.len() > 0 {
                            log::error!("Blocked path: {:?}", blocked_pos);
                        }*/

                        //log::info!("We found a SHORT avoidance path!");

                        // Get every position AFTER the avoidance goal in the old path
                        let old_path = path
                            .path
                            .iter()
                            .skip_while(|pos| *pos != &avoidance_goal)
                            .cloned()
                            .collect::<Vec<UVec3>>();

                        // Combine the new path with the old path
                        let mut combined_path = new_path.path().to_vec();
                        combined_path.extend(old_path);

                        if combined_path.len() == 0 {
                            log::error!("Combined path is empty for entity: {:?}", entity);
                            continue;
                        }

                        // Replace the path with the combined path
                        *path = Path::from_slice(&combined_path, new_path.cost());
                    } else {
                        // If we can't avoid locally we need to repath the whole thing
                        //log::info!("Can't find a path to the avoidance goal, trying to astar repath the whole thing...");

                        let new_path = grid.get_astar_path(position.0, pathfind.goal, &blocking.0, true);
                        if let Some(new_path) = new_path {
                            //log::info!("We found a new full Astar full path!");
                            *path = new_path;
                        }  else {
                            let elapsed_time = start_time.elapsed().as_secs_f64();
                            stats.add_collision(elapsed_time, path.cost() as f64);
                            log::error!("SECOND repathing failed for {:?}: no path found, avoidance_goal: {:?}", entity, avoidance_goal);
                            continue;
                        }
                    }
                } else {
                    // ummm.. try to astar path to goal?
                    //log::info!("No avoidance goal found? trying to repath the whole thing...");
                    let new_path = grid.get_astar_path(position.0, pathfind.goal, &blocking.0, true);
                    if let Some(new_path) = new_path {
                        //log::info!("We found a new full astar path!");
                        *path = new_path;
                    } else {
                        let elapsed_time = start_time.elapsed().as_secs_f64();
                        stats.add_collision(elapsed_time, path.cost() as f64);
                        log::error!("Full ASTAR Repathing failed for {:?}: no path found, goal: {:?}", entity, pathfind.goal);
                        continue;
                    }
                }
            } 
            /*else {
                log::info!("Nothing blocked, just moving on!");
            }*/
        }

        // We must have gotten to this point because of partial paths
        if path.path.is_empty() {
            //log::info!("Final path is empty, doing ONE MORE FINAL PATH");

            // if goal is in blocking
            if blocking.0.contains_key(&pathfind.goal) {
                //log::error!("FINAL FINAL pathing failed for {:?}: goal is blocked, goal: {:?}", name, pathfind.goal);
                continue;
            }

            let new_path = grid.get_path(position.0, pathfind.goal, &blocking.0, false);

            if let Some(new_path) = new_path {
                *path = new_path;
            } else {
                let elapsed_time = Instant::now().elapsed().as_secs_f64();
                stats.add_collision(elapsed_time, path.cost() as f64);
                //log::error!("FINAL FINAL pathing failed for {:?}: no path found, goal: {:?}", name, pathfind.goal);
                continue;
            }
        }

        let potential_next = path.path.front().unwrap();

        if blocking.0.contains_key(potential_next) && settings.collision {
            log::error!("The next position is in the blocking map, we shouldn't get to here. Next position {:?}", potential_next);
            continue;
        }

        let elapsed_time = start_time.elapsed().as_secs_f64();
        stats.add_collision(elapsed_time, path.cost() as f64);

        let next = path.pop();

        if let Some(next) = next {
            blocking.0.remove(&position.0);
            blocking.0.insert(next, entity);
            commands.entity(entity).insert(Next(next));
        } else {
            log::error!("No next position found for entity: {:?}", entity);
        }
    }
} */

fn update_blocking_map(
    mut blocking_set: ResMut<BlockingMap>,
    query: Query<(Entity, &Position)> // WHY DID I HAVE THIS Changed<Position>>,
) {
    blocking_set.0.clear();

    query.iter().for_each(|(entity, position)| {
        blocking_set.0.insert(position.0, entity);
    });
}
