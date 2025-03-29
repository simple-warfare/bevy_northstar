#[cfg(feature = "stats")]
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
                reroute_path::<N>,
            )
            .chain()
            .in_set(PathingSet),
        )
        .insert_resource(BlockingMap::default());

        app.insert_resource(Stats::default());
        app.insert_resource(DirectionMap::default());
    }
}

#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct PathingSet;

#[derive(Resource, Default)]
pub struct BlockingMap(pub HashMap<UVec3, Entity>);

#[derive(Resource, Default)]
pub struct DirectionMap(pub HashMap<Entity, Vec3>);

fn pathfind<N: Neighborhood>(
    grid: Option<Res<Grid<N>>>,
    mut commands: Commands,
    mut query: Query<(Entity, &Position, &Pathfind), Changed<Pathfind>>,
    blocking: Res<BlockingMap>,
    settings: Res<NorthstarSettings>,
    #[cfg(feature = "stats")] mut stats: ResMut<Stats>,
) where
    N: 'static + Neighborhood,
{
    let grid = match grid {
        Some(grid) => grid,
        None => return,
    };

    query.iter_mut().for_each(|(entity, start, pathfind)| {
        if start.0 == pathfind.goal {
            commands.entity(entity).remove::<Pathfind>();
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
    mut query: Query<(Entity, &mut Path, &Position, &Pathfind), (Without<Next>, Without<AvoidanceFailed>, Without<RerouteFailed>)>,
    grid: Option<Res<Grid<N>>>,
    mut blocking: ResMut<BlockingMap>,
    mut direction: ResMut<DirectionMap>,
    settings: Res<NorthstarSettings>,
    mut commands: Commands,
    #[cfg(feature = "stats")] mut stats: ResMut<Stats>,
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

            let success = avoidance(&grid, entity, &mut path, pathfind, position.0, &blocking.0, &direction.0, settings.avoidance_distance);

            if !success {
                commands.entity(entity).insert(AvoidanceFailed);
                continue;
            }

            #[cfg(feature = "stats")]
            let elapsed = start.elapsed().as_secs_f64();
            #[cfg(feature = "stats")]
            stats.add_collision(elapsed, path.cost() as f64);

            let potential_next = path.path.front().unwrap();


            if blocking.0.contains_key(potential_next) && blocking.0[potential_next] != entity {
                log::info!("Blocking: {:?}", blocking.0);
                log::error!("Next position is blocked for entity: {:?}, other entity: {:?}", entity, blocking.0[potential_next]);
            }

            path.pop()
        } else {
            path.pop()
        };
        
        if let Some(next) = next{
            if blocking.0.contains_key(&next) && settings.collision {
                log::error!("Next position is blocked for entity, we should never get here but we did: {:?}", entity);
                commands.entity(entity).insert(AvoidanceFailed);
                continue;
            }

            // Calculate the dot product direction
            direction.0.insert(entity, next.as_vec3() - position.0.as_vec3());

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
    direction: &HashMap<Entity, Vec3>,
    avoidance_distance: usize,
) -> bool
where
    N: 'static + Neighborhood,
{
    if path.is_empty() {
        log::error!("Path is empty for entity: {:?}", entity);
        return false;
    }

    // Check if the next few positions are blocked
    let count = if path.path().len() > avoidance_distance {
        avoidance_distance
    } else {
        path.path().len()
    };

    // Precompute the dot product for the current position
    let next_position = path.path.front().unwrap();

    if path.path.len() == 1 && blocking.contains_key(next_position) {
        info!("The next position is the goal and is blocked, all we can do is wait");
        return false;
    }

    let difference = next_position.as_vec3() - position.as_vec3();


    let unblocked_pos: Vec<UVec3> = path
        .path
        .iter()
        .take(count)
        .filter(|pos| {
            if let Some(blocking_entity) = blocking.get(*pos) {
                // Too risky to move to the next position regardless of the direction
                if *pos == next_position {
                    return false;
                }

                if let Some(blocking_dir) = direction.get(blocking_entity) {
                    let dot = difference.dot(*blocking_dir);
                    if dot <= 0.0 {
                        return false;
                    }
                } else {
                    // They have no direction
                    return false;
                }
            }
            true
        })
        .cloned()
        .collect();

    // If we have a blocked position in the path, repath
    if unblocked_pos.len() < count {
        let avoidance_goal = {
            let _ = info_span!("avoidance_goal", name = "avoidance_goal").entered();
         
            // Get the first unlocked position AFTER skipping the count, we don't care about the direction
            path.path.iter()
                .skip(count)
                .find(|pos| blocking.contains_key(&**pos) == false)
        };
        
        // If we have an avoidance goal, astar path to that
        if let Some(avoidance_goal) = avoidance_goal {
            let new_path = grid.get_astar_path(position, *avoidance_goal, &blocking, false);

            // Replace the first few positions of path until the avoidance goal
            if let Some(new_path) = new_path {
                // Get every position AFTER the avoidance goal in the old path
                let old_path = path
                    .path
                    .iter()
                    .skip_while(|pos| *pos != avoidance_goal)
                    .cloned()
                    .collect::<Vec<UVec3>>();

                // Combine the new path with the old path
                let mut combined_path = new_path.path().to_vec();
                combined_path.extend(old_path);

                if combined_path.len() == 0 {
                    log::error!("Combined path is empty for entity: {:?}", entity);
                    return false;
                }

                let graph_path = path.graph_path.clone();

                // Replace the path with the combined path
                *path = Path::from_slice(&combined_path, new_path.cost());
                path.graph_path = graph_path;
            } else {
                return false;
            }
        } else {
            // No easy avoidance astar path to the next entrance
            return false;
        }
    }

    // DELETE THIS
    // We must have gotten to this point because of partial paths
    if path.path.is_empty() {
        // if goal is in blocking
        if blocking.contains_key(&pathfind.goal) {
            return false;
        }

        let new_path = grid.get_path(position, pathfind.goal, &blocking, false);

        if let Some(new_path) = new_path {
            *path = new_path;
            return true;
        } else {
            return false;
        }
    }

    true
}

fn reroute_path<N: Neighborhood>(
    mut query: Query<(Entity, &Position, &Pathfind, &Path), With<AvoidanceFailed>>,
    grid: Res<Grid<N>>,
    blocking: Res<BlockingMap>,
    mut commands: Commands,
    #[cfg(feature = "stats")] mut stats: ResMut<Stats>,
) 
where 
    N: 'static + Neighborhood,
{
    for (entity, position, pathfind, path) in query.iter_mut() {
        #[cfg(feature = "stats")]
        let start = Instant::now();

        // Let's just try a repath
        //let new_path = grid.get_path(position.0, pathfind.goal, &blocking.0, true);

        // Let's reroute the path
        let new_path = grid.reroute_path(path, position.0, pathfind.goal, &blocking.0);

        if let Some(new_path) = new_path {

            // if the last point in the path is not the goal...
            if new_path.path().last().unwrap() != &pathfind.goal {
                log::error!("WE HAVE A PARTIAL ROUTE ISSUE: {:?}", entity);
            }

            #[cfg(feature = "stats")]
            let elapsed = start.elapsed().as_secs_f64();
            #[cfg(feature = "stats")]
            stats.add_collision(elapsed, new_path.cost() as f64);

            commands.entity(entity).insert(new_path);
            commands.entity(entity).remove::<AvoidanceFailed>();
        } else {
            commands.entity(entity).insert(RerouteFailed);
            commands.entity(entity).remove::<AvoidanceFailed>(); // Try again next frame

            #[cfg(feature = "stats")]
            let elapsed = start.elapsed().as_secs_f64();
            #[cfg(feature = "stats")]
            stats.add_collision(elapsed, 0.0);
        }
    }
}

fn update_blocking_map(
    mut blocking_set: ResMut<BlockingMap>,
    query: Query<(Entity, &Position)> // WHY DID I HAVE THIS Changed<Position>>,
) {
    blocking_set.0.clear();

    query.iter().for_each(|(entity, position)| {
        blocking_set.0.insert(position.0, entity);
    });
}
