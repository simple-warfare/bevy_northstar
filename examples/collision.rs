use bevy::{
    dev_tools::fps_overlay::{FpsOverlayConfig, FpsOverlayPlugin}, log, prelude::*, text::FontSmoothing, utils::HashMap
};

use bevy_northstar::prelude::*;

use bevy_ecs_tiled::prelude::*;
use bevy_ecs_tilemap::prelude::*;
use rand::seq::SliceRandom;

#[derive(Resource, Debug, Default)]
pub struct Stats {
    average_time: f64,
    average_length: f64,
    pathfind_time: Vec<f64>,
    pathfind_length: Vec<f64>,
    count: u64,
}

#[derive(Resource, Debug, Default)]
pub struct Config {
    use_astar: bool,
}

impl Stats {
    pub fn add(&mut self, time: f64, length: f64) {
        self.pathfind_time.push(time);
        self.pathfind_length.push(length);

        self.average_time =
            self.pathfind_time.iter().sum::<f64>() / self.pathfind_time.len() as f64;
        self.average_length =
            self.pathfind_length.iter().sum::<f64>() / self.pathfind_length.len() as f64;
        self.count += 1;
    }

    pub fn reset(&mut self) {
        self.average_time = 0.0;
        self.average_length = 0.0;
        self.pathfind_time.clear();
        self.pathfind_length.clear();
        self.count = 0;
    }
}

#[derive(Clone, Debug, Default, Hash, Eq, States, PartialEq)]
pub enum State {
    #[default]
    Loading,
    Playing,
}

#[derive(Resource, Debug, Default)]
pub struct Walkable {
    pub tiles: Vec<Vec3>,
}

fn main() {
    App::new()
        // Bevy default plugins
        .add_plugins(DefaultPlugins)
        .add_plugins(FpsOverlayPlugin {
            config: FpsOverlayConfig {
                text_config: TextFont {
                    font_size: 32.0,
                    font_smoothing: FontSmoothing::default(),
                    font: default(),
                },
                text_color: Color::srgb(0.0, 1.0, 0.0),
                enabled: true,
            },
        })
        // bevy_ecs_tilemap and bevy_ecs_tiled main plugins
        .add_plugins(TilemapPlugin)
        .add_plugins(TiledMapPlugin::default())
        // bevy_northstar plugins
        .add_plugins(NorthstarPlugin::<OrdinalNeighborhood>::default())
        .add_plugins(NorthstarDebugPlugin::<OrdinalNeighborhood> {
            config: NorthstarDebugConfig {
                grid_width: 8,
                grid_height: 8,
                draw_entrances: false,
                draw_chunks: false,
                draw_cached_paths: false,
                draw_path: true,
                draw_points: false,
                draw_goals: true,
                ..Default::default()
            },
            ..Default::default()
        })
        .insert_resource(Grid::<OrdinalNeighborhood>::new(&GridSettings {
            width: 16,
            height: 16,
            depth: 1,
            chunk_size: 8,
            chunk_depth: 1,
            chunk_ordinal: false,
            default_cost: 1,
            default_wall: true,
            jump_height: 1,
        }))
        // Observe the LayerCreated event to build the grid
        .add_observer(layer_created)
        // Add our systems and run the app!
        .add_systems(Startup, startup)
        .add_systems(OnEnter(State::Playing), spawn_minions)
        .add_systems(Update, input)
        .add_systems(Update, move_pathfinders.after(PathingSet))
        .add_systems(Update, tick.run_if(in_state(State::Playing)))
        //.add_systems(Update, pathfind_minions.run_if(in_state(State::Playing)))
        .add_systems(Update, set_new_goal.run_if(in_state(State::Playing)))
        .add_systems(Update, update_stat_text.run_if(in_state(State::Playing)))
        .add_systems(
            Update,
            update_pathfind_type_test.run_if(in_state(State::Playing)),
        )
        .add_event::<Tick>()
        .insert_state(State::Loading)
        .insert_resource(Walkable::default())
        .insert_resource(Stats::default())
        .insert_resource(Config::default())
        .insert_resource(NorthstarSettings {
            collision: true,
            avoidance_distance: 4,
        })
        .add_systems(Update, debug_bad_positions)
        .run();
}

#[derive(Event, Default)]
struct Tick;

// Generate a tick event
fn tick(time: Res<Time>, mut tick_writer: EventWriter<Tick>) {
    if time.elapsed_secs() % 0.25 < time.delta_secs() {
        tick_writer.send_default();
    }
}

#[derive(Component, Debug)]
struct StatText;

#[derive(Component, Debug)]
struct PathfindTypeText;

fn startup(mut commands: Commands, asset_server: Res<AssetServer>) {
    // Spawn a 2D camera (required by Bevy)
    commands.spawn(Camera2d);

    // Load the map ...
    let map_handle: Handle<TiledMap> = asset_server.load("demo_16.tmx");

    // ... then spawn it !
    let mut map_entity = commands.spawn(TiledMapHandle(map_handle));

    // You can eventually add some extra settings to your map
    map_entity.insert((
        TiledMapSettings {
            layer_positioning: LayerPositioning::Centered,
            ..default()
        },
        TilemapRenderSettings {
            render_chunk_size: UVec2::new(32, 32),
            ..Default::default()
        },
    ));

    commands
        .spawn((
            Text::new("Key [p]| Algorithm: "),
            TextFont {
                font_size: 24.0,
                ..default()
            },
            Node {
                position_type: PositionType::Absolute,
                bottom: Val::Px(50.0),
                left: Val::Px(0.0),
                ..default()
            },
        ))
        .with_child((
            TextSpan::default(),
            TextFont {
                font_size: 24.0,
                ..default()
            },
            PathfindTypeText,
        ));

    commands
        .spawn((
            Text::new("Avg Path Time: "),
            TextFont {
                font_size: 24.0,
                ..default()
            },
            Node {
                position_type: PositionType::Absolute,
                bottom: Val::Px(100.0),
                left: Val::Px(0.0),
                ..default()
            },
        ))
        .with_child((
            TextSpan::default(),
            TextFont {
                font_size: 24.0,
                ..default()
            },
            StatText,
        ));
}

fn update_stat_text(stats: Res<Stats>, mut query: Query<&mut TextSpan, With<StatText>>) {
    for mut span in &mut query {
        **span = format!("{:.2}ms", stats.average_time * 1000.0);
    }
}

fn update_pathfind_type_test(
    config: Res<Config>,
    mut query: Query<&mut TextSpan, With<PathfindTypeText>>,
) {
    for mut span in &mut query {
        **span = if config.use_astar {
            "A*".to_string()
        } else {
            "HPA*".to_string()
        };
    }
}

fn debug_bad_positions(
    query: Query<(Entity, &Position, &Path)>,
) {
    let mut positions = HashMap::new();

    for (entity, position, path) in query.iter() {        
        if positions.contains_key(&position.0) {
            //log::error!("{:?}", positions);
            log::error!("Entity {:?} has the same position as another entity: {:?}", entity, path);
        } else {
            positions.insert(position.0, (entity, path));
        }
    }
}

fn move_pathfinders(
    mut commands: Commands,
    mut query: Query<(Entity, &mut Position, &Next)>,
    mut tick_reader: EventReader<Tick>,
) {
    for _ in tick_reader.read() {
        for (entity, mut position, next) in query.iter_mut() {
            position.0 = next.0;
            let translation = Vec3::new(next.0.x as f32 * 8.0 + 4.0, next.0.y as f32 * 8.0 + 4.0, 4.0);

            commands
                .entity(entity)
                .insert(Transform::from_translation(translation))
                .remove::<Next>();
        }
    }
}

fn set_new_goal(
    mut commands: Commands,
    mut minions: Query<Entity, (Without<Path>, Without<Goal>)>,
    walkable: Res<Walkable>,
) {
    for entity in minions.iter_mut() {
        let new_goal = walkable.tiles.choose(&mut rand::thread_rng()).unwrap();

        commands.entity(entity).insert(Goal(UVec3::new((new_goal.x / 8.0) as u32, (new_goal.y / 8.0) as u32, 0)));
    }
}

fn spawn_minions(
    mut commands: Commands,
    grid: Res<Grid<OrdinalNeighborhood>>,
    layer_entity: Query<Entity, With<TiledMapTileLayer>>,
    asset_server: Res<AssetServer>,
    mut walkable: ResMut<Walkable>,
) {
    let layer_entity = layer_entity.iter().next().unwrap();

    walkable.tiles = Vec::new();
    for x in 0..grid.get_width() {
        for y in 0..grid.get_height() {
            if grid.get_point(UVec3::new(x, y, 0)).wall == false {
                let position = Vec3::new(x as f32 * 8.0, y as f32 * 8.0, 0.0);

                walkable.tiles.push(position);
            }
        }
    }

    let mut count = 0;

    while count < 6 {
        let position = walkable.tiles.choose(&mut rand::thread_rng()).unwrap();
        //let goal = walkable.tiles.choose(&mut rand::thread_rng()).unwrap();

        let transform = Vec3::new(position.x + 4.0, position.y + 4.0, 4.0);

        // Generate random color
        let color = Color::srgb(
            rand::random::<f32>(),
            rand::random::<f32>(),
            rand::random::<f32>(),
        );

        commands
            .spawn(Sprite {
                image: asset_server.load("tile_0018_edit.png"),
                color: color,
                ..Default::default()
            })
            .insert(Name::new(format!("{:?}", color)))
            .insert(DebugColor(color))
            .insert(Blocking)
            .insert(Transform::from_translation(transform))
            .insert(Position(UVec3::new((position.x / 8.0) as u32, (position.y / 8.0) as u32, 0)))
            //.insert(Goal(UVec3::new((goal.x / 8.0) as u32, (goal.y / 8.0) as u32, 0)))
            .set_parent(layer_entity);

        count += 1;
    }
}

fn layer_created(
    trigger: Trigger<TiledLayerCreated>,
    q_layer: Query<&Name, With<TiledMapLayer>>,
    map_asset: Res<Assets<TiledMap>>,
    mut grid: ResMut<Grid<OrdinalNeighborhood>>,
    mut state: ResMut<NextState<State>>,
) {
    // We can either access the layer components
    if let Ok(name) = q_layer.get(trigger.event().layer) {
        info!("Received TiledLayerCreated event for layer '{}'", name);
    }

    // Or directly the underneath Tiled Layer structure
    let layer = trigger.event().layer(&map_asset);
    layer.as_tile_layer().map(|layer| {
        let width = layer.width().unwrap();
        let height = layer.height().unwrap();

        for x in 0..width {
            for y in 0..height {
                let tile = layer.get_tile(x as i32, y as i32);
                if tile.is_some() {
                    let tile_id = tile.unwrap().id();

                    if tile_id == 14 {
                        grid.set_point(
                            UVec3::new(x as u32, height - 1 - y as u32, 0),
                            Point::new(1, false),
                        );
                    } else {
                        grid.set_point(
                            UVec3::new(x as u32, height - 1 - y as u32, 0),
                            Point::new(0, true),
                        );
                    }
                }
            }
        }
    });

    info!("Loaded layer: {:?}", layer);
    grid.build();

    state.set(State::Playing);
}

pub fn input(
    time: Res<Time>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&mut Transform, &mut OrthographicProjection), With<Camera>>,
    mut config: ResMut<Config>,
    mut stats: ResMut<Stats>,
) {
    for (mut transform, mut ortho) in query.iter_mut() {
        let mut direction = Vec3::ZERO;

        if keyboard_input.pressed(KeyCode::KeyA) {
            direction -= Vec3::new(1.0, 0.0, 0.0);
        }

        if keyboard_input.pressed(KeyCode::KeyD) {
            direction += Vec3::new(1.0, 0.0, 0.0);
        }

        if keyboard_input.pressed(KeyCode::KeyW) {
            direction += Vec3::new(0.0, 1.0, 0.0);
        }

        if keyboard_input.pressed(KeyCode::KeyS) {
            direction -= Vec3::new(0.0, 1.0, 0.0);
        }

        if keyboard_input.pressed(KeyCode::KeyZ) {
            ortho.scale += 0.1;
        }

        if keyboard_input.pressed(KeyCode::KeyX) {
            ortho.scale -= 0.1;
        }

        if keyboard_input.just_pressed(KeyCode::KeyP) {
            config.use_astar = !config.use_astar;
            stats.reset();
        }

        if ortho.scale < 0.25 {
            ortho.scale = 0.25;
        }

        let z = transform.translation.z;
        transform.translation += time.delta_secs() * direction * 500.;
        // Important! We need to restore the Z values when moving the camera around.
        // Bevy has a specific camera setup and this can mess with how our layers are shown.
        transform.translation.z = z;
    }
}
