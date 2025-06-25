use bevy::{
    dev_tools::fps_overlay::{FpsOverlayConfig, FpsOverlayPlugin},
    prelude::*,
    text::FontSmoothing,
};

use bevy_northstar::prelude::*;

use bevy_ecs_tiled::prelude::*;
use bevy_ecs_tilemap::prelude::*;
use rand::seq::IndexedRandom;

mod shared;

fn main() {
    App::new()
        // Bevy default plugins
        .add_plugins(DefaultPlugins)
        // FPS Overlay
        .add_plugins(FpsOverlayPlugin {
            config: FpsOverlayConfig {
                text_config: TextFont {
                    font_size: 32.0,
                    font_smoothing: FontSmoothing::default(),
                    font: default(),
                    ..default()
                },
                text_color: Color::srgb(0.0, 1.0, 0.0),
                enabled: true,
                ..default()
            },
        })
        // bevy_ecs_tilemap and bevy_ecs_tiled main plugins
        .add_plugins((TilemapPlugin, TiledMapPlugin::default()))
        // bevy_northstar plugins
        .add_plugins((
            NorthstarPlugin::<CardinalNeighborhood>::default(),
            NorthstarDebugPlugin::<CardinalNeighborhood>::default(),
        ))
        // Add the SharedPlugin for unrelated pathfinding systems shared by the examples
        .add_plugins(shared::SharedPlugin::<CardinalNeighborhood>::default())
        // Observe the LayerCreated event to build the grid from the Tiled layer
        .add_observer(layer_created)
        // Startup and State Systems
        .add_systems(Startup, startup)
        .add_systems(OnEnter(shared::State::Playing), spawn_minions)
        // Update Systems
        .add_systems(
            Update,
            (
                move_pathfinders.before(PathingSet),
                set_new_goal.run_if(in_state(shared::State::Playing)),
                handle_reroute_failed.run_if(in_state(shared::State::Playing)),
            ),
        )
        .run();
}

fn startup(mut commands: Commands, asset_server: Res<AssetServer>) {
    // Get our anchor positioning calculated
    let anchor = TilemapAnchor::Center;

    let tilemap_size = TilemapSize { x: 128, y: 128 };
    let tilemap_gridsize = TilemapGridSize { x: 8.0, y: 8.0 };
    let tilemap_tilesize = TilemapTileSize { x: 8.0, y: 8.0 };

    let offset = anchor.as_offset(
        &tilemap_size,
        &tilemap_gridsize,
        &tilemap_tilesize,
        &TilemapType::Square,
    );

    let camera_offset = Vec3::new(
        offset.x + (tilemap_size.x as f32 * tilemap_gridsize.x) / 2.0,
        offset.y + (tilemap_size.y as f32 * tilemap_gridsize.y) / 2.0,
        1.0,
    );

    // Spawn a 2D camera and set the position based on the centered anchor offset
    commands.spawn(Camera2d).insert(Transform {
        translation: camera_offset,
        ..Default::default()
    });

    // Load the map ...
    let map_handle: Handle<TiledMap> = asset_server.load("demo_128.tmx");

    let mut map_entity = commands.spawn((TiledMapHandle(map_handle), anchor));

    let grid_settings = GridSettingsBuilder::new_2d(128, 128)
        .chunk_size(16)
        .allow_corner_clipping()
        .enable_collision()
        .avoidance_distance(4)
        .build();

    // Insert the grid as a child of the map entity. This won't currently affect anything, but in the future
    // we may want to have the grid as a child of the map entity so that multiple grids can be supported.
    map_entity.insert((
        TilemapRenderSettings {
            render_chunk_size: UVec2::new(32, 32),
            ..Default::default()
        },
        Grid::<CardinalNeighborhood>::new(&grid_settings),
    ));

    // Add the debug map as a child of the map entity
    // Set the translation to offset the the debug gizmos.
    map_entity.with_child((
        DebugMap {
            tile_width: 8,
            tile_height: 8,
            map_type: DebugMapType::Square,
            draw_chunks: true,
            draw_points: false,
            draw_entrances: true,
            draw_cached_paths: false,
        },
        DebugOffset(offset.extend(0.0)),
    ));
}

fn layer_created(
    trigger: Trigger<TiledLayerCreated>,
    map_asset: Res<Assets<TiledMap>>,
    grid: Single<&mut Grid<CardinalNeighborhood>>,
    mut state: ResMut<NextState<shared::State>>,
) {
    let mut grid = grid.into_inner();

    let layer = trigger.event().get_layer(&map_asset);
    if let Some(layer) = layer {
        if let Some(tile_layer) = layer.as_tile_layer() {
            let width = tile_layer.width().unwrap();
            let height = tile_layer.height().unwrap();

            for x in 0..width {
                for y in 0..height {
                    let tile = tile_layer.get_tile(x as i32, y as i32);
                    if let Some(tile) = tile {
                        let tile_id = tile.id();

                        if tile_id == 14 {
                            grid.set_point(UVec3::new(x, height - 1 - y, 0), Point::new(1, false));
                        } else {
                            grid.set_point(UVec3::new(x, height - 1 - y, 0), Point::new(0, true));
                        }
                    }
                }
            }
        }
    }

    info!("Loaded layer: {:?}", layer);
    grid.build();

    state.set(shared::State::Playing);
}

fn spawn_minions(
    mut commands: Commands,
    grid: Query<&Grid<CardinalNeighborhood>>,
    layer_entity: Query<Entity, With<TiledMapTileLayer>>,
    asset_server: Res<AssetServer>,
    mut walkable: ResMut<shared::Walkable>,
    config: Res<shared::Config>,
) {
    let grid = if let Ok(grid) = grid.single() {
        grid
    } else {
        return;
    };

    let layer_entity = layer_entity.iter().next().unwrap();

    walkable.tiles = Vec::new();
    for x in 0..grid.width() {
        for y in 0..grid.height() {
            if !grid.point(UVec3::new(x, y, 0)).solid {
                let position = Vec3::new(x as f32 * 8.0, y as f32 * 8.0, 0.0);

                walkable.tiles.push(position);
            }
        }
    }

    let mut count = 0;

    while count < 128 {
        let position = walkable.tiles.choose(&mut rand::rng()).unwrap();
        let goal = walkable.tiles.choose(&mut rand::rng()).unwrap();

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
                color,
                ..Default::default()
            })
            .insert(Name::new(format!("{:?}", color)))
            .insert(DebugPath {
                tile_width: 8,
                tile_height: 8,
                map_type: DebugMapType::Square,
                color,
                draw_unrefined: false,
            })
            .insert(Blocking)
            .insert(Transform::from_translation(transform))
            .insert(GridPos(UVec3::new(
                (position.x / 8.0) as u32,
                (position.y / 8.0) as u32,
                0,
            )))
            .insert(Pathfind {
                goal: UVec3::new((goal.x / 8.0) as u32, (goal.y / 8.0) as u32, 0),
                use_astar: config.use_astar,
            })
            .insert(ChildOf(layer_entity));

        count += 1;
    }
}

fn move_pathfinders(
    mut commands: Commands,
    mut query: Query<(Entity, &mut GridPos, &NextPos)>,
    tilemap: Single<(
        &TilemapSize,
        &TilemapTileSize,
        &TilemapGridSize,
        &TilemapAnchor,
    )>,
    mut tick_reader: EventReader<shared::Tick>,
) {
    let (map_size, tile_size, grid_size, anchor) = tilemap.into_inner();

    for _ in tick_reader.read() {
        let offset = anchor.as_offset(map_size, grid_size, tile_size, &TilemapType::Square);

        for (entity, mut position, next) in query.iter_mut() {
            position.0 = next.0;

            let translation = Vec3::new(
                next.0.x as f32 * grid_size.x + offset.x,
                next.0.y as f32 * grid_size.y + offset.y,
                4.0,
            );

            commands
                .entity(entity)
                .insert(Transform::from_translation(translation))
                .remove::<NextPos>();
        }
    }
}

fn set_new_goal(
    mut commands: Commands,
    mut minions: Query<Entity, (Without<Path>, Without<Pathfind>)>,
    walkable: Res<shared::Walkable>,
    config: Res<shared::Config>,
) {
    for entity in minions.iter_mut() {
        let new_goal = walkable.tiles.choose(&mut rand::rng()).unwrap();

        commands.entity(entity).insert(Pathfind {
            goal: UVec3::new((new_goal.x / 8.0) as u32, (new_goal.y / 8.0) as u32, 0),
            use_astar: config.use_astar,
        });
    }
}

fn handle_reroute_failed(
    mut commands: Commands,
    mut query: Query<(Entity, &Pathfind, &RerouteFailed)>,
    mut tick_reader: EventReader<shared::Tick>,
) {
    for _ in tick_reader.read() {
        for (entity, pathfind, _) in query.iter_mut() {
            commands.entity(entity).remove::<RerouteFailed>();
            commands.entity(entity).insert(Pathfind {
                goal: pathfind.goal,
                use_astar: true,
            });
        }
    }
}
