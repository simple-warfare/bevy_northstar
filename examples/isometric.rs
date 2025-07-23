// Example of a "3d" isometric tilemap using Bevy Northstar
use bevy::{ecs::query::QueryData, log, prelude::*};
use bevy_ecs_tiled::prelude::*;
use bevy_ecs_tilemap::prelude::*;
use bevy_northstar::prelude::*;

// Portal is at 1,36,4

// Game state
#[derive(Clone, Debug, Default, Hash, Eq, States, PartialEq)]
pub enum State {
    #[default]
    Loading,
    Playing,
}

// Tiled tile properties component
#[derive(Component, Default, Debug, Reflect)]
#[reflect(Component)]
pub struct Tile {
    pub info: TileInfo,
}

// Tiled TileInfo properties
#[derive(Component, Default, Debug, Reflect, Clone)]
#[reflect(Component)]
pub struct TileInfo {
    pub height: i32,
    pub ramp: bool,
}

// Tiled Portal property
#[derive(Component, Default, Debug, Reflect)]
#[reflect(Component)]
pub struct PortalInfo {
    pub target: (u16, u16),
}

// Player marker
#[derive(Component)]
pub struct Player;

// For holding z offset data
#[derive(Component, Reflect, Debug)]
pub struct YSort(pub f32);

// Place the pivot point for the player sprite for correct y-sorting
#[derive(Component, Debug, Reflect)]
pub struct Pivot(pub Vec2);


// Event that lets other systems know to wait until animations are completed.
#[derive(Debug, Event)]
pub struct AnimationWaitEvent;

// Event that signals the completion of the tilemap loading.
#[derive(Debug, Event)]
pub struct LoadCompleteEvent;

// Cursor resource to track the mouse position and hovered tile
#[derive(Default, Resource, Reflect)]
pub struct Cursor {
    pub position: Vec2,
    pub tile: Option<UVec3>,
}

// Common map query to find positioning
#[derive(QueryData)]
#[query_data(derive(Debug))]
struct MapQuery {
    grid_size: &'static TilemapGridSize,
    map_size: &'static TilemapSize,
    tile_size: &'static TilemapTileSize,
    map_type: &'static TilemapType,
    anchor: &'static TilemapAnchor,
}

const LAYER_Z_OFFSET: f32 = 16.0; // Z offset for each layer
const HEIGHT_OFFSET: f32 = 4.0;
const MAX_HEIGHT: u32 = 9;
// We need to adjust the z offset based on the layer that the height is in.
const PLAYER_CENTER_OFFSET: f32 = 4.0; // Center offset for player sprite

pub const LERP_SPEED: f32 = 22.0;
pub const POSITION_TOLERANCE: f32 = 0.1;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(ImagePlugin::default_nearest()))
        .add_plugins(TiledMapPlugin::default())
        // Add the Northstar Plugin with an isometric neighborhood for 3D pathfinding.
        .add_plugins(NorthstarPlugin::<OrdinalNeighborhood3d>::default())
        // Add the Debug Plugin to visualize the grid and pathfinding.
        .add_plugins(NorthstarDebugPlugin::<OrdinalNeighborhood3d>::default())
        .add_event::<LoadCompleteEvent>()
        .add_event::<AnimationWaitEvent>()
        .add_systems(Startup, startup)
        .add_systems(
            PreUpdate,
            (
                input,
                debug_input,
                update_cursor,
                move_pathfinders,
            ).run_if(in_state(State::Playing))
        )
        .add_systems(
            Update,
            (
                animate_move,
                pathfind_error,
            )
                .run_if(in_state(State::Playing)),
        )
        .add_systems(
            PostUpdate,
            (
                y_sort,
                camera_follow_player,
            ).run_if(in_state(State::Playing)),
        )
        .add_observer(tile_created)
        .add_observer(loading_complete)
        .register_type::<Tile>()
        .register_type::<TileInfo>()
        .register_type::<PortalInfo>()
        .insert_resource(Cursor::default())
        .insert_state(State::Loading)
        .run();
}

fn startup(mut commands: Commands, asset_server: Res<AssetServer>) {
    commands.spawn(Camera2d);

    let grid_settings = GridSettingsBuilder::new_3d(64, 64, 10)
        .chunk_size(16)
        .default_impassable()
        // This is a great example of when to use a neighbor filter.
        // Since we're Y Sorting, we don't want to allow the player to move diagonally around walls as the sprite will z transition through the wall.
        .add_neighbor_filter(filter::NoCornerCutting)
        .build();

    let map_handle: Handle<TiledMap> = asset_server.load("isotilemap.tmx");

    commands.spawn((
        TiledMapHandle(map_handle),
        TilemapAnchor::Center,
        TilemapRenderSettings {
            render_chunk_size: UVec2::new(1, 1),
            y_sort: true,
        },
        TiledMapLayerZOffset(LAYER_Z_OFFSET),
        Grid::<OrdinalNeighborhood3d>::new(&grid_settings),
    ));
}

fn tile_created(
    trigger: Trigger<TiledTileCreated>,
    mut query: Query<(&mut TileInfo, &TilePos)>,
    grid: Single<&mut OrdinalGrid3d>,
    mut commands: Commands,
) {
    let mut grid = grid.into_inner();

    if let Ok((mut tile_info, tile_pos)) = query.get_mut(trigger.event().entity) {
        // HACK: I'm not aware of anyway to know when all the tiles have been fully loaded.
        // There's a hiddden tile with a height of 99 in the corner of the map on layer 2
        // with a height of 99 that is used to determine the end of the tilemap loading.
        if tile_info.height == 99 {
            commands.trigger(LoadCompleteEvent);
            return;
        }

        let layer = trigger.event().layer;

        // Treat the higher layers as having a higher height offset.
        let layer_height_offset = if layer.id == 1 { 4 } else { 0 };

        // Readjust the tile_info height based on the layer.
        tile_info.height += layer_height_offset as i32;

        if tile_info.ramp {
            grid.set_nav(
                UVec3::new(tile_pos.x, tile_pos.y, tile_info.height as u32),
                Nav::Portal(Portal::new(
                    UVec3::new(tile_pos.x, tile_pos.y, 4 + layer_height_offset),
                    1,
                    false,
                )),
            );

            // Ensure that the elevation change destination is set to passable
            let target_pos = UVec3::new(tile_pos.x, tile_pos.y, 4 + layer_height_offset);
            if !grid.in_bounds(target_pos) {
                log::warn!(
                    "Target position {:?} is out of bounds for elevation change",
                    target_pos
                );
            } else {
                grid.set_nav(
                    target_pos,
                    Nav::Portal(Portal::new(
                        UVec3::new(tile_pos.x, tile_pos.y, tile_info.height as u32),
                        1,
                        false,
                    )),
                );
            }
        } else {
            let pos = UVec3::new(tile_pos.x, tile_pos.y, tile_info.height as u32);
            grid.set_nav(pos, Nav::Passable(1));

            if tile_info.height > 4 {
                // We need to squash the few positions below so creating maps isn't a pain.
                for z in 2..tile_info.height {
                    let squash_pos = UVec3::new(tile_pos.x, tile_pos.y, z as u32);
                    if grid.in_bounds(squash_pos) {
                        grid.set_nav(squash_pos, Nav::Impassable);
                    }
                }
            }
        }
    }
}

fn loading_complete(
    _: Trigger<LoadCompleteEvent>,
    map_query: Query<MapQuery>,
    camera: Single<(&mut Transform, &mut Projection), With<Camera>>,
    grid: Single<(Entity, &mut OrdinalGrid3d)>,
    asset_server: Res<AssetServer>,
    mut state: ResMut<NextState<State>>,
    mut commands: Commands,
) {
    let (entity, mut grid) = grid.into_inner();

    grid.build();

    let player_start = UVec3::new(32, 25, 0);

    // Insert the debug grid as a child to the grid entity
    if let Some(map) = map_query.iter().next() {
        commands.entity(entity).with_child((
            DebugGridBuilder::new(32, 16)
                .isometric()
                .build(),
            DebugOffset(
                map.anchor
                    .as_offset(map.map_size, map.grid_size, map.tile_size, map.map_type)
                    .extend(0.0),
            ),
            DebugDepthYOffsets(
                (0..=MAX_HEIGHT)
                    .map(|h| (h, h as f32 * HEIGHT_OFFSET))
                    .collect(),
            ),
        ));
    }

    let center = map_query
        .iter()
        .next()
        .map(|map| {
            TilePos::new(player_start.x, player_start.y).center_in_world(
                map.map_size,
                map.grid_size,
                map.tile_size,
                map.map_type,
                map.anchor,
            )
        })
        .unwrap();

    log::info!("Spawning player at: {:?}", center);

    commands.spawn((
        Player,
        Sprite::from_image(asset_server.load("player.png")),
        AgentPos(player_start),
        Transform::from_translation(Vec3::new(center.x, center.y + PLAYER_CENTER_OFFSET, 0.0)),
        YSort(0.0),
        Pivot(Vec2::new(0.0, -10.0)),
    ));

    // Zoom camera into the player
    let (mut transform, mut projection) = camera.into_inner();
    if let Projection::Orthographic(ref mut ortho) = &mut *projection {
        ortho.scale = 0.5;
        transform.translation = Vec3::new(center.x, center.y + PLAYER_CENTER_OFFSET, 10.0);
    }

    state.set(State::Playing);
}

fn update_cursor(
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform, &Transform), With<Camera>>,
    mut cursor: ResMut<Cursor>,
    map_query: Query<MapQuery>,
    tile_storage: Query<&TileStorage>,
    tile_info: Query<&TileInfo>,
) {
    let window = window.into_inner();
    let (camera, camera_transform, _) = camera.into_inner();

    if let Some(cursor_position) = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world_2d(camera_transform, cursor).ok())
    {
        let map = map_query.iter().next().expect("No map found in the query");

        let offset = Vec2::new(0.0, map.grid_size.y / 2.0);

        let mut selected_tile = None;

        // Test from highest to lowest height
        for test_height in (0..=MAX_HEIGHT).rev() {
            let height_visual_offset = test_height as f32 * HEIGHT_OFFSET;

            let adjusted_cursor = cursor_position + offset + Vec2::new(0.0, -height_visual_offset);

            if let Some(tile_pos) = TilePos::from_world_pos(
                &adjusted_cursor,
                map.map_size,
                map.grid_size,
                map.tile_size,
                map.map_type,
                map.anchor,
            ) {
                // Find the tile with the highest height at this position
                let mut top_tile: Option<(Entity, i32)> = None;
                for storage in tile_storage.iter() {
                    if let Some(tile) = storage.get(&tile_pos) {
                        if let Ok(info) = tile_info.get(tile) {
                            if top_tile.is_none() || info.height > top_tile.unwrap().1 {
                                top_tile = Some((tile, info.height));
                            }
                        }
                    }
                }

                if let Some((_, height)) = top_tile {
                    if height as u32 == test_height as u32 {
                        selected_tile =
                            Some(UVec3::new(tile_pos.x, tile_pos.y, test_height as u32));
                        break; // Found the topmost tile
                    }
                }
            }
        }

        cursor.position = cursor_position;
        cursor.tile = selected_tile;
    }
}

fn input(
    player: Single<Entity, With<Player>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    camera: Single<&mut Transform, With<Camera>>,
    cursor: Res<Cursor>,
    mut commands: Commands,
) {
    let player = player.into_inner();
    let mut camera_transform = camera.into_inner();

    if keyboard.pressed(KeyCode::KeyW) {
        // Move the camera up
        camera_transform.translation.y += 10.0;
    }
    if keyboard.pressed(KeyCode::KeyS) {
        // Move the camera down
        camera_transform.translation.y -= 10.0;
    }
    if keyboard.pressed(KeyCode::KeyA) {
        // Move the camera left
        camera_transform.translation.x -= 10.0;
    }
    if keyboard.pressed(KeyCode::KeyD) {
        // Move the camera right
        camera_transform.translation.x += 10.0;
    }

    if mouse.just_pressed(MouseButton::Left) {
        if let Some(tile) = cursor.tile {
            log::info!("Pathfinding to tile: {:?}", tile);
            commands
                .entity(player)
                .insert(Pathfind::new_3d(tile.x, tile.y, tile.z));
        }
    }
}


fn pathfind_error(query: Query<Entity, With<PathfindingFailed>>, mut commands: Commands) {
    for entity in query.iter() {
        log::error!("Pathfinding failed for entity: {:?}", entity);
        commands
            .entity(entity)
            .remove::<PathfindingFailed>()
            .remove::<Pathfind>()
            .remove::<NextPos>();
    }
}

fn debug_input(keyboard: Res<ButtonInput<KeyCode>>, mut debug_query: Query<&mut DebugGrid>) {
    if let Ok(mut debug_grid) = debug_query.single_mut() {
        if keyboard.just_pressed(KeyCode::Backquote) {
            debug_grid.toggle_cells();
            debug_grid.toggle_chunks();
            debug_grid.toggle_entrances();
        }

        if keyboard.just_pressed(KeyCode::Equal) {
            let new_depth = debug_grid.depth() + 1;

            if new_depth <= MAX_HEIGHT {
                debug_grid.set_depth(new_depth);
            }
        }

        if keyboard.just_pressed(KeyCode::Minus) {
            let current_depth = debug_grid.depth();
            debug_grid.set_depth(current_depth.saturating_sub(1));
        }
    }
}


fn move_pathfinders(
    mut commands: Commands,
    mut query: Query<(Entity, &mut AgentPos, &NextPos)>,
    animation_reader: EventReader<AnimationWaitEvent>,
) {
    if animation_reader.len() > 0 {
        return;
    }

    for (entity, mut position, next) in query.iter_mut() {
        position.0 = next.0;

        commands.entity(entity).remove::<NextPos>();
    }
}


fn animate_move(
    mut query: Query<(&AgentPos, &mut Transform, &mut YSort)>,
    map_query: Query<MapQuery>,
    time: Res<Time>,
    mut ev_wait: EventWriter<AnimationWaitEvent>,
) {
    let map = map_query.iter().next().expect("No map found in the query");

    for (position, mut transform, mut ysort) in query.iter_mut() {
        let tile_pos = TilePos {
            x: position.0.x as u32,
            y: position.0.y as u32,
        };

        let base_vec = TilePos::center_in_world(
            &tile_pos,
            map.map_size,
            map.grid_size,
            map.tile_size,
            map.map_type,
            map.anchor,
        );

        let height_offset = position.0.z as f32 * HEIGHT_OFFSET;

        ysort.0 = height_offset - (MAX_HEIGHT - 1) as f32 * HEIGHT_OFFSET;

        let target = Vec3::new(
            base_vec.x,
            base_vec.y + height_offset + PLAYER_CENTER_OFFSET, // Apply elevation and center offset here
            transform.translation.z,
        );

        let d = (target - transform.translation).length();
        let animating = if d > POSITION_TOLERANCE {
            transform.translation = transform
                .translation
                .lerp(target, LERP_SPEED * time.delta_secs());
            true
        } else {
            transform.translation = target;
            false
        };

        if animating {
            ev_wait.write(AnimationWaitEvent);
        }
    }
}

fn y_sort(
    mut query: Query<(&mut Transform, &YSort, &Pivot)>,
    map_query: Query<(&TilemapSize, &TilemapTileSize)>,
) {
    let Some((map_size, tile_size)) = map_query.iter().next() else { return; };

    let max_y = map_size.y as f32 * tile_size.y;

    for (mut transform, ysort, pivot) in query.iter_mut() {
        let y = transform.translation.y + pivot.0.y;

        transform.translation.z = ysort.0 + (1.0 - (y / max_y));
    }
}

const CAMERA_MARGIN: f32 = 25.0;
const CAMERA_LERP_SPEED: f32 = 8.0;

fn camera_follow_player(
    time: Res<Time>,
    windows: Query<&Window>,
    mut camera_query: Query<&mut Transform, (With<Camera2d>, Without<Player>)>,
    player_query: Single<&Transform, With<Player>>,
) {
    let Some(window) = windows.iter().next() else {
        return;
    };

    let Some(mut camera_transform) = camera_query.iter_mut().next() else {
        return;
    };

    let player_transform = player_query.into_inner();

    let player_pos = player_transform.translation.truncate();
    let camera_pos = camera_transform.translation.truncate();

    let quarter_width = window.width() / 8.0;
    let quarter_height = window.height() / 8.0;

    let left_edge = camera_pos.x - quarter_width + CAMERA_MARGIN;
    let right_edge = camera_pos.x + quarter_width - CAMERA_MARGIN;
    let bottom_edge = camera_pos.y - quarter_height + CAMERA_MARGIN;
    let top_edge = camera_pos.y + quarter_height - CAMERA_MARGIN;

    let mut target_camera_pos = camera_pos;

    if player_pos.x < left_edge {
        target_camera_pos.x = player_pos.x + quarter_width - CAMERA_MARGIN;
    } else if player_pos.x > right_edge {
        target_camera_pos.x = player_pos.x - quarter_width + CAMERA_MARGIN;
    }

    if player_pos.y < bottom_edge {
        target_camera_pos.y = player_pos.y + quarter_height - CAMERA_MARGIN;
    } else if player_pos.y > top_edge {
        target_camera_pos.y = player_pos.y - quarter_height + CAMERA_MARGIN;
    }

    // Smoothly interpolate the camera position
    let lerped = camera_pos.lerp(target_camera_pos, CAMERA_LERP_SPEED * time.delta_secs());
    camera_transform.translation.x = lerped.x;
    camera_transform.translation.y = lerped.y;
}