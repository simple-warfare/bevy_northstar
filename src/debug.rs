//! Plugin to add systems for drawing gizmos. For debugging pathfinding.
use bevy::{color::palettes::css, math::Vec2, prelude::*};

use crate::{
    components::{DebugMap, DebugPath},
    grid::Grid,
    neighbor::Neighborhood,
    path::Path,
};

/// Required to calculate how to draw the debug gizmos
#[derive(Reflect, Debug, Clone, Default)]
pub enum DebugMapType {
    #[default]
    Square,
    Isometric,
}

// #[derive(Reflect, Debug, Clone, Default)]
// pub enum DebugMapAnchor {
//     #[default]
//     None,
//     Center,
//     BottomLeft,
//     BottomCenter,
//     BottomRight,
//     CenterLeft,
//     CenterRight,
//     TopLeft,
//     TopCenter,
//     TopRight,
//     Custom(Vec2),
// }

/// Debug plugin for the Northstar pathfinding library.
/// Add this plugin to your app to add the systems required to draw the debug helper gizmos.
///
/// # Example
///
/// ```
/// use bevy::prelude::*;
/// use bevy_northstar::prelude::*;
///
/// fn main() {
///    App::new()
///       .add_plugins(DefaultPlugins)
///       .add_plugins(NorthstarDebugPlugin::<CardinalNeighborhood>::default())
///       .add_systems(Startup, setup);
/// }
///
/// fn setup(mut commands: Commands) {
///   commands.spawn(DebugMap {
///       tile_width: 32,
///       tile_height: 32,
///       map_type: DebugMapType::Square,
///       draw_chunks: true,
///       draw_points: true,
///       draw_entrances: true,
///       draw_cached_paths: true,
///   });
/// }
#[derive(Clone)]
pub struct NorthstarDebugPlugin<N: Neighborhood + 'static> {
    /// Debug gizmos configuration
    pub _marker: std::marker::PhantomData<N>,
}

impl<N: Neighborhood + 'static> Default for NorthstarDebugPlugin<N> {
    fn default() -> Self {
        NorthstarDebugPlugin {
            _marker: std::marker::PhantomData,
        }
    }
}

impl<N: Neighborhood + 'static> Plugin for NorthstarDebugPlugin<N> {
    fn build(&self, app: &mut bevy::prelude::App) {
        app.add_systems(Update, (draw_debug_map::<N>, draw_debug_paths::<N>))
            .register_type::<DebugMap>()
            .register_type::<DebugMapType>();
            //.register_type::<DebugMapAnchor>();
    }
}

// / Draw the debug gizmos for the chunks, points, entrances, and cached paths.
fn draw_debug_map<N: Neighborhood + 'static>(
    query: Query<(&Transform, &DebugMap)>,
    grid: Query<&Grid<N>>,
    mut gizmos: Gizmos,
) {
    let grid = if let Ok(grid) = grid.single() {
        grid
    } else {
        return;
    };

    for (transform, debug_map) in query.iter() {
        //let offset = transform.translation.truncate();
        let offset = transform.translation.truncate();

        if debug_map.draw_chunks {
            // Draw chunk boundaries
            let chunk_size = grid.chunk_size();
            let chunk_width = grid.width() / chunk_size;
            let chunk_height = grid.height() / chunk_size;

            match debug_map.map_type {
                DebugMapType::Square => {
                    for x in 0..chunk_width {
                        for y in 0..chunk_height {
                            let bottom_left = Vec2::new(
                                (x * chunk_size) as f32 * debug_map.tile_width as f32,
                                (y * chunk_size) as f32 * debug_map.tile_height as f32,
                            );
                            let bottom_right = Vec2::new(
                                ((x + 1) * chunk_size) as f32 * debug_map.tile_width as f32,
                                (y * chunk_size) as f32 * debug_map.tile_height as f32,
                            );
                            let top_left = Vec2::new(
                                (x * chunk_size) as f32 * debug_map.tile_width as f32,
                                ((y + 1) * chunk_size) as f32 * debug_map.tile_height as f32,
                            );
                            let top_right = Vec2::new(
                                ((x + 1) * chunk_size) as f32 * debug_map.tile_width as f32,
                                ((y + 1) * chunk_size) as f32 * debug_map.tile_height as f32,
                            );

                            let bottom_left = bottom_left + offset - debug_map.tile_width as f32 * 0.5;
                            let bottom_right = bottom_right + offset - debug_map.tile_height as f32 * 0.5;
                            let top_left = top_left + offset - debug_map.tile_width as f32 * 0.5;
                            let top_right = top_right + offset - debug_map.tile_height as f32 * 0.5;

                            gizmos.line_2d(bottom_left, bottom_right, css::WHITE);
                            gizmos.line_2d(bottom_right, top_right, css::WHITE);
                            gizmos.line_2d(top_right, top_left, css::WHITE);
                            gizmos.line_2d(top_left, bottom_left, css::WHITE);
                        }
                    }
                }
                DebugMapType::Isometric => {
                    for x in 1..chunk_width {
                        for y in 1..chunk_height {
                            let position = Vec2::new(
                                (y as f32 + x as f32)
                                    * (debug_map.tile_width as f32 * 0.5)
                                    * chunk_size as f32,
                                (y as f32 - x as f32)
                                    * (debug_map.tile_height as f32 * 0.5)
                                    * chunk_size as f32,
                            );

                            let top = position
                                + Vec2::new(0.0, debug_map.tile_height as f32 * chunk_size as f32);
                            let right = position
                                + Vec2::new(debug_map.tile_width as f32 * chunk_size as f32, 0.0);
                            let bottom = position
                                - Vec2::new(0.0, debug_map.tile_height as f32 * chunk_size as f32);
                            let left = position
                                - Vec2::new(debug_map.tile_width as f32 * chunk_size as f32, 0.0);

                            gizmos.line_2d(top + offset, right + offset, css::WHITE);
                            gizmos.line_2d(right + offset, bottom + offset, css::WHITE);
                            gizmos.line_2d(bottom + offset, left + offset, css::WHITE);
                            gizmos.line_2d(left + offset, top + offset, css::WHITE);
                        }
                    }
                }
            }
        }

        if debug_map.draw_points {
            // Draw point gizmos
            for x in 0..grid.width() {
                for y in 0..grid.height() {
                    let point = grid.point(UVec3::new(x, y, 0));
                    let color = if point.wall { css::RED } else { css::WHITE };

                    let position = match debug_map.map_type {
                        DebugMapType::Square => Vec2::new(
                            (x * debug_map.tile_width) as f32,
                            (y * debug_map.tile_height) as f32,
                        ),
                        DebugMapType::Isometric => Vec2::new(
                            (y as f32 + x as f32) * (debug_map.tile_width as f32 * 0.5),
                            (y as f32 - x as f32) * (debug_map.tile_height as f32 * 0.5),
                        ),
                    };

                    gizmos.circle_2d(position + offset, 2.0, color);
                }
            }
        }

        if debug_map.draw_entrances {
            // Draw graph nodes
            for node in grid.graph().nodes() {
                let position = match debug_map.map_type {
                    DebugMapType::Square => Vec2::new(
                        (node.pos.x * debug_map.tile_width) as f32,
                        (node.pos.y * debug_map.tile_height) as f32,
                    ),
                    DebugMapType::Isometric => Vec2::new(
                        (node.pos.y as f32 + node.pos.x as f32)
                            * (debug_map.tile_width as f32 * 0.5),
                        (node.pos.y as f32 - node.pos.x as f32)
                            * (debug_map.tile_height as f32 * 0.5),
                    ),
                };

                gizmos.circle_2d(position + offset, 2.0, css::MAGENTA);

                // Draw the node connection only to nodes in other chunks
                for edge in node.edges() {
                    let neighbor = grid.graph().node_at(edge);
                    if let Some(neighbor) = neighbor {
                        if neighbor.chunk != node.chunk {
                            let neighbor_position = match debug_map.map_type {
                                DebugMapType::Square => Vec2::new(
                                    (neighbor.pos.x * debug_map.tile_width) as f32,
                                    (neighbor.pos.y * debug_map.tile_height) as f32,
                                ),
                                DebugMapType::Isometric => Vec2::new(
                                    (neighbor.pos.y as f32 + neighbor.pos.x as f32)
                                        * (debug_map.tile_width as f32 * 0.5),
                                    (neighbor.pos.y as f32 - neighbor.pos.x as f32)
                                        * (debug_map.tile_height as f32 * 0.5),
                                ),
                            };
                            gizmos.line_2d(
                                position + offset,
                                neighbor_position + offset,
                                css::GREEN,
                            );
                        }
                    }
                }
            }
        }

        if debug_map.draw_cached_paths {
            let path_colors = [
                css::RED,
                css::PURPLE,
                css::YELLOW,
                css::PINK,
                css::DARK_CYAN,
                css::MAGENTA,
                css::GREY,
                css::GREEN,
                css::ORANGE,
                css::NAVY,
            ];

            let mut color_index = 0;

            // Draw cached paths
            for path in grid.graph().all_paths() {
                if debug_map.draw_entrances && !debug_map.draw_cached_paths {
                    if path.len() > 2 {
                        continue;
                    }
                }

                // Iterate over path.path() drawing a line from one point to the next point until completed
                let mut iter = path.path().iter();
                let mut prev = iter.next().unwrap();

                for next in iter {
                    let prev_position = match debug_map.map_type {
                        DebugMapType::Square => Vec2::new(
                            (prev.x * debug_map.tile_width) as f32,
                            (prev.y * debug_map.tile_height) as f32,
                        ),
                        DebugMapType::Isometric => Vec2::new(
                            (prev.y as f32 + prev.x as f32) * (debug_map.tile_width as f32 * 0.5),
                            (prev.y as f32 - prev.x as f32) * (debug_map.tile_height as f32 * 0.5),
                        ),
                    };

                    let next_position = match debug_map.map_type {
                        DebugMapType::Square => Vec2::new(
                            (next.x * debug_map.tile_width) as f32,
                            (next.y * debug_map.tile_height) as f32,
                        ),
                        DebugMapType::Isometric => Vec2::new(
                            (next.y as f32 + next.x as f32) * (debug_map.tile_width as f32 * 0.5),
                            (next.y as f32 - next.x as f32) * (debug_map.tile_height as f32 * 0.5),
                        ),
                    };

                    let mut color = css::BLUE;

                    if debug_map.draw_entrances && debug_map.draw_cached_paths {
                        color = path_colors[color_index % path_colors.len()];
                    }

                    gizmos.line_2d(
                        prev_position + offset,
                        next_position + offset,
                        color,
                    );

                    prev = next;
                }

                color_index += 1;
            }
        }
    }
}

// Draw the debug gizmos for [`DebugPath`]s.
fn draw_debug_paths<N: Neighborhood + 'static>(
    query: Query<(&DebugPath, &Path)>,
    debug_map: Single<&Transform, With<DebugMap>>,
    mut gizmos: Gizmos,
) {
    let transform = debug_map.into_inner();

    for (debug_path, path) in query.iter() {
        if path.is_empty() {
            continue;
        }

        /*let center_offset = match debug_path.map_type {
            DebugMapType::Square => Vec2::new(
                debug_path.tile_width as f32 * 0.5,
                debug_path.tile_height as f32 * 0.5,
            ),
            DebugMapType::Isometric => Vec2::new(debug_path.tile_width as f32 * 0.5, 0.0),
        };*/

        let center_offset = transform.translation.truncate();

        // Iterate over path.path() drawing a line from one point to the next point until completed
        let mut iter = path.path().iter();
        let mut prev = iter.next().unwrap();

        for next in iter {
            let prev_position = match debug_path.map_type {
                DebugMapType::Square => Vec2::new(
                    (prev.x * debug_path.tile_width) as f32,
                    (prev.y * debug_path.tile_height) as f32,
                ),
                DebugMapType::Isometric => Vec2::new(
                    (prev.y as f32 + prev.x as f32) * (debug_path.tile_width as f32 * 0.5),
                    (prev.y as f32 - prev.x as f32) * (debug_path.tile_height as f32 * 0.5),
                ),
            };

            let next_position = match debug_path.map_type {
                DebugMapType::Square => Vec2::new(
                    (next.x * debug_path.tile_width) as f32,
                    (next.y * debug_path.tile_height) as f32,
                ),
                DebugMapType::Isometric => Vec2::new(
                    (next.y as f32 + next.x as f32) * (debug_path.tile_width as f32 * 0.5),
                    (next.y as f32 - next.x as f32) * (debug_path.tile_height as f32 * 0.5),
                ),
            };

            let x = prev_position + center_offset;
            let y = next_position + center_offset;

            gizmos.line_2d(x, y, debug_path.color);

            prev = next;
        }

        if debug_path.draw_unrefined {
            let mut iter = path.graph_path.iter();
            let mut prev = if let Some(p) = iter.next() {
                p
            } else {
                continue;
            };

            let inverted_color = debug_path.color.to_srgba();
            let inverted_color = Color::srgba(
                1.0 - inverted_color.red,
                1.0 - inverted_color.green,
                1.0 - inverted_color.blue,
                inverted_color.alpha,
            );

            for next in iter {
                let prev_position = match debug_path.map_type {
                    DebugMapType::Square => Vec2::new(
                        (prev.x * debug_path.tile_width) as f32,
                        (prev.y * debug_path.tile_height) as f32,
                    ),
                    DebugMapType::Isometric => Vec2::new(
                        (prev.y as f32 + prev.x as f32) * (debug_path.tile_width as f32 * 0.5),
                        (prev.y as f32 - prev.x as f32) * (debug_path.tile_height as f32 * 0.5),
                    ),
                };

                let next_position = match debug_path.map_type {
                    DebugMapType::Square => Vec2::new(
                        (next.x * debug_path.tile_width) as f32,
                        (next.y * debug_path.tile_height) as f32,
                    ),
                    DebugMapType::Isometric => Vec2::new(
                        (next.y as f32 + next.x as f32) * (debug_path.tile_width as f32 * 0.5),
                        (next.y as f32 - next.x as f32) * (debug_path.tile_height as f32 * 0.5),
                    ),
                };

                let x = prev_position + center_offset;
                let y = next_position + center_offset;

                gizmos.line_2d(x, y, inverted_color);

                prev = next;
            }
        }
    }
}
