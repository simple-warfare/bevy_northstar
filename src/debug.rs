//! This module contains some tools to help you debug your application.
//!
use bevy::math::Vec2;
use bevy::{color::palettes::css, prelude::*};

use crate::grid::Grid;
use crate::neighbor::Neighborhood;
use crate::path::Path;

#[derive(Reflect, Debug, Clone, Default)]
pub enum MapType {
    #[default]
    Square,
    Isometric,
}

#[derive(Reflect, Component)]
#[require(Transform)]
pub struct DebugMap {
    pub tile_width: u32,
    pub tile_height: u32,
    pub map_type: MapType,
    //
    pub draw_chunks: bool,
    pub draw_points: bool,
    pub draw_entrances: bool,
    pub draw_cached_paths: bool,
}

impl Default for DebugMap {
    fn default() -> Self {
        DebugMap {
            tile_width: 16,
            tile_height: 16,
            map_type: MapType::Square,
            draw_chunks: true,
            draw_points: false,
            draw_entrances: false,
            draw_cached_paths: false,
        }
    }
}

#[derive(Component)]
pub struct DebugPath {
    pub tile_width: u32,
    pub tile_height: u32,
    pub map_type: MapType,
    pub color: Color,
}

impl Default for DebugPath {
    fn default() -> Self {
        DebugPath {
            tile_width: 16,
            tile_height: 16,
            map_type: MapType::Square,
            color: bevy::prelude::Color::Srgba(css::RED),
        }
    }
}

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
        //app.add_systems(Update, (draw_debug_grid::<N>, draw_debug_paths::<N>, draw_goals::<N>));
        app
            .add_systems(
                Update, 
                (
                    draw_debug_map::<N>,
                    draw_debug_paths::<N>
                )
            )
            .register_type::<DebugMap>()
            .register_type::<MapType>();
    }
}

fn draw_debug_map<N: Neighborhood + 'static>(
    query: Query<(&Transform, &DebugMap)>,
    grid: Option<Res<Grid<N>>>,
    mut gizmos: Gizmos,
) {
    let grid = match grid {
        Some(grid) => grid,
        None => return,
    };

    for (transform, debug_map) in query.iter() {
        //let offset = transform.translation.truncate();
        let offset = transform.translation.truncate();

        let center_offset = match debug_map.map_type {
            MapType::Square => offset + Vec2::new(debug_map.tile_width as f32 * 0.5, debug_map.tile_height as f32 * 0.5),
            MapType::Isometric => offset + Vec2::new(debug_map.tile_width as f32 * 0.5, 0.0),
        };

        if debug_map.draw_chunks {
            // Draw chunk boundaries
            let chunk_size = grid.get_chunk_size();
            let chunk_width = grid.get_width() / chunk_size;
            let chunk_height = grid.get_height() / chunk_size;

            match debug_map.map_type {
                MapType::Square => {
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
    
                            gizmos.line_2d(bottom_left + offset, bottom_right + offset, css::WHITE);
                            gizmos.line_2d(bottom_right + offset, top_right + offset, css::WHITE);
                            gizmos.line_2d(top_right + offset, top_left + offset, css::WHITE);
                            gizmos.line_2d(top_left + offset, bottom_left + offset, css::WHITE);
                        }
                    }  
                }
                MapType::Isometric => {
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
            for x in 0..grid.get_width() {
                for y in 0..grid.get_height() {
                    let point = grid.get_point(UVec3::new(x, y, 0));
                    let color = if point.wall { css::RED } else { css::WHITE };
    
                    let position = match debug_map.map_type {
                        MapType::Square => Vec2::new(
                            (x * debug_map.tile_width) as f32,
                            (y * debug_map.tile_height) as f32,
                        ),
                        MapType::Isometric => Vec2::new(
                            (y as f32 + x as f32) * (debug_map.tile_width as f32 * 0.5),
                            (y as f32 - x as f32) * (debug_map.tile_height as f32 * 0.5),
                        ),
                    };
    
                    gizmos.circle_2d(position + center_offset, 2.0, color);
                }
            }
        }
    
        if debug_map.draw_entrances {
            // Draw graph nodes
            for node in grid.graph.get_nodes() {
                let position = match debug_map.map_type {
                    MapType::Square => Vec2::new(
                        (node.pos.x * debug_map.tile_width) as f32,
                        (node.pos.y * debug_map.tile_height) as f32,
                    ),
                    MapType::Isometric => Vec2::new(
                        (node.pos.y as f32 + node.pos.x as f32) * (debug_map.tile_width as f32 * 0.5),
                        (node.pos.y as f32 - node.pos.x as f32) * (debug_map.tile_height as f32 * 0.5),
                    ),
                };
    
                gizmos.circle_2d(position + center_offset, 2.0, css::MAGENTA);
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
            for path in grid.graph.get_all_paths() {
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
                        MapType::Square => Vec2::new(
                            (prev.x * debug_map.tile_width) as f32,
                            (prev.y * debug_map.tile_height) as f32,
                        ),
                        MapType::Isometric => Vec2::new(
                            (prev.y as f32 + prev.x as f32) * (debug_map.tile_width as f32 * 0.5),
                            (prev.y as f32 - prev.x as f32) * (debug_map.tile_height as f32 * 0.5),
                        ),
                    };
    
                    let next_position = match debug_map.map_type {
                        MapType::Square => Vec2::new(
                            (next.x * debug_map.tile_width) as f32,
                            (next.y * debug_map.tile_height) as f32,
                        ),
                        MapType::Isometric => Vec2::new(
                            (next.y as f32 + next.x as f32) * (debug_map.tile_width as f32 * 0.5),
                            (next.y as f32 - next.x as f32) * (debug_map.tile_height as f32 * 0.5),
                        ),
                    };
    
                    let mut color = css::BLUE;
    
                    if debug_map.draw_entrances && debug_map.draw_cached_paths {
                        color = path_colors[color_index % path_colors.len()];
                    }
    
                    gizmos.line_2d(prev_position + center_offset, next_position + center_offset, color);
    
                    prev = next;
                }
    
                color_index += 1;
            }
        }
    }
}

fn draw_debug_paths<N: Neighborhood + 'static>(
    query: Query<(&DebugPath, &Path)>,
    mut gizmos: Gizmos,
) {
    for (debug_path, path) in query.iter() {
        if path.is_empty() {
            continue;
        }
        
        let center_offset = match debug_path.map_type {
            MapType::Square => Vec2::new(debug_path.tile_width as f32 * 0.5, debug_path.tile_height as f32 * 0.5),
            MapType::Isometric => Vec2::new(debug_path.tile_width as f32 * 0.5, 0.0),
        };

        // Iterate over path.path() drawing a line from one point to the next point until completed
        let mut iter = path.path().iter();
        let mut prev = iter.next().unwrap();

        for next in iter {
            let prev_position = match debug_path.map_type {
                MapType::Square => Vec2::new(
                    (prev.x * debug_path.tile_width) as f32,
                    (prev.y * debug_path.tile_height) as f32,
                ),
                MapType::Isometric => Vec2::new(
                    (prev.y as f32 + prev.x as f32) * (debug_path.tile_width as f32 * 0.5),
                    (prev.y as f32 - prev.x as f32) * (debug_path.tile_height as f32 * 0.5),
                ),
            };

            let next_position = match debug_path.map_type {
                MapType::Square => Vec2::new(
                    (next.x * debug_path.tile_width) as f32,
                    (next.y * debug_path.tile_height) as f32,
                ),
                MapType::Isometric => Vec2::new(
                    (next.y as f32 + next.x as f32) * (debug_path.tile_width as f32 * 0.5),
                    (next.y as f32 - next.x as f32) * (debug_path.tile_height as f32 * 0.5),
                ),
            };

            let x = prev_position + center_offset;
            let y = next_position + center_offset;

            gizmos.line_2d(
                x, y,
                debug_path.color
            );

            prev = next;
        }
    }
}
