//! Raycasting and pathfinding utilities for 2D/3D grids.
use bevy::math::UVec3;
use ndarray::ArrayView3;

use crate::grid::Point;

/// Check if there is a line of sight between two points in the grid allowing diagonal movement.
///
/// Arguments:
/// * `grid` - A 3D array view of [`Point`]s representing the grid.
/// * `start` - The starting [`UVec3`].
/// * `end` - The ending point [`UVec3`].
///
/// Returns:
/// * `true` if there is a line of sight between the two points.
/// * `false` if there is an obstacle between the two points.
///
/// # Example
///
/// ```
/// use bevy::math::UVec3;
/// use ndarray::Array3;
/// use bevy_northstar::grid::Point;
/// use bevy_northstar::raycast::line_of_sight;
///
/// let mut grid = Array3::from_elem((10, 10, 1), Point::new(1, false));
/// grid[[5, 5, 0]] = Point::new(1, true);
///
/// let start = UVec3::new(0, 0, 0);
/// let end = UVec3::new(9, 9, 0);
///
/// assert_eq!(line_of_sight(&grid.view(), start, end), false);
/// ```
pub fn line_of_sight(grid: &ArrayView3<Point>, start: UVec3, end: UVec3) -> bool {
    // TDDO: This can be optimized using integers
    let start = start.as_vec3();
    let end = end.as_vec3();

    let mut x = start.x;
    let mut y = start.y;
    let mut z = start.z;

    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let dz = end.z - start.z;

    let steps = dx.abs().max(dy.abs()).max(dz.abs());
    if steps == 0.0 {
        return true;
    }

    let x_step = dx / steps;
    let y_step = dy / steps;
    let z_step = dz / steps;

    for _ in 0..=steps as i32 {
        let grid_x = x.round() as usize;
        let grid_y = y.round() as usize;
        let grid_z = z.round() as usize;

        if grid[[grid_x, grid_y, grid_z]].solid {
            return false; // Hit an obstacle
        }

        x += x_step;
        y += y_step;
        z += z_step;
    }

    true
}

// Generates the positions of a path segment between two points in a 3D grid.
pub(crate) fn generate_path_segment_ordinal(start: UVec3, end: UVec3) -> Vec<UVec3> {
    // TODO: This can be optimized using integers I believe
    let mut segment = Vec::new();

    let mut current = start;
    segment.push(current);

    let dx = (end.x as isize - start.x as isize).abs();
    let dy = (end.y as isize - start.y as isize).abs();
    let dz = (end.z as isize - start.z as isize).abs();

    let steps = dx.max(dy).max(dz); // Longest axis determines the number of steps

    if steps == 0 {
        return segment; // Start and end are the same
    }

    let step_x = (end.x as isize - start.x as isize) as f32 / steps as f32;
    let step_y = (end.y as isize - start.y as isize) as f32 / steps as f32;
    let step_z = (end.z as isize - start.z as isize) as f32 / steps as f32;

    let mut x = start.x as f32;
    let mut y = start.y as f32;
    let mut z = start.z as f32;

    for _ in 0..steps {
        x += step_x;
        y += step_y;
        z += step_z;

        let next = UVec3 {
            x: x.round() as u32,
            y: y.round() as u32,
            z: z.round() as u32,
        };

        if next != current {
            segment.push(next);
            current = next;
        }
    }

    segment
}

// Generates the positions of a path segment between two points in a 3D grid using cardinal movement.
pub(crate) fn generate_path_segment_cardinal(start: UVec3, end: UVec3) -> Vec<UVec3> {
    let mut segment = Vec::new();
    segment.push(start);

    let mut pos = start.as_ivec3();
    let end = end.as_ivec3();

    while (pos.x, pos.y, pos.z) != (end.x, end.y, end.z) {
        if pos.x != end.x {
            pos.x += (end.x - pos.x).signum();
        } else if pos.y != end.y {
            pos.y += (end.y - pos.y).signum();
        } else if pos.z != end.z {
            pos.z += (end.z - pos.z).signum();
        }

        let next = UVec3 {
            x: pos.x as u32,
            y: pos.y as u32,
            z: pos.z as u32,
        };
        segment.push(next);
    }

    segment
}

// Trace a line from start to goal and get the Bresenham path only if the path doesn't collide with a wall
// This should take into account the Neighborhood and the grid
pub(crate) fn bresenham_path(
    grid: &ArrayView3<Point>,
    start: UVec3,
    goal: UVec3,
    ordinal: bool,
    allow_corner_clipping: bool,
) -> Option<Vec<UVec3>> {
    let mut path = Vec::new();
    let mut current = start;

    let (width, height, depth) = (
        grid.shape()[0] as u32,
        grid.shape()[1] as u32,
        grid.shape()[2] as u32,
    );

    // Differences in each dimension
    let dx = (goal.x as i32 - start.x as i32).abs();
    let dy = (goal.y as i32 - start.y as i32).abs();
    let dz = if depth > 1 {
        (goal.z as i32 - start.z as i32).abs()
    } else {
        0 // Ignore z axis if grid depth is 1
    };

    let sx: i32 = if start.x < goal.x { 1 } else { -1 };
    let sy: i32 = if start.y < goal.y { 1 } else { -1 };
    let sz: i32 = if depth > 1 && start.z < goal.z { 1 } else { -1 };

    let mut err_xy = dx - dy;
    let mut err_xz = dx - dz;

    while current != goal {
        // Bounds check
        if current.x >= width || current.y >= height || current.z >= depth {
            return None;
        }

        path.push(current);

        if grid[[current.x as usize, current.y as usize, current.z as usize]].solid {
            return None;
        }

        /*if blocking.contains_key(&current) {
            log::info!("Bresenham path blocked by entity: {:?}, {:?}", blocking[&current], blocking);
            return None;
        }*/

        // Error-based stepping
        let double_err_xy = 2 * err_xy;
        let double_err_xz = 2 * err_xz;

        if ordinal {
            /*if !allow_corner_clipping {
                // Determine the tentative next cell (but do NOT update current yet)
                let mut next = current;

                if double_err_xy >= -dy && double_err_xz >= -dz {
                    next.x = next.x.saturating_add_signed(sx);
                }
                if double_err_xy < dx {
                    next.y = next.y.saturating_add_signed(sy);
                }
                if depth > 1 && double_err_xz < dx {
                    next.z = next.z.saturating_add_signed(sz);
                }

                let delta = next.as_ivec3() - current.as_ivec3();

                // Check 2D XY corner clipping
                if delta.x != 0 && delta.y != 0 {
                    let step_x = UVec3::new((current.x as i32 + delta.x) as u32, current.y, current.z);
                    let step_y = UVec3::new(current.x, (current.y as i32 + delta.y) as u32, current.z);
                    if grid[[step_x.x as usize, step_x.y as usize, step_x.z as usize]].solid ||
                       grid[[step_y.x as usize, step_y.y as usize, step_y.z as usize]].solid {
                        return None;
                    }
                }

                // Check XZ
                if depth > 1 && delta.x != 0 && delta.z != 0 {
                    let step_x = UVec3::new((current.x as i32 + delta.x) as u32, current.y, current.z);
                    let step_z = UVec3::new(current.x, current.y, (current.z as i32 + delta.z) as u32);
                    
                    if grid[[step_x.x as usize, step_x.y as usize, step_x.z as usize]].solid ||
                       grid[[step_z.x as usize, step_z.y as usize, step_z.z as usize]].solid {
                        return None;
                    }
                }

                // Check YZ
                if depth > 1 && delta.y != 0 && delta.z != 0 {
                    let step_y = UVec3::new(current.x, (current.y as i32 + delta.y) as u32, current.z);
                    let step_z = UVec3::new(current.x, current.y, (current.z as i32 + delta.z) as u32);

                    if grid[[step_y.x as usize, step_y.y as usize, step_y.z as usize]].solid ||
                       grid[[step_z.x as usize, step_z.y as usize, step_z.z as usize]].solid {
                        return None;
                    }
                }

                // Optional: XYZ diagonal clipping (for full 3D corner case)
                if depth > 1 && delta.x != 0 && delta.y != 0 && delta.z != 0 {
                    let xy = UVec3::new((current.x as i32 + delta.x) as u32, (current.y as i32 + delta.y) as u32, current.z);
                    let xz = UVec3::new((current.x as i32 + delta.x) as u32, current.y, (current.z as i32 + delta.z) as u32);
                    let yz = UVec3::new(current.x, (current.y as i32 + delta.y) as u32, (current.z as i32 + delta.z) as u32);
                    
                    if grid[[xy.x as usize, xy.y as usize, xy.z as usize]].solid ||
                       grid[[xz.x as usize, xz.y as usize, xz.z as usize]].solid ||
                       grid[[yz.x as usize, yz.y as usize, yz.z as usize]].solid {
                        return None;
                    }
                }
            }*/

            if double_err_xy >= -dy && double_err_xz >= -dz {
                // Move along x-axis
                err_xy -= dy;
                err_xz -= dz;
                current.x = current.x.saturating_add_signed(sx);
            }
            if double_err_xy < dx {
                // Move along y-axis
                err_xy += dx;
                current.y = current.y.saturating_add_signed(sy);
            }
            if depth > 1 && double_err_xz < dx {
                // Move along z-axis (if applicable)
                err_xz += dx;
                current.z = current.z.saturating_add_signed(sz);
            }
        } else if double_err_xy >= -dy && double_err_xz >= -dz {
            // Move along x-axis
            err_xy -= dy;
            err_xz -= dz;
            current.x = current.x.saturating_add_signed(sx);
        } else if double_err_xy < dx {
            // Move along y-axis
            err_xy += dx;
            current.y = current.y.saturating_add_signed(sy);
        } else if depth > 1 && double_err_xz < dx {
            // Move along z-axis (if applicable)
            err_xz += dx;
            current.z = current.z.saturating_add_signed(sz);
        }
    }

    path.push(goal);

    Some(path)
}

/*pub fn line_of_sight(grid: &ArrayView3<Point>, start: UVec3, end: UVec3) -> bool {
    let (x0, y0, z0) = (start.x as isize, start.y as isize, start.z as isize);
    let (x1, y1, z1) = (end.x as isize, end.y as isize, end.z as isize);

    let dx = (x1 - x0).abs();
    let dy = (y1 - y0).abs();
    let dz = (z1 - z0).abs();

    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let sz = if z0 < z1 { 1 } else { -1 };

    let mut x = x0;
    let mut y = y0;
    let mut z = z0;

    let mut err_xy = dx - dy;
    let mut err_xz = dx - dz;

    loop {
        // Check bounds and wall
        if x < 0
            || y < 0
            || z < 0
            || x >= grid.shape()[0] as isize
            || y >= grid.shape()[1] as isize
            || z >= grid.shape()[2] as isize
        {
            return false; // Out of bounds
        }
        if grid[(x as usize, y as usize, z as usize)].wall {
            return false; // Wall blocks line of sight
        }
        if x == x1 && y == y1 && z == z1 {
            return true; // Reached the target
        }

        // Update error terms and move in the direction of the ray
        let e2_xy = 2 * err_xy;
        let e2_xz = 2 * err_xz;

        if e2_xy > -dy {
            err_xy -= dy;
            x += sx;
        }
        if e2_xy < dx {
            err_xy += dx;
            y += sy;
        }
        if e2_xz > -dz {
            err_xz -= dz;
            x += sx;
        }
        if e2_xz < dx {
            err_xz += dx;
            z += sz;
        }
    }
}*/

#[cfg(test)]
mod tests {
    use bevy::math::UVec3;
    use ndarray::Array3;

    use crate::{
        grid::{
            ChunkSettings, CollisionSettings, CostSettings, GridInternalSettings, GridSettings,
            NeighborhoodSettings,
        },
        prelude::*,
        raycast::{bresenham_path, line_of_sight},
    };

    const GRID_SETTINGS: GridSettings = GridSettings(GridInternalSettings {
        dimensions: UVec3::new(12, 12, 1),
        chunk_settings: ChunkSettings {
            size: 4,
            depth: 1,
            diagonal_connections: false,
        },
        cost_settings: CostSettings {
            default_cost: 1,
            default_solid: false,
        },
        collision_settings: CollisionSettings {
            enabled: true,
            avoidance_distance: 4,
        },
        neighborhood_settings: NeighborhoodSettings {
            allow_corner_clipping: true,
            smooth_corner_paths: false,
        },
    });

    #[test]
    fn test_line_of_sight() {
        let mut grid = Array3::from_elem((10, 10, 1), Point::new(1, false));
        grid[[5, 5, 0]] = Point::new(1, true);

        let start = UVec3::new(0, 0, 0);
        let end = UVec3::new(9, 9, 0);

        assert!(!line_of_sight(&grid.view(), start, end));
    }

    #[test]
    fn test_bresenhan_path() {
        let mut grid: Grid<OrdinalNeighborhood> = Grid::new(&GRID_SETTINGS);

        grid.build();

        let path = bresenham_path(
            &grid.view(),
            UVec3::new(0, 0, 0),
            UVec3::new(10, 10, 0),
            grid.neighborhood.is_ordinal(),
            false,
        );

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 11);

        let mut grid: Grid<CardinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.build();

        let path = bresenham_path(
            &grid.view(),
            UVec3::new(0, 0, 0),
            UVec3::new(10, 10, 0),
            grid.neighborhood.is_ordinal(),
            false,
        );

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 21);

        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.set_point(UVec3::new(5, 5, 0), Point::new(1, true));
        grid.build();

        let path = bresenham_path(
            &grid.view(),
            UVec3::new(0, 0, 0),
            UVec3::new(10, 10, 0),
            grid.neighborhood.is_ordinal(),
            false,
        );

        assert!(path.is_none());
    }
}
