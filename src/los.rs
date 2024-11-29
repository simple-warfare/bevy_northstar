use bevy::math::UVec3;
use ndarray::ArrayView3;

use crate::Point;

/// Check if there is a line of sight between two points in a 3D grid.
/// - `grid`: A 3D ndarray grid with a shape (x, y, z), where each cell is `true` if it contains a wall, else `false`.
/// - `start`: (x, y, z) coordinates of the starting point.
/// - `end`: (x, y, z) coordinates of the target point.
/// 
/// Returns `true` if there is an unobstructed line of sight, `false` otherwise.
pub fn line_of_sight(grid: &ArrayView3<Point>, start: UVec3, end: UVec3) -> bool {
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
        if x < 0 || y < 0 || z < 0 || x >= grid.shape()[0] as isize || y >= grid.shape()[1] as isize || z >= grid.shape()[2] as isize {
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
}
