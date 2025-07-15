use ndarray::{Array2, ArrayView2};

use crate::nav::NavCell;

#[allow(dead_code)]
pub(crate) fn flood_fill_connected_regions(face: ArrayView2<NavCell>) -> Vec<Vec<(usize, usize)>> {
    let mut visited = Array2::from_elem(face.raw_dim(), false);
    let mut groups = Vec::new();

    for ((x, y), cell) in face.indexed_iter() {
        if visited[(x, y)] || cell.is_impassable() {
            continue;
        }

        let mut group = Vec::new();
        let mut stack = vec![(x, y)];

        while let Some((cx, cy)) = stack.pop() {
            if cx >= face.shape()[0] || cy >= face.shape()[1] {
                continue;
            }

            if visited[(cx, cy)] || face[(cx, cy)].is_impassable() {
                continue;
            }

            visited[(cx, cy)] = true;
            group.push((cx, cy));

            for dx in -1..=1 {
                for dy in -1..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }

                    let nx = cx as isize + dx;
                    let ny = cy as isize + dy;

                    if nx >= 0
                        && ny >= 0
                        && (nx as usize) < face.shape()[0]
                        && (ny as usize) < face.shape()[1]
                    {
                        stack.push((nx as usize, ny as usize));
                    }
                }
            }
        }

        if !group.is_empty() {
            groups.push(group);
        }
    }

    groups
}

pub(crate) fn flood_fill_bool_mask(mask: ArrayView2<bool>) -> Vec<Vec<(usize, usize)>> {
    let mut visited = Array2::from_elem(mask.raw_dim(), false);
    let mut groups = Vec::new();

    for ((x, y), &passable) in mask.indexed_iter() {
        if visited[(x, y)] || !passable {
            continue;
        }

        let mut group = Vec::new();
        let mut stack = vec![(x, y)];

        while let Some((cx, cy)) = stack.pop() {
            if cx >= mask.shape()[0] || cy >= mask.shape()[1] {
                continue;
            }

            if visited[(cx, cy)] || !mask[(cx, cy)] {
                continue;
            }

            visited[(cx, cy)] = true;
            group.push((cx, cy));

            for dx in -1..=1 {
                for dy in -1..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }

                    let nx = cx as isize + dx;
                    let ny = cy as isize + dy;

                    if nx >= 0
                        && ny >= 0
                        && (nx as usize) < mask.shape()[0]
                        && (ny as usize) < mask.shape()[1]
                    {
                        stack.push((nx as usize, ny as usize));
                    }
                }
            }
        }

        if !group.is_empty() {
            groups.push(group);
        }
    }

    groups
}
