# Grid Updates
You can modify the navigation data of individual cells at runtime using `Grid::set_nav()`. 
After making changes, you **must** call `Grid::build()` to update the internal state.

When `Grid::set_nav()` is called, the affected chunk and its adjacent chunks are automatically marked as dirty. During the next `Grid::build()` call, only those dirty chunks are rebuilt. If the default parallel feature is enabled, chunk rebuilds are performed in parallel for better performance.

Calling any of the `Grid` pathfinding methods on a dirty grid will log an error and return `None`.

Example:
```rust,no-run
if input.just_pressed(MouseButton::Right) {
    if let Some(position) = clicked_tile {
        let mut grid = grid.into_inner();

        if let Some(nav) = grid.nav(position) {
            if !matches!(nav, Nav::Impassable) {
                // If the cell is passable, we set it to impassable.
                grid.set_nav(position, Nav::Impassable);
            } else {
                // If the cell is impassable, we set it to passable with a cost of 1.
                grid.set_nav(position, Nav::Passable(1));
            }
        } else {
            return;
        }
        // You must call `build` after modifying the grid to update the internal state.
        grid.build();
    }
}
```

## Performance Notes
Rebuilding a single chunk takes approximately **0.2ms** on modern systems. Note that updating a cell in a single chunk may require updating neighboring chunks if it touches an edge. If you enable `GridSettingsBuilder::diagonal_connections()` or use an ordinal `Neighborhood`, the number of adjacent chunks needing rebuilds may increase.

* With the default `parallel` feature enabled, multiple dirty chunks can often be rebuilt in the same frame with a ~35% speedup.
* If `parallel` is disabled (e.g. for WASM), rebuilding a large number of chunks sequentially may exceed your frame budget. In that case, consider:

    * Limiting updates to small, localized areas per frame.

    * Spreading updates across multiple frames.

    * Adjusting chunk size to find the best performance fo your use case.
