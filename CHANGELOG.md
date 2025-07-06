## v0.3.1
### Features
* The grid can now be updated during gameplay. After calling `Grid::set_nav()` to change navigation at a position just call `Grid::build()` again. 
* `Grid::build()` now rebuilds only areas affected by `set_nav()` when calling it after the initial build.
* `Grid::build()` now processes chunks in parallel! Large performance boost for the initial build and allows rebuilding larger areas of the grid in a single frame.
* `parallel` feature has been added and is enabled by default. You can disable default features if targetting WASM and multithreading gives you issue.
* Right-click a tile in the basic example to change the navigation data and dynamically rebuild the grid.

## v0.3.0
**BREAKING CHANGES**
`GridSettings` has been replaced with a builder `GridSettingsBuilder`.
`GridPos` component has been renamed to `AgentPos`.
`Point` has been removed and fully reworked as `Nav` and `NavCell`.
`Pathfind` component has been reworked with a fluent/chain constructor pattern.
`DebugMap` component has been renamed to `DebugGrid` and reworked.
`DebugMapType` has been renamed to `DebugTilemapType`.
`DebugPath` has removed tile_width, tile_height, and map_type.

See the [migration guide](https://jtothethree.github.io/bevy_northstar/migrations/001_v0.3.0.html) for more details on how to migrate.

### Features
* Updated the basic example to demonstrate more realistic usage of the crate.
* `NorthstarPlugin` systems now stagger agent processing based on the `NorthstarPluginSettings` configuration.
* Added the `NorthstarPluginSettings` resource to configure how many agents can be processed per frame by the pathfinding systems.
* Added the `NeighborFilter` trait. Filters can now be applied to neighbor generation—for example, to prevent corner clipping. Multiple filters can be chained.
* Added `NoCornerClipping`, `NoCornerCutting` filters which customize diagonal movement around and/or through wall corners.
* Neighbors are now precomputed as bitmasks and cached during `Grid::build()`. This provides a general performance boost and a significant improvement when using neighbor filters.
* Added `PathfindMode::Coarse`, which returns a hierarchical path without refinement. Refinement is the step that optimizes a path for shortest distance
* `AgentOfGrid` relationship has been created to relate an entity with a specific `Grid`. This will allow supporting multiple grids in the future.
* HPA* path refinement has been optimized for performance. A* performance has been further optimized.
* Optimized the collision avoidance system by limiting checks to a localized search area, reducing stutters from full A* searches.

### Debug Features
* `DebugPath` now uses the `AgentOfGrid` relationship and child relationships making it cleaner to setup.
* Added `DebugGridBuilder` to simplify the creation and configuration of `DebugGrid`.

## v0.2.1
**Fixes**
* `Plugin::update_blocking_map` system now properly queries `Blocking` (previously was including all `GridPos` entities as blockers for collision). Credit [#11 @AwfulToTheEar](https://github.com/JtotheThree/bevy_northstar/pull/11).
* `pathfind` now checks grid bounds; logs an error and return `None` if out of bounds.
* Fixed repeated pathfind attempts due to unintended component reinsertion in `NorthstarPlugin`. Use `PathfindingFailed` to handle failures gracefully.
* Fixed a spawning issue in the examples where entities were not spawning with transforms adjusted for the bevy_ecs_tilemap offset.

**Features**
* Added `Grid::is_path_viable`. Returns `true` if there's a path possible from the start to the goal while skipping the refinement step that the standard `pathfind` functions runs. This should be a more performant call if you just need to test if path exists.
* Added `BlockingMap` to `prelude`

## v0.2.0
First official public release