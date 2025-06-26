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