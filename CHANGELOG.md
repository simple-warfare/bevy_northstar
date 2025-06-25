## v0.2.1
**Fixes**
* Fixed a system query issue that was pulling in all entities with `GridPos` into the collision `BlockingMap` instead of only entities with `Blocking`. Credit [#11 @AwfulToTheEar](https://github.com/JtotheThree/bevy_northstar/pull/11).
* Added bounds checking to `pathfind`. If the start or goal isn't in the grid bounds it returns `None` for the path and logs an error.

**Features**
* Added `Grid::is_path_viable`. Returns `true` if there's a path possible from the start to the goal while skipping the refinement that the standard `pathfind` functions runs.

## v0.2.0
First official public release