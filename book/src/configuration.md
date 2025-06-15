# Configuration Guide

## GridSettings

`width`: Set this to tile count width of your tilemap.

`height`: The tile count height of your tilemap.

`depth`: Set to 1 for 2d tilemaps. If using a 3d tilemap set this to the height of your tilemap.

`chunk_size`: How many chunks to divide the tilemap into for HPA*. You'll want to find a good middle ground for this value depending on the size of your map. Larger chunks will assist with performance but provide less optimal paths, smaller chunks will give more optimal paths while hitting performance.

`chunk_ordinal`: If set to true, entrances will be added to the corners of chunks. This will make `Grid::build` take a significantly longer for large maps. In most cases this isn't necessary, but may be considered if your map is noisy with a lot of corners and you allow diagonal movement.

For example a map where wall corners meet often with a diagonal gap.

|||
|---|---|
|x|o|
|o|x|

`default_cost`: This sets default cost for every point of the grid. Set to 0 or 1 for cheap movement. Set to 255 if you want the cost to be the highest.

`default_wall`: Set to true if you want every point in the grid to a be a wall by default. 

`collision`: Set to true to allow the plugins pathfinding systems to ensure entities aren't pathing through each other.

`avoidance_distance`: The plugin uses local collision avoidance mainly for performance purposes. This sets the lookahead distance in grid points to check for any possible blocking entity in it's path.

