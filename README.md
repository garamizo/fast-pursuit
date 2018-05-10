# Fast Pursuit

## Pursuit on 3D Space

A fleet of autonomous aerial vehicles protect a target from an intruder. This intruder tries to reach the target while evading the fleet of pursuers.

This code proposes to solve this problem using numerical solvers. To find the most likely interception point, the trajectory of the pursuers and evader are optimized jointly. From all the points in the feasable region - that the evader can reach before the pursuers (Figure 1, point cloud) - the evader should choose the point closest to the target (Figure 1, hottest regions) to increase its chance of success.

### Pursuer and evader have the same speed:

![alt text](images/free_space_convergence1.png "Interception point is the hot region")

In the optimization procedure, an initial interception guess (orange spheres) are iterated to decrease the path-distance to the target, while moving towards the feasable region.

### Evader is faster than pursuer:

![alt text](images/convergence_diff_speed.png "Drones have different analysis")

The 

### Fleet of pursuers vs 1 evader in obstructed map:

![alt text](images/convergence.png "Fleet on obstructed map")

### Benchmark 