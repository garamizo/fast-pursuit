# Fast Pursuit

## Pursuit on 3D Space

A fleet of autonomous aerial vehicles protect a target from an intruder. This intruder tries to reach the target while evading the fleet of pursuers.

This code proposes to solve this problem using numerical solvers. To find the most likely interception point, the trajectory of the pursuers and evader are optimized jointly. From all the points in the feasible region - that the evader can reach before the pursuers (Figure 1, point cloud) - the evader should choose the point closest to the target (Figure 1, hottest regions) to increase its chance of success.

### Pursuer and evader have the same speed:

![alt text](images/free_space_convergence1.png "Interception point is the hot region")

In the optimization procedure, an initial interception guess (orange spheres) are iterated to decrease the path-distance to the target, while moving towards the feasible region.

### Evader is faster than pursuer:

![alt text](images/convergence_diff_speed.png "Drones have different analysis")

When the drones have different speeds, the feasible region becomes a curved surface.

### Fleet of pursuers vs 1 evader in obstructed map:

![alt text](images/convergence.png "Fleet on obstructed map")

When there are multiple pursuers or obstruction, the feasible region becomes complex.

### Benchmark 

On a ThinkPad W530 laptop (2.7 GHz CPU, 8GB RAM).

| Node Density [node/m] | Number of Nodes | Convergence Rate [%] | Execution Time [ms/solution] |
| --- | --- | --- | --- |
| 0.33 | 6k  | 70.01 ± 4.41  | 1.82 ± 0.07 | 
| 0.40  | 8.9k  | 70.49 ± 2.81  | 1.94 ± 0.12 | 
| 0.50  | 14k  | 57.17 ± 5.94 | 2.79 ± 0.11 | 
| 0.67  | 25k  | 58.97 ± 4.69  | 3.40 ± 0.17 | 
| 1.00  | 59k  | 46.76 ± 4.81  | 5.70 ± 0.28 | 
| 2.00  | 247k  | 87.86 ± 2.24  | 9.94 ± 0.40 | 
| 3.33  | 689k  | 77.56 ± 3.74  | 15.45 ± 0.35 | 

### How to use

#### Plotting results

```bash
roslaunch pursuit3d demo.launch
```

### Benchmark

```bash
rosrun pursuit3d benchmark 10.0 1.0 2.0 -0.3
```