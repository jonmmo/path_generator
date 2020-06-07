# path_generator :ship:
For a stepwise path-generation problem, the vessel starts at a departure waypoint and only knows the next target waypoint. During the voyage, the vessel knows all earlier and the next waypoint. When it approaches the next waypoint, say within a circle of acceptance, it will have some time to decide for the waypoint after that. The task of path-generation is then to generate a feasible, smooth path between the waypoints online. It has to be computationally efficient and robust.

Two methods are implemented: a pragmatic solution and a solution involving optimization.

## Dependencies

### MATLAB
Version:
- Built on 2019a

## Contributors
:star: Magnus Knaedal

Part of master thesis. Spring 2020, NTNU.

