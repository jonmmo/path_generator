# path_generator :ship:
Project work during TMR4510 Marine Cybernertics, Specialization project. Fall 2019, NTNU.

For a stepwise path-generation problem, the vessel starts at a departure waypoint and only knows the next target waypoint. During the voyage, the vessel knows all earlier and the next waypoint. When it approaches the next waypoint, say within a circle of acceptance, it will have some time to decide for the waypoint after that. The task of path-generation is then to generate a feasible, smooth path between the waypoints online. It has to be computationally efficient and robust.

The focus in this project has been the development of a stepwise path-generation algorithm, using Bezier curve as the basis function. Two methods have been proposed. First, a pragmatic solution, second, a solution involving optimization. To be used with a maneuvering control system, the methods are designed to generate a \mathcal{C}^3-continuous path. The algorithms are tunable with respect to the dynamical constraints imposed by the vessel.

An analysis was performed indicating a septic Bezier curve was the most suitable for a path generator to be used by a maneuvering controller. The results obtained show that it is possible to combine the Bezier curve and optimization to incorporate the dynamics of the vessel to produce the desired behavior for a stepwise path-generation problem.

## Dependencies

### MATLAB
Version:
- Built on 2019a

## Contributors
:star: Magnus Knaedal
