# Motion-Planning-in-CSpace

## Overview:
This project focused on Motion Planning in CSpace, employing MATLAB and Peter Corke’s Robotics Toolbox to address collision avoidance and navigate robots through a 2-D workspace. The assignment spanned from understanding configuration spaces to optimizing trajectories and implementing collision-free paths.

### C0: Configuration Space Basics
* Defined a 2-D configuration space for two square robots moving along fixed tracks.
* Specified axes, axis limits, and configurations in the configuration space.

<img width="489" alt="image" src="https://github.com/PanchalM19/Motion-Planning-in-CSpace/assets/115374409/565bcc61-de0b-405f-8c38-2f4e0da37038">

### C1: Robot Visualization
* Plotted robot configurations in the workspace at specified angles.
* Utilized MATLAB's polyshape class for polygon representation.

### C2: Configuration Space Discretization
* Converted the problem into configuration space by discretizing it.
* Checked for collisions at each discrete grid point by intersecting polygons.

### C3-C5: Path Planning
* Computed distance transform from the nearest grid point to the goal.
* Generated a path from the start configuration to the goal using the distance transform.
* Converted the path from grid point indices to configurations and visualized the trajectory.

<img width="242" alt="image" src="https://github.com/PanchalM19/Motion-Planning-in-CSpace/assets/115374409/4cc9bbe1-b566-4689-9932-71e66e29f45d">

### C6: Swept-Volume Collisions
* Detected and visualized swept-volume collisions along the path.
* Utilized convex hulls of robot links’ 2-D polygons for collision checking.

### C7: Conservative Trajectory
* Avoided close collisions by padding obstacles in configuration space.
* Verified the resulting trajectory for collision-free swept volumes.

<img width="237" alt="image" src="https://github.com/PanchalM19/Motion-Planning-in-CSpace/assets/115374409/4dff6f11-b39f-4575-839c-b4fdbc87f96c">

