# Offline Trajectory Tool

This tool enables you to:

1. Create, edit, and save spline-based trajectories and export to target trajectory line (TTL).
2. Run a simulation to fill the optimal speeds along a TTL.
3. :construction: Run an optimization to modify the trajectory for shorter lap time.

## Install

1. Clone this repository
2. `cd offline-trajectory-tools`
3. `pip install -e .`

Note: this will install in experimental mode. Any changes done to the local repository will be reflected in the package.

## Getting Started

1. Create a new directory as your trajectory workspace.
2. In a terminal, change to the new directory, and execute `trajectory_init`.
3. Edit the configuration and regions file as needed.

### Prepare References

To start using the tool, you first need some references:

- Some reference trajectory or track boundary files in CSV format where the first two columns are the X and Y positions. It could be a TTL or a simple trajectory file in local cartesian coordinate. It does not need to be in high detail, but should be accurate. It will be displayed in the background for your own convenience when drawing your new trajectory.
- Some regions in a YAML file with the name, code, and vertices. Refer to `regions.yaml` on how to format it. They are used to fill the region field of the exported TTL and will be displayed in the background. It is good practice to slightly overlap the regions as to not leave out waypoints when the trajectory transitions to a different region. The waypoints outside of any region will be assigned with region code 0.

### Create New Trajectory

1. Execute `trajectory_edit`.
2. Click on "Display Reference TTL" and "Display Reference Regions". You can display multiple TTLs in multiple files, but only one region file should be selected. Verify the lines and regions. If you don't provide any reference, the default graphing area will be limited to a 1 by 1 square, although you can use the Matplotlib figure tools on the bottom-left corner later to resize the graph.
3. Enter the number of nodes for the new trajectory. A trajectory is made up of multiple cubic bezier curves with $C^1$ continuity. Every node is the end of the last bezier curve and the beginning of a new one. If unsure how many nodes to use, reserve two for each turn, and use more for turns with changing curvatures.
4. Click on "New" and the main editing window will appear, with the nodes in a circle. drag the nodes to fit the trajectory. For each turn try putting one node at the entry and one at the exit. Try moving the two handles on the node (which are the Beizer curve control points) and see how the curve changes.
5. Use the Matplotlib figure tools on the bottom-left corner to adjust the zoom and pan. Click on the buttons again to resume editing. Click on the home button to go back to the initial view.
6. Click on "Lock Heading" to fine tune the curve without changing the yaw at the node. Click on "Hide Vertices" to pause editing and preview the trajectory.
7. When done, click on "Save" to save the trajectory which preserves the Bezier curve information and can be loaded again.
8. Click on "Export" to export the trajectory to TTL with curvature, distance and region information populated. You will be prompted to enter the TTL number and the sampling interval of each waypoint. Note that the sampling interval may not be exact.

### Load and Edit Existing Trajectory

1. Execute `trajectory_edit`.
2. Open the reference TTL and region files.
3. Click on "Open Saved" and load the saved trajectory.
4. Edit, save, and export the trajectory.

### Run Speed Optimization

1. Modify `simulation_config.yaml` to reflect vehicle dynamics, and specify input and output TTLs.
2. Execute `trajectory_sim`.