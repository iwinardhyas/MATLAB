### 1. 3D Trajectory Visualization

![image](https://github.com/iwinardhyas/MATLAB/blob/main/drone3/img/1.png)

Explanation:

The figure shows the drone’s 3D trajectory (blue line) starting from the initial position (green text on the right) at (5, 5, 2) meters.

The drone moves smoothly towards the target at the origin (0, 0, 0), marked with red text on the left.

The labels display:

Start state: position (5, 5, 2) m, initial velocity [-1, 0, 0.1] m/s.

End state: position (0, 0, 0) m, final velocity approximately [-1, 0, 0] m/s.

Total time: ~46.98 seconds to complete the maneuver.

The path is curved and smooth, which indicates that MPC is actively optimizing control inputs to ensure stable drone motion under constraints (acceleration, velocity, etc.).

Importantly, the drone does not overshoot or oscillate, showing good stability of the MPC controller.

### Insight: This visualization demonstrates that the MPC successfully guides the drone from start to target while ensuring smooth and dynamically feasible motion.

### 2. Position Tracking Over Time

![image](https://github.com/iwinardhyas/MATLAB/blob/main/drone3/img/2.png)

Explanation:

This plot shows the time response of X, Y, and Z positions compared to their respective targets.

Blue Line (X Position): Moves from 5 m → 0 m over time, converging smoothly.

Green Line (Y Position): Starts at 5 m → 0 m, converges exponentially to the target.

Red Line (Z Position): Starts at 2 m → rises to ~5 m, then decreases to 0 m as the final goal.

Dashed lines represent the target trajectories, while solid lines are the actual drone states.

### Insight:

The tracking error is minimal: actual positions follow the desired trajectories closely.

The smooth convergence indicates that MPC manages state constraints and optimizes future control inputs effectively.

The Z-axis profile shows MPC’s ability to handle altitude control, even when changes are non-monotonic (climb then descend).

### Overall Interpretation

MPC ensures smooth, feasible trajectories by solving optimization problems at each timestep.

Both 3D trajectory and position tracking confirm:

Minimal tracking error.

Stable convergence to targets.

Feasible drone dynamics without abrupt oscillations.

This proves that MPC is suitable for drone trajectory tracking and stabilization tasks in real-time.
## Authors

- [@iwinardhyas](https://www.github.com/iwinardhyas)

erwin.ardias@gmail.com