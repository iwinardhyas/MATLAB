### 1. 3D Trajectory Visualization

![App Screenshot](https://via.placeholder.com/468x300?text=App+Screenshot+Here)


# Results of NMPC Drone Control with ODE Solver and Wind Disturbance

## 1. **3D Trajectory Tracking**

![3D Trajectory](Screenshot%202025-08-23%20120750.png)

* **Description**:
  This plot shows the **3D path of the quadrotor** under NMPC control.

  * **Blue solid line** → Actual trajectory (affected by system dynamics + wind).
  * **Red dashed line** → Reference trajectory (ideal path).
  * **Green dot** → Start position.
  * **Red dot** → Final position.

* **Observations**:

  * The quadrotor follows the reference trajectory well, but some **tracking error** exists due to **wind noise** and nonlinear dynamics.
  * NMPC adapts at each step by solving the optimization problem with system constraints, which helps minimize deviations.

* **Insight**:
  The result demonstrates that **NMPC handles nonlinear dynamics better than standard MPC**, maintaining trajectory tracking despite disturbances.

---

## 2. **Control Inputs (Rotor Thrusts)**

![Control Inputs](Screenshot%202025-08-23%20120833.png)

* **Description**:
  These four subplots show the **control inputs** applied to each of the quadrotor’s rotors:

  * **Blue line** → Actual computed thrust from NMPC.
  * **Red line** → Hover/reference thrust (baseline).

* **Observations**:

  * The rotor thrusts vary significantly, showing NMPC’s **real-time adaptation**.
  * Inputs are not smooth because NMPC optimizes based on the **nonlinear ODE dynamics** at each time step.
  * Fluctuations reflect how the controller rejects **wind disturbance**.

* **Insight**:
  The NMPC optimizer **predicts future states** and allocates thrust unevenly to maintain stability, which is why oscillations are visible.

---

## 3. **State Tracking (Positions, Angles, and Wind Disturbances)**

![States](Screenshot%202025-08-23%20132548.png)

* **Description**:
  These plots compare **actual states** vs **reference values**:

  * **Top row**: X, Y, Z positions and roll (φ) angle.
  * **Middle row**: Pitch (θ), Yaw (ψ), and X-wind effect.
  * **Bottom row**: Y-wind and Z-wind disturbance effects.

* **Observations**:

  * **Position (X, Y, Z)**: NMPC tracks the reference closely, with slight delay and small steady-state error due to wind.
  * **Angles (φ, θ, ψ)**: The quadrotor changes orientation to counteract wind. Oscillations reflect **NMPC corrections** to stabilize flight.
  * **Wind disturbance plots**: Show how NMPC reacts to random disturbances; actual responses don’t match perfectly but stay bounded and stable.

* **Insight**:
  The NMPC system successfully stabilizes the quadrotor in a **nonlinear, disturbed environment**, proving robustness in both **trajectory tracking** and **disturbance rejection**.

---

# Key Takeaways

1. **Trajectory tracking in 3D** is achieved with high accuracy, even under wind noise.
2. **Control inputs (rotor thrusts)** fluctuate more than linear MPC, showing NMPC’s ability to handle nonlinear dynamics.
3. **State responses** confirm that NMPC stabilizes the system while compensating for wind disturbances.

Overall, NMPC with an ODE solver provides **robust, predictive, and nonlinear-aware control**, making it more reliable for quadrotor applications in uncertain environments.


## Authors

- [@iwinardhyas](https://www.github.com/iwinardhyas)

erwin.ardias@gmail.com