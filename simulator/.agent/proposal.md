# Proposal: Docking Simulator (Task 1)

This proposal outlines the implementation plan for the **Docking Simulator** as described in [requirements.md](file:///data/copilot/workspace/robot1/requirements.md) and [task1.txt](file:///data/copilot/workspace/robot1/docs/task1.txt).

---

## 1. Architecture & Technology Stack

We propose two options for the simulator, with **Option A (Web-based)** as the recommended path:

### Option A: Web-based Interactive Simulator (Recommended)
* **Tech Stack**: HTML5, Vanilla CSS (sleek dark mode, premium typography), JavaScript (ES6+), and HTML5 Canvas for real-time 2D rendering.
* **Why it fits**:
  * **Interactive Visualization**: Smooth rendering of the robot's trajectory, sensor cones, and real-time coverage.
  * **Rich Configurator**: Form controls to easily tune geometries, speeds, and all noise/sensor error variables on the fly.
  * **Batch Run Visualization**: Execute batches of 100-1000 runs instantly and view results as charts/tables directly in the UI.
  * **Portability**: Runs immediately in any browser without needing Python package installations (`pip install pygame`, etc.).

### Option B: Python-based Simulator
* **Tech Stack**: Python 3.x, standard libraries, with `matplotlib` or `pygame` for visualization.
* **Why it fits**:
  * If the ultimate docking algorithms are to be copy-pasted directly into Python-based robot firmware, this option might require less porting.
  * *Downside*: Harder to provide a highly interactive GUI with real-time controls and premium styling without overhead.

---

## 2. Mathematical & Physics Models

### Robot Geometry
The robot's dimensions are $12 \text{ cm (width)} \times 16 \text{ cm (length)}$. We will support two footprint models:
1. **Rectangular Footprint**: A simple bounding box.
2. **Chamfered Polygon Footprint**: Based on [diagram.svg](file:///data/copilot/workspace/robot1/docs/diagram.svg). The vertices in cm relative to the robot's center $(0,0)$ (with face pointing along the positive Y-axis) are:
   $$\begin{aligned}
   P_1 &= (-6.0, -5.5) && \text{(Back-left)} \\
   P_2 &= (-6.0,  5.5) \\
   P_3 &= (-3.5,  8.0) && \text{(Front-left chamfer start)} \\
   P_4 &= ( 3.5,  8.0) && \text{(Front-right chamfer start)} \\
   P_5 &= ( 6.0,  5.5) \\
   P_6 &= ( 6.0, -5.5) && \text{(Back-right)} \\
   P_7 &= ( 3.5, -8.0) && \text{(Back-right chamfer start)} \\
   P_8 &= (-3.5, -8.0) && \text{(Back-left chamfer start)}
   \end{aligned}$$

### Target Geometry
* Rectangular or square mat with configurable dimensions: $w \in [1, 9]\text{ cm}$ and $h \in [1, 9]\text{ cm}$.
* Configurable rotation angle $\theta_{\text{target}}$.

### Geometric Coverage Calculation
To compute the exact `coverage_ratio = IntersectionArea(Robot, Target) / TargetArea`:
* For the **Web-based simulator**, we will implement a robust polygon clipping algorithm (Sutherland-Hodgman) to find the intersection polygon of the robot footprint and the target mat. We will then compute the area of this intersection polygon.
* For the **Python-based simulator**, we can use the `shapely` library or a custom polygon intersection function.

---

## 3. Motion & Sensor Models with Noise

### Motion State
The state of the robot is defined by $(x, y, \theta)$, representing position in cm and heading in degrees.
* **Ideal Motion**:
  $$x_{t+1} = x_t + v \cdot \cos(\theta_t) \cdot \Delta t$$
  $$y_{t+1} = y_t + v \cdot \sin(\theta_t) \cdot \Delta t$$
  $$\theta_{t+1} = \theta_t + \omega \cdot \Delta t$$
  where $v$ is forward speed ($\text{cm/s}$) and $\omega$ is rotational speed ($\text{deg/s}$).

* **Noise & Slip Integration**:
  * **Forward Slip**: Actual forward velocity is scaled by a random factor: $v_{\text{noisy}} = v \cdot (1 - \text{slip\_percent} \cdot \text{rand}())$.
  * **Turn Error**: Rotational velocity is perturbed: $\omega_{\text{noisy}} = \omega \cdot (1 + \text{turn\_error\_percent} \cdot \text{gaussian}())$.
  * **Position Drift**: Adds random walk drift at each step: $x = x + \mathcal{N}(0, \sigma_{\text{pos}})$, $y = y + \mathcal{N}(0, \sigma_{\text{pos}})$.
  * **Heading Drift**: Adds angular drift: $\theta = \theta + \mathcal{N}(0, \sigma_{\text{heading}})$.
  * **Stop Delay**: Commands to stop are delayed by $t_{\text{delay}}$, continuing motion momentarily.

### Sensor Model
The algorithms will only access the virtual sensors, which can be configured with error rates:
1. `target_visible`: True if the target center falls within the sensor cone (defined by range and angle parameters). Subject to false positive/negative rates.
2. `target_angle_deg`: Relative bearing to the target, perturbed by `sensor_angle_noise_deg`.
3. `target_distance_cm`: Distance to target center, perturbed by `sensor_range_noise_cm`.
4. `target_under_robot`: True if the target's center lies inside the robot's polygon.
5. `termination_indicator`: Set to active when the algorithm decides to terminate. (Must only be triggered when stationary, or a $-5$ penalty is applied).

---

## 4. Docking Algorithms

We will implement the following five algorithms:
1. **Straight Walk**: Face forward and drive straight. (Baseline/Failure reference).
2. **Random Walk**: Drive forward; if no target is found within a timeout, turn a random angle and repeat.
3. **Rotate-Scan then Drive**: Rotate in place to find the angle that minimizes `target_angle_deg` or maximizes visibility, then drive straight towards it.
4. **Expanding Spiral / Square**: Walk in an expanding spiral or Archimedean square to search the area.
5. **Detect, Approach, Slow Final Correction**: 
   * Rotate-scan to acquire target direction.
   * Drive towards the target at full speed.
   * As distance decreases, slow down.
   * Once `target_under_robot` is true, perform micro-corrections (short forward/backward/turn adjustments) to maximize target coverage before stopping and triggering the termination indicator.

---

## 5. UI & Run Modes

If Option A (Web-based) is selected, the application will provide:
1. **Interactive Sandbox (Single Run)**:
   * Play, Pause, Step controls.
   * Visual canvas showing the robot, its path, the target, sensor cones, and real-time coverage area.
   * Live dashboard showing: current state $(x, y, \theta)$, speeds, sensor outputs, coverage ratio, current score, and simulation time.
2. **Batch Simulator**:
   * Input to specify number of trials (e.g., 100 or 1000).
   * A "Run Batch" button that executes runs asynchronously to prevent UI freeze.
   * Summary card showing:
     * **Success Rate (100% Coverage)**
     * **Partial Coverage Rate**
     * **Touched But Failed Rate**
     * **Moving Termination Failures**
     * **Average Score**
     * **Average Time to Dock**
3. **Beautiful Design**: A premium, responsive interface featuring CSS grid layouts, HSL color variables (sleek dark mode), and micro-animations on interactive elements.

---

## Next Steps & Decisions

Please review this proposal and let us know:
1. **Which option do you prefer?** We highly recommend **Option A (Web-based)** for its outstanding visualization, debugging usefulness, and compliance with the design aesthetics guidelines.
2. **Algorithm details**: Do you have any specific constraints on how the docking algorithms should be structured, or are you happy with our planned five baselines?
3. **Plan location**: We have stored this proposal in the `.agent` folder inside the workspace. Once you approve the plan, we will start building the simulator.
