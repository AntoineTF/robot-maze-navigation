# BOMR Project – Autonomous Thymio Mission

## Project Overview

**Goal.** Develop a fully autonomous Thymio robot capable of perceiving its environment, planning a safe route to a target, and executing the mission using onboard sensing and control.  
**Approach.** The system combines computer vision, A* path planning, rule-based local navigation, and PID motion control, achieving consistent autonomous navigation in a real-world arena.

This repository packages the complete perception–planning–control stack along with the exploratory notebooks and report used to analyse performance and tune the hardware/software loop.

<p align="center">
  <img src="img/general_diagram.jpeg" alt="System overview" width="700"/>
  <br>
  <em>Full perception–planning–control loop implemented for Thymio autonomy.</em>
</p>

## Key Highlights

| Aspect | Description |
| --- | --- |
| Mission objective | Autonomous Thymio navigation from take-off to goal using onboard sensing |
| Vision | Camera calibration via ArUco markers + HSV color segmentation |
| Global planner | 8-neighbour A* with visual overlay and debugging tools |
| Local navigation | Finite-state machine for obstacle avoidance and waypoint following |
| Control | PID-based wheel actuation with Kalman filter smoothing |
| Result | Reliable mission completion with ~90% path adherence in noisy lighting |

## Results Demonstration

| Global map | Path planning overlay | Final trajectory |
| --- | --- | --- |
| ![Environment map](img/env_map.jpg) | ![Detected markers and objects](img/vision_init_result.jpg) | ![Local navigation FSM](img/model_local_nav.jpeg) |

## Repository Structure

```text
.
├── Global.py              # Global A* planner and path reconstruction utilities
├── Local_nav_model.jpeg   # Standalone copy of the local navigation diagram
├── MotionControl.py       # Thymio motor interface, LED helpers, and PID controller
├── kalman_filter.py       # Constant-velocity Kalman filter tuned for Thymio kinematics
├── local_nav.py           # Local navigation state machine and obstacle handling maneuvers
├── main.ipynb             # Mission notebook tying perception, planning, and control
├── Report.ipynb           # Final project report with data, figures, and discussion
├── vision/                # Computer-vision pipeline (aruco, color segmentation, helpers)
└── img/                   # All figures used in the README and notebooks
```

## End-to-End Pipeline

### 1. Perception (`vision/`)
- Calibrates the camera view using four ArUco markers before warping the board to a 10×7 grid (`vision.functions.perspective`).
- Segments color-coded shapes to detect obstacles (red), intermediate targets (green), and goal (blue) via HSV filtering (`vision.detection.colored_object_extraction`).
- Estimates the Thymio pose from marker corners and converts everything into grid coordinates, ready for planning (`vision.VisionObjects`).
- `vision.functions.prepare_output` packages an occupancy grid, waypoints, and the robot state for downstream modules.

### 2. Global Planning (`Global.py`)
- Builds the heuristic lookup and explores the grid with an 8-neighbour A* (`A_Star`) while tracking expanded nodes for debug overlays.
- Returns both path indices and real-world coordinates so the local navigator can translate the plan into motion commands.
- Utility helpers (`reconstruct_path`, `_get_movements_4n/8n`) make it easy to adjust neighborhood definitions when the board layout changes.

### 3. Local Navigation (`local_nav.py`)
- Implements a state machine that steps through the global path, monitors proximity sensors, and executes avoidance manoeuvres.
- Dedicated handlers (e.g., `handle_target_left/right`, `handle_target_behind`) switch direction, rotate, and advance precise distances to skirt obstacles or align with targets.
- Tunable constants (`Speed_Factor`, `Short_distance`, `Threshold_detection`) let you adapt the behaviour to new arena scales or motor calibrations.

### 4. Motion Control & State Estimation
- `MotionControl.Control` wraps the `tdmclient` API to stream motor targets, fetch proximity data, and drive LEDs for feedback.
- `pid_controller` dynamically balances throttle and turning to center the robot on its heading, cropping corrections when hard turns are required.
- `kalman_filter.KalmanFilter` fuses wheel odometry with the estimated heading, smoothing position and velocity before each control step.

## Getting Started

1. **Install dependencies**
   ```bash
   python -m venv .venv
   source .venv/bin/activate
   pip install numpy opencv-python tdmclient ipykernel ipywidgets
   ```
   Add other packages you use in the notebooks (e.g., `matplotlib`, `scipy`).

2. **Collect calibration assets**
   - Place four ArUco markers (IDs 1–4) on the board corners and the marker with ID 5 on the Thymio.
   - Ensure lighting is uniform so HSV thresholds defined in `vision/detection.py` stay reliable.

3. **Launch the mission notebook**
   - Start the Thymio firmware and connect via `tdmclient`.
   - Open `main.ipynb` to run the live pipeline: initialize vision, fetch the occupancy grid, plan, and step through execution.

4. **Review the full report**
   - `Report.ipynb` explains the approach, experiments, and analysis. Execute it to regenerate visualizations or adapt them for presentations.

## Operating Tips

- Use `vision/utils.py` helpers (`display_image`, `draw_on_image`) to sanity-check perception results while tuning HSV ranges or morphology kernels.
- Log the closed-set explored by `A_Star` to understand why certain paths were rejected; overlaying the returned `closedSet` on `img/map.jpeg` is handy for debugging.
- Adjust the Kalman `time_normalization_factor` when changing camera frame rates so prediction updates stay stable.
- The PID constants in `MotionControl.py` were tuned for ~80 mm/s cruise speed; re-run a short Ziegler–Nichols sweep if you retrofit different wheels or battery packs.

## Extending the Project

- Swap in a probabilistic roadmap or D* for dynamic environments while reusing the same interface expected by `local_nav.py`.
- Record image sequences and develop an automated vision regression suite to guard against lighting regressions.
- Export `main.ipynb` as a Python script for kiosk-style demos where notebooks are impractical.

## Project Context

This project was completed as part of the **EPFL “Mobile Robotics” course** (MICRO-452) under the supervision of **Prof. Mondada Francesco**. The system was developed and tested in the Thymio arena to explore end-to-end autonomy, from perception through motion execution.

## Acknowledgements

This README builds on the figures, images, and analysis already produced in the notebooks (`Report.ipynb`, `main.ipynb`). Keep them alongside this document so the visual explanations remain accurate.

Special thanks to Prof. Mondada Francesco, the MICRO-452 teaching team, and all teammates for their collaboration and guidance.

