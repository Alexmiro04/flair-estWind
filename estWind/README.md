# estWind Project

## Overview
The `estWind` project provides a framework for developing custom controllers for UAVs based on FlAIR. It includes a skeleton structure for the project, with the `myCtrl` directory containing a customizable controller. In this example, the controller implements a PID-based position controller (with gravity compensation) and an attitude controller. The project also includes utilities for controller saturation and motor constant conversion. The example task becomes from the CircleFollower demo.

## File Structure
- **`uav/src/estWind`**: This class contains the skeleton of the project.
- **`uav/src/myCtrl`**: This class defines the customized controller.

## Build Instructions
The project includes a `utils/build.sh` script for building the project. Before running the script, ensure that you modify it to specify the desired build path with the variable `FOLDER_PATH` and the project name with `PROJECT_NAME`.

## Wind perturbations
The controller procedurally generates an 18-second wind profile that mimics the MATLAB prototype shared by the project authors. Three Gaussian segments are stitched together: 0–5 s (μ = 0.11, σ = 0.92), 5–10 s (μ = −0.24, σ = 1.37) and 10–18 s (μ = −0.53, σ = 1.78). The samples are produced at 0.1 s and applied along the X axis.

1. Open the “Wind perturbations” box in the GUI and press **Start wind** to inject the disturbance. Press **Stop wind** at any time to end the perturbation early.
2. Every generated velocity (vx, vy, vz) is logged under the names `wind_vx`, `wind_vy` and `wind_vz`. You can export these signals from Fl-AIR logs and plot them in MATLAB.

This approach removes the dependency on external CSV files while keeping the simulator deterministic (the generator uses a fixed random seed).

