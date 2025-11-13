# estWind Project

## Overview
The `estWind` project provides a framework for developing custom controllers for UAVs based on FlAIR. It includes a skeleton structure for the project, with the `myCtrl` directory containing a customizable controller. In this example, the controller implements a PID-based position controller (with gravity compensation) and an attitude controller. The project also includes utilities for controller saturation and motor constant conversion. The example task becomes from the CircleFollower demo.

## File Structure
- **`uav/src/estWind`**: This class contains the skeleton of the project.
- **`uav/src/myCtrl`**: This class defines the customized controller.

## Build Instructions
The project includes a `utils/build.sh` script for building the project. Before running the script, ensure that you modify it to specify the desired build path with the variable `FOLDER_PATH` and the project name with `PROJECT_NAME`.

## Replaying wind disturbances
You can replay recorded wind disturbances by providing a CSV file that contains the following columns: time (in seconds), wind speed on the X axis, wind speed on the Y axis and wind speed on the Z axis. The CSV file can optionally include a header row.
1. Place the provided `wind_uvw_1s_30min.csv` file inside `estWind/uav/data/` (create the directory if it does not exist). The controller will automatically look for that name in the project tree when no explicit path is given.
2. Launch the UAV controller with the `--wind-csv` option:
   ```bash
   ./estWind --wind-csv /path/to/wind_profile.csv
   ```
   Alternatively, you can set the `WIND_CSV_PATH` environment variable before starting the controller. If neither option is provided, the executable will try to locate `wind_uvw_1s_30min.csv` in common locations such as `estWind/uav/data/` relative to the launch directory.
3. In the “Wind disturbance” section of the controller GUI, enable the playback checkbox and tune the optional time scaling, offset and gain values to suit your scenario.
