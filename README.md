# ENPH 353 Team 6 Repository

Authors: Shreya Shah & Jonah Lee

## Quick Start

This project contains packages meant for use in conjunction with the Gazebo/ROS environment at
https://github.com/ENPH353/2025_competition.

To set up the project clone this repository and https://github.com/ENPH353/2025_competition in the same directory (~/ros_ws/src).

Next, build the packages with catkin_make.

```bash
cd ~/ros_ws
catkin_make
```

To run the competition code, a helper script is provided: `run_time_trials.sh`. This will both start the main Gazebo world using `2025_competition/enph353/enph353_utils/scripts/run_sim.sh` and start the `score_tracker` node from the `enph353_utils` package using `roslaunch`.

**Note**: The score_tracker node can only be launched from the `/2025_competition/enph353/enph353_utils/scripts` directory because it assumes this location when searching for the required `.ui` file.

## Project Structure

The root of this repository is split into packages and scripts.

### 1. `packages`

This directory contains various ROS packages for collecting data.

Some packages may be robots, such as `eyeinthesky`. See the package readme for
how to run the robot in place of the default `robbie` bot.

Other packages are python nodes which interface with the existing models in the world.
To run them, there use the launch file, e.g. `roslaunch time_trials time_trials.launch`.

### 2. `scripts`

This directory is for helper scripts -- for example, they may automatically set up terminals running multiple nodes.