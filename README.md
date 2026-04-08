# ENPH 353 Team 6 Repository

Authors: Shreya Shah & Jonah Lee

## Quick Start

### Environment Setup

This project contains packages meant for use in conjunction with the Gazebo/ROS environment at
https://github.com/ENPH353/2025_competition.

To set up the project clone this repository and https://github.com/ENPH353/2025_competition in the same directory (~/ros_ws/src).

Next, build the packages with catkin_make.

```bash
cd ~/ros_ws
catkin_make
```

### Running the Competition

To run the competition code, a helper script is provided: `run_time_trials.sh`. This will both start the main Gazebo world using `2025_competition/enph353/enph353_utils/scripts/run_sim.sh` and start the `score_tracker` node from the `enph353_utils` package using `roslaunch`.

**Note**: The score_tracker node can only be launched from the `/2025_competition/enph353/enph353_utils/scripts` directory because it assumes this location when searching for the required `.ui` file.

In addition, two changes are required to add the custom drone:

1. Change the selected robot from Robbie to Eyeinthesky (`robots.launch`, lines 15-24)

```xml
<!-- Launch eyeinthesky -->
<group ns="B1">
<param name="tf_prefix" value="B1_tf" />
<include file="$(find eyeinthesky)/launch/eyeinthesky.launch">
    <arg name="init_pose" value="-x 5.5 -y 2.5 -z 0.2 -R 0.0 -P 0.0 -Y -1.57" />
    <arg name="robot_name"  value="B1" />
</include>
```

2. Change the wind scaling to 1% (`353.world`, line 586)

```xml
<force_approximation_scaling_factor>0.01</force_approximation_scaling_factor>
```

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

### 3. `data`

This is for training data for models. Data may not be pushed; this is also the output for
data generation scripts like `collect_images.py`.