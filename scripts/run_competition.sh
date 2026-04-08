#!/bin/bash
SOURCE_CMD="source ~/ros_ws/devel/setup.bash"
ENTER_DIR="cd ~/ros_ws/src"
SETUP_CMD="$SOURCE_CMD;$ENTER_DIR"

# 1. Launch Gazebo in a new xfce4-terminal window
xfce4-terminal -T "GAZEBO_SIM" -e \
"bash -c '$SETUP_CMD; bash $HOME/ros_ws/src/2025_competition/enph353/enph353_utils/scripts/run_sim.sh -vpgw; exec bash'" &
# Wait until the ROS Master and Gazebo physics are actually alive
echo "Waiting for Gazebo topic /clock..."
until rostopic list 2>/dev/null | grep -q "/clock"; do
    sleep 1
    echo -n "."
done
echo -e "\nGazebo is ready!"

# 2. Run the score tracker
SCORE_TRACKER_DIR="/home/fizzer/ros_ws/src/2025_competition/enph353/enph353_utils/scripts"
SCORE_TRACKER_CMD="$SETUP_CMD; cd $SCORE_TRACKER_DIR; python3 score_tracker.py"
xfce4-terminal -T "SCORE_TRACKER" -e \ "bash -c '$SCORE_TRACKER_CMD; exec bash'" &

# 3. Run the competition controller node
xfce4-terminal -T "COMP_CONTROLLER" -e \ "bash -c '$SETUP_CMD; roslaunch team6_utils competition.launch; exec bash'" &
