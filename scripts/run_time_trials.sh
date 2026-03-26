#!/bin/bash
SOURCE_CMD="source ~/ros_ws/devel/setup.bash"
ENTER_DIR="cd ~/ros_ws/src"
SETUP_CMD="$SOURCE_CMD;$ENTER_DIR"
confirm() {
    # Default prompt/response
    local prompt="${1:-Are you sure?}"
    local response

    while true; do
        read -p "$prompt [Y/n]: " response
        case "${response,,}" in # ${var,,} converts to lowercase
            y|yes|"") return 0 ;; # Success (True)
            n|no)      return 1 ;; # Failure (False)
            *)         echo "Please answer 'y' or 'n'." ;;
        esac
    done
}

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
xfce4-terminal -T "SCORE_TRACKER" -e \
"bash -c '$SETUP_CMD; cd /home/fizzer/ros_ws/src/2025_competition/enph353/enph353_utils/scripts; python3 ./score_tracker.py; exec bash'" &

# 3. Launch Time Trials in the main window when ready
if confirm "Launch time trials node?"; then
    xfce4-terminal -T "TIME_TRIALS" -e \
    "bash -c '$SETUP_CMD; roslaunch time_trials time_trials.launch; exec bash'" &
else
    echo "Cancelled time trials! Run '$SETUP_CMD; roslaunch time_trials time_trials.launch' to start it."
fi