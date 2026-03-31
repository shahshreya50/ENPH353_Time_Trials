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
if confirm "Start gazebo world?"; then
    xfce4-terminal -T "GAZEBO_SIM" -e \
    "bash -c '$SETUP_CMD; bash $HOME/ros_ws/src/2025_competition/enph353/enph353_utils/scripts/run_sim.sh -vpgw; exec bash'" &
    # Wait until the ROS Master and Gazebo physics are actually alive
    echo "Waiting for Gazebo topic /clock..."
    until rostopic list 2>/dev/null | grep -q "/clock"; do
        sleep 1
        echo -n "."
    done
    echo -e "\nGazebo is ready!"
else
    echo "Skipped starting gazebo world!"
fi


# 2. Run rqt_image_iew
# Note: configure the number of images and other settings in collect_images.py
IMAGE_VIEW_CMD="$SETUP_CMD; rqt_image_view"
if confirm "Start rqt_image_view?"; then
    xfce4-terminal -T "IMAGE_VIEWER" -e \
    "bash -c '$IMAGE_VIEW_CMD; exec bash'" &
else
    echo "Skipped starting rqt_image_view! Run '$IMAGE_VIEW_CMD' to start it."
fi


# 3. Run the image collection script!
# Note: configure the number of images and other settings in collect_images.py
COLLECT_IMAGES_CMD="$SETUP_CMD; python3 /home/fizzer/ros_ws/src/team6/packages/team6_utils/nodes/collect_images.py"
if confirm "Start image collection?"; then
    xfce4-terminal -T "COLLECT_IMAGES" -e \
    "bash -c '$COLLECT_IMAGES_CMD; exec bash'" &
else
    echo "Skipped starting image collection! Run '$COLLECT_IMAGES_CMD' to start it."
fi
