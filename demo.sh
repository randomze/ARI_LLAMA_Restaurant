#!/bin/env bash

# Import the convenience commands
source ari-convenience-docker/environment
source RODGER/environment_rodger

# Create a folder structure
mkdir containers_fs

mkdir containers_fs/gallium
mkdir containers_fs/gallium/src
mkdir containers_fs/gallium/bridge

mkdir containers_fs/rodger

# Populate the folder structure
sn -l cloud_interaction containers_fs/gallium/src/cloud_interaction
sn -l RODGER/RODGER_src containers_fs/rodger/src

# Create the containers
make_container gallium ari-llama-gallium-container ./containers_fs/gallium/src ./containers_fs/gallium/bridge
make_rodger ari-llama-rodger-container ./containers_fs/rodger/src

# Start the containers
start_container ari-llama-gallium-container
start_container ari-llama-rodger-container

# New aliases because it's annoying to write too much
run_gallium() {
  run_command ari-llama-gallium-container ${@:1}
}

run_rodger() {
  run_command ari-llama-rodger-container ${@:1}
}

# Compile the ROS workspace
run_gallium "cd catkin_ws; catkin build"

# Start the Ollama server in the background and hide its output
run_rodger "" &>/dev/null &

# Start the ROS package
run_gallium "source catkin_ws/devel/setup.bash; rosrun cloud_interaction experiment_design" &

# Wait for the user to write "end" in order to finish the interaction
USER_STRING=""
while [ $USER_STRING != "end" ]; do
  read USER_STRING
done
run_gallium "source catkin_ws/devel/setup.bash; rostopic pub -1 /set_flag_end std_msgs/String finish"

# Sleep for a bit to give it time to wrap up
sleep 10s

# Stop the containers
stop_container ari-llama-gallium-container
stop_container ari-llama-rodger-container
