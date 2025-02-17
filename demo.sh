#!/bin/env bash

# Import the convenience commands
source ari-convenience-docker/environment

# Create a folder structure
mkdir -p containers_fs

mkdir -p containers_fs/gallium
mkdir -p containers_fs/gallium/src
mkdir -p containers_fs/gallium/bridge

# Populate the folder structure
cp -r $(pwd)/cloud_interaction $(pwd)/containers_fs/gallium/src

# Create the containers
make_container gallium ari-llama-gallium-container ./containers_fs/gallium/src ./containers_fs/gallium/bridge

# Do this manually for RODGER because it deviates a bit from the poor abstractions of ari-convenience
create_image desktop ./RODGER
make_container_adv desktop ari-llama-rodger-container ./RODGER/RODGER_src /home/$USER/RODGER_src ollama /root/.ollama

# Start the containers
start_container ari-llama-gallium-container
start_container ari-llama-rodger-container

# New aliases because it's annoying to write too much
run_gallium() {
  run_command ari-llama-gallium-container "$@"
}

run_rodger() {
  run_command ari-llama-rodger-container "$@"
}

# Compile the ROS workspace
run_gallium /bin/bash -c "source /opt/pal/gallium/setup.bash; cd catkin_ws; catkin build; python3 -m pip install python-statemachine"

# Start the Ollama server in the background and hide its output
run_rodger /bin/bash -c "python server/ollama_app.py" &>rodger_output.txt &

# Start the ROS package
run_gallium /bin/bash -c "source catkin_ws/devel/setup.bash; roscore" &>/dev/null &
run_gallium /bin/bash -c "source catkin_ws/devel/setup.bash; rosrun cloud_interaction experiment_design.py" &

# Wait for the user to write "end" in order to finish the interaction
echo "Write 'end' when you wish to terminate the experiment"
USER_STRING=""
while [ "$USER_STRING" != "end" ]; do
  read USER_STRING
done
run_gallium /bin/bash -c "source catkin_ws/devel/setup.bash; rostopic pub -1 /set_flag_end std_msgs/String finish"

# Sleep for a bit to give it time to wrap up
sleep 10s

# Stop the containers
close_container ari-llama-gallium-container
close_container ari-llama-rodger-container
