#!/bin/bash

# Source the workspace
source install/setup.bash

ros2 service call /set_up_new_episode turtle_interfaces/srv/SetUpNewEpisode "episode_num: 0"

# Train a model based on initial world configuration
ros2 run turtle_rl training_node --ros-args -p training_mode:=train -p epoch:=0

for i in {1..100}; do
  echo "Iteration $i"
  # Delete all obstacles in the arena
  ros2 service call /delete_all_obstacles std_srvs/srv/Empty
  # Sleep 3s
  sleep 3
  # Set up the new world configuration in the arena
  ros2 service call /set_up_new_episode turtle_interfaces/srv/SetUpNewEpisode "episode_num: $i"
  # Sleep 3s
  sleep 3
  # Retrain the model based on new world configuration
  ros2 run turtle_rl training_node --ros-args -p training_mode:=retrain -p epoch:=$i
done
