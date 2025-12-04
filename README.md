for the training and implementation model of a dqn for navigation with gazebo in ROS2:
in the github it was 5 nodes in a src for being compiled and prepare to test the model of navigation

for begin it needs the follow requirements:

## Environment Setup

### Prerequisites

Ensure you have the following installed:

```bash
# Check ROS2 Humble installation
ros2 --version

# Check if TurtleBot3 packages are installed
ros2 pkg list | grep turtlebot3

# Required Python packages
pip3 install scikit-learn numpy matplotlib
```

### TurtleBot3 Setup

```bash
# Install TurtleBot3 packages if not already installed
sudo apt install ros-humble-turtlebot3*

# Set TurtleBot3 model
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

# Set Gazebo model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

### Create Workspace

```bash
# Create and build workspace
mkdir -p ~/dqn_navigation_ws/src
cd ~/dqn_navigation_ws/src
```

the following steps are to copy the content of the github
- copy all the files of the github in the workspace
- make a colcon build
```bash
colcon build
source install/setup.bash
```
- in the path **src/dqn_robot_nav/dqn_robot_nav/environment.py** in the line 34 copy the path from the model **src/dqn_robot_nav/models/goal_red/model.sdf** of your devide and replace in the code of the enviroment

once al this steps will be complet it will be ready for training or testing

For training:
- in the **training_node.py** are all the parameters to training a model, the important values are; n_episodes, max_steps_per_episode, in the agent are anoter parameters for a more advanced configuration of the simulation.
  and in the **environment.py** in the line 43 are the actions, that set the action in linear or angular velocity, it can be added or deleted, its important that the same quantity of actions are defined in the line 21 in **train_node.py**, it willl need to be the same.
- once all the parameters are defined it needs to run
  
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
in one terminal and in the workspace
```bash
colcon build
source install/setup.bash
ros2 run dqn_robot_nav train_node
```
and the training will be saved

For testing:
All the workspace is ready to be compiled, it only needs:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
in one terminal and in the workspace
```bash
colcon build
source install/setup.bash
ros2 run dqn_robot_nav test_node
```
