<img src="https://img.shields.io/badge/license-MIT-green&style=plastic"> <img src="https://img.shields.io/badge/noetic-passing-green&style=plastic">

# MTE-Robotics-Lab-2 - Group-1 Miniproject 

![Python](https://img.shields.io/badge/-Python-black?style=plastic&logo=Python)
<img alt="Ubuntu" src="https://img.shields.io/badge/-Ubuntu-E95420?style=plastic&logo=Ubuntu&logoColor=white">
<img alt="GitHub" src="https://img.shields.io/badge/-GitHub-181717?style=plastic&logo=GitHub">

A collective codebase inclusive of all experiments and post lab exercises

## Installation

* Clone the repository at your `$WORKSPACE/src`:

      git clone --single-branch --branch=LSD https://github.com/leander-dsouza/MTE-Robotics-Lab-2.git
      
* Build the newly cloned packages at the root of your workspace:

      cd ../ && catkin_make
      
* Install `python3-rosdep` for resolving your workspaces' dependencies (**Don't install `rosdep2` as it will break your ros system**):

      sudo apt-get install python3-rosdep && sudo rosdep init && rosdep update
      
* To install all the dependencies of your workspace:

      rosdep install --from-paths src --ignore-src -r -y   

* To prevent repeated sourcing of your workspace's devel bash scripts, add the corresponding line and finally execute the entire bash script:

      echo "source $WORKSPACE_DIR/devel/setup.bash" >> ~/.bashrc && source ~/.bashrc

## Usage       

### 1) Multi-robot

* To launch the multi-robot simulator, launch the world with the bots in scene:

      roslaunch integrator main.launch

* To move all the robots using arrow keys:

      rosrun integrator key_drive.py


### 2) PCL Obstacle Avoidance

* Execute the launching script to deploy the robot with an Intel RealSense camera:

      roslaunch atreus xacro_robot.launch camera_enabled:=true

* Then run the obstacle avoidance script:

      rosrun atreus pcl_obstacle_avoidance.py

* Note that the robot wil move towards the resultant vector, and hence forward if no obstacle is present. So place an object in front of it, to see it avoid.


### 3) Pre-Planned Scene:

* Make sure you have [**gzsatellite**](https://github.com/plusk01/gzsatellite) in your `$WORKSPACE/src`:

      cd $WORKSPACE/src && git clone https://github.com/plusk01/gzsatellite && cd ../ && catkin_make

* To visualize the world, run the launch file:

      roslaunch atreus xacro_robot.launch world_name:=ab1.world
