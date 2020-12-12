# MTE-Robotics-Lab-2
A collective codebase inclusive of all experiments and post lab exercises

## Installation

* Clone the repository at your `$WORKSPACE/src`:

      git clone https://github.com/leander-dsouza/MTE-Robotics-Lab-2.git
      
* Build the newly cloned packages at the root of your workspace:

      cd ../ && catkin_make
      
* To install all the dependencies of your workspace:

      rosdep install --from-paths src --ignore-src -r -y   

* To prevent repeated sourcing of your workspace's devel bash scripts, add the corresponding line and finally execute the entire bash script:

      echo "source $WORKSPACE_DIR/devel/setup.bash" >> ~/.bashrc && source ~/.bashrc

## Usage       

To learn more about the packages and how to run them, refer the [Wiki](https://github.com/leander-dsouza/MTE-Robotics-Lab-2/wiki) page of this repository.

## Disclaimer

Make sure you change the shebang from `python3` to `python`, as I'm using noetic-distro

