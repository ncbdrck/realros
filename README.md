# ROS_RL: A Comprehensive Framework for Real-World Robotic Reinforcement Learning

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

ROS_RL, is a unified ([ROS](http://wiki.ros.org/))-based open-source Python framework to create real-world robotics environments for reinforcement learning (RL) applications.

This framework extends the functionality of ROS by introducing additional Python bindings necessary for automatically launching environments in the real world, eliminating the need for manual configuration of the ROS's low-level features.

This framework provides the following features:
 1. A modular architecture that promotes reproducibility and encourages code reuse, 
 2. Real-time training with any RL simulation framework (Agnostic of any RL simulation framework) 
 3. Tools to execute concurrent environments and maintain communication.

## Prerequisites

Before installing ROS_RL, make sure you have the following prerequisites:

### ROS Installation

ROS_RL requires a working installation of ROS. If you haven't installed ROS yet, please follow the official [ROS installation guide](http://wiki.ros.org/ROS/Installation) for your specific operating system. This package has been tested with [ROS Noetic](http://wiki.ros.org/noetic) version, and the following instructions will guide you through the installation of ROS Noetic on Ubuntu 20.04:
```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```
### Catkin Tools
ROS_RL uses Catkin as the build system for ROS packages. Install Catkin Tools by running the following command:
```shell
sudo apt-get install python3-catkin-tools
```
### Other Packages 
ROS_RL also requires the following additional packages:
- XTerm for terminal emulation:
    ```shell
    sudo apt install xterm
    ```
- [MoveIt](https://moveit.ros.org/) for motion planning:
    ```shell
    sudo apt install ros-noetic-moveit
    ```
  
### Create ROS Workspace
Before using ROS_RL, you need to create a ROS workspace to build and run your ROS packages. Follow these steps to create a workspace:
```shell
cd ~
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

You are now ready to proceed with the installation and usage of ROS_RL.

Please note that the instructions assume you are using Ubuntu 20.04 and ROS Noetic. If you are using a different operating system or ROS version, make sure to adapt the commands accordingly.

## Installation

To get started with ROS_RL, follow these steps:

1. Clone the repository:
    ```shell
    cd ~/catkin_ws/src
    git clone https://github.com/ncbdrck/ros_rl.git
    ```

2. ROS_RL relies on several Python packages. You can install them by running the following command:

    ```shell
    # Install pip if you haven't already by running this command
    sudo apt-get install python3-pip

    # install the required Python packages for MultiROS by running
    cd ~/catkin_ws/src/ros_rl/
    pip3 install -r requirements.txt
    ```
3. Build the ROS packages and source the environment:
    ```shell
   cd ~/catkin_ws/
   rosdep install --from-paths src --ignore-src -r -y
   catkin build
   source devel/setup.bash
    ```

## License

ROS_RL is released under the [MIT License](https://opensource.org/licenses/MIT). Please see the LICENSE file for more details.

## Acknowledgements

We would like to thank the following projects and communities for their valuable contributions, as well as the authors of relevant libraries and tools used in MultiROS.
- [ROS (Robot Operating System)](https://www.ros.org/)
- [MoveIt](https://moveit.ros.org/)


## Cite

If you use MultiROS in your research or work and would like to cite it, you can use the following citation:

Repository
```bibtex
@misc{ros_rl,
  author = {Kapukotuwa, Jayasekara},
  booktitle = {GitHub repository},
  publisher = {GitHub},
  title = {ROS_RL: A Comprehensive Framework for Real-World Robotic Reinforcement Learning},
  url = {https://github.com/ncbdrck/ros_rl},
  year = {2023}
}
```

## Contact

For questions, suggestions, or collaborations, feel free to reach out to the project maintainer at [j.kapukotuwa@research.ait.ie](mailto:j.kapukotuwa@research.ait.ie).
