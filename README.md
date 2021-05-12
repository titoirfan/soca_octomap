# soca_octomap

An ultraviolet-C (UV-C) irradiation simulation and evaluation implementation in Gazebo and ROS.

<img src="./docs/demo.png" height="200"/> <img src="./docs/map_building.gif" height="200"/> 

<img src="./docs/map_meas.png" height="200"/> <img src="./docs/map_ref.png" height="200"/> <img src="./docs/map_diff.png" height="200"/> 

# 1. Acknowledgement
The development of this software won't be possible without the following open-source libraries and tools
- [octomap](https://github.com/OctoMap/octomap)
- [octomap_mapping](https://github.com/OctoMap/octomap_mapping)
- [octomap_ros](https://github.com/OctoMap/octomap_ros)
- [octomap_msgs](https://github.com/OctoMap/octomap_msgs)
- [octomap_rviz_plugins](https://github.com/OctoMap/octomap_rviz_plugins)
- [velodyne_simulator](https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/)
- [hector_localization](https://github.com/tu-darmstadt-ros-pkg/hector_localization)
- [rotors_simulator](https://github.com/ethz-asl/rotors_simulator)
- [tinycolormap](https://github.com/yuki-koyama/tinycolormap)

# 2. Prerequisite

We have tested the implementation in Ubuntu 20.04 with ROS Noetic and Gazebo 11.1.0.

## C++11 Compiler
We use functionalities provided by C++11.

## Octomap
You will need to install [Octomap](https://octomap.github.io/) separately in your system.

# 3. Building soca_octomap

Create a catkin workspace
```bash
mkdir -p catkin_ws/src && cd ./catkin_ws/src
```

Clone the repository
```bash
git clone https://github.com/titoirfan/soca_octomap
```

Build and source the repository
```bash
cd .. && catkin_make && source ./devel/setup.bash 
```