# DynaPRM for Robot Exploration

This repo contains the implementation of Dynamic Environment Planner (DEP) which aims at robotic exploration in dynamic and unknown environments in 
[ROS](https://www.ros.org/) and [Gazebo](http://gazebosim.org/). This work is adapted from this paper: https://ieeexplore.ieee.org/document/9362184. Alongwith re-implementation this project provide two extensions - 1. Spars RoadMap 2.Narrow Biased Planner

<img src="https://user-images.githubusercontent.com/55560905/111251884-77929700-85e6-11eb-8c98-3a28ba0d06d5.gif" alt="cafe_faster" width="278" height="180"><img src="https://user-images.githubusercontent.com/55560905/111250037-09000a00-85e3-11eb-9c04-7b81c4badc74.gif" alt="maze_faster" width="270" height="180"><img src="https://user-images.githubusercontent.com/55560905/111252221-03a4be80-85e7-11eb-8fcc-cab48a055426.gif" alt="office_faster" width="278" height="180">

# Installation

## Option 1: Use pre-built docker image (Recommended)

### Step1: Pull docker image:
Pull the latest image from docker hub, this step assumes you have docker service in your system, if not follow this [link](https://docs.docker.com/desktop/setup/install/linux/ubuntu/) to install it
```bash
docker pull hsdhillon1313/mp_dep:latest
```
### Step 2: Run the container
Modify the command to mount the correct folder to your cloned repository directory

```bash
xhost +local:docker

# Then run your docker command
docker run -it \
    --env="DISPLAY=$DISPLAY"" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/path/to/your/DEP:/root/catkin_ws/src/DEP" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume="/dev/dri:/dev/dri:rw" \
    --security-opt seccomp=unconfined \
    --cap-add=SYS_PTRACE \
    --privileged \
    --network=host \
    --name=dep_container \
    hsdhillon1313/mp_dep
``` 

## Option 2: Docker Installation

### Step 1: Build the Docker Image
Copy the provided Dockerfile into your project directory and build it. Note that building the image will take around 20-30 minutes.

```bash
sudo docker build -t dep_ros .
```

### Step 2: Run the Container
Enable X11 forwarding to see Gazebo and Rviz on your system (ensure you have Gazebo and Rviz installed on your host system):

```bash
xhost +local:docker

docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/dev/dri:/dev/dri:rw" \
    --volume="/path/to/your/DEP:/root/catkin_ws/src/DEP" \
    --privileged \
    --network=host \
    --name=dep_container \
    dep_ros
```

Note: Replace `/path/to/your/DEP` with the actual path to your DEP repository on your host system for docker volume mount. You don't need to follow native installation steps further. You can start with running 5 terminals for planner simulation in Rviz and Gazebo.

## Option 2: Native Installation

This package has been tested on Ubuntu 16.04/18.04 LTS with ROS Kinetic/Melodic. Make sure you have installed the compatible ROS version.

### Simulation Environments for Native Installation

To run the planner in simulation, please install the drone_gazebo, which requires octomap_server:

```bash
sudo apt-get install ros-kinetic-octomap-server
cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/drone_gazebo.git

# Build
cd ~/catkin_ws
catkin_make

# Add Gazebo Model Path
export GAZEBO_MODEL_PATH=path/to/drone_gazebo/models:$GAZEBO_MODEL_PATH
```

### Dependencies for Native Installation

This package relies on octomap_ros, voxblox_ros, and nlopt.

1. Install octomap_ros:
```bash
sudo apt-get install ros-kinetic-octomap-ros
```

2. Install voxblox_ros (Modified based on the original installation instruction):
```bash
mkdir -p ~/tsdf_ws/src
cd ~/tsdf_ws

catkin init
catkin config --extend /opt/ros/kinetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel

cd ~/tsdf_ws/src/
git clone https://github.com/ethz-asl/voxblox.git
wstool init . ./voxblox/voxblox_https.rosinstall
wstool update

cd ~/tsdf_ws/src/
catkin build voxblox_ros
```

Then, add `esdf.launch` file into the folder `~/tsdf_ws/src/voxblox/voxblox_ros/launch`, which contains the following contents:

```xml
<launch>
    <node name="esdf_node" pkg="voxblox_ros" type="esdf_server" output="screen" args="-alsologtostderr" clear_params="true">
      <remap from="pointcloud" to="/camera/depth/points"/>
      <remap from="esdf_node/esdf_map_out" to="esdf_map" />
      <param name="tsdf_voxel_size" value="0.2" />
      <param name="tsdf_voxels_per_side" value="16" />
      <param name="publish_esdf_map" value="true" />
      <param name="publish_pointclouds" value="true" />
      <param name="use_tf_transforms" value="true" />
      <param name="update_mesh_every_n_sec" value="1.0" />
      <param name="clear_sphere_for_planning" value="true" />
      <param name="world_frame" value="world" />
      <param name="sensor_frame" value="camera_link"/>
    </node>
</launch>
```

3. Install nlopt:
```bash
cd path/to/nlopt
mkdir build
cd build
cmake ..
make

cd path/to/nlopt/build
sudo make install
```

### Building the Planner

After installing all the required packages, compile the planner:

```bash
cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/DEP.git
```

**PLEASE MAKE SURE TO MODIFY THE FOLLOWING CONTENTS IN CMakeList.txt FOR SUCCESSFUL COMPILATION**
```
# Replace ALL "/home/zhefan/Desktop/nlopt/build" to "path/to/nlopt/build"
# Replace ALL "/home/zhefan/Desktop/nlopt/build/libnlopt.so" to "path/to/nlopt/build/libnlopt.so"
```

Also, since this package relies on catkin_make instead of catkin build, we need to create a catkin workspace overlay:

1. DELETE the build and devel folder in your ~/catkin_ws
2. Compile the planner:
```bash
source ~/tsdf_ws/devel/setup.bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

# How to Use

## For Docker Installation
Run the following commands in separate container terminals:

```bash
# Terminal 1
roslaunch drone_gazebo cafe.launch

# Terminal 2 (new container terminal)
roslaunch voxblox_ros esdf.launch

# Terminal 3 (new container terminal)
rosrun drone_gazebo cafe_warmup.sh

# Terminal 4 (new container terminal)
roslaunch DEP exploration.launch

# Terminal 5 (new container terminal)
rosrun DEP move_and_rotate.py
```

To open additional terminals for the running container:
```bash
docker ps  # Get the container ID
docker exec -it CONTAINER_ID bash
```

## For Native Installation
First, launch the simulation environment (E.g. Cafe). You need to turn on the panel for visualization in [Rviz](http://wiki.ros.org/rviz).
```bash
roslaunch drone_gazebo cafe.launch
roslaunch voxblox_ros esdf.launch # in a separate terminal
```

Let the robot(drone) get an initial scan:
```bash
rosrun drone_gazebo cafe_warmup.sh
```

Finally, run the planner:
```bash
roslaunch DEP exploration.launch
rosrun DEP move_and_rotate.py # in a separate terminal
```

# Visualization
To visualize the exploration process, below are the ros topics you need to add in [Rviz](http://wiki.ros.org/rviz):

- `/voxblox_mesh`: The ESDF map generated from voxblox_ros.
- `/occupied_vis_array` (Optional): The voxel map generated from octomap_server.
- `/map_vis_array`: This is the incremental PRM mentioned in the paper.
- `/path_vis_array`: The generated path from the DEP planner.
- `/closebound_vis_array`: Collision line (only for Narrow Space Biased PRM).
- `/inner_map_vis_array`: Child map in narrow space (only for Narrow Space Biased PRM).
  - `/edge_vis_array`: entry nodes  in narrow space (only for Narrow Space Biased PRM).

# Restricting the Exploration Range
Sometimes, you may want to explore a confined space (defined by a cubic). In order to achieve that, simply modify the `include/env.h` file to change the corresponding dimension of the desired space and then run `catkin_make` in `~/catkin_ws`.

# Bonus
We provide additional capabilities for DynaPRM like adaptive thresholding and Narrow space bias. These can be found in `feature/adaptive_thresholding` and `narrow_dep` branch. If using zipped folder Unzip the bonus zip folder and replace the include and src folders with adaptive thresolding folder or narrow folder based on which variant you want to visualize performance.

# Troubleshooting for Docker Installation
If you are having Docker X11 forwarding issue and not able to visualize Rviz and  gazebo on your system after termnial 1 command then follow these steps.

0. Stop and remove the existing container

1. First terminal (keep running):
```bash
socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CONNECT:/tmp/.X11-unix/X1
```

2. Second terminal, the exact working Docker command(replace mount path from host system):
```bash
docker run -it \
    --env="DISPLAY=host.docker.internal:0.0" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XDG_RUNTIME_DIR=/tmp/runtime-root" \
    --env="LIBGL_ALWAYS_SOFTWARE=1" \
    --add-host=host.docker.internal:host-gateway \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/dev/dri:/dev/dri:rw" \
    --volume="/home/lucifer/WPI/Fall_courses/MP/Dynamic-exploration-planning:/root/catkin_ws/src/DEP" \
    --privileged \
    --network=host \
    --name=dep_container \
    hsdhillon1313/mp_dep
```

3. Inside container, the exact steps that worked:
```bash
# Create runtime directory
mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root

# Install exact packages we used before
apt-get update && apt-get install -y \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    qt5-default

# Launch ROS (exactly as before)
roslaunch drone_gazebo cafe.launch
```

Raise a github issue if encountering any problems.

## Team
- Harmeet Dhillon
- Harshal Bhat
- Manav Bichu
- Dharmik Raval

---
Project developed at Worcester Polytechnic Institute
Robotics Engineering Department

