# Dynamic-exploration-planning
This repository consists of final project for Motion-Planning course at WPI

## Steps to run the code

# Step 1: Copy the provided docker file into a folder and build it
Note: Ensure you docker daemon is running. Building the image will take around 20-30 mins, so please be patient.

```bash
sudo docker build -t dep_ros .
```

# Step 2: Enable running X11 forwarding to see Gazebo and Rviz on your system. I expect you already have Gazebo and Rviz installed on your system
```bash
xhost +local:docker

docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/dev/dri:/dev/dri:rw" \
    --volume="/home/lucifer/WPI/Fall_courses/MP/Motion-Planning-Group-projects/FinalProject/DEP:/root/catkin_ws/src/DEP" \
    --privileged \
    --network=host \
    --name=dep_container \
    dep_ros
```
# Step 3: Once inside the container you run the Exploration system

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

Note: To open additional terminals for the running container:
```bash
docker ps  # Get the container ID
docker exec -it CONTAINER_ID bash
```

Open Rviz and you can visualize the octomap grid, map_vis and other topics for analysis.
