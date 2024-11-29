# Use ROS Melodic with Ubuntu 18.04 as base image
FROM osrf/ros:melodic-desktop-full

# Set shell to bash and make it non-interactive
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Set environment variables
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV QT_X11_NO_MITSHM=1
ENV GAZEBO_MODEL_PATH=""

# Install system dependencies including debugging tools
RUN apt-get update && apt-get install -y \
    python-catkin-tools \
    python-wstool \
    python-pip \
    git \
    cmake \
    wget \
    unzip \
    build-essential \
    rsync \
    libyaml-cpp-dev \
    libprotobuf-dev \
    protobuf-compiler \
    autoconf \
    libtool \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    pulseaudio \
    alsa-utils \
    gdb \
    valgrind \
    vim \
    cmake-curses-gui \
    lldb \
    && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-melodic-octomap-server \
    ros-melodic-octomap-ros \
    ros-melodic-joy \
    ros-melodic-geodesy \
    ros-melodic-cv-bridge \
    ros-melodic-image-transport \
    ros-melodic-teleop-twist-keyboard \
    ros-melodic-message-to-tf \
    ros-melodic-tf2-geometry-msgs \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace structure
RUN mkdir -p /root/catkin_ws/src
RUN mkdir -p /root/tsdf_ws/src

# Initialize tsdf workspace with debug flags
WORKDIR /root/tsdf_ws
RUN catkin init \
    && catkin config --extend /opt/ros/melodic \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-g -O0" \
    && catkin config --merge-devel

# Clone and build voxblox
WORKDIR /root/tsdf_ws/src
RUN git clone https://github.com/ethz-asl/voxblox.git \
    && wstool init . ./voxblox/voxblox_https.rosinstall \
    && wstool update

# Create launch directory
RUN mkdir -p /root/tsdf_ws/src/voxblox/voxblox_ros/launch

# Create esdf.launch file directly
RUN echo '<launch>\n\
    <node name="esdf_node" pkg="voxblox_ros" type="esdf_server" output="screen" args="-alsologtostderr" clear_params="true">\n\
      <remap from="pointcloud" to="/camera/depth/points"/>\n\
      <remap from="esdf_node/esdf_map_out" to="esdf_map" />\n\
      <param name="tsdf_voxel_size" value="0.2" />\n\
      <param name="tsdf_voxels_per_side" value="16" />\n\
      <param name="publish_esdf_map" value="true" />\n\
      <param name="publish_pointclouds" value="true" />\n\
      <param name="use_tf_transforms" value="true" />\n\
      <param name="update_mesh_every_n_sec" value="1.0" />\n\
      <param name="clear_sphere_for_planning" value="true" />\n\
      <param name="world_frame" value="world" />\n\
      <param name="sensor_frame" value="camera_link"/>\n\
    </node>\n\
</launch>' > /root/tsdf_ws/src/voxblox/voxblox_ros/launch/esdf.launch

# Build voxblox with memory limits
WORKDIR /root/tsdf_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash \
    && export MAKEFLAGS="-j1" \
    && export CATKIN_MAKE_FLAGS="-j1" \
    && catkin build voxblox_ros \
    --cmake-args -DCMAKE_BUILD_TYPE=Debug \
    && catkin config --blacklist \
    && catkin build voxblox_ros -DCMAKE_BUILD_TYPE=Debug -j1'

# Install nlopt
WORKDIR /root
RUN wget https://github.com/stevengj/nlopt/archive/v2.7.1.tar.gz \
    && tar -xvf v2.7.1.tar.gz \
    && cd nlopt-2.7.1 \
    && mkdir build \
    && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Debug \
    && make -j1 \
    && make install

# Clone required repositories
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/Zhefan-Xu/drone_gazebo.git \
    && git clone https://github.com/Zhefan-Xu/DEP.git

# Download and install Gazebo models
RUN mkdir -p ~/.gazebo/models \
    && cd ~/.gazebo/models \
    && wget https://github.com/osrf/gazebo_models/archive/refs/heads/master.zip \
    && unzip master.zip \
    && mv gazebo_models-master/* . \
    && rm -rf gazebo_models-master master.zip

# Modify CMakeLists.txt to use the correct nlopt path and add debug flags
RUN sed -i 's|/home/zhefan/Desktop/nlopt/build|/root/nlopt-2.7.1/build|g' /root/catkin_ws/src/DEP/CMakeLists.txt \
    && sed -i 's|set(CMAKE_BUILD_TYPE Release)|set(CMAKE_BUILD_TYPE Debug)|g' /root/catkin_ws/src/DEP/CMakeLists.txt \
    && echo "add_compile_options(-g)" >> /root/catkin_ws/src/DEP/CMakeLists.txt

# Create .vscode directory
RUN mkdir -p /root/catkin_ws/src/DEP/.vscode

# Create launch.json
RUN echo '{\n\
    "version": "0.2.0",\n\
    "configurations": [\n\
        {\n\
            "name": "ROS: map_vis debug",\n\
            "type": "cppdbg",\n\
            "request": "launch",\n\
            "program": "/root/catkin_ws/devel/lib/DEP/map_vis",\n\
            "args": [],\n\
            "stopAtEntry": false,\n\
            "cwd": "/root/catkin_ws",\n\
            "environment": [\n\
                {\n\
                    "name": "ROS_MASTER_URI",\n\
                    "value": "http://localhost:11311"\n\
                }\n\
            ],\n\
            "externalConsole": false,\n\
            "MIMode": "gdb",\n\
            "setupCommands": [\n\
                {\n\
                    "description": "Enable pretty-printing for gdb",\n\
                    "text": "-enable-pretty-printing",\n\
                    "ignoreFailures": true\n\
                }\n\
            ]\n\
        }\n\
    ]\n\
}' > /root/catkin_ws/src/DEP/.vscode/launch.json

# Create tasks.json
RUN echo '{\n\
    "version": "2.0.0",\n\
    "tasks": [\n\
        {\n\
            "label": "catkin_make",\n\
            "type": "shell",\n\
            "command": "cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Debug",\n\
            "group": {\n\
                "kind": "build",\n\
                "isDefault": true\n\
            }\n\
        }\n\
    ]\n\
}' > /root/catkin_ws/src/DEP/.vscode/tasks.json

# Set up environment variables
ENV GAZEBO_MODEL_PATH=/root/.gazebo/models:/root/catkin_ws/src/drone_gazebo/models:${GAZEBO_MODEL_PATH}
ENV QT_X11_NO_MITSHM=1

# Build the workspace with debug symbols
WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash \
    && . /root/tsdf_ws/devel/setup.bash \
    && export MAKEFLAGS="-j1" \
    && catkin_make -DCMAKE_BUILD_TYPE=Debug'

# Source ROS workspace in bashrc
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
    && echo "source /root/tsdf_ws/devel/setup.bash" >> ~/.bashrc \
    && echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Create a script to set up audio
RUN echo '#!/bin/bash\n\
pulseaudio -D --exit-idle-time=-1\n\
pacmd load-module module-virtual-sink sink_name=dummy\n\
pacmd set-default-sink dummy\n\
pacmd set-default-source dummy.monitor\n\
' > /root/setup_audio.sh \
    && chmod +x /root/setup_audio.sh

# Set the default command
CMD ["/bin/bash"]
