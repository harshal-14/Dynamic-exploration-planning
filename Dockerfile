# Use ROS Melodic with Ubuntu 18.04 as base image
FROM osrf/ros:melodic-desktop-full

# Set shell to bash and make it non-interactive
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Set environment variables
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
ENV QT_X11_NO_MITSHM=1

# Install system dependencies
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

# Initialize tsdf workspace
WORKDIR /root/tsdf_ws
RUN catkin init \
    && catkin config --extend /opt/ros/melodic \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin config --merge-devel

# Clone and build voxblox
WORKDIR /root/tsdf_ws/src
RUN git clone https://github.com/ethz-asl/voxblox.git \
    && wstool init . ./voxblox/voxblox_https.rosinstall \
    && wstool update

# Add esdf.launch file
RUN mkdir -p /root/tsdf_ws/src/voxblox/voxblox_ros/launch
COPY <<EOF /root/tsdf_ws/src/voxblox/voxblox_ros/launch/esdf.launch
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
EOF

# Build voxblox
WORKDIR /root/tsdf_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin build voxblox_ros'

# Install nlopt
WORKDIR /root
RUN wget https://github.com/stevengj/nlopt/archive/v2.7.1.tar.gz \
    && tar -xvf v2.7.1.tar.gz \
    && cd nlopt-2.7.1 \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
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

# Modify CMakeLists.txt to use the correct nlopt path
RUN sed -i 's|/home/zhefan/Desktop/nlopt/build|/root/nlopt-2.7.1/build|g' /root/catkin_ws/src/DEP/CMakeLists.txt

# Set up environment variables
ENV GAZEBO_MODEL_PATH=/root/.gazebo/models:/root/catkin_ws/src/drone_gazebo/models:${GAZEBO_MODEL_PATH}
ENV QT_X11_NO_MITSHM=1

# Build the workspace
WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; . /root/tsdf_ws/devel/setup.bash; catkin_make'

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
