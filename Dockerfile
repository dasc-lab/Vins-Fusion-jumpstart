FROM ros:kinetic-perception

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND noninteractive

ENV CERES_VERSION="1.12.0"
ENV CATKIN_WS=/root/catkin_ws


RUN   apt-get update && apt-get install -y \
      cmake \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      python-catkin-tools \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-tf && \
      rm -rf /var/lib/apt/lists/* && \
      # Build and install Ceres
      git clone https://ceres-solver.googlesource.com/ceres-solver && \
      cd ceres-solver && \
      git checkout tags/${CERES_VERSION} && \
      mkdir build && cd build && \
      cmake .. && \
      make -j install && \
      rm -rf ../../ceres-solver && \
      mkdir -p $CATKIN_WS/src/VINS-Fusion/

#RUN apt-get update && apt-get install -y --no-install-ros \
#  ${ROS_DISTRO}-recommends-realsense2-camera \
#  ros-${ROS_DISTRO}-realsense2-description \
#  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-realsense2-camera ros-${ROS_DISTRO}-realsense2-description && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install ros-${ROS_DISTRO}-rqt-common-plugins -y

RUN apt-get update && apt-get install -y --no-install-recommends \ 
  vim tmux \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rviz && rm -rf /var/lib/apt/lists/*


# Copy VINS-Fusion
#COPY ./ $CATKIN_WS/src/VINS-Fusion/
# use the following line if you only have this dockerfile
# RUN git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git

# Build VINS-Fusion
#WORKDIR $CATKIN_WS
#ENV TERM xterm
#ENV PYTHONIOENCODING UTF-8
#RUN catkin config \
#      --extend /opt/ros/${ROS_DISTRO} \
#      --cmake-args \
#        -DCMAKE_BUILD_TYPE=Release && \
#    catkin build && \
#    sed -i '/exec "$@"/i \
#            source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh
            
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc
RUN echo 'source /root/catkin_ws/devel/setup.bash' >> ~/.bashrc

## add useful commands
RUN echo 'alias vins_camera="roslaunch vins rs_camera.launch"' >> ~/.bashrc
RUN echo 'alias vins_path="rosrun vins vins_node /root/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml"' >> ~/.bashrc
RUN echo 'alias vins_loop="rosrun loop_fusion loop_fusion_node /root/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml"' >> ~/.bashrc
RUN echo 'alias vins_rviz="roslaunch vins vins_rviz.launch"' >> ~/.bashrc



RUN apt-get update && apt-get install -y \ 
  software-properties-common \
  apt-transport-https

RUN  apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

RUN apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

# Update the package list and install required dependencies
RUN apt-get update && apt-get install -y \
    ros-kinetic-octomap \
    ros-kinetic-octomap-server \
    && rm -rf /var/lib/apt/lists/*

