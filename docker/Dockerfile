# Multi-stage Dockerfile for Rover Software
# Base stage contains all common dependencies and setup
# Development and production stages extend from base with specific configurations

ARG L4T_VERSION_MAJOR
ARG L4T_VERSION_MINOR
ARG L4T_VERSION_PATCH
ARG L4T_VERSION=${L4T_VERSION_MAJOR}.${L4T_VERSION_MINOR}.${L4T_VERSION_PATCH}

#####################################
###         BASE STAGE            ###
#####################################

FROM nvcr.io/nvidia/l4t-jetpack:r${L4T_VERSION} AS base

# Avoid prompts with apt-get
ARG DEBIAN_FRONTEND=noninteractive

###################################
###         Install ROS         ###
###################################

# Update system locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

# Set Python3 as default
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

# 
RUN apt install software-properties-common -y && \
    add-apt-repository universe
    
# Add the ROS 2 GPG key with apt.
RUN apt update && apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list.
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

RUN apt update && apt upgrade -y && \
    apt install ros-humble-desktop -y

# Install pip3 
RUN apt-get update && apt-get install -y python3-pip

#####################################
###         Install PyTorch       ###
#####################################

# Install OpenBLAS and other dependencies for PyTorch
RUN apt-get update && apt-get install -y \
    libopenblas-base \
    libopenblas-dev \
    libopenblas0-pthread \
    libomp-dev && \
    ln -sf /usr/lib/aarch64-linux-gnu/libopenblas.so /usr/lib/aarch64-linux-gnu/libopenblas.so.0

ARG PYTORCH_URL=https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
ARG PYTORCH_WHL=torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
RUN wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${PYTORCH_URL} -O ${PYTORCH_WHL} && \
    pip3 install --no-cache-dir --verbose ${PYTORCH_WHL} && \
    rm ${PYTORCH_WHL}

# Fix NumPy compatibility issue with PyTorch - install specific compatible version
RUN pip3 install --force-reinstall numpy==1.24.3

###################################
###     Install cuSPARSELt      ###
###################################

ARG cuSPARSELt_URL=https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb
ARG cuSPARSELt_DEB=cuda-keyring_1.1-1_all.deb

RUN wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${cuSPARSELt_URL} -O ${cuSPARSELt_DEB} && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    rm ${cuSPARSELt_DEB} && \
    apt-get update && \
    apt-get -y install libcusparselt0 libcusparselt-dev

#####################################
###            ZED-SDK            ###
#####################################

ARG ZED_SDK_MAJOR
ARG ZED_SDK_MINOR
ARG ZED_SDK_PATCH
ARG ZED_SDK_VERSION=${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}.${ZED_SDK_PATCH}
ARG L4T_VERSION_MAJOR
ARG L4T_VERSION_MINOR

# print URL for ZED SDK download
RUN echo "Downloading ZED SDK version ${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}.${ZED_SDK_PATCH} for L4T version ${L4T_VERSION_MAJOR}.${L4T_VERSION_MINOR}.${L4T_VERSION_PATCH}"
RUN apt-get update || true ; apt-get install --no-install-recommends lsb-release wget less udev zstd sudo apt-transport-https build-essential cmake -y && \
     echo "# R${L4T_VERSION} (release), REVISION: ${L4T_VERSION}" > /etc/nv_tegra_release ; \
     wget -q --no-check-certificate -O ZED_SDK_Linux.run https://download.stereolabs.com/zedsdk/${ZED_SDK_VERSION}/l4t${L4T_VERSION_MAJOR}.${L4T_VERSION_MINOR}/jetsons && \
     chmod +x ZED_SDK_Linux.run ; ./ZED_SDK_Linux.run silent skip_tools skip_drivers && \
     rm -rf /usr/local/zed/resources/* \
     rm -rf ZED_SDK_Linux.run && \
     rm -rf /var/lib/apt/lists/*

RUN python3 /usr/local/zed/get_python_api.py

#This symbolic link is needed to use the streaming features on Jetson inside a container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

#####################################
###   ZED and UBlox ROS2 Wrapper  ###
#####################################

# ZED ROS2 Wrapper dependencies version
ARG XACRO_VERSION=2.0.8
ARG DIAGNOSTICS_VERSION=4.0.0
ARG AMENT_LINT_VERSION=0.12.11
ARG ROBOT_LOCALIZATION_VERSION=3.5.3
ARG ZED_MSGS_VERSION=5.0.0
ARG NMEA_MSGS_VERSION=2.0.0
ARG ANGLES_VERSION=1.15.0
ARG GEOGRAPHIC_INFO_VERSION=1.0.6
ARG POINTCLOUD_TRANSPORT_VERSION=1.0.18
ARG POINTCLOUD_TRANSPORT_PLUGINS_VERSION=1.0.11
ARG RMW_CYCLONEDDS_VERSION=1.3.4
ARG BACKWARD_ROS_VERSION=1.0.7

ARG ROS2_WRAPPER_FOLDER
ARG ZED_ROS2_WRAPPER_BRANCH
ARG ZED_ROS2_WRAPPER_URL

RUN apt-get update && apt-get install -y git python3-colcon-common-extensions lsb-release wget less udev sudo build-essential cmake python3 python3-dev python3-pip python3-wheel git jq libpq-dev zstd usbutils python3-rosdep libasio-dev
RUN mkdir -p ${ROS2_WRAPPER_FOLDER}/src && \
   git clone --recurse-submodule --branch ${ZED_ROS2_WRAPPER_BRANCH} ${ZED_ROS2_WRAPPER_URL} ${ROS2_WRAPPER_FOLDER}/src

WORKDIR ${ROS2_WRAPPER_FOLDER}/src
RUN wget https://github.com/ros/xacro/archive/refs/tags/${XACRO_VERSION}.tar.gz -O - | tar -xvz && mv xacro-${XACRO_VERSION} xacro && \
    wget https://github.com/ros/diagnostics/archive/refs/tags/${DIAGNOSTICS_VERSION}.tar.gz -O - | tar -xvz && mv diagnostics-${DIAGNOSTICS_VERSION} diagnostics && \
    wget https://github.com/ament/ament_lint/archive/refs/tags/${AMENT_LINT_VERSION}.tar.gz -O - | tar -xvz && mv ament_lint-${AMENT_LINT_VERSION} ament-lint && \
    wget https://github.com/cra-ros-pkg/robot_localization/archive/refs/tags/${ROBOT_LOCALIZATION_VERSION}.tar.gz -O - | tar -xvz && mv robot_localization-${ROBOT_LOCALIZATION_VERSION} robot-localization && \
    wget https://github.com/stereolabs/zed-ros2-interfaces/archive/refs/tags/${ZED_MSGS_VERSION}.tar.gz -O - | tar -xvz && mv zed-ros2-interfaces-${ZED_MSGS_VERSION} zed-ros2-interfaces && \
    wget https://github.com/ros-drivers/nmea_msgs/archive/refs/tags/${NMEA_MSGS_VERSION}.tar.gz -O - | tar -xvz && mv nmea_msgs-${NMEA_MSGS_VERSION} nmea_msgs && \
    wget https://github.com/ros/angles/archive/refs/tags/${ANGLES_VERSION}.tar.gz -O - | tar -xvz && mv angles-${ANGLES_VERSION} angles && \
    wget https://github.com/ros-perception/point_cloud_transport/archive/refs/tags/${POINTCLOUD_TRANSPORT_VERSION}.tar.gz -O - | tar -xvz && mv point_cloud_transport-${POINTCLOUD_TRANSPORT_VERSION} point_cloud_transport && \
    wget https://github.com/ros-perception/point_cloud_transport_plugins/archive/refs/tags/${POINTCLOUD_TRANSPORT_PLUGINS_VERSION}.tar.gz -O - | tar -xvz && mv point_cloud_transport_plugins-${POINTCLOUD_TRANSPORT_PLUGINS_VERSION} point_cloud_transport_plugins && \
    wget https://github.com/ros2/rmw_cyclonedds/archive/refs/tags/${RMW_CYCLONEDDS_VERSION}.tar.gz -O - | tar -xvz && mv rmw_cyclonedds-${RMW_CYCLONEDDS_VERSION} rmw_cyclonedds && \
    wget https://github.com/ros-geographic-info/geographic_info/archive/refs/tags/${GEOGRAPHIC_INFO_VERSION}.tar.gz -O - | tar -xvz && mv geographic_info-${GEOGRAPHIC_INFO_VERSION} geographic-info && \
    wget https://github.com/pal-robotics/backward_ros/archive/refs/tags/${BACKWARD_ROS_VERSION}.tar.gz -O - | tar -xvz && mv backward_ros-${BACKWARD_ROS_VERSION} backward_ros && \
    cp -r geographic-info/geographic_msgs/ . && \
    rm -rf geographic-info

RUN python3 -m pip install --upgrade cython

# Clone the ublox-zedf9p repository for ublox zed f9p support
RUN git clone --branch master https://github.com/AAU-Space-Robotics/ros2-ublox-zedf9p.git

WORKDIR ${ROS2_WRAPPER_FOLDER}

RUN rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
  colcon build --parallel-workers $(nproc) --symlink-install \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' \
  ' --no-warn-unused-cli' "

###################################
###         pip packages         ###
###################################

# Install pip and Python packages
RUN apt-get update && apt-get install -y python3-pip && \
    pip3 install setuptools Cython wheel colcon-common-extensions canopen Jetson.GPIO

RUN apt-get install busybox && apt-get install iproute2 -y && \
    apt-get install net-tools -y

RUN pip3 install tritonclient[all]

#####################################
###         Install EXTRAS       ###
#####################################

RUN apt-get update && apt-get install -y ros-humble-rosbridge-server ros-humble-rmw-zenoh-cpp ros-humble-twist-mux

# Clean up
RUN rm -rf /var/lib/apt/lists/*

#####################################
###      DEVELOPMENT STAGE        ###
#####################################

FROM base AS development

# Copy unified entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

#####################################
###      PRODUCTION STAGE         ###
#####################################

FROM base AS production

###################################
###    Copy and Build Rover SW   ###
###################################

# Create workspace directory
RUN mkdir -p /home/workspace

# Copy the source code
COPY ./src /home/workspace/src

# Set working directory
WORKDIR /home/workspace

# Install dependencies for the rover software
RUN /bin/bash -c "apt-get update && \
    source /opt/ros/humble/setup.bash && \
    source ${ROS2_WRAPPER_FOLDER}/install/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble"

# Build the rover software
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source ${ROS2_WRAPPER_FOLDER}/install/setup.bash && \
    colcon build --parallel-workers $(nproc) \
    --event-handlers console_direct+ \
    --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
    ' --no-warn-unused-cli'"

# Copy unified entrypoint script
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
