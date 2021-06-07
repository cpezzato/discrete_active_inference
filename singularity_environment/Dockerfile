
ARG TARGET_ROS_VERSION=melodic

ARG DEBIAN_FRONTEND=noninteractive

FROM osrf/ros:$TARGET_ROS_VERSION-desktop-full

# redeclare as FROM resets everything
ARG TARGET_ROS_VERSION=melodic
ARG WS_BASE=/retail_store_ws

# temp: use tud mirrors for ubuntu && ROS
#RUN sed -i 's|archive.ubuntu.com/ubuntu/|ftp.tudelft.nl/archive.ubuntu.com/|g' /etc/apt/sources.list && \
#    sed -i 's|packages.ros.org/ros/ubuntu|ftp.tudelft.nl/ros/ubuntu|g' /etc/apt/sources.list.d/ros1-latest.list

RUN apt-get update && apt-get install -y -q \
      cmake \
      git \
      less \
      libcanberra-gtk-module \
      mc \
      mesa-utils \
      nano \
      overlay-scrollbar \
      overlay-scrollbar-gtk2 \
      python-catkin-tools \
      python-vcstool \
      screen \
      tmux \
      tree \
      wget \
      uuid-runtime \
      dbus \
      libzmq3-dev \
      libboost-dev \
      ros-melodic-behaviortree-cpp-v3 \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# upgrade all installed ROS packages
# TODO: does this consider upgrades to pkgs that are dependencies?
RUN apt-get update && apt-get install -y -q \
      --only-upgrade \
      "ros-${ROS_VERSION}*" \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

WORKDIR $WS_BASE/src
COPY pkgs ./

# # make rosdep install all our dependencies
# RUN vcs import < retail_store_lightweight_sim.repos 

WORKDIR $WS_BASE
# make rosdep install all our dependencies
RUN apt-get update \ 
 && rosdep init || true \
 && rosdep update \
 && rosdep install --from-paths $WS_BASE/src --ignore-src --rosdistro melodic -y --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3" \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*


# TODO: use a catkin_tools (white/black list, --extend)


# build workspace and install into /opt/ros/${ROS_VERSION}

RUN bash -c "\
      source /opt/ros/$TARGET_ROS_VERSION/setup.bash && \
      catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/$TARGET_ROS_VERSION install && \
      rosclean purge -y \
    " 

# reverse what we did earlier
#RUN sed -i 's|ftp.tudelft.nl/archive.ubuntu.com/|archive.ubuntu.com/ubuntu/|g' /etc/apt/sources.list \
# && sed -i 's|ftp.tudelft.nl/ros/ubuntu|packages.ros.org/ros/ubuntu|g' /etc/apt/sources.list.d/ros1-latest.list

WORKDIR /
# we don't want this in the image, so remove it
RUN rm -r $WS_BASE

# This fixes some X issue
RUN dbus-uuidgen > /var/lib/dbus/machine-id

ENV LC_NUMERIC="en_US.UTF-8"
