FROM ros:kinetic-robot

MAINTAINER Paul Harter <paul@earthrover.cc>

# python build stuff
RUN apt-get update -y
RUN apt-get -y install autoconf automake wget
RUN apt-get install -y git python python-dev python-pip build-essential

# some apt packages
RUN apt-get install -y libgps-dev libyaml-cpp-dev ros-kinetic-geographic-msgs

## make a folder to work in
RUN mkdir -p /opt/earth-rover/ws/src
WORKDIR /opt/earth-rover/ws/src

# build GeographicLib from source
RUN wget https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.49.tar.gz/download
RUN tar xfpz download
RUN cd GeographicLib-1.49 && mkdir BUILD
RUN cd GeographicLib-1.49/BUILD && cmake .. && make && make install

# a dependency on gps_umd
RUN git clone --recursive --depth=1 https://github.com/swri-robotics/gps_umd.git

# a dependency on Piksi RTK GPS module
RUN git clone --recursive --depth=1 https://github.com/ethz-asl/ethz_piksi_ros.git

# and robot_localisation
RUN git clone --recursive --depth=1 https://github.com/cra-ros-pkg/robot_localization.git

WORKDIR /opt/earth-rover/ws

# update and install the dependencies' dependencies
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && rosdep update"
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && rosdep install --from-paths src/ --ignore-src --rosdistro kinetic"

# build the dependencies
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"

# install the dependencies
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic install"

# python deps for piksi and xsens
RUN pip install pyserial
RUN pip install numpy
RUN pip install sbp
RUN pip install zope

# add the earth_rover_localization source code
COPY earth_rover_localization src/earth_rover_localization

# add the earth_rover_localization lib submodules
COPY libs src/libs

# build earth_rover_localization
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=\"piksi_multi_rtk_ros;piksi_rtk_msgs;xsens_driver;GeographicLib;gps_common;xsens_msgs;geographic_lib;catkin_simple;earth_rover_localization\""

# install earth_rover_localization
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic install"

# clean up
RUN rm -rf devel
RUN rm -rf build

