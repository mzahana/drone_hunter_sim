#!/bin/bash

# This scripts sets up the simulation environment of the autonomous drone hunter project
# Author: Mohamed Abdelkader, mohamedashraf123@gmail.com

echo "Setting up the simulation environment of the autonomous drone hunter project..."


# For coloring terminal output
# RED='\033[0;31m'
# NC='\033[0m' # No Color

# if [ -z "${SUDO_PASS}" ]; then
#     echo -e "${RED} Please enter the sudo passwrod as the 1st arg of this script ${NC}" && echo
#     exit 10
# fi

# if [ -z "${GIT_TOKEN}" ]; then
#     echo -e "${RED} Please enter the GIT_TOKEN as the 2nd arg of this script ${NC}" && echo
#     exit 10
# fi

#
# Install some ROS pkgs
#
sudo apt-get update \
	&& sudo apt-get -y install \
		geographiclib-tools \
		libeigen3-dev \
		libgeographic-dev \
		libopencv-dev \
		libyaml-cpp-dev \
		python3-rosdep \
		python3-catkin-tools \
		python3-catkin-lint \
		ros-$ROS_DISTRO-gazebo-ros-pkgs \
		ros-$ROS_DISTRO-mavlink \
		ros-$ROS_DISTRO-mavros \
		ros-$ROS_DISTRO-mavros-extras \
		ros-$ROS_DISTRO-octomap \
		ros-$ROS_DISTRO-octomap-msgs \
		ros-$ROS_DISTRO-pcl-conversions \
		ros-$ROS_DISTRO-pcl-msgs \
		ros-$ROS_DISTRO-pcl-ros \
		ros-$ROS_DISTRO-ros-base \
		ros-$ROS_DISTRO-rostest \
		ros-$ROS_DISTRO-rosunit \
        ros-$ROS_DISTRO-tf-conversions \
        ros-$ROS_DISTRO-rqt-tf-tree \
		ros-$ROS_DISTRO-rviz \
		xvfb \
	&& geographiclib-get-geoids egm96-5 \
	&& sudo apt-get -y autoremove \
	&& sudo apt-get clean autoclean \
	&& sudo rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

pip3 install -U osrf-pycommon

#
# Create catkin_ws at $HOME
#
if [ ! -d "$HOME/catkin_ws" ]; then
    mkdir -p $HOME/catkin_ws/src
fi

cd $HOME/catkin_ws \
    && catkin init \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin config --merge-devel \
    && catkin config --extend /opt/ros/$ROS_DISTRO \
    && catkin build

#
# source catkin_ws inside .bashrc
#
sed -i '/source ~\/catkin_ws\/devel\/setup.bash/d' $HOME/.bashrc
sed -i 's+source /opt/ros/noetic/setup.bash+source /opt/ros/noetic/setup.bash\nsource ~/catkin_ws/devel/setup.bash+g' $HOME/.bashrc

#
# Create $HOME/src for cloning and installing somce development pkgs
#
if [ ! -d "$HOME/src" ]; then
    mkdir -p $HOME/src; fi

#
# Setting up PX4 Firmware, v1.11.2
#
if [ ! -d "${HOME}/PX4-Autopilot" ]; then
    cd ${HOME} && git clone https://github.com/PX4/PX4-Autopilot.git --recursive
else
    echo "PX4-Autopilot directory already exists. Just pulling latest upstream...." \
    && cd ${HOME}/PX4-Autopilot \
    && git pull
fi
#
# Build PX4
#
cd ${HOME}/PX4-Autopilot \
    && git checkout v1.11.2  \
    && git submodule update --recursive \
    && make clean && make distclean \
    && DONT_RUN=1 make px4_sitl gazebo
    # && git submodule update --recursive

# RUN DONT_RUN=1 make px4_sitl gazebo
#
# Install xmlstarlet for Gazebo sdf models
#
sudo apt install -y xmlstarlet

#
# Link python command to python3
#
sudo apt install -y python-is-python3

#
# Export neccessary env variables
#
echo "source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default" >> ${HOME}/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/PX4-Autopilot" >> ${HOME}/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo" >> ${HOME}/.bashrc
echo "export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins" >> ${HOME}/.bashrc

source $HOME/.bashrc

# Path to the PX4 Frimware folder. It assumes that ROS is installed properly,
#  and the PX4 is in ROS_PACKAGE_PATH
PX4_PATH=$(rospack find px4)
if [ -z "${PX4_PATH}" ]; then
	echo
	echo "Could not find PX4 Firmware folder"
	echo "Setting PX4_PATH to the default PX4_PATH='${HOME}/Firmware'"
	PX4_PATH="${HOME}/PX4-Autopilot"
	echo "PX4_PATH=${PX4_PATH}"
	echo
else
	echo "Found PX4 folder at ${PX4_PATH}"
	echo
fi

# Copy the 6012_psu_iris_depth_cam PX4 startup file
if [ ! -d "${PX4_PATH}" ]; then
    echo "${RED} [ERROR] ${PX4_PATH} does not exist. It seems the PX4 Firmware is not installed in the Home directory. Exiting the setup.${NC}"
    exit 10
else
    cp $HOME/catkin_ws/src/drone_hunter_sim/config/6012_psu_iris_depth_cam ${PX4_PATH}/ROMFS/px4fmu_common/init.d-posix
    echo "6012_psu_iris_depth_cam is copied to ${PX4_PATH}/ROMFS/px4fmu_common/init.d-posix" && echo
fi
echo " " && echo "Adding drone_hunter_sim/models to GAZEBO_MODEL_PATH..." && echo " "

grep -xF 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/catkin_ws/src/drone_hunter_sim/models' ${HOME}/.bashrc || echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$HOME/catkin_ws/src/drone_hunter_sim/models" >> ${HOME}/.bashrc

# clean catkin_ws
cd $HOME/catkin_ws && catkin clean -y

# Clone packages
PKGS="drone_hunter_perception mpc_tracker trajectory_prediction custom_trajectory_msgs"
for p in $PKGS; do
    if [ ! -d "$HOME/catkin_ws/src/$p" ]; then
        echo "Didn't find $p. Cloning it..."
        cd $HOME/catkin_ws/src
        git clone https://${GIT_USER}:${GIT_TOKEN}@github.com/mzahana/$p
    else
        echo "$p is found. Pulling latest code..."
        cd $HOME/catkin_ws/src/$p && git pull
    fi
done

cd $HOME/catkin_ws/src
git clone https://${GIT_USER}:${GIT_TOKEN}@github.com/mzahana/multi_target_kf.git
cd $HOME/catkin_ws/src/multi_target_kf && git checkout const_vel_model

# Setup requirements for perception package
cd ${HOME}/catkin_ws/src/drone_hunter_perception/scripts && ./setup.sh

# Setup requirements for control package
cd ${HOME}/catkin_ws/src/mpc_tracker
cd ${HOME}/catkin_ws/src/mpc_tracker/scripts && ./setup.sh
cd 

# Build catkin_ws
cd ${HOME}/catkin_ws && catkin build

echo && echo "Execute this command:   source \$HOME/.bashrc" && echo " "

echo && echo "To run the simulation: roslaunch drone_hunter_sim run_full_system.launch" && echo

echo && echo "Setup is complete." && echo
