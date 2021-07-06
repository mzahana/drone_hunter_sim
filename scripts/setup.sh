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



# Path to the PX4 Frimware folder. It assumes that ROS is installed properly,
#  and the PX4 is in ROS_PACKAGE_PATH
PX4_PATH=$(rospack find px4)
if [ -z "${PX4_PATH}" ]; then
	echo
	echo "Could not find PX4 Firmware folder"
	echo "Setting PX4_PATH to the default PX4_PATH='${HOME}/Firmware'"
	PX4_PATH="${HOME}/Firmware"
	echo "PX4_PATH=${PX4_PATH}"
	echo
else
	echo "Found PX4 folder at ${PX4_PATH}"
	echo
fi

# Copy the 10017_psu_drone PX4 startup file
if [ ! -d "${PX4_PATH}" ]; then
    echo "${RED} [ERROR] ${PX4_PATH} does not exist. It seems the PX4 Firmware is not installed in the Home directory. Exiting the setup.${NC}"
    exit 10
else
    cp $HOME/catkin_ws/src/drone_hunter_sim/config/10017_psu_drone ${PX4_PATH}/ROMFS/px4fmu_common/init.d-posix
    echo "10017_psu_drone is copied to ${PX4_PATH}/ROMFS/px4fmu_common/init.d-posix" && echo
fi
echo " " && echo "Adding drone_hunter_sim/models to GAZEBO_MODEL_PATH..." && echo " "

grep -xF 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/catkin_ws/src/drone_hunter_sim/models' ${HOME}/.bashrc || echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$HOME/catkin_ws/src/drone_hunter_sim/models" >> ${HOME}/.bashrc



# Cloning the perception package
if [ ! -d "${HOME}/catkin_ws/src/drone_hunter_perception" ]; then
    echo "Didn't find drone_hunter_perception. Cloning it..."
    cd ${HOME}/catkin_ws/src/
    git clone https://${GIT_TOKEN}@github.com/riotu-lab/drone_hunter_perception.git
else
    echo "drone_hunter_perception is found. Pulling latest code..."
    cd ${HOME}/catkin_ws/src/drone_hunter_perception
    git pull
fi

# Setup requirements for perception package
cd ${HOME}/catkin_ws/src/drone_hunter_perception/scripts && ./setup.sh ${SUDO_PASS} ${GIT_TOKEN}

# Cloning control package
if [ ! -d "${HOME}/catkin_ws/src/drone_hunter_control" ]; then
    echo "Didn't find drone_hunter_control. Cloning it..."
    cd ${HOME}/catkin_ws/src/
    git clone https://${GIT_TOKEN}@github.com/riotu-lab/drone_hunter_control.git
else
    echo "drone_hunter_control is found. Pulling latest code..."
    cd ${HOME}/catkin_ws/src/drone_hunter_control
    git pull
fi

# Setup requirements for control package
cd ${HOME}/catkin_ws/src/drone_hunter_control && git checkout mpc_tracker
cd ${HOME}/catkin_ws/src/drone_hunter_control/scripts && ./setup.sh

# Build catkin_ws
cd ${HOME}/catkin_ws && catkin build

echo && echo "Execute this command:   source \$HOME/.bashrc" && echo " "

echo && echo "To run the basic simulation: roslaunch drone_hunter_sim basic_sim.launch" && echo

echo && echo "Setup is complete." && echo