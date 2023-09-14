#!/usr/bin/bash
#
#
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade

echo "\n\n\n"
echo "Select installation type(int)"
echo "\t1) desktop - ROS + RViz + demos + tutorials"
echo "\t2) base - communication libraries + message packages + command line tools"
echo "\t3) development - compilers and tools for building ROS packages"

read installtype

if [[ "$installtype" -eq 1 ]]; then
    sudo apt install ros-humble-desktop

elif [[ "$installtype" -eq 2 ]]; then
    sudo apt install ros-humble-ros-base

elif [[ "$installtype" -eq 3 ]]; then
    sudo apt install ros-dev-tools

else
    echo "erroneous argument"
fi

# install colcon
sudo apt install python3-colcon-common-extensions

echo "NOTE: your next step should be testing the installation with 'source /opt/ros/humble/setup.sh'"
echo "\t* select the correct version of setup.* for your shell. setup.zsh and setup.bash are also available"
