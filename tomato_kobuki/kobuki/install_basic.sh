#!/usr/bin/env sh

#ref https://github.com/gaunthan/Turtlebot2-On-Melodic/blob/master/install_basic.sh


git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_msgs.git
git clone https://github.com/turtlebot/turtlebot_apps.git
git clone https://github.com/turtlebot/turtlebot_simulator.git

git clone https://github.com/yujinrobot/kobuki_msgs.git
git clone --single-branch --branch melodic https://github.com/yujinrobot/kobuki.git
mv kobuki/kobuki_description kobuki/kobuki_node kobuki/kobuki_keyop kobuki/kobuki_safety_controller kobuki/kobuki_bumper2pc ./
rm -rf kobuki

git clone --single-branch --branch melodic https://github.com/yujinrobot/kobuki_desktop.git
mv kobuki_desktop/kobuki_gazebo_plugins ./
rm -rf kobuki_desktop

git clone https://github.com/yujinrobot/yujin_ocs.git
mv yujin_ocs/yocs_cmd_vel_mux yujin_ocs/yocs_controllers yujin_ocs/yocs_velocity_smoother .
rm -rf yujin_ocs

sudo apt-get install ros-$ROS_DISTRO-kobuki-* -y
sudo apt-get install ros-$ROS_DISTRO-ecl-streams -y

# necessary for build and gazebo
git clone --single-branch --branch melodic-devel https://github.com/ros-perception/depthimage_to_laserscan.git
sudo apt install ros-$ROS_DISTRO-joy -y


if [ $ROS_DISTRO = noetic ]; then
    # 'python' -> 'python3'
    find ./ -type f -exec sed -i '1s/^#!\/usr\/bin\/env python$/#!\/usr\/bin\/env python3/' {} \;
fi
