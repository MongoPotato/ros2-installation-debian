#!/bin/bash

locale  # check for UTF-8

sudo apt update && sudo apt install -y locales

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common -y

#Now add the ROS 2 GPG key with apt.
sudo apt update
sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Then add the repository ros ubuntu to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo bookworm) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
# Install common packages.

sudo apt update
sudo apt upgrade -y

sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools



# install more python requirements
sudo apt-get install -y \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures

sleep 5

sudo apt update
sudo apt upgrade -y

mv ./ros2_humble ~/

cd ~/ros2_humble

sudo rosdep init
sudo rosdep update
sudo rosdep install --from-paths src --ignore-src -y --skip-keys "python3-catkin-pkg-modules python3-rosdistro-modules libignition-math6-dev fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-" --os=debian:bookworm --rosdistro humble

if ! sudo colcon build --symlink-install --packages-skip rviz_common rviz_default_plugins rviz_ogre_vendor rviz_rendering rviz_rendering_tests rviz_visual_testing_framework rviz2 --cmake-args "-DCMAKE_SHARED_LINKER_FLAGS='-latomic'" "-DCMAKE_EXE_LINKER_FLAGS='-latomic'" "-DGOOGLETEST_PATH=/usr/lib" "-DBENCHMARK_ENABLE_TESTING=OFF"; then
    echo "Build failed, creating directory and retrying..."
    sudo mkdir -p ~/ros2_humble/build/mimick_vendor/mimick_vendor_install
    sudo colcon build --symlink-install --packages-skip rviz_common rviz_default_plugins rviz_ogre_vendor rviz_rendering rviz_rendering_tests rviz_visual_testing_framework rviz2 --cmake-args "-DCMAKE_SHARED_LINKER_FLAGS='-latomic'" "-DCMAKE_EXE_LINKER_FLAGS='-latomic'" "-DGOOGLETEST_PATH=/usr/lib" "-DBENCHMARK_ENABLE_TESTING=OFF"
fi