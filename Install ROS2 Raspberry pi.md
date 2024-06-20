# Manual Install instructions

source https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

You need to have a locale UTF-8, does not require to be en_US

### Installation

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

install software-properties to have access to ros packages

```bash
sudo apt install software-properties-common
```

Now add the ROS 2 GPG key with apt.

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository to your sources list.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo bookworm) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```


```bash
sudo apt update
sudo apt upgrade
```

### Install development tools and ROS tools

Install common packages.

```bash
sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
```

install more python requirements

```bash
sudo apt install -y \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures
```

### Get ROS 2 Code
Create a workspace and clone all repos:

Clone my custom made repository with all the CMakeLists.txt modifications to be ready to compile on armhf on raspbian (bookworm 32 bit)

```bash
git clone https://gitlab.stud.idi.ntnu.no/master-thesis-ros2-k3s/ros-2-raspbian-install.git
cd ros2_humble/
```

### Install dependencies using rosdep

ROS 2 packages are built on frequently updated debian systems. It is always recommended that you ensure your system is up to date before installing new packages.

```bash
sudo apt update
sudo apt upgrade
```

Starting rosdep that installs all the ros dependencies we specify our os with the ros distribution

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "python3-catkin-pkg-modules python3-rosdistro-modules libignition-math6-dev fastcdr rti-connext-dds-6.0.1 urdfdom_headers" --os=debian:bookworm --rosdistro humble
```

### Build the code in the workspace 

Be mindfull, you should install this in a fresh environment, make sure that ROS 2 is not sourced with the command `printenv | grep -i ROS`. The output should be empty.

Build command important to be sudo or it will fail, we also skip rviz packages there are not needed for our projects. 

If it does not work, then go to "if need of modifications" and modify accordingly.

```bash
sudo colcon build --symlink-install --packages-skip rviz_common rviz_default_plugins rviz_ogre_vendor rviz_rendering rviz_rendering_tests rviz_visual_testing_framework rviz2 --cmake-args "-DCMAKE_SHARED_LINKER_FLAGS='-latomic'" "-DCMAKE_EXE_LINKER_FLAGS='-latomic'" "-DGOOGLETEST_PATH=/usr/lib" "-DBENCHMARK_ENABLE_TESTING=OFF"
```

### Source the setup script

Set up your environment by sourcing the following file.

```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source ~/ros2_humble/install/setup.bash
```

Check using ``ros2`` command.

```bash
ros2 
```

## If need of modifications 

### Keyboard_handler

```bash
cd ~/ros2_humble/src/ros-tooling/keyboard_handler/keyboard_handler
nano CMakeLists.txt
```

modify one line in target_link_libraries and add "atomic" to it like this.
```cmake
# modify
target_link_libraries(test_keyboard_handler ${PROJECT_NAME} atomic)
```

### mimick_vendor

```bash
cd ~/ros2_humble/src/ros2/mimick_vendor
nano CMakeLists.txt
```

Replace under CMAKE_ARGS and add these lines to force it to choose armhf architecture (32 bit arm)

```cmake
# add this to CMAKE_ARGS
CMAKE_ARGS
    -DCMAKE_SYSTEM_NAME=Linux
    -DCMAKE_SYSTEM_PROCESSOR=armv7l
    -DCMAKE_C_COMPILER=arm-linux-gnueabihf-gcc
    -DCMAKE_CXX_COMPILER=arm-linux-gnueabihf-g++
```

Create a folder mimick_vendor_install because it lacks the folder for some reason

```bash
sudo mkdir ~/ros2_humble/build/mimick_vendor/mimick_vendor_install 
```

### rcutils

```bash
cd ~/ros2_humble/src/ros2/rcutils
nano CMakeLists.txt
```

Add at each target_link_libraries() "atomic" at the end like this

```cmake
# modify
target_link_libraries(test_keyboard_handler ${PROJECT_NAME} atomic)
```

