# ros-2-raspbian-install
Cloned repository of ros 2 source with modification to be able to compile from source on a raspbian (debian bookworm) distribution

## Requirements

- python 3
- git
- raspberry pi with raspbian 32 bit (debian:bookworm)

## To start 

To start the installation, clone the modified ros project.

```bash
cd ~/
mkdir install
cd install/
git clone https://gitlab.stud.idi.ntnu.no/master-thesis-ros2-k3s/ros-2-raspbian-install.git
cd ros-2-raspbian-install/
```

To install and build all the dependencies for the ros 2 project use the custom shell script ``install.sh``. If 
the installation does not work for any reason check [here](Install%20ROS2%20Raspberry%20pi.md#if-need-of-modifications)

```bash
chmod +x install.sh
./install.sh
```

## To remove

```bash
cd ~/
sudo rm -r ros-2-raspbian-install
```
