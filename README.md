# Strawberry Stacker EYRC 22
This is my solution to Strawberry Stacker tasks.

Note: Will update documentation for each task.

## Installation Instructions
### General Dependencies
Before PX4, let us install some pre requisite packages.

Apt packages
```bash
  sudo apt install -y \
	ninja-build \
	exiftool \
	python3-empy \
	python3-toml \
	python3-numpy \
	python3-yaml \
	python3-dev \
	python3-pip \
	ninja-build \
	protobuf-compiler \
	libeigen3-dev \
	genromfs \
    libignition-rendering3 \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    xmlstarlet 
```
Python3 packages
```bash
pip3 install \
	pandas \
	jinja2 \
	pyserial \
	cerberus \
	pyulog \
	numpy \
	toml \
	pyquaternion \
    kconfiglib \
    --user packaging \
    --user jsonschema
```

### MAVROS Installation
MAVROS is a communication node based on MAVLink for ROS that is specially designed for communication between the drone and the companion computer. To install it, follow the following instructions:

```bash 
sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y 
```

**Step 1.** Create the workspace:
```bash
mkdir -p ~/ss_workspace/src
cd ~/ss_workspace
catkin init
wstool init src
```

**Step 2.** Install MAVLink: we use the Kinetic reference for all ROS distros as it’s not distro-specific and up to date
```bash
rosinstall_generator --rosdistro noetic mavlink | tee /tmp/mavros.rosinstall
```

**Step 3.** Install MAVROS: get source (upstream - released)
```bash
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
```
Alternatively, you can get the latest development version:
```bash
rosinstall_generator --upstream-development mavros | tee -a /tmp/mavros.rosinstall
```

**Step 4.** Create workspace & deps
```bash
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y
```
**Step 5.** Install GeographicLib datasets:
```bash
sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

**Step 6.** Build source
```bash
catkin build
```

**Step 7.** Make sure that you use setup.bash/zsh
```bash
source devel/setup.bash
```
or 
```bash
source devel/setup.zsh
```
### PX4 Firmware Installation
Clone the PX4 Repository:
```bash
cd ~/ss_workspace/src
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot/
DONT_RUN=1 make px4_sitl_default gazebo
```

Some extra tools to install:

```bash
pip3 install px4tools pymavlink
```

Build the workspace:
```bash
cd ~/ss_workspace
catkin build
```

Modifying your ‘bashrc’ or 'zshrc' so that your environment remains the same every time you open a new terminal:
```bash
source ~/ss_workspace/devel/setup.bash # Or source ~/ss_workspace/devel/setup.zsh
source ~/ss_workspace/src/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/ss_workspace/src/PX4-Autopilot/ ~/ss_workspace/src/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ss_workspace/src/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ss_workspace/src/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
```

### Strawberry Stacker
Clone the repository:
```bash
cd ~/ss_workspace/src
git clone https://github.com/Aryan-Satpathy/StrawberryStacker.git
cd ..
catkin build
```

Reopen the terminal to automatically source everything(if you followed "Modifying you 'bashrc' ..." instruction). Start any task as follows:
```bash
roslaunch task_2 task2_2.launch
```

## Latest
### Task 3.1 :
  The task just got released.

### Task 2.2 :
  Submitted.
<!--  Working code is ready. Refactoring, documentation and submission files are ready.
  
  Submission is all that is left.
-->

<!--
### Task 2.1 :
  Submitted.
-->

## Progress
*Current Task : Task 3.1*

*Work hasn't started yet, we just got the task, and I just got my laptop.*
<!--
*Current Task : Task 2.2*

*Working code is ready. Refactoring, documentation and submission files are ready.*

*Submission is all that is left.*
-->

- **Task 3.1**

    - [ ] Prepare code
    - [ ] Document the code
    - [ ] Record bag file
    - [ ] Record video
    - [ ] Get everything ready to submit
    - [ ] Submitted

- **Task 2.2**

    - [x] Prepare code
    - [x] Document the code
    - [x] Record bag file
    - [x] Record video
    - [x] Get everything ready to submit
    - [x] Submitted

- **Task 2.1**

    - [x] Prepare code
    - [x] Document the code
    - [x] Record bag file
    - [x] Record video
    - [x] Get everything ready to submit
    - [x] Submitted

- **Task 1.2**

    - [x] Prepare code
    - [x] Document the code
    - [x] Record bag file
    - [x] Record video
    - [x] Get everything ready to submit
    - [x] Submitted

- **Task 1.1**

    - [x] Prepare code
    - [x] Document the code
    - [x] Modify the code for bonus task
    - [x] Record a video as input for bonus task
    - [x] Get everything ready to submit
    - [x] Submitted

## Tasks

- **Task 3.1**

    - The gazebo world consist of a drone and a strawberry box.

    - You have to put the drone in Offboard mode and publish the positions of the drone as setpoints.

          The location of the box is 3m 0m 0m, the drone needs to do the following
          - Takeoff and the initial position to the height of 3m
          - Go to the coordinate 3m 0m 3m
          - Land on the box and pick the box
          - Takeoff at the height of 3m and go to 3m 3m 3m
          - Land at 3m 3m 0m and drop the box
          - Takeoff again to height of 3m
          - Head back to the start position ie 0m 0m 3m
          - Finally land the drone at 0m 0m 0m and then disarm
      
    - After landing on the box, you need to check that the drone is above the box in the allowable range to pick the box. To do that, you need to subscribe to the rostopic /gripper_check. If the value is true, you can now pick the box and if the value is false,the drone is not correctly placed and you need to correct the position.

    - If the result of gripper_check is true, to pick the box, you need to call the rosservice /activate_gripper and pass the value true to attach the box. If the box is attached, you will get a result from the rosservice as true. If the result is false, which means the drone is not correctly placed over the box.

    - To detach the box, you need to use the same rosservice /activate_gripper and pass the value false. This will detach the box from the drone.

- **Task 2.2**

  Drive the drone in offboard mode in a square of 10 m and land it back.

- **Task 2.1**

  Drive the drone in auto mission mode along the given waypoints.
    
      The waypoints are as follows
      - Takeoff at the home position to 10 meters
      - Go to 19.134641, 72.911706, 10  
      - Go to 19.134617, 72.911886, 10
      - Go to 19.134434, 72.911817, 10
      - Go to 19.134423, 72.911763, 10
      - Land at the last coordinate

- **Task 1.2**
  
  Detect aruco tag in Gazebo simulation and publish its id, position and yaw to the rostopic */marker_info*.

- **Task 1.1**

  Detect aruco tags from a given set of images and annotate their id, yaw and corners.
  
  **Bonus Task** : Do the same on a video having an aruco tag.
