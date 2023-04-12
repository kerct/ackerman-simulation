# ackermann-simulation
Aim: to create a simulation in RViz for Ackermann steering (WORK IN PROGRESS)

## Required Packages
This code is built and tested on ROS Noetic

1. RViz
	```bash
	sudo apt-get install ros-noetic-rviz
	```
1. teleop_twist_keyboard
	```bash
	sudo apt-get install ros-noetic-teleop-twist-keyboard
	```

## Setup
```bash
# clone the repo
git clone https://github.com/kerct/ackermann-simulation.git

# make files executable
chmod +x clean_make.sh
chmod +x make.sh
chmod +x run.sh

# to build the program, simply call make.sh (or clean_make.sh for a clean build)
./make.sh
```

## Run
1. Start ROS master
	```bash
	roscore
	```
1. In another terminal, run the program
	```bash
	./run.sh
	```
1. To control the robot, run teleop
	```bash
	rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.2 _turn:=0.5
	```
