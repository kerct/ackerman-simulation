# ackermann-simulation
Aim: to create a simulation in RViz for Ackermann steering (WORK IN PROGRESS)

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
