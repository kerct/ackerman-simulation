# change to current workspace
export ACKERMANN_WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
cd `echo $ACKERMANN_WS`

source devel/setup.bash

# TODO use roslaunch?
rosrun rviz rviz &
rosrun ackermann solver &
rosrun ackermann visualizer &
wait
