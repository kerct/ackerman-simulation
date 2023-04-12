# change to current workspace
export ACKERMANN_WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
cd `echo $ACKERMANN_WS`

source devel/setup.bash

# can also use roslaunch
rosparam load "src/ackermann/config/robot.yaml"
rosrun rviz rviz -d "src/ackermann/config/rviz_config.rviz" &
rosrun ackermann solver &
rosrun ackermann visualizer &
wait
