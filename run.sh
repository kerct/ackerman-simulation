# change to current workspace
export ACKERMANN_WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
cd `echo $ACKERMANN_WS`

source devel/setup.bash

rosrun ackermann solver &
rosrun rviz rviz &
wait
