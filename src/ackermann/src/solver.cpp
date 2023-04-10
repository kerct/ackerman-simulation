#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "solver");
  ros::NodeHandle nh;

  // TODO
  ros::Publisher pose_pub = nh.advertise<std_msgs::String>("pose", 1, true);

  // ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // pose_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
