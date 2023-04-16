#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

bool path_tracking;

double x, y;
void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  x = msg->pose.position.x;
  y = msg->pose.position.y;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_tracker");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  if (!nh.param("path_tracking", path_tracking, false)) {
    ROS_WARN(" TMAIN : Param path_tracking not found, set to false");
  }

  if (!path_tracking) {
    return 0;
  }

  ros::Subscriber pose_sub = nh.subscribe("pose", 1, &cbPose);
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

  geometry_msgs::Twist twist_rbt;

  while (ros::ok()) {
    twist_rbt.linear.x = 0.01;
    twist_rbt.angular.z = 0.01;

    twist_pub.publish(twist_rbt);

    // update topics
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
