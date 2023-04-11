#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "visualizer");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  // TODO
  ros::Subscriber pose_sub = nh.subscribe("pose", 1, &cbPose);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  while (ros::ok()) {
    // ROS_INFO("visualizer running");

    // update topics
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
