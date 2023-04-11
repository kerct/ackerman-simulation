#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "solver");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1, true);

  // TODO get inputs
  double l = 2;
  double w = 1;
  double speed = 1;
  double phi = 0;

  // prepare message to publish
  geometry_msgs::PoseStamped pose_rbt;
  pose_rbt.header.frame_id = "world";

  while (ros::ok()) {
    // convert quaternion to theta
    auto &q = pose_rbt.pose.orientation;
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double theta = atan2(siny_cosp, cosy_cosp);

    pose_rbt.pose.position.x += speed * cos(theta);
    pose_rbt.pose.position.y += speed * sin(theta);

    theta += speed / l * tan(theta);

    // convert theta to quaternion
    pose_rbt.pose.orientation.w = cos(theta / 2);
    pose_rbt.pose.orientation.z = sin(theta / 2);

    pose_pub.publish(pose_rbt);
    
    ROS_INFO("Pose(%7.3f, %7.3f, %7.3f)", pose_rbt.pose.position.x, pose_rbt.pose.position.y, theta);

    // update topics
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
