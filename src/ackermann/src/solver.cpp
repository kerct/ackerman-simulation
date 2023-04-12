#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

double wheel_base = 2;
double wheel_separation = 1; // actually not used

double speed = 0;
double phi = 0;
void cbVel(const geometry_msgs::Twist::ConstPtr &msg) {
  // adapted from http://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots 
  speed = msg->linear.x;
  double ang_vel = msg->angular.z;

  if (speed == 0 || ang_vel == 0) {
    phi = 0;
  } else {
    double radius = speed / ang_vel;
    phi = atan(wheel_base / radius);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "solver");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  ros::Subscriber vel_cmd = nh.subscribe("cmd_vel", 1, &cbVel);
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1, true);

  // prepare message to publish
  geometry_msgs::PoseStamped pose_rbt;
  pose_rbt.header.frame_id = "world";

  while (ros::ok()) {
    // convert quaternion to theta
    auto &q = pose_rbt.pose.orientation;
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double theta = atan2(siny_cosp, cosy_cosp);

    // TODO
    double dt = 0.1;

    pose_rbt.pose.position.x += speed * cos(theta) * dt;
    pose_rbt.pose.position.y += speed * sin(theta) * dt;

    theta += speed / wheel_base * tan(theta) * dt;

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
