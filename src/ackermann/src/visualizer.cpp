#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

bool newPose = false;
double x, y;
void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  x = msg->pose.position.x;
  y = msg->pose.position.y;
  newPose = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "visualizer");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  ros::Subscriber pose_sub = nh.subscribe("pose", 1, &cbPose);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "world";
  line_strip.ns = "trajectory";
  line_strip.id = 0;
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.03; // line width
  // blue colour
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  while (ros::ok()) {
    line_strip.header.stamp = ros::Time::now();

    if (newPose) {
      geometry_msgs::Point p;
      p.x = x;
      p.y = y;
      line_strip.points.push_back(p);

      marker_pub.publish(line_strip);
      newPose = false;
    }

    // update topics
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
