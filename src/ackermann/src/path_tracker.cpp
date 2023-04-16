#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

bool path_tracking;

struct Position {
  double x;
  double y;
};

Position curr_pos;
void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  curr_pos.x = msg->pose.position.x;
  curr_pos.y = msg->pose.position.y;
}

double dist_euc(Position p1, Position p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return sqrt(dx * dx + dy * dy);
}

double heading(Position src, Position tgt) {
  double dx = tgt.x - src.x;
  double dy = tgt.y - src.y;
  return atan2(dy, dx);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_tracker");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  if (!nh.param("path_tracking", path_tracking, false)) {
    ROS_WARN("PATH_TRACKER: Param path_tracking not found, set to false");
  }

  if (!path_tracking) {
    return 0;
  }

  double close_enough;
  if (!nh.param("close_enough", close_enough, 0.1)) {
    ROS_WARN("PATH_TRACKER: Param close_enough not found, set to 0.1");
  }

  ros::Subscriber pose_sub = nh.subscribe("pose", 1, &cbPose);
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("target_path", 1, true);

  // create path to track
  nav_msgs::Path nav_path;
  nav_path.header.frame_id = "world";
  geometry_msgs::PoseStamped path_pose;
  std::vector<Position> path;
  double dx = 0.5;
  for (int i = 0; i < 10; i++) {
    Position pt;
    pt.x = (dx * i);
    pt.y = (pt.x) * (pt.x);
    path.push_back(pt);

    path_pose.pose.position.x = pt.x;
    path_pose.pose.position.y = pt.y;
    nav_path.poses.push_back(path_pose);
  }

  path_pub.publish(nav_path);

  geometry_msgs::Twist twist_rbt;

  int idx = 0;
  int total_pts = path.size();

  while (ros::ok()) {
    if (path.empty()) {
      ROS_WARN("PATH_TRACKER: Path to track is empty");
      return 0;
    }

    if (dist_euc(curr_pos, path[idx]) < close_enough) {
      idx++;
      if (idx == total_pts) {
        ROS_INFO("PATH_TRACKER: Path completed");
        return 0;
      }
    }

    // simple proportional controller, needs more work
    twist_rbt.linear.x = dist_euc(curr_pos, path[idx]) * 0.2;
    twist_rbt.angular.z = heading(curr_pos, path[idx]) * 0.1;

    twist_pub.publish(twist_rbt);

    // update topics
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
