#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "ros/ros.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/timer.h"
#include "nav_msgs/Odometry.h"

#include "executor.h"

using namespace executor;

const double DISTANCE_BETWEEN_TRAJECTORY_WAYPOINTS = 0.50;
Executor* executor_ = nullptr;

std::vector<executor::waypoint> generate_straight_trajectory(Eigen::Vector2d start, Eigen::Vector2d end) {

    std::vector<executor::waypoint> waypoints;

    // Compute the direction vector from start to end
    Eigen::Vector2d direction = end - start;
    double total_distance = direction.norm();
    direction.normalize();

    // Calculate the number of intermediate points
    int num_points = static_cast<int>(total_distance / DISTANCE_BETWEEN_TRAJECTORY_WAYPOINTS);

    // Generate intermediate points along the straight line
    for (int i = 0; i <= num_points; i++) {
        Eigen::Vector2d intermediate_point = start + direction * (DISTANCE_BETWEEN_TRAJECTORY_WAYPOINTS * i);
        executor::waypoint pose(intermediate_point.x(), intermediate_point.y(), 0.0); // Assuming zero orientation for simplicity
        waypoints.push_back(pose);
    }

    return waypoints;
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  executor_->UpdateOdometry(
      Eigen::Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y),
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
      Eigen::Vector2f(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
      msg.twist.twist.angular.z);
}


int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  // Initialize ROS.
  ros::init(argc, argv, "executor", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;

  executor_ = new Executor(&n);

  auto trajectory = generate_straight_trajectory(Eigen::Vector2d(0, 0), Eigen::Vector2d(10.0, 0));
  std::cout << "Generated Trajectory:" << std::endl;
    for (size_t i = 0; i < trajectory.size(); ++i) {
        std::cout << "Pose " << i << ": x = " << trajectory[i].loc.x() << ", y = " << trajectory[i].loc.y() << ", theta = " << trajectory[i].theta << std::endl;
    }

    ros::Subscriber odom_sub =
        n.subscribe("/odom", 1, &OdometryCallback);

  RateLoop loop(20.0);
  while (ros::ok()) {
    ros::spinOnce();
    executor_->Run();
    loop.Sleep();
  }
  return 0;
}