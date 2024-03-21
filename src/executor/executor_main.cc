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

const float DISTANCE_BETWEEN_TRAJECTORY_WAYPOINTS = 1.00;
Executor* executor_ = nullptr;

std::vector<executor::waypoint> generate_random_trajectory(const waypoint& start, const waypoint& end, vector2f max_deviation_from_start) {

    std::vector<executor::waypoint> waypoints;
    std::srand(std::time(nullptr));


    bool done = false;
    waypoint current_point = start;

    while (! done) {
        float angle_offset = ((float) rand() / RAND_MAX - 0.5) * M_PI / 2;
        float angle_to_goal = (end.loc - current_point.loc).angle();
        float angle_to_next_waypoint = angle_to_goal + angle_offset;

        vector2f intermediate_loc = vector2f(current_point.loc[0] + std::cos(angle_to_next_waypoint) * DISTANCE_BETWEEN_TRAJECTORY_WAYPOINTS, 
                                             current_point.loc[1] + std::sin(angle_to_next_waypoint) * DISTANCE_BETWEEN_TRAJECTORY_WAYPOINTS);

        vector2f drift = start.loc - intermediate_loc;
        while (std::abs(drift[0]) - max_deviation_from_start[0] > 0 || std::abs(drift[1]) - max_deviation_from_start[1] > 0) {
            angle_offset = ((float) rand() / RAND_MAX - 0.5) * M_PI;
            angle_to_next_waypoint = angle_to_goal + angle_offset;
            intermediate_loc = vector2f(current_point.loc[0] + std::cos(angle_to_goal) * DISTANCE_BETWEEN_TRAJECTORY_WAYPOINTS, 
                                        current_point.loc[1] + std::sin(angle_to_goal) * DISTANCE_BETWEEN_TRAJECTORY_WAYPOINTS);
        }

        waypoint intermediate_point = waypoint(intermediate_loc, 0.0);
        current_point = intermediate_point;


        if (! waypoints.empty()) {
            waypoints.back().theta = angle_to_next_waypoint;
        }
        waypoints.push_back(intermediate_point);

        if ((end.loc - intermediate_loc).length() < DISTANCE_BETWEEN_TRAJECTORY_WAYPOINTS) {
            waypoints.push_back(end);
            done = true;
        } 
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

  waypoint start = waypoint(vector2f(0.0, 0.0), 0.0);
  waypoint end = waypoint(vector2f(5.0, 0.0), 0.0);
  vector2f bounds = vector2f(5.0, 7.0);
  auto trajectory = generate_random_trajectory(start, end, bounds);
  std::cout << "Generated Trajectory:" << std::endl;
  std::ofstream traj_file("waypoints.txt");
  for (size_t i = 0; i < trajectory.size(); ++i) {
      traj_file << trajectory[i].loc[0] << "," << trajectory[i].loc[1] << "," << trajectory[i].theta << std::endl;
  }
  traj_file.close();

    ros::Subscriber odom_sub =
        n.subscribe("/odom", 1, &OdometryCallback);

  executor_->SetTrajectory(trajectory);

  RateLoop loop(20.0);
  while (ros::ok()) {
    ros::spinOnce();
    executor_->Run();
    loop.Sleep();
  }
  return 0;
}