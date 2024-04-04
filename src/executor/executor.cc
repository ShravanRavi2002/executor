
#include <math.h>

#include "motion.h"
#include "executor.h"
#include "cobot_msgs/CobotDriveMsg.h"
#include "shared/ros/ros_helpers.h"


namespace {
    ros::Publisher drive_pub_;
    cobot_msgs::CobotDriveMsg drive_msg_;
} // namespace

namespace executor {

Executor::Executor(ros::NodeHandle* n) {
    drive_pub_ = n->advertise<cobot_msgs::CobotDriveMsg>(
      "/Cobot/Drive", 1);
    ros_helpers::InitRosHeader("drive", &drive_msg_.header);
}


void DumpStateToFile(const std::vector<Eigen::Vector2f>& cloud, vector2f robot_loc, float robot_angle, vector2f cart_loc, float cart_angle) {
   std::ofstream scan_file("scan.csv", std::ios::app);
   for (const auto& point : cloud) {
        scan_file << "(" << point[0] << "," << point[1] << ") ";
    }
    scan_file << endl;
    scan_file.close();

    std::ofstream loc_file("location.csv", std::ios::app);
    loc_file << robot_loc[0] <<  "," << robot_loc[1] <<  "," << robot_angle << "," << cart_loc[0] <<  "," << cart_loc[1] << "," <<  cart_angle << endl;
    loc_file.close();
}



void Executor::UpdateCartographerOdometry(const Eigen::Vector2f& loc, float angle) {
  cart_loc_ = vector2f(loc.x(), loc.y());
  cart_angle_ = angle;
}

void Executor::UpdateOdometry(const Eigen::Vector2f& loc,
                                float angle,
                                const Eigen::Vector2f& vel,
                                float ang_vel) {
  robot_ang_vel_ = ang_vel;
  robot_vel_ = vector2f(vel.x(), vel.y());
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = vector2f(loc.x(), loc.y());
    odom_initialized_ = true;
    odom_loc_ = vector2f(loc.x(), loc.y());
    odom_angle_ = angle;

    trajectory_odom_start_ = odom_start_loc_;
    trajectory_odom_angle_start_ = odom_start_angle_;
    robot_loc_file.open("trajectory_loc.csv");
    return;
  }
  odom_loc_ = vector2f(loc.x(), loc.y());
  odom_angle_ = angle;
}

void Executor::ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                                   float time) {

  point_cloud_ = cloud;

  if (prev_key_frame_scan_.size() == 0) {
    prev_key_frame_scan_ = cloud;
    return;
  }

  vector2f relative_loc = odom_loc_ - prev_key_frame_loc_;
  float relative_angle = angle_diff(odom_angle_, prev_key_frame_angle_) * 180 / CV_PI;
  if (relative_loc.length() > 0.15 || relative_angle > 1000.0) {
    DumpStateToFile(cloud, odom_loc_, odom_angle_, cart_loc_, cart_angle_);
  }
}



void Executor::SetTrajectory(std::vector<waypoint> trajectory) {
  trajectory_ = trajectory;
  trajectory_set_ = true;
}

std::vector<float> Executor::Run2dTOC(const vector2f& target_loc, const float& target_angle) {

    vector2f robot_loc(0, 0);
    // Starting robot location.
    vector2f start_loc(0, 0);
    // Current robot angle in the trajectory coordinates.
    float robot_angle(0);
    // Starting robot angle.
    float start_angle(0);

    // Get current pose from odometry.
    
    if (use_cart_localization_) {
      robot_loc = cart_loc_;
      robot_angle = cart_angle_;
      cout << "cart localization" << endl;
    } else {
      robot_loc = odom_loc_;
      robot_angle = odom_angle_;
      start_loc = trajectory_odom_start_;
      start_angle = trajectory_odom_angle_start_;
      robot_loc = (robot_loc - start_loc).rotate(-start_angle);
      robot_angle = angle_diff(robot_angle, start_angle);
    }
    

    // Current robot velocity in the trajectory coordinates.
    vector2f robot_vel(0, 0);
    // Current robot angular velocity.
    float robot_ang_vel(0);

    // cobot->getOdometryVelocity(robot_vel, robot_ang_vel);
    robot_vel = robot_vel_;
    robot_ang_vel = robot_ang_vel_;

    // Rotate vel to trajectory coorinates.
    robot_vel = robot_vel.rotate(robot_angle);


    const AccelLimits& trans_limit = AccelLimits(1.0, 2.5, 0.75);
    const AccelLimits& ang_limit = AccelLimits(3 * M_PI, 1.5 * M_PI, 3 * M_PI);

    // Angular velocity command for the next frame period.

    // Translation velocity command for the next frame period.
    vector2f trans_cmd(0, 0);

    // Compute next translation velocity command.
    // The time it will take for the translation motion to the next waypoint.
    double trans_time(0);
    // The frame in which the near time-optimal control motion profile is
    // computed.
    vector2f frame(1,0);
    trans_cmd = CalcMotion2D(
        trans_limit,
        FRAME_PERIOD + DRIVE_LATENCY,
        robot_loc,
        robot_vel,
        target_loc,
        vector2f(0,0),
        trans_time,
        frame,
        NULL,
        NULL);

    // Compute next angular velocity command.
    // The time it will take for the angular motion to the next waypoint.
    double angle_time(0);
    // The delta between the current and the target angle.
    float ang_delta = angle_diff(target_angle, robot_angle);
    // cout << "ang delta: " << ang_delta << endl;
    float ang_cmd = CalcMotion1D(
        ang_limit,
        FRAME_PERIOD + DRIVE_LATENCY,
        ang_delta,
        robot_ang_vel,
        0.0,
        angle_time,
        NULL);

    // If there is a time difference of more than a frame period between
  // translation and angular motions, slow down the faster of the two until the
  // times match.
  const bool relax_trans = (trans_time < angle_time - FRAME_PERIOD);
  const bool relax_angle = (angle_time < trans_time - FRAME_PERIOD);
  if (false && (relax_angle || relax_trans)) {
    AccelLimits scaled_trans_limit = trans_limit;
    AccelLimits scaled_ang_limit = ang_limit;
    static const float ScaleStep = 0.05;
    for (float scale = 1.0;
        scale > 0.0 &&
        ((relax_trans && (trans_time < angle_time - FRAME_PERIOD)) ||
        (relax_angle && (angle_time < trans_time - FRAME_PERIOD)));
        scale -= ScaleStep) {
      if (relax_angle) {
        scaled_ang_limit = ang_limit * scale;
        ang_cmd = CalcMotion1D(
            scaled_ang_limit,
            FRAME_PERIOD + DRIVE_LATENCY,
            ang_delta,
            robot_ang_vel,
            0.0,
            angle_time,
            NULL);
      } else if (relax_trans) {
        scaled_trans_limit = trans_limit * scale;
        trans_cmd = CalcMotion2D(
            scaled_trans_limit,
            FRAME_PERIOD + DRIVE_LATENCY,
            robot_loc,
            robot_vel,
            target_loc,
            vector2f(0,0),
            trans_time,
            frame,
            NULL,
            NULL);
      }
    }
  }
  trans_cmd = trans_cmd.rotate(-robot_angle);
    std::vector<float> ret{trans_cmd[0], trans_cmd[1], ang_cmd};
    return ret;
}

void Executor::Run() {

    if (!trajectory_started_) {
      if (! (odom_initialized_ && trajectory_set_)) return;
      trajectory_odom_angle_start_ = odom_angle_;
      trajectory_odom_start_ = odom_loc_;
      trajectory_index_ = 0;
      trajectory_started_ = true;
    }

    vector2f relative_loc = odom_loc_ - trajectory_odom_start_;
    float relative_angle = angle_diff(odom_angle_, trajectory_odom_angle_start_);

    while (trajectory_index_ < trajectory_.size() &&
      (relative_loc - trajectory_[trajectory_index_].loc).sqlength() <
      sq(WAYPOINT_LOC_THRESHOLD) &&
      angle_dist(relative_angle, trajectory_[trajectory_index_].theta) <
      WAYPOINT_ANGLE_THRESHOLD) {

      trajectory_index_++;
    }

    if (trajectory_index_ == trajectory_.size()) {
      trajectory_started_ = false;
      trajectory_set_ = false;
      robot_loc_file.close();
    }

    waypoint target = trajectory_[trajectory_index_];

    std::cout << "Current Loc: " << "(" << relative_loc[0] << " " << relative_loc[1] << ")" << " Angle: " << relative_angle  << std::endl;

    robot_loc_file << relative_loc[0] << "," << relative_loc[1] << "," << relative_angle << std::endl;

    std::cout << "Goal Loc: " << "(" << (target.loc - relative_loc)[0] << " " << (target.loc - relative_loc)[1] << ")" << " Angle: " << angle_diff(target.theta, relative_angle)  << std::endl;
    std::vector<float> cmds = Run2dTOC(target.loc, target.theta);

    std::cout << "Cmds: " << "x: " << cmds[0] << " y: " << cmds[1] << " t: " << cmds[2] << std::endl;
    std::cout << "Target Loc: " << "(" << target.loc[0] << " " << target.loc[1] << ")" << " Angle: " << target.theta << std::endl;

    float min_accuated_vel = 0.2;
    if (cmds[0] < min_accuated_vel && cur_spin_index_ % 2 == 0) {
      cmds[0] *= 2;
      cmds[0] = std::max(cmds[0], min_accuated_vel);
    }
    if (cmds[1] < min_accuated_vel && cur_spin_index_ % 2 == 0) {
      cmds[1] *= 2;
      cmds[1] = std::max(cmds[0], min_accuated_vel);
    }
    if (cmds[2] < min_accuated_vel && cur_spin_index_ % 2 == 0) {
      cmds[2] *= 2;
      cmds[2] = std::max(cmds[0], min_accuated_vel);
    }

    drive_msg_.velocity_x = cmds[0];
    drive_msg_.velocity_y = cmds[1];
    drive_msg_.velocity_r = cmds[2];
    drive_msg_.transMaxAcceleration = 1.0;
    drive_msg_.transMaxDeceleration = 1.0;
    drive_msg_.transMaxVelocity = 1.5;
    drive_msg_.rotMaxAcceleration = 3 * M_PI;
    drive_msg_.rotMaxDeceleration = 1.5 * M_PI;
    drive_msg_.rotMaxVelocity = 3 * M_PI;

    drive_msg_.header.stamp = ros::Time::now();
    drive_pub_.publish(drive_msg_);
    cur_spin_index_++;
}





} // namespace executor
