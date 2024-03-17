
#include "motion.h"
#include "executor.h"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "shared/ros/ros_helpers.h"


namespace {
    ros::Publisher drive_pub_;
    amrl_msgs::AckermannCurvatureDriveMsg drive_msg_;
} // namespace

namespace executor {

Executor::Executor(ros::NodeHandle* n) {
    drive_pub_ = n->advertise<amrl_msgs::AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
    ros_helpers::InitRosHeader("base_link", &drive_msg_.header);
}

void Executor::UpdateOdometry(const Eigen::Vector2f& loc,
                                float angle,
                                const Eigen::Vector2f& vel,
                                float ang_vel) {
//   robot_omega_ = ang_vel;
//   robot_vel_ = vel;
//   if (!odom_initialized_) {
//     odom_start_angle_ = angle;
//     odom_start_loc_ = loc;
//     odom_initialized_ = true;
//     prev_odom_loc_ = loc;
//     odom_loc_ = loc;
//     odom_angle_ = angle;
//     return;
//   }
//   odom_loc_ = loc;
//   odom_angle_ = angle;
    cout << "Loc: " << "(" << loc.x() << " " << loc.y() << ")" << " Angle: " << angle << endl;
}


// void Executor::InterpolateTrajectoryFromWayPoints(std::vector<waypoint> waypoints) {

//     std::vector<double> x, y
//     for (auto p : waypoints) {
//         x.push_back(p.loc.x());
//         y.push_back(p.loc.y());
//     }

//     spline_traj = gsl_spline_alloc(gsl_interp_cspline, waypoints.size());
//     gsl_spline_init(spline_traj, x.data(), y.data(), waypoints.size());
// }

void Executor::Run2dTOC(const vector2f& target_loc, const float& target_angle) {

    vector2f robot_loc(0, 0);
    // Starting robot location.
    vector2f start_loc(0, 0);
    // Current robot angle in the trajectory coordinates.
    float robot_angle(0);
    // Starting robot angle.
    float start_angle(0);

    // Get current pose from odometry.
    // cobot->getOdometryLocation(robot_loc, robot_angle);
    // Set starting pose to starting odometry pose.
    start_loc = trajectory_odom_start;
    start_angle = trajectory_odom_angle_start;


    robot_loc = (robot_loc - start_loc).rotate(-start_angle);
    robot_angle = angle_diff(robot_angle, start_angle);

    // Current robot velocity in the trajectory coordinates.
    vector2f robot_vel(0, 0);
    // Current robot angular velocity.
    float robot_ang_vel(0);

    // cobot->getOdometryVelocity(robot_vel, robot_ang_vel);

    // Rotate vel to trajectory coorinates.
    robot_vel = robot_vel.rotate(robot_angle);


    const AccelLimits& trans_limit = AccelLimits(1.0, 1.0, 0.5);
    const AccelLimits& ang_limit = AccelLimits(1.0, 1.0, 0.5);

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
    float ang_cmd = CalcMotion1D(
        ang_limit,
        FRAME_PERIOD + DRIVE_LATENCY,
        ang_delta,
        robot_ang_vel,
        0.0,
        angle_time,
        NULL);
    std::cout << ang_cmd << endl;
}

void Executor::Run() {
    drive_msg_.velocity = 0.5;
    drive_msg_.curvature = 0;
    drive_msg_.header.stamp = ros::Time::now();
    drive_pub_.publish(drive_msg_);



}





} // namespace executor
