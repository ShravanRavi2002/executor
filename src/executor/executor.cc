
#include <math.h>

#include "motion.h"
#include "executor.h"
#include "cobot_msgs/CobotDriveMsg.h"
#include "sensor_msgs/LaserScan.h"
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
  // cout << "Loc: " << "(" << loc.x() << " " << loc.y() << ")" << " Angle: " << angle << endl;
}

void Executor::SetTrajectory(std::vector<waypoint> trajectory) {
  // std::cout << "Trajectory set with " << trajectory.size() << " waypoints." << std::endl;
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
    // cobot->getOdometryLocation(robot_loc, robot_angle);
    robot_loc = odom_loc_;
    robot_angle = odom_angle_;
    // Set starting pose to starting odometry pose.
    start_loc = trajectory_odom_start_;
    start_angle = trajectory_odom_angle_start_;


    robot_loc = (robot_loc - start_loc).rotate(-start_angle);
    robot_angle = angle_diff(robot_angle, start_angle);
    // cout << "Start angle: " <<  start_angle << endl;

    // Current robot velocity in the trajectory coordinates.
    vector2f robot_vel(0, 0);
    // Current robot angular velocity.
    float robot_ang_vel(0);

    // cobot->getOdometryVelocity(robot_vel, robot_ang_vel);
    robot_vel = robot_vel_;
    robot_ang_vel = robot_ang_vel_;

    // Rotate vel to trajectory coorinates.
    robot_vel = robot_vel.rotate(robot_angle);


    const AccelLimits& trans_limit = AccelLimits(1.0, 2.5, 0.5);
    const AccelLimits& ang_limit = AccelLimits(1 * M_PI, 1.5 * M_PI,  M_PI / 4);

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

void Executor::DumpStateToFile(const std::vector<Eigen::Vector2f>& cloud, cv::Matx44d T, vector2f robot_loc, float robot_angle, vector2f loc_factor, float angle_factor) {
   std::ofstream scan_file("scan.csv", std::ios::app);
   for (const auto& point : cloud) {
        scan_file << "(" << point[0] << "," << point[1] << ") ";
    }
    scan_file << endl;
    scan_file.close();

    std::ofstream transformation_file("transformation.csv", std::ios::app);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            transformation_file << T(i, j) << ",";
        }
    }
    transformation_file << endl;
    transformation_file.close();

    std::ofstream loc_file("location.csv", std::ios::app);
    loc_file << robot_loc[0] <<  "," << robot_loc[1] <<  "," << robot_angle << "," << prev_key_frame_loc_[0] <<  "," << prev_key_frame_loc_[1] << "," <<  prev_key_frame_angle_ << endl;
    loc_file.close();

    std::ofstream factors("factors.csv", std::ios::app);
    factors << loc_factor[0] << "," << loc_factor[1] << "," << angle_factor << "," << odom_loc_[0] << ","  << odom_loc_[1] << "," << odom_angle_ << "," << cur_node_idx << endl;
    factors.close();
}

cv::Mat Transform2DLidarToOpenCVWithNormals(const std::vector<Eigen::Vector2f> & cloud, float z) {
  cv::Mat pc(cloud.size(), 3, CV_32F);
  for (uint i = 0; i < cloud.size(); i++)
  { 
    auto p = cloud[i];
      
    pc.at<float>(i, 0) = p[0];
    pc.at<float>(i, 1) = p[1];
    pc.at<float>(i, 2) = z; 
  }

  cv::Mat pc_with_normals;
  int num_neighbors = 10;
  bool flip_viewpoint = false;
  cv::Vec3f viewpoint(0, 0, 0);

  cv::ppf_match_3d::computeNormalsPC3d(pc, pc_with_normals, num_neighbors, flip_viewpoint, viewpoint);
  return pc_with_normals;
}

void Executor::ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                                   float time) {

  point_cloud_ = cloud;

  if (prev_key_frame_scan_.size() == 0) {
    prev_key_frame_scan_ = cloud;
    start_to_prev_key_frame = cv::Matx44d::eye();
    return;
  }

  vector2f relative_loc = odom_loc_ - prev_key_frame_loc_;
  float relative_angle = angle_diff(odom_angle_, prev_key_frame_angle_) * 180 / CV_PI;
  // cout << "Translation: " << t.norm() << " Angle: " << abs(angle) << " Error: " << error << endl;
  if (relative_loc.length() > 0.15 || relative_angle > 1000.0) {

    std::vector<Eigen::Vector2f> cur_scan;
    std::vector<Eigen::Vector2f> prev_scan;
    for (size_t i = 0; i < cloud.size(); i++) {
      if (prev_key_frame_scan_[i].norm() < 20 && cloud[i].norm() < 20) {
        prev_scan.push_back(prev_key_frame_scan_[i]);
        cur_scan.push_back(cloud[i]);
      }
    }

    cout << "Bounded Scan Size: " << prev_scan.size() << endl;

   
    cv::Mat src = Transform2DLidarToOpenCVWithNormals(prev_scan, 0.0f);
    cv::Mat dst = Transform2DLidarToOpenCVWithNormals(cur_scan, 0.1f);

    double error;
    cv::Matx44d T;
    icp_solver_->registerModelToScene(src, dst, error, T);
    T(2, 3) -= 0.1;

    // float angle = std::atan2(T(1, 0), T(0, 0));
    Eigen::Vector2f t(T(0, 3), T(1, 3));


    vector2f relative_loc = odom_loc_ - prev_key_frame_loc_;
    float relative_angle = angle_diff(odom_angle_, prev_key_frame_angle_);
    cout << relative_loc[0] << " " << relative_loc[1] << endl;
    DumpStateToFile(cloud, T, odom_loc_, odom_angle_, relative_loc, relative_angle);
    gtsam::Pose2 relative_motion_from_icp = gtsam::Pose2(-t.x(), -t.y(), relative_angle);
    cout << "Adding Constraint: " << relative_motion_from_icp << endl;
    pose_graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(cur_node_idx, cur_node_idx + 1, relative_motion_from_icp, odometry_noise_));

    vector2f lidar_displacement = vector2f(std::cos(odom_angle_) * 0.17, std::sin(odom_angle_) * 0.17);
    relative_loc = relative_loc + lidar_displacement;
    raw_odometry_.insert(cur_node_idx + 1, gtsam::Pose2(relative_loc[0], relative_loc[1], relative_angle));
    cur_node_idx++;

    prev_key_frame_scan_ = cloud;
    prev_key_frame_loc_ = odom_loc_;
    prev_key_frame_angle_ = odom_angle_;
    start_to_prev_key_frame = start_to_prev_key_frame * T;
  }
}


void Executor::OptimizePoseGraph() {
  // Step 1: Optimize the Factor Graph
    gtsam::LevenbergMarquardtOptimizer optimizer(pose_graph_, raw_odometry_);
    gtsam::Values optimizedEstimate = optimizer.optimize();

    // Step 2: Extract Optimized Poses
    // Assuming the poses are of type Pose2
    // You need to modify this according to your pose type
    // Pose2 optimizedPoseCur = optimizedEstimate.at<Pose2>(cur_node_idx);
    // Pose2 optimizedPoseNext = optimizedEstimate.at<Pose2>(cur_node_idx + 1);

    // Step 3: Write Results to File
    std::ofstream outputFile("optimized_poses.csv");
    if (outputFile.is_open()) {
        // Write optimized poses to the file

        for (int i = 0; i < cur_node_idx; i++) {
          outputFile << optimizedEstimate.at<gtsam::Pose2>(i) << endl;
        }
        outputFile.close();
    } else {
        // cout << "Unable to open file for writing!" << endl;
    }

    // std::ofstream outputGraphFile("optimized_graph.txt");
    optimizedEstimate.print("Optimized Values:");
    pose_graph_.print("Pose Graph");

}

void Executor::Run() {

    if (!trajectory_started_) {
      if (! (odom_initialized_ && trajectory_set_)) return;
      trajectory_odom_angle_start_ = odom_angle_;
      trajectory_odom_start_ = odom_loc_;
      trajectory_index_ = 0;
      trajectory_started_ = true;
      cur_node_idx = 0;
      raw_odometry_.insert(cur_node_idx, gtsam::Pose2(odom_loc_[0], odom_loc_[1], odom_angle_));
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
      cout << "finished trajectory" << endl;
      robot_loc_file.close();
      OptimizePoseGraph();
    }

    waypoint target = trajectory_[trajectory_index_];

    robot_loc_file << relative_loc[0] << "," << relative_loc[1] << "," << relative_angle << std::endl;
    std::vector<float> cmds = Run2dTOC(target.loc, target.theta);


    drive_msg_.velocity_x = cmds[0];
    drive_msg_.velocity_y = cmds[1];
    drive_msg_.velocity_r = cmds[2];
    drive_msg_.transMaxAcceleration = 1.0;
    drive_msg_.transMaxDeceleration = 1.0;
    drive_msg_.transMaxVelocity = 1.0;
    drive_msg_.rotMaxAcceleration = 3 * M_PI;
    drive_msg_.rotMaxDeceleration = 1.5 * M_PI;
    drive_msg_.rotMaxVelocity = 3 * M_PI;

    drive_msg_.header.stamp = ros::Time::now();
    drive_pub_.publish(drive_msg_);
}





} // namespace executor
