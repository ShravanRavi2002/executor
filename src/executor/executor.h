#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/surface_matching/icp.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>


#include "ros/ros.h"
#include "motion.h"

#include "eigen3/Eigen/Dense"

namespace executor {

typedef struct waypoint {
    vector2f loc;
    float theta;

    waypoint(float x, float y, float theta) : loc(x, y), theta(theta) {}
    waypoint(const vector2f& loc, float theta) : loc(loc), theta(theta) {}

} waypoint;

class Executor {
    public:
    
        Executor(ros::NodeHandle* n);
        void Run();
        std::vector<float> Run2dTOC(const vector2f& target_loc, const float& target_angle);
        void UpdateOdometry(const Eigen::Vector2f& loc,
                                float angle,
                                const Eigen::Vector2f& vel,
                                float ang_vel);
        waypoint FetchNextGoal();
        void SetTrajectory(std::vector<waypoint> trajectory);
        void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud, float time);
        void OptimizePoseGraph();
        void DumpStateToFile(const std::vector<Eigen::Vector2f>& cloud, cv::Matx44d T, vector2f robot_loc, float robot_angle, vector2f prev_key_frame_loc, float prev_key_frame_angle);
    
        // void InterpolateTrajectoryFromWayPoints(std::vector<waypoint> waypoints);

    private:

        const double FRAME_PERIOD = 1.0 / 20.0;
        const double DRIVE_LATENCY = 0.1;
        const double WAYPOINT_LOC_THRESHOLD = 0.25;
        const double WAYPOINT_ANGLE_THRESHOLD = 0.25 * M_PI;


        vector2f robot_vel_;
        float robot_ang_vel_;
        vector2f odom_start_loc_;
        float odom_start_angle_;
        vector2f odom_loc_;
        float odom_angle_;
        bool odom_initialized_ = false;

        bool trajectory_set_ = false;
        bool trajectory_started_ = false;
        vector2f trajectory_odom_start_;
        float trajectory_odom_angle_start_;

        std::vector<waypoint> trajectory_;
        size_t trajectory_index_;

        std::vector<Eigen::Vector2f> point_cloud_;
        std::vector<Eigen::Vector2f> prev_key_frame_scan_;
        vector2f prev_key_frame_loc_;
        float prev_key_frame_angle_;
        cv::Matx44d start_to_prev_key_frame;
        cv::ppf_match_3d::ICP* icp_solver_ = new cv::ppf_match_3d::ICP(1000, 0.01);

        std::ofstream robot_loc_file;

        gtsam::NonlinearFactorGraph pose_graph_;
        gtsam::Values raw_odometry_;
        gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));
        int cur_node_idx = 1;
};



}