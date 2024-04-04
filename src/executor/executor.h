#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/surface_matching/icp.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>

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
        void UpdateCartographerOdometry(const Eigen::Vector2f& loc,
                                float angle);
        waypoint FetchNextGoal();
        void SetTrajectory(std::vector<waypoint> trajectory);
        void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                                   float time);

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

        vector2f cart_loc_;
        float cart_angle_;
        bool use_cart_localization_ = false;

        vector2f prev_key_frame_loc_;
        float prev_key_frame_angle_;
        std::vector<Eigen::Vector2f> prev_key_frame_scan_;

        std::vector<waypoint> trajectory_;
        size_t trajectory_index_;

        std::vector<Eigen::Vector2f> point_cloud_;

        int cur_spin_index_ = 0;

        std::ofstream robot_loc_file;

};



}