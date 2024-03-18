#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/surface_matching/icp.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>

#include "ros/ros.h"
#include "motion.h"

#include "eigen3/Eigen/Dense"

namespace executor {

typedef struct waypoint {
    Eigen::Vector2d loc;
    double theta;

    waypoint(double x, double y, double theta) : loc(x, y), theta(theta) {}
    waypoint(const Eigen::Vector2d& loc, double theta) : loc(loc), theta(theta) {}

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
        // void InterpolateTrajectoryFromWayPoints(std::vector<waypoint> waypoints);

    private:

        const double FRAME_PERIOD = 1.0 / 20.0;
        const double DRIVE_LATENCY = 0.1;

        vector2f robot_vel_;
        float robot_ang_vel_;
        vector2f odom_start_loc_;
        float odom_start_angle_;
        vector2f odom_loc_;
        float odom_angle_;
        bool odom_initialized_ = false;

        bool target_set = false;
        vector2f target_loc_;
        float target_angle_;

        vector2f trajectory_odom_start;
        float trajectory_odom_angle_start;

};



}