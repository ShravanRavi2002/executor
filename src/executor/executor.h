#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/surface_matching/icp.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>

#include "ros/ros.h"
#include "motion.h"

#include <Eigen/Dense>
#include <gsl/gsl_spline.h>

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
        void Run2dTOC(const vector2f& target_loc, const float& target_angle);
        void UpdateOdometry(const Eigen::Vector2f& loc,
                                float angle,
                                const Eigen::Vector2f& vel,
                                float ang_vel);
        // void InterpolateTrajectoryFromWayPoints(std::vector<waypoint> waypoints);

    private:

        const double FRAME_PERIOD = 1.0 / 20.0;
        const double DRIVE_LATENCY = 0.1;

        vector2f trajectory_odom_start;
        double trajectory_odom_angle_start;
        gsl_spline* spline_traj;

};



}