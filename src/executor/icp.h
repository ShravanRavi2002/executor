
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <cassert>
#include <random>
#include <chrono>
#include <tuple>

Eigen::MatrixXd icp(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);