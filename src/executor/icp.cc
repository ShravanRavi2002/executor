#include <iostream>
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

#include "icp.h"

using namespace Eigen;

// Calculate the least-squares best-fit transform
std::tuple<MatrixXd, MatrixXd, VectorXd> best_fit_transform(const MatrixXd& A, const MatrixXd& B) {
    // Assert shapes are same
    assert(A.rows() == B.rows() && A.cols() == B.cols());

    int m = A.cols(); // Number of dimensions

    // Translate points to their centroids
    VectorXd centroid_A = A.colwise().mean();
    VectorXd centroid_B = B.colwise().mean();
    MatrixXd AA = A.rowwise() - centroid_A.transpose();
    MatrixXd BB = B.rowwise() - centroid_B.transpose();

    // Compute the rotation matrix
    MatrixXd H = AA.transpose() * BB;
    JacobiSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);
    MatrixXd U = svd.matrixU();
    MatrixXd V = svd.matrixV();
    MatrixXd R = V * U.transpose();

    // Special reflection case
    if (R.determinant() < 0) {
        V.col(m - 1) *= -1;
        R = V * U.transpose();
    }

    // Compute translation
    VectorXd t = centroid_B - R * centroid_A;

    // Homogeneous transformation
    MatrixXd T = MatrixXd::Identity(m + 1, m + 1);
    T.block(0, 0, m, m) = R;
    T.block(0, m, m, 1) = t;

    return std::make_tuple(T, R, t);
}

// Find the nearest neighbor in dst for each point in src
std::pair<VectorXd, VectorXi> nearest_neighbor(const MatrixXd& src, const MatrixXd& dst) {
    assert(src.cols() == dst.cols());

    int N = src.rows();
    VectorXd distances(N);
    VectorXi indices(N);

    // Iterate over each point in src
    for (int i = 0; i < N; ++i) {
        double min_dist = std::numeric_limits<double>::infinity();
        int min_index = -1;

        // Calculate distance to each point in dst
        for (int j = 0; j < dst.rows(); ++j) {
            double dist = (src.row(i) - dst.row(j)).norm();
            if (dist < min_dist) {
                min_dist = dist;
                min_index = j;
            }
        }

        // Save the nearest neighbor distance and index
        distances(i) = min_dist;
        indices(i) = min_index;
    }

    return std::make_pair(distances, indices);
}

// The Iterative Closest Point method
MatrixXd icp(const MatrixXd& A, const MatrixXd& B) {
    const MatrixXd& init_pose = MatrixXd(); 
    int max_iterations = 1000;
    double tolerance = 0.0001;
    
    assert(A.rows() == B.rows() && A.cols() == B.cols());

    int m = A.cols(); // Number of dimensions

    // Make points homogeneous
    MatrixXd src(m + 1, A.rows());
    src.topRows(m) = A.transpose();
    src.row(m).setOnes();

    MatrixXd dst(m + 1, B.rows());
    dst.topRows(m) = B.transpose();
    dst.row(m).setOnes();

    // Apply initial pose estimation
    if (!init_pose.isZero())
        src = init_pose * src;

    double prev_error = 0;

    for (int i = 0; i < max_iterations; ++i) {
        // Find the nearest neighbors between the current source and destination points
        auto [distances, indices] = nearest_neighbor(src.topRows(m).transpose(), dst.topRows(m).transpose());

        // Compute the transformation between the current source and nearest destination points
        auto [T, R, t] = best_fit_transform(src.topRows(m).transpose(), dst.topRows(m).transpose());

        // Update the current source
        src = T * src;

        // Check error
        double mean_error = distances.mean();
        if (std::abs(prev_error - mean_error) < tolerance)
            break;
        prev_error = mean_error;
    }

    // Calculate final transformation
    auto [T, R, t] = best_fit_transform(A, src.topRows(m).transpose());

    return T;
}

// int main() {
//     // Example usage
//     MatrixXd A(3, 3);
//     A << 1, 2, 3,
//          4, 5, 6,
//          7, 8, 9;

//     MatrixXd B(3, 3);
//     B << 9, 8, 7,
//          6, 5, 4,
//          3, 2, 1;

//     auto [T, distances, iterations] = icp(A, B);

//     std::cout << "Transformation Matrix T:\n" << T << std::endl;
//     std::cout << "Euclidean Distances of Nearest Neighbors:\n" << distances << std::endl;
//     std::cout << "Number of Iterations: " << iterations << std::endl;

//     return 0;
// }