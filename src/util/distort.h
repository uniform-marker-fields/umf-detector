#ifndef __UMF_DISTORT_H
#define __UMF_DISTORT_H
#include <Eigen/Core>
#include <vector>

namespace umf {

/**
 * Undistort 2D points based on internal camera matrix and distortion coefficients
 *
 * @param _src The input array of projected points 
 * @param _dst The undistorted points are stored here
 * @param _cameraMatrix The 3x3 internal matrix
 * @param _distCoeffs The distortion coefficients (at most 8 are considered)
 */
void undistortPoints( std::vector<Eigen::Vector2f> &_src,
                      std::vector<Eigen::Vector2f> &_dst,
                      const Eigen::Matrix3f &_cameraMatrix,
                      const Eigen::VectorXf &_distCoeffs );

/**
 * Distort 2D points based on internal camera matrix and distortion coefficients
 *
 * @param _src The input array of undistorted points 
 * @param _dst The distorted points are stored here
 * @param _cameraMatrix The 3x3 internal matrix
 * @param _distCoeffs The distortion coefficients (at most 8 are considered)
 */
void distortPoints(std::vector<Eigen::Vector2f> &src,
                   std::vector<Eigen::Vector2f> &dst,
                   const Eigen::Matrix3f &cameraMatrix,
                   const Eigen::VectorXf &distCoeffs );

}
#endif // DISTORT_H
