#ifndef _UMF_HOMOGRAPHY_2D_
#define _UMF_HOMOGRAPHY_2D_

#include <Eigen/Core>
#include <vector>
#include "../defines.h"

namespace umf {

/**
 * Compute the homography between pairs of points in different imagesize
 * 
 * @param src Set of points from one image
 * @param dst Corresponding points from the other image
 * @param H the resulting homography from src to dst
 */
UMF void computeHomography2d(const std::vector<Eigen::Vector2f> &src,const std::vector<Eigen::Vector2f> &dst, Eigen::Matrix3f &H);

}
#endif