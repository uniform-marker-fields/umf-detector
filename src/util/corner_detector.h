#ifndef __UMF_CORNER_DETECTOR_H
#define __UMF_CORNER_DETECTOR_H

#include <Eigen/Core>
#include <vector>
#include "image.h"

namespace umf {


/**
 * Search for an edge point between two modules of the marker. We can effectively find such a point
 * based on the assumption, that the modules are unichrome.
 * 
 * @param image The input image
 * @param edge The output edge location in image pixels
 * @param left The first module center
 * @param right The other module center
 * @param searchThreshold The minimum intensity difference between modules
 * @param angleThreshold The edge direction between modules should be parallel to the line joining the module centers. The dot product should be smaller than this
 * @return if an edge was found
 */
template<class T, int NCHAN>
bool binSearchEdge(Image<T, NCHAN> *image,
                   Eigen::Vector2i &edge,
                   Eigen::Vector2i left,
                   Eigen::Vector2i right,
                   const int searchThreshold = 0,
				   const float angleThreshold = -1.f); 


/**
 * Find a corner using only the module information - Find 4 edge points between modules. The intersection of lines joining 
 * two opposite edge points are selected as corner points
 *
 * @param srcImg The input image
 * @param points 4 module centers
 * @param cornerType See marker.h for corner types
 * @param intensityThreshold the intensity threshold between neighbor modules (used by binSearchEdge)
 * @param binSearchDotThreshold See binSearchEdge angleThreshold
 * @param angleThreshold the minimum dot product between edge direction and the line joining neighbor points (should be parallel)
 * @return The corner coordinates
 */
template<class T, int NCHAN>
Eigen::Vector2f findMarkerCorner(Image<T, NCHAN> *srcImg, std::vector<Eigen::Vector2f> &points, int cornerType, const int intensityThreshold, const float binsearchDotThreshold, const float angleThreshold);

/**
 * find the corners with subpixel precision
 *
 * @param srcImg The source image
 * @param points The vector of corners that should be refined with sub-pixel precision. If the search failed, the point is set to (-1,-1)
 * @param windowSizeHalf The radius of the region considered for sub-pixel search
 * @return the number of successfully refined points
 */
template<class T, int NCHAN>
int findCornersSubpixel(Image<T, NCHAN> *srcImg, std::vector<Eigen::Vector2f> &points, const unsigned int windowSizeHalf = 8);


template<class fptype, class T, int NCHAN>
void getSobelResponseRGB(Image<T, NCHAN> *input, int x, int y, fptype &h, fptype &v);


template<int p, class T, int NCHAN>
void getSobelResponseRGB(Image<T, NCHAN> *input, int x, int y, fixedpoint::fixed_point<p> &h, fixedpoint::fixed_point<p> &v);


template<class T, int NCHAN>
void getSobelResponseMax(Image<T, NCHAN> *input, int x, int y, float &h, float &v);

}
#endif // CORNER_DETECTOR_H
