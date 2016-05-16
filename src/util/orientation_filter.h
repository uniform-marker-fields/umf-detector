#ifndef _UMFUTIL_ORIENTATION_FILTER_H
#define _UMFUTIL_ORIENTATION_FILTER_H

#include <Eigen/Core>
#include <vector>
#include "image.h"
#include "../defines.h"

namespace umf {

/**
 * Simple edgel and edge pixel filter based on the two angles (in radians) provided in the update function.
 * These angles should correspond to the marker's orientation. (the angle between the x axis and the edge orientation)
 * Useble for mobil phones - the orientations can be guessed based on the gravity direction and the assumed marker orientation.
 */
class UMF OrientationFilter
{
public:
    OrientationFilter();

    void update(float directions[2]);
    inline void enable(bool enable = true) { this->enabled = enable; }

    template<class T, int NCHAN>
    bool filterPoints(Image<T, NCHAN> *img, std::vector<Eigen::Vector2i> &points);
    bool filterEdgels(std::vector<Edgel> &edgels);

private:
    Eigen::Vector2f mainAngles[2];
    bool enabled;
    float threshold;
};




}//end of namespace

#endif // ORIENTATION_FILTER_H
