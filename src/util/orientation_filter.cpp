#include "orientation_filter.h"
#include "corner_detector.h"
#include "../edgel_detector.h"
#include <Eigen/Core>
#include <cmath>
#include <iostream>

#ifndef M_PI_2
#define M_PI_2 1.5707963267948966192313216916398
#endif

#ifndef M_PI_4
#define M_PI_4 0.78539816339744830961566084581988
#endif

namespace umf {


OrientationFilter::OrientationFilter()
{
    this->enabled = false;
    this->threshold = static_cast<float> (cos(M_PI_2 - M_PI_2/6));
    float directions[2];
    directions[0] = static_cast<float> (M_PI_4);
    directions[1] = static_cast<float> (3*M_PI_4);
    this->update(directions);
}

void OrientationFilter::update(float directions[])
{
    this->mainAngles[0][0] = cos(directions[0]);
    this->mainAngles[0][1] = sin(directions[0]);
    this->mainAngles[1][0] = cos(directions[1]);
    this->mainAngles[1][1] = sin(directions[1]);
}

template <class T, int NCHAN>
bool OrientationFilter::filterPoints(Image<T,NCHAN> *img, std::vector<Eigen::Vector2i> &points)
{
    if(!this->enabled)
    {
        return this->enabled;
    }

    for(int pindex = (int) points.size() - 1; pindex >= 0; pindex--)
    {
        float h,v;
        getSobelResponseRGB(img, points[pindex][0], points[pindex][1], h, v);
        Eigen::Vector2f normal(v, h);
        normal.normalize();

        if(!(std::abs(this->mainAngles[0].dot(normal)) < threshold
                 || std::abs(this->mainAngles[1].dot(normal)) < threshold))
        {
            points.erase(points.begin() + pindex);
        }

    }

    return this->enabled;
}

bool OrientationFilter::filterEdgels(std::vector<Edgel> &edgels)
{
    if(!this->enabled)
    {
        return this->enabled;
    }

    for(int pindex = (int) edgels.size() - 1; pindex >= 0; pindex--)
    {
        Edgel &curre = edgels[pindex];
        Eigen::Vector2f normal(curre.line[0], curre.line[1]);
        normal.normalize();

        float p1 = std::abs(this->mainAngles[0].dot(normal));
        float p2 = std::abs(this->mainAngles[1].dot(normal));

        if(!(p1 < threshold
                 || p2 < threshold))
        {
            edgels.erase(edgels.begin() + pindex);
        }
    }

    return this->enabled;
}

//instancing

template bool OrientationFilter::filterPoints(ImageRGB *img, std::vector<Eigen::Vector2i> &points);
template bool OrientationFilter::filterPoints(ImageGray *img, std::vector<Eigen::Vector2i> &points);


}//end of namespace
