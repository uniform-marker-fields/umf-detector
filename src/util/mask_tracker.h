#ifndef __UMF_UTIL_MASK_TRACKER_H_
#define __UMF_UTIL_MASK_TRACKER_H_

#include <Eigen/Core>
#include <vector>

namespace umf {

/**
 * Simple filter to mask out all corners that lie outside of the tracked marker.
 */
class MaskTracker
{
public:
	MaskTracker();

    //set the corners of the image region, where the marker was detected in the previous step
	void update(std::vector<Eigen::Vector2f> &corners);
	inline void disable() { this->enabled = false; }

    //filter out all points outside the tracked image region
	bool filterPoints(std::vector<Eigen::Vector2i> &points);

	void show();

private:
	Eigen::Vector2f prevPos[4];
	bool enabled;
};




}//end of namespace


#endif