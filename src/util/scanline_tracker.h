#ifndef _UMF_SCANLINE_TRACKER_H__
#define _UMF_SCANLINE_TRACKER_H__

#include <vector>
#include <Eigen/Core>
#include "../model.h"
#include "line_iterator.h"
#include <iostream>

namespace umf {

/**
 * Simple marker tracking, by replacing the horizontal/vertical scanlines in the initial step
 * by scanlines that are constrained by the marker's position in the previous frame and should
 * be perpendicular to the edges inside the marker. This approach then maximizes the quality and
 * number of edges found, and also fewer scanlines can be used
 */
class ScanlineTracker
{
public:
	ScanlineTracker(int N = 10, int extraStep = 100){ //N is the maximum number of scanlines in each direction add every hundreds line a point
        this->maxCountHalf = N;
        this->enabled = false;
        this->extraStep = extraStep;
    }

    /**
     * Generate iterators that generate the sample points for scanline detection
     */
    template<class T, int CHAN>
    void getScanlines(Model<CHAN> &model, Image<T, CHAN> *img, std::vector< LineIterator< Image<T, CHAN> > > &scanlines);
	
    inline void disable() { this->enabled = false; }
    inline void enable(bool enable = true) { this->enabled = enable; }
    inline bool isEnabled() { return this->enabled; }

private:
	bool enabled;
    int maxCountHalf;
    int extraStep;
};

}

#endif