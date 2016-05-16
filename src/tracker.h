#ifndef _UMF_TRACKER_H_
#define _UMF_TRACKER_H_

#include "util/point_tracker.h"
#include "model.h"

namespace umf {


class UMF Tracker 
{
public:
	enum { TRACKING_SUCCESS, TRACKING_FAILURE };


    enum TRACKER_SUBSAMPLING {
        SUBSAMPLE_1 = 1,
        SUBSAMPLE_2 = 2,
        SUBSAMPLE_4 = 4
    };


    int getSubSampling() { return this->subsampling; }
    void setSubSampling(int subsampling) { this->subsampling = subsampling; }

	bool getUseRANSAC() { return this->useRansac; }
	void enableRANSAC(bool enable = true) { this->useRansac = enable; }

	Tracker();
	~Tracker();

    //should be called after the marker was successfully detected
	template<class T, int N>
    int start(Image<T, N> *img, CorrespondenceSet &correspondences, ImageGray* mask = NULL);
    
    //while tracking is active, this should be called
	template<class T, int N>
    int track(Image<T, N> *img, Model<N> *model, ImageGray *mask = NULL);

	bool isTrackingActive() const { return this->tracking; }

	void setMaxPointCount(int count) { this->maxPointCount = count; }
	int getMaxPointCount() { return this->maxPointCount;}

	void setMinPointCount(int count) { this->minPointCount = count; }
	int getMinPointCount() { return this->minPointCount;}

	bool needMorePoints() { return this->mneedMorePoints; }

private:

    int copySubsample(ImageGray *newImg);

	PointTracker ptracker;
	bool tracking; //if tracking is active
	bool useRansac; //if ransac shouldb be used to filter out outliers
	unsigned int maxPointCount; //maximum number of tracked points
	unsigned int minPointCount; //minimum number of tracked points
	bool mneedMorePoints; //flag if the tracker needs more points for tracking
    int subsampling; //subsampling in each dimension 2 means half width half height
	unsigned int windowSize; //diameter of neighbourhood in subsampled image region

    ImageGray *imgSubsampled; //subsampled cached image
};



}

#endif
