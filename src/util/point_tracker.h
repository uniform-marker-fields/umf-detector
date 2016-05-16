#ifndef _UMF_POINT_TRACKER_H_
#define _UMF_POINT_TRACKER_H_

#include "../defines.h"
#include "image.h"
#include <Eigen/Core>
#include <vector>

namespace umf
{

class PointTracker
{
public:
	enum {TRACK_SUCCESS, TRACK_OOB, TRACK_ITERATION, TRACK_SMALL_DET, TRACK_RESIDUE, TRACK_NOT_CORNER};
	enum {SUBSAMPLE_NO = 1, SUBSAMPLE_2 = 2, SUBSAMPLE_4 = 4};
#ifdef UMF_USE_OPENCV
	static const int SUBSAMPLE = SUBSAMPLE_NO;
#else
    static const int SUBSAMPLE = SUBSAMPLE_NO;
#endif

	PointTracker();
	~PointTracker();
	void setInitialImage(ImageGray *img);
	int trackPoints(ImageGray *img, std::vector<Eigen::Vector2f> &points);

private:
	typedef float GradientType;
	typedef unsigned char IMGType;
	typedef short IMGDiffType;

	int trackPoint(Image<IMGType, 1>  *nextImg, Image<GradientType, 1>  *nextGradX, Image<GradientType, 1>  *nextGradY,
		Image<IMGType, 1>  *prevImg, Image<GradientType, 1>  *prevGradX, Image<GradientType, 1>  *prevGradY,
		float xloc, float yloc, float *xlocout, float *ylocout);

	Image<IMGType, 1> *prevImg;
	Image<GradientType, 1> *prevGradX;
	Image<GradientType, 1> *prevGradY;

	Image<IMGType, 1> *newImg;
    Image<IMGType, 1> *subImg;
	Image<GradientType, 1> *newGradX;
	Image<GradientType, 1> *newGradY;

	int windowSize;
	float displacementThreshold;
	int maxIterations;
	float minEigenRatio;
	bool lightingInsensitive;
	float stepFactor;
	float maxResidue;

	Image<IMGDiffType, 1> *imgDiff;
	Image<GradientType, 1> *wGradX;
	Image<GradientType, 1> *wGradY;
};


}


#endif
