
#include "tracker.h"
#include "util/homography2d.h"
#include "util/umfdebug.h"
#include "util/native_x.h"
#include "util/corner_detector.h"
#include <list>

namespace umf {

Tracker::Tracker()
{
	this->tracking = false;
	this->mneedMorePoints = false;
	this->useRansac = false;

	this->imgSubsampled = NULL;
	this->subsampling = SUBSAMPLE_2;
	this->windowSize = 8;

#ifdef UMF_ANDROID
	this->maxPointCount = 50;
	this->minPointCount = 25;
#else
	this->maxPointCount = 200;
	this->minPointCount = 50;
#endif
}

Tracker::~Tracker()
{
}

template<>
int Tracker::track(ImageRGB *img, Model<3> *model, ImageGray *mask)
{
	//TODO implement
	assert(false);
	return TRACKING_FAILURE;
}


template<>
int Tracker::track(ImageGray *img, Model<1> *model, ImageGray *mask)
{
	int status = TRACKING_FAILURE;
	this->tracking = false;

	//first we should filter out the correspondences based on the reprojection error
	CorrespondenceSet &c = model->getCorrespondences();

	std::sort(c.begin(), c.end(), corrSortByScore);

	std::vector<Eigen::Vector2f> points;
	std::vector<Eigen::Vector3f> models;
	std::vector<float> scores;
	
	points.reserve(c.size());
	models.reserve(c.size());
	scores.reserve(c.size());
	unsigned int counter = 0;
    for(CorrespondenceSet::iterator it = c.begin(); it != c.end(); it++)
	{
		if(counter >= this->maxPointCount)
		{
			break;
        }
        if(mask && *(mask->get2D(it->px, it->py)) != 255){
            continue;
        }
		points.push_back(Eigen::Vector2f(it->px / subsampling, it->py / subsampling));
		models.push_back(Eigen::Vector3f(it->mx, it->my, it->mz));
		scores.push_back(it->score);

        counter++;
	}
#ifdef UMF_DEBUG_TIMING
	UMFDebug *dbg = UMFDSingleton::Instance();
	int logid = dbg->logEventStart();

#endif

    this->copySubsample(img);
    int pointCount = this->ptracker.trackPoints(this->imgSubsampled, points);

	pointCount = findCornersSubpixel(this->imgSubsampled, points, this->windowSize / this->subsampling);

#ifdef UMF_DEBUG_TIMING
    dbg->logEventEnd(logid, "TRP");
#endif
	if(pointCount < 4)
	{
		this->tracking = false;
		return TRACKING_FAILURE;
	}

	c.clear();

	for(unsigned int i = 0; i < counter; i++)
	{
		if(points[i][0] <= 0 && points[i][1] <= 0)
		{
			continue;
		}
        c.push_back(Correspondence(points[i][0]*subsampling, points[i][1]*subsampling, models[i][0], models[i][1], models[i][2], scores[i]));
	}
	status = TRACKING_SUCCESS;

	if(status == TRACKING_SUCCESS)
	{
		this->tracking = true;
	}

	if (this->useRansac)
	{
		//filter simply by homography calculations
		const unsigned int SET_SIZE = 6;

		if (c.size() < SET_SIZE)
		{
			this->mneedMorePoints = true;
			return status;
		}

		std::vector<Eigen::Vector2f> srcPoints(SET_SIZE);
		std::vector<Eigen::Vector2f> dstPoints(SET_SIZE);
		Eigen::Matrix3f H;
		const int RANSAC_ITERATIONS = 10;
		const float DISTANCE_THRESHOLD = 0.04f;
		const int MIN_INLIER_COUNT = c.size() - 2;
		int index = 0, inliers = 0;
		CorrespondenceSet bestSet;
		std::vector<int> bestOutliers;
		int bestInliers = -1;
		std::vector<int> outliers;
		for (int iter = 0; iter < RANSAC_ITERATIONS; iter++, index += SET_SIZE)
		{
			inliers = 0;
			outliers.clear();
			if (index + SET_SIZE >= c.size())
			{
				std::random_shuffle(c.begin(), c.end());
				index = 0;
			}
			for (unsigned int i = 0; i < SET_SIZE; i++)
			{
				srcPoints[i][0] = c[i + index].px;
				srcPoints[i][1] = c[i + index].py;
				dstPoints[i][0] = c[i + index].mx;
				dstPoints[i][1] = c[i + index].my;
			}
			computeHomography2d(srcPoints, dstPoints, H);

			for (unsigned int i = 0; i < c.size(); i++)
			{
				Eigen::Vector3f curr(c[i].px, c[i].py, 1.0f);
				curr = H*curr;
				float dist = (Eigen::Vector2f(curr[0] / curr[2], curr[1] / curr[2]) - Eigen::Vector2f(c[i].mx, c[i].my)).squaredNorm();
				if (dist < DISTANCE_THRESHOLD)
				{
					inliers++;
				}
				else {
					outliers.push_back(i);
				}
			}

			if (inliers > bestInliers)
			{
				bestInliers = inliers;
				bestOutliers = outliers;
				bestSet = c;
			}

			if (inliers >= MIN_INLIER_COUNT)
			{
				break;
			}
		}

		if (bestInliers < 4)
		{
			this->tracking = false;
			return TRACKING_FAILURE;
		}


		if (bestInliers > -1)
		{
			c = bestSet;
			for (std::vector<int>::reverse_iterator it = bestOutliers.rbegin(); it != bestOutliers.rend(); it++)
			{
				c.erase(c.begin() + *it);
			}
		}
	}
	


    this->mneedMorePoints = c.size() <= this->minPointCount;

	return status;
}

template <>
int Tracker::start(ImageGray *img, CorrespondenceSet &/*correspondences*/, ImageGray* )
{
    if(this->imgSubsampled != NULL)
    {
        if (this->imgSubsampled->width*this->subsampling != img->width && this->subsampling != SUBSAMPLE_1)
        {
            delete this->imgSubsampled;
            this->imgSubsampled = NULL;
        }
    }

    if(this->imgSubsampled == NULL && this->subsampling != SUBSAMPLE_1)
    {
        this->imgSubsampled = new ImageGray(img->width/this->subsampling, img->height/this->subsampling);
    }

    this->copySubsample(img);

    this->ptracker.setInitialImage(imgSubsampled);
	this->tracking = true;
	return TRACKING_SUCCESS;
}

template <>
int Tracker::start(ImageRGB *img, CorrespondenceSet &/*correspondences*/, ImageGray*)
{
    //NOT SUPPORTED
	assert(false);
	return TRACKING_FAILURE;
}

int Tracker::copySubsample(ImageGray *img)
{

    switch(this->subsampling){
        case SUBSAMPLE_1:
            this->imgSubsampled = img;
            break;
        case SUBSAMPLE_2:
            assert(img->widthstep/2 <= imgSubsampled->widthstep && img->height/2 <= imgSubsampled->height);
#ifdef UMF_USE_NATIVE
            umfnative::h_subsample2((unsigned char*) img->data, img->width, img->height,(unsigned char*)  this->imgSubsampled->data);
#else 
			umf::subSample2(img, this->imgSubsampled);
#endif
            break;
        case SUBSAMPLE_4:
            assert(img->widthstep/4 <= imgSubsampled->widthstep && img->height/4 <= imgSubsampled->height);
#ifdef UMF_USE_NATIVE
            umfnative::h_subsample4((unsigned char*) img->data, img->width, img->height,(unsigned char*)  this->imgSubsampled->data);
#else
			umf::subSample4(img, this->imgSubsampled);
#endif
            break;
        default:
            return -1;
        }
    return 0;
}

}
