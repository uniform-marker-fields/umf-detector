#ifndef __UMF_EDGE_DIR_DETECTOR_H
#define __UMF_EDGE_DIR_DETECTOR_H

#include <Eigen/Core>
#include <vector>
#include "util/image.h"
#include "marker.h"

namespace umf {

typedef float EDGEDIR_FPTYPE;

//The number of samples used for edge direction identification
enum EDGE_DIR_SAMPLE_COUNTS
{
    EDGE_DIR_SAMPLE_COUNT_FAST = 4,
    EDGE_DIR_SAMPLE_COUNT_SIMPLE = 8,
    EDGE_DIR_SAMPLE_COUNT_CENTERED = 16
};

enum EDGE_DIR_SCORE_DECIDERS
{
    EDGE_DIR_SCORE_DECIDER_FAST = 2,
    EDGE_DIR_SCORE_DECIDER_SIMPLE = 5,
    EDGE_DIR_SCORE_DECIDER_CENTERED = 11
};

//see CPP for documentation

template<int NCHAN, class fptype=EDGEDIR_FPTYPE>
class UMF EdgeDirDetector
{
public:
    typedef fptype fpsampling;
    typedef Eigen::Matrix<fptype, 2, 1> Vector2fp;

    EdgeDirDetector();

    template<class T>
    void extract(Image<T, NCHAN> *img, std::vector<Eigen::Vector3f> &pencil1, std::vector<Eigen::Vector3f> &pencil2, ImageGray* mask = NULL, bool show = false);

    void setFieldDiffThreshold(Eigen::Array<int, NCHAN, 1> threshold) { this->fieldDiffThreshold = threshold; }
    Eigen::Array<int, NCHAN, 1> getFieldDiffThreshold() { return this->fieldDiffThreshold; }

    std::vector< Vector2fp > &getExtractionPoints(){ return this->extractionPoints; }

    std::vector< typename Marker<NCHAN>::DirectionType > &getEdgeDirections() { return this->edgeDirections; }
    unsigned int getRows(){ return this->rows; }
    unsigned int getCols(){ return this->cols; }

	inline unsigned int getSampleCount() { return this->sampleCount; }
	void setSampleCount(unsigned int sampleCount) {
		this->sampleCount = sampleCount;
		if (sampleCount < EDGE_DIR_SAMPLE_COUNT_FAST) {
			this->scoreDecider = sampleCount;
		}
		else if (sampleCount < EDGE_DIR_SAMPLE_COUNT_SIMPLE) {
			this->scoreDecider = EDGE_DIR_SCORE_DECIDER_FAST;
		}
		else if (sampleCount < EDGE_DIR_SAMPLE_COUNT_CENTERED) {
			this->scoreDecider = EDGE_DIR_SCORE_DECIDER_SIMPLE;
		}
		else {
			this->scoreDecider = EDGE_DIR_SCORE_DECIDER_CENTERED;
		}
	}

private:

    void showFieldCenters(ImageGray* mask = NULL);
    void showEdgeDirections(ImageGray* mask = NULL);

    void getSamplingPoints(const Vector2fp &p1,
                           const Vector2fp &p2,
                           const Vector2fp &otherDirection,
                           std::vector<Vector2fp> &samples1,
                           std::vector<Vector2fp> &samples2);

    template<class T>
    Eigen::Matrix<int, NCHAN, 1> getScore(const Image<T, NCHAN> *img, const std::vector<Vector2fp> &samples1, const std::vector<Vector2fp> &samples2, bool show = false);

    std::vector< Vector2fp > extractionPoints;

    //size is ((width - 1)*height + width*(height-1))*channels theoretically but for better indexing we use width*height*2*channels
    std::vector< typename Marker<NCHAN>::DirectionType > edgeDirections;
    unsigned int rows;
    unsigned int cols;

    Eigen::Array<int, NCHAN, 1> fieldDiffThreshold;
    unsigned int sampleCount;
    int scoreDecider;
};

}

#endif // EDGE_DIR_DETECTOR_H
