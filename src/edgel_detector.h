#ifndef __UMF_EDGEL_DETECTOR_H
#define __UMF_EDGEL_DETECTOR_H

#include <vector>
#include <Eigen/Core>
#include "util/image.h"
#include "util/orientation_filter.h"
#include "util/fixed_class.h"
#include "util/line_iterator.h"
#include "util/scanline_tracker.h"

namespace umf {

/**
 * @brief The EdgelDetector class
 *
 * Serves as a class for scanline based edge detection and using these
 * as seeds for edgel detection.
 */
class UMF EdgelDetector
{
public:
    EdgelDetector();

    template <class T, int NCHAN>
    void detectEdges(Image<T, NCHAN> *image, ImageGray *mask = NULL, bool show = false);

    template <class T, int NCHAN>
    void detectEdges(Image<T, NCHAN> *image, std::vector< LineIterator<Image<T,NCHAN> > > &scanlines, ImageGray *mask = NULL, bool show = false);

    template <class T, int NCHAN>
    void findEdgels(Image<T, NCHAN> *image, ImageGray *mask = NULL, bool show = false);

    std::vector<Edgel> &getEdgels() { return this->edgels; }
	void setEdgels(std::vector<Edgel> &edgels) { this->edgels = edgels; }

	std::vector<Eigen::Vector2i> &getEdges() { return this->points; }
	void setEdges(std::vector<Eigen::Vector2i> &edges) { this->points = edges; }

    void setAdaptiveThreshold(short threshold) {this->adaptiveThreshold = threshold; }
    short getAdaptiveThreshold() { return this->adaptiveThreshold; }

    void setScanlineStep(short step) { this->scanlineStep = step; }
    short getScanlineStep() { return this->scanlineStep; }

    void setScanlineWindow(short window) { this->scanlineWindow = window; }
    short getScanlineWindow() { return this->scanlineWindow; }

    void setCompareWindow(short window) { this->compareWindow = window; }
    short getCompareWindow() { return this->compareWindow; }

    void setEdgelSearchStep(short step) { this->edgelStep = step; }
    void setEdgelSearchRadius(short radius) { this->edgelRadius = radius; }
    void setEdgelSearchThreshold(short threshold) { this->edgelThreshold = threshold; }
	void setEdgelFieldValueTolerance(short threshold) { this->edgelFieldValueTolerance = threshold; }
    void setEdgelLengthThreshold(short threshold) { this->edgelLengthThreshold = threshold; }
    void setEdgelDotThreshold(float threshold) { this->edgelDotThreshold = threshold; }

    short getEdgelSearchStep() { return this->edgelStep; }
    short getEdgelSearchRadius() { return this->edgelRadius; }
    short getEdgelSearchThreshold() { return this->edgelThreshold; }
	short getEdgelFieldValueTolerance() { return this->edgelFieldValueTolerance; }
    short getEdgelLengthThreshold() { return this->edgelLengthThreshold; }
    float getEdgelDotThreshold() { return this->edgelDotThreshold; }

    OrientationFilter &getOrientationFilter() { return this->orientationFilter; }
    ScanlineTracker &getScanlineTracker() { return this->scanTracker; }

private:

    template<class T, int NCHAN>
    bool findEdgel(Image<T, NCHAN> *image,
                   Edgel &edgel,
                   const Eigen::Vector2i &point,
                   ImageGray* mask = NULL);

    void showEdgels();

    std::vector<Eigen::Vector2i> points;
    std::vector<Edgel> edgels;

    //scanline step
    short adaptiveThreshold;
    short scanlineWindow;
    short compareWindow;
    short scanlineStep;

    //edgel search step
    short edgelMaxSteps; //the number of steps taken in each direction at maximum
    short edgelStep; //the distance between the point and parallel lines when detecting the line direction
    short edgelRadius; // the distance from the detected line, that we use for detecting edges
    short edgelThreshold;// the threshold between points that are on different sides of the detected line; if in doubt set it to adaptiveThreshold
	short edgelFieldValueTolerance; // if -1 no check is done, if around the searched points the marker field elements are constant, or it's on an edge
    short edgelLengthThreshold; //the minimum length of the edgel
    float edgelDotThreshold; //the threshold for the sobel gradient at found edge points and between the current normal


    OrientationFilter orientationFilter;
    ScanlineTracker scanTracker;
};


}

#endif // EDGEL_DETECTOR_H
