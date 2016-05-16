#ifndef __UMF_UMF_H__
#define __UMF_UMF_H__

#include "util/stream_factory.h"
#include "util/image.h"
#include "util/exceptions.h"
#include "util/umfdebug.h"
#include "edgel_detector.h"
#include "grid_detector.h"
#include "edge_dir_detector.h"
#include "model.h"
#include "tracker.h"
#include <vector>
#include <exception>
#include <limits>
#include <Eigen/StdVector>


namespace umf
{

enum DETECTOR_FLAGS
{
    UMF_FLAG_TRACK_POS = 0x02, //track position of the marker
    UMF_FLAG_SUBWINDOWS = 0x04, //use subwindows to detect smaller markers or cluttered scenes
    UMF_FLAG_ITER_REFINE = 0x08, //iteratively refine camera pose
    UMF_FLAG_SUBPIXEL = 0x10, //find corners with subpixel precision 
    UMF_FLAG_CHROMAKEY = 0x20, //the input is an RGB image with a marker mapped into greenscreen
    UMF_FLAG_ORIENTATION = 0x40, //filter markers based on orientation
	UMF_FLAG_HOMOGRAPHY = 0x80, // compute also the homography between the current frame and marker
	UMF_FLAG_MAX_PRECISION = 0x100 //find as many points as reference points as possible - makes the algorithm slower
};

enum TRACKING_FLAGS
{
    UMF_TRACK_MARKER = 0x01, //track only the marker position
    UMF_TRACK_LOCAL_POS = 0x03, //track the successfully detected part of the marker
    UMF_TRACK_SCANLINES = 0x04, //change the scanlines
	UMF_TRACK_CORNERS = 0x08 //track the corners using klt
};

class DetectionTimeoutException: public exception
{
  virtual const char* what() const throw()
  {
    return "DetectionTimout";
  }
};

template< int NCHAN >
class UMF UMFDetector
{
public:
    UMFDetector(int flags);
    ~UMFDetector();

    bool loadMarker(const char* marker_str);
	bool loadMarkerXML(const char* marker_xml);

    /**
     * Update the model camera pose based on the image provided
     * this includes lucas-canade corner tracking backed up by UMF detection
     * and camera pose estimation
     *
     * @param image - the input image used for UMF detection
     * @param timeout  - optional timeout, after which the processing should stop
     */
	template<class T>
	bool update(Image<T, NCHAN> *image, float timeout = -1);

    /**
     * Detect the marker in the image (does not update camera pose)
     *
     * @param image - the input image used for UMF detection
     * @param timeout  - optional timeout, after which the processing should stop
     */
    template< class T >
    bool detect(Image<T, NCHAN> *image, float timeout = -1) /*throw (DetectionTimeoutException)*/ ;

    /**
     * Detect the position of a given image point. Does not support chromakeying
     * 
     * @param imgPos - the requested position in pixels, the algorithm should find the correspoding marker coordinates
     * @param modelPos - the computed model position
     * @return if the grid was successfully detected
     */
    template< class T >
    bool detectPosition(Image<T, NCHAN> *image, std::vector<Eigen::Vector2f> &imgPos, std::vector<Eigen::Vector2f> &modelPos);

	EdgelDetector edgelDetect;
    GridDetector gridDetect;
    EdgeDirDetector<NCHAN> edgeDirDetect;
    Model<NCHAN> model;
	Tracker tracker;

    void setTrackingFlags(int flags) { this->trackFlags = flags; }
    int getTrackingFlags() { return this->trackFlags; }

	void setSubWindowVerticalCount(int count) { this->subWindowVerticalCount = count; }
	int getSubWindowVerticalCount() { return this->subWindowVerticalCount; }
	
    void setFlags(int flags) { this->flags = flags; }
    int getFlags() { return this->flags; }

private:

    Timer detectionTimer;
    bool checkTimeout(float timeout, bool shouldThrow = true) /*throw(DetectionTimeoutException)*/ ;

    template <class T >
    int processSubWindow(Image<T, NCHAN> *image, Eigen::Vector2i &offset, Eigen::Vector2i &size, Location &loc, ImageGray *mask = NULL, bool show = false);

    //generate subwindows to search for UMF marker
    void getSubwindowOffsets(const Eigen::Vector2i &imgSize,
                             std::vector<Eigen::Vector2i> &offsets,
                             Eigen::Vector2i &subwindowSize);

                             
    //get the position of a point only based on the detected grid
    void getPointPosition(Location &reflocation, Eigen::Vector2f &imgPos, Eigen::Vector2f &modelPos);

    int flags;
    int trackFlags;
    int subWindowVerticalCount;
    ImageGray *filterMask;
	ImageGray *chromeMap;
};

}

#endif
