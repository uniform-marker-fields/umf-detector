#include "umf.h"
#include "util/umfdebug.h"
#include "util/draw.h"
#include "util/chromakey.h"
#include "util/mask_tracker.h"
#include "util/grid_util.h"
#include "util/kalman_filter.h"
#include "util/median_filter.h"
#include "util/scanline_tracker.h"
#include "util/rapidxml.hpp"
#include "util/native_x.h"
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <sstream>
#include <iostream>


/**
 * @mainpage UMF detector
 *
 * @section umf_usg Usage
 *  requires Eigen3 and optionally OpenCV
 *
 * @section umf_gath Gallery and Theory
 *  The main process goes like this:
 *
 *  -# source comes from opencv/firewire etc see factories \link umf::StreamFactory \endlink
 *  \image html 0_source.png
 *  -# sparse scanlines and edge detection \link umf::EdgelDetector::detectEdges \endlink
 *  \image html 1_scanline.png
 *  -# use edges along scanlines as seeds for edgel detection (scanlines 200) \link umf::EdgelDetector::findEdgel \endlink
 *  \image html 2_edgelgrow.png
 *  \image html 3_edgels.png
 *  -# separate into two groups \link umf::GridDetector::separateTwoGroups \endlink
 *  \image html 4_groups.png
 *  -# find vanish for both groups and filter out outliers using RANSAC \link umf::GridDetector::findVanish \endlink
 *  -# detect pencils of lines \link umf::GridDetector::detectMesh \endlink
 *  \image html 5_mesh.png
 *  -# extract edge directions in RGB \link umf::EdgeDirDetector \endlink
 *  \image html 6_edgedir.png
 *  -# match position using decision tree \link umf::DecisionTree \endlink, \link umf::Model \endlink
 *  -# camera pose estimation \link umf::Model \endlink
 *  -# Extra stuff - like iterative refinement by trying to detect more of the map ( for larger maps) )
 *
 */



namespace umf {

template <int NCHAN>
UMFDetector<NCHAN>::UMFDetector(int flags)
{
    this->flags = flags;
    this->subWindowVerticalCount = 2;
    this->filterMask = NULL;
	this->chromeMap = NULL;
}

template <int NCHAN>
UMFDetector<NCHAN>::~UMFDetector()
{
    if(this->filterMask)
    {
        delete this->filterMask;
    }

	if(this->chromeMap)
	{
		delete this->chromeMap;
	}
}

//find subwindows to search for in the image
template <int NCHAN>
void UMFDetector<NCHAN>::getSubwindowOffsets(const Eigen::Vector2i &imgSize,
                                             std::vector<Eigen::Vector2i> &offsets,
                                             Eigen::Vector2i &subwindowSize)
{
	const int windowSizeWidth = imgSize[0] / this->subWindowVerticalCount;
    const int windowSizeHeight = imgSize[1]/this->subWindowVerticalCount; //~200x200 window for VGA
    const int windowSizeHalfWidth = windowSizeWidth/2;
	const int windowSizeHalfHeight = windowSizeHeight/2;
    subwindowSize = Eigen::Vector2i(windowSizeWidth, windowSizeHeight);

    int gridWidth = this->subWindowVerticalCount;
    int gridHeight = this->subWindowVerticalCount;
    int gridMidWidth = gridWidth - 1;
	int gridMidHeight = gridHeight - 1;

    offsets.resize(gridWidth*gridHeight + gridMidWidth*gridMidHeight);

    int counter = 0;
    //normal grid
    for(int j = 0; j < gridHeight; j++)
    {
        for(int i = 0; i < gridWidth; i++)
        {
            offsets[counter++] = Eigen::Vector2i(i*windowSizeWidth, j*windowSizeHeight);
        }
    }

    //overlap
    for(int j = 0; j < gridMidHeight; j++)
    {
        for(int i = 0; i < gridMidWidth; i++)
        {
            offsets[counter++] = Eigen::Vector2i(i*windowSizeWidth + windowSizeHalfWidth, j*windowSizeHeight + windowSizeHalfHeight);
        }
    }
}

//get the position of a point only based on the detected grid
template <int NCHAN>
void UMFDetector<NCHAN>::getPointPosition(Location &refLocation, Eigen::Vector2f &imgPos, Eigen::Vector2f &modelPos)
{

    std::vector<Eigen::Vector3f> &rows = this->gridDetect.getPencil(0);
    std::vector<Eigen::Vector3f> &cols = this->gridDetect.getPencil(1);

    Eigen::Vector3f v1 = rows.front().cross(rows.back());
    Eigen::Vector3f v2 = cols.front().cross(cols.back());
    Eigen::Vector3f horizont = v1.cross(v2);

    Eigen::Vector3f rowLine = v1.cross(Eigen::Vector3f(imgPos[0], imgPos[1], 1));
    Eigen::Vector3f colLine = v2.cross(Eigen::Vector3f(imgPos[0], imgPos[1], 1));

    float rowK = lineGetKPsI(static_cast<float>(rows.size()), horizont, rows.front(), rows.back());
    float colK = lineGetKPsI(static_cast<float>(cols.size()), horizont, cols.front(), cols.back());

    float pointRowK = lineGetKPsI(1.f, horizont, rows.front(), rowLine);
    float pointColK = lineGetKPsI(1.f, horizont, cols.front(), colLine);

    float rowOffset = rowK/pointRowK;
    float colOffset = colK/pointColK;

    modelPos[0] = colOffset + refLocation.c;
    modelPos[1] = rowOffset + refLocation.r;
    changeBackLocationf(modelPos, refLocation.rotation, this->model.getMarker()->w , this->model.getMarker()->h, 0, 0);
}

/**
 * Detect the position of a given image point. Does not support chromakeying
 * 
 * @param imgPos - the requested position in pixels, the algorithm should find the correspoding marker coordinates
 * @param modelPos - the computed model position
 * @return if the grid was successfully detected
 */
template <int NCHAN> template<class T>
bool UMFDetector<NCHAN>::detectPosition(Image<T, NCHAN> *image, std::vector<Eigen::Vector2f> &imgPos, std::vector<Eigen::Vector2f> &modelPos)
{
#ifdef UMF_DEBUG_TIMING
    UMFDebug *dbg = UMFDSingleton::Instance();
#endif

    const unsigned int POINT_COUNT = imgPos.size();

#ifdef UMF_DEBUG_TIMING
    int ovid = dbg->logEventStart();

    int logid = dbg->logEventStart();
#endif

    this->edgelDetect.detectEdges(image, NULL, false);

    //Orientation filter
    if(this->flags & UMF_FLAG_ORIENTATION)
    {
        OrientationFilter &track = this->edgelDetect.getOrientationFilter();
        track.enable(true);
        //track.filterPoints(image, this->edgelDetect.getEdges());
    }

    this->edgelDetect.findEdgels(image, NULL, true);

    //Orientation filter 2
    if(this->flags & UMF_FLAG_ORIENTATION)
    {
        OrientationFilter &track = this->edgelDetect.getOrientationFilter();
        track.filterEdgels(this->edgelDetect.getEdgels());
    }


#ifdef UMF_DEBUG_TIMING
    dbg->logEventEnd(logid, "EE");
    logid = dbg->logEventStart();
#endif
    //global pass first
    Location bestLoc;
    int bestCount = -1;
    Eigen::Vector2i poffset(0, 0);
    Eigen::Vector2i psize(image->width, image->height);
    std::vector<Eigen::Vector2f> bestPosition(imgPos.size());

    this->model.setUseCornerSearch(false);
    int pcount = this->processSubWindow(image, poffset, psize, bestLoc, NULL, false);
    bool success = pcount != -1;
    success = false;

#ifdef UMF_DEBUG_TIMING
    dbg->logEvent(1, "COUNT");
    dbg->logEventEnd(logid, "FULL");
#endif

    if(success)
    {
        for(unsigned int i = 0; i < POINT_COUNT; i++)
        {
            this->getPointPosition(bestLoc, imgPos[i], bestPosition[i]);
        }
    }
    else if(success == false && (this->flags & UMF_FLAG_SUBWINDOWS) != 0)
    {
#ifdef UMF_DEBUG_TIMING
        logid = dbg->logEventStart();
#endif
        //processing in subwindows
        int bestSubWindow = -1;

        std::vector<Eigen::Vector2i> offsets;
        Eigen::Vector2i wSize;
        this->getSubwindowOffsets(Eigen::Vector2i(image->width, image->height), offsets, wSize);
#ifdef UMF_DEBUG_TIMING
        dbg->logEvent(offsets.size(), "COUNT");
#endif
        for(unsigned int subI = 0; subI < offsets.size(); subI++)
        {
            Location loc;
            int pcount = this->processSubWindow(image, offsets[subI], wSize, loc, NULL, subI == 5);

            if(pcount > bestCount)
            {
                bestCount = pcount;
                bestSubWindow = subI;
                bestLoc = loc;

                for(unsigned int i = 0; i < POINT_COUNT; i++)
                {
                    this->getPointPosition(bestLoc, imgPos[i], bestPosition[i]);
                }
            }
        }
#ifdef UMF_DEBUG_TIMING
        dbg->logEventEnd(logid, "SUB");
#endif

        success = bestSubWindow != -1;
    }

    //use a kalman filter to smooth the results, since
    //the positions are noisy by not considering subpixel precise edge corners or
    //full homography calculations
    static std::vector<MedianFilter<float, 2> > filter(imgPos.size());
    typedef SimpleState<2> mstate_t;
    typedef SimpleState<4> sstate_t;
    typedef ConstantProcess<sstate_t> process_t;
    static std::vector< KalmanFilter<sstate_t, process_t>, Eigen::aligned_allocator<KalmanFilter<sstate_t, process_t> > > kf(imgPos.size());
    static bool kfinited = false;
    const float kfMeasurementVariance[2] = {2.0, 1.0};
    const double dt = 0.5;
    const float medianThreshold = 10.f;
    const float medianThresholdSquared = medianThreshold*medianThreshold;
    if(success)
    {
        for(unsigned int i = 0; i < POINT_COUNT; i++)
        {
            modelPos[i] = bestPosition[i];
            Eigen::Vector2f med = filter[i].filter(bestPosition[i]);

            if((med - modelPos[i]).squaredNorm() > medianThresholdSquared)
            {
                //std::cout << "Median diff" << (med - modelPos).norm() << std::endl;
                modelPos[i] = med;
            }
        }

        if(!kfinited)
        {
            kfinited = true;
            std::vector<float> processModelVariance(imgPos.size(), 0.1f);
            processModelVariance[0] = 5e-4f;
            for(unsigned int i = 0; i < POINT_COUNT; i++)
            {
                kf[i].state.x = Eigen::Vector4d(modelPos[i][0], modelPos[i][1], 0, 0);

                kf[i].processModel.sigma = sstate_t::VecState::Constant(processModelVariance[i]);
                kf[i].processModel.jacobian << 1, 0, 1, 0,
                        0, 1, 0, 1,
                        0, 0, 1, 0,
                        0, 0, 0, 1;
            }
        }

        for(unsigned int i = 0; i < POINT_COUNT; i++)
        {
            kf[i].predict(dt);

            AbsoluteMeasurement<mstate_t, sstate_t> meas;
            meas.measurement = modelPos[i].template cast<double>();
            meas.covariance = Eigen::Vector2d::Constant(kfMeasurementVariance[i]).asDiagonal();

            kf[i].correct(meas);

            modelPos[i][0] = static_cast<float>(kf[i].state.x[0]);
            modelPos[i][1] = static_cast<float>(kf[i].state.x[1]);
        }
    }

#ifdef UMF_DEBUG_TIMING
    dbg->logEventEnd(ovid, "OVRL");
#endif
    return success;
}

 /**
 * Update the model camera pose based on the image provided
 * this includes lucas-canade corner tracking backed up by UMF detection
 * and camera pose estimation
 *
 * @param image - the input image used for UMF detection
 * @param timeout  - optional timeout, after which the processing should stop
 *      - (might be important on mobile and real-time apps)
 */
template <int NCHAN> template <class T>
bool UMFDetector<NCHAN>::update(Image<T, NCHAN> *image,  float timeout)
{
#ifdef UMF_DEBUG_TIMING
	UMFDebug *dbg = UMFDSingleton::Instance();
#endif
    this->detectionTimer.start();
	
#ifdef UMF_DEBUG_TIMING
    int ovid = dbg->logEventStart();
	int chromaid = dbg->logEventStart();
#endif

	 if(this->flags & UMF_FLAG_CHROMAKEY)
	{
		if(image->channels == 3)
		{
			
			if(this->filterMask == NULL)
			{
				this->filterMask = new ImageGray(image->width, image->height);
			}
			getChromeMask(image, this->filterMask);
		} else 
		{
#ifdef UMF_ANDROID
			
			if(this->filterMask == NULL)
			{
                this->filterMask = new ImageGray(image->width, image->height*2/3);
			}
			if(this->chromeMap == NULL)
			{
				this->chromeMap = new ImageGray(image->width, image->height*2/3);
			}

#ifdef UMF_USE_NATIVE
            umfnative::getChromeMaskNV21((ImageGray *) image, this->filterMask, this->chromeMap);
#else
			getChromeMaskNV21((ImageGray *) image, this->filterMask, this->chromeMap);
#endif
			image = (Image<T, NCHAN>*) this->chromeMap; //replace the image we are processing with the map
#else
			
			if(this->filterMask == NULL)
			{
				this->filterMask = new ImageGray(image->width/3, image->height);
			}
			if(this->chromeMap == NULL)
			{
				this->chromeMap  = new ImageGray(image->width/3, image->height);
			}


#ifdef UMF_USE_NATIVE
            umfnative::getChromeMaskYCbCr((ImageGray *) image, this->filterMask, this->chromeMap);
#else

            getChromeMaskYCbCr((ImageGray *) image, this->filterMask, this->chromeMap);
#endif
			image = (Image<T, NCHAN>*) this->chromeMap;
#endif
		}
	 }


#ifdef UMF_DEBUG_TIMING
	 dbg->logEventEnd(chromaid, "CHRO");
	 int procid = dbg->logEventStart();
    int logid = dbg->logEventStart();
#endif


	bool success = false;
	bool shouldTrack = (this->flags & UMF_FLAG_TRACK_POS) != 0 && (this->trackFlags & UMF_TRACK_CORNERS);
	if( shouldTrack && this->tracker.isTrackingActive())
	{ //tracking enable try tracking
        int track_result = this->tracker.track(image, &(this->model), this->filterMask);
		success =  track_result == Tracker::TRACKING_SUCCESS;
#ifdef UMF_DEBUG_MSG
		if(success == false)
		{
			std::stringstream s;
			s << "Tracking failed for reason: " << track_result;
			dbg->logMsg(s.str());
		}
#endif
	}
	
#ifdef UMF_DEBUG_TIMING
    dbg->logEventEnd(logid, "TRK");
#endif

	if(success == false)
	{
		success = this->detect(image, timeout);
#ifdef UMF_DEBUG_MSG
		if(success == false)
		{
			dbg->logMsg("FAILED detection");
		}
#endif
	}
	

#ifdef UMF_DEBUG_TIMING
	dbg->logEventEnd(procid, "PROC");
#endif


#ifdef UMF_DEBUG_TIMING
	logid = dbg->logEventStart();
#endif
    if(success)
    {
		
		bool shouldRefine = ((this->flags & UMF_FLAG_ITER_REFINE) > 0) && !checkTimeout(timeout);
		if(shouldRefine && shouldTrack && this->tracker.isTrackingActive())
		{
			if(!this->tracker.needMorePoints())
			{
				shouldRefine = false;
			}
		}
		if (this->flags & UMF_FLAG_MAX_PRECISION) {
			shouldRefine = true;
		}

        success = this->model.computeCameraPosition(image,
                                                    this->edgeDirDetect.getCols(),
                                                    this->edgeDirDetect.getRows(),
                                                    shouldRefine, true, this->filterMask);


		if ((this->flags & UMF_FLAG_HOMOGRAPHY) != 0)
		{
			success = this->model.computeHomography(false);
		}
		
    }

#ifdef UMF_DEBUG_TIMING
	dbg->logEventEnd(logid, "CAM");
#endif

    if(this->flags & UMF_FLAG_TRACK_POS)
    {
        if(success)
        {
            this->model.updateMaskTracker(Eigen::Vector2i(image->width, image->height));
            this->edgelDetect.getScanlineTracker().enable();
        } else {
            MaskTracker &track = this->model.getMaskTracker();
            track.disable();
            this->edgelDetect.getScanlineTracker().disable();
        }
    }

	
	if(success && shouldTrack && !this->tracker.isTrackingActive())
	{
		this->tracker.start(image, this->model.getCorrespondences());
	}

	if(success && (this->flags & UMF_FLAG_TRACK_POS) != 0)
	{
		this->model.setPnPFlags(this->model.getPnPFlags() | PNP_FLAG_USE_LAST);
	} else {
		this->model.setPnPFlags(this->model.getPnPFlags() & (~PNP_FLAG_USE_LAST));
	}


#ifdef UMF_DEBUG_TIMING
    dbg->logEventEnd(ovid, "OVRL");
#endif


	return success;
}

 /**
     * Detect the marker in the image (does not update camera pose)
     *
     * @param image - the input image used for UMF detection
     * @param timeout  - optional timeout, after which the processing should stop
     */
template <int NCHAN> template<class T>
bool UMFDetector<NCHAN>::detect(Image<T, NCHAN> *image,  float timeout) /*throw (DetectionTimeoutException) */
{
#if defined(UMF_DEBUG_TIMING) || defined(UMF_DEBUG_MSG)
    UMFDebug *dbg = UMFDSingleton::Instance();
#endif

    //EDGEL DETECTION
#ifdef UMF_DEBUG_TIMING
    int logid = dbg->logEventStart();
#endif

    const bool showEdges = false;

    if( ((this->flags & UMF_FLAG_TRACK_POS) != 0) && 
        ((this->trackFlags & UMF_TRACK_SCANLINES) != 0) &&
        this->edgelDetect.getScanlineTracker().isEnabled())
    {
        std::vector< LineIterator< Image<T, NCHAN> > > iterators;
        this->edgelDetect.getScanlineTracker().getScanlines(this->model, image, iterators);
        this->edgelDetect.detectEdges(image, iterators, this->filterMask, showEdges);
    } else {
        this->edgelDetect.detectEdges(image, this->filterMask, showEdges);
    }

#ifdef UMF_DEBUG_TIMING
    dbg->logEventEnd(logid, "EDGE");
    logid = dbg->logEventStart();
#endif

#ifdef UMF_DEBUG_MSG
	std::stringstream s;
	s << "Number of edges: " << this->edgelDetect.getEdges().size();
	dbg->logMsg(s.str());
#endif

    //MASK TRACKING
    if( ((this->flags & UMF_FLAG_TRACK_POS) != 0) && 
        ((this->trackFlags & UMF_TRACK_MARKER) != 0) )
    {
        MaskTracker &track = this->model.getMaskTracker();
        track.filterPoints(this->edgelDetect.getEdges());

#ifdef UMF_DEBUG_DRAW
        track.show();
#endif
    }

    const bool showEdgels = false;

    this->edgelDetect.findEdgels(image, this->filterMask, showEdgels);

    this->checkTimeout(timeout);

#ifdef UMF_DEBUG_MSG
	s.str("");
	s << "Number of edgels: " << this->edgelDetect.getEdgels().size();
	dbg->logMsg(s.str());
#endif

#ifdef UMF_DEBUG_TIMING
    dbg->logEvent(this->edgelDetect.getEdgels().size(), "EDGEL_COUNT");
    dbg->logEventEnd(logid, "EDGEL");
    
    logid = dbg->logEventStart();
#endif

    bool success = false;

    this->model.setUseSubPixel((this->flags & UMF_FLAG_SUBPIXEL) > 0);

    const bool showMainProcessing = true;

    //global pass first
	Location goodLoc;
    Eigen::Vector2i poffset(0, 0);
    Eigen::Vector2i psize(image->width, image->height);
    int pcount = this->processSubWindow(image, poffset, psize, goodLoc, filterMask, showMainProcessing);
#ifdef UMF_DEBUG_TIMING
    dbg->logEventEnd(logid, "WIN0");
    logid = dbg->logEventStart();
#endif
    success = pcount != -1;

    if(success == false && (this->flags & UMF_FLAG_SUBWINDOWS) != 0)
    {
        //check for timeout
        this->checkTimeout(timeout);
        
        //processing in subwindows
        int bestSubWindow = -1;
        CorrespondenceSet bestSet;
        Location bestLoc;

        //generate offset
        //foreach subset
        std::vector<Eigen::Vector2i> offsets;
        Eigen::Vector2i wSize;
        this->getSubwindowOffsets(Eigen::Vector2i(image->width, image->height),
                                  offsets,
                                  wSize);

#ifdef UMF_DEBUG_TIMING
        dbg->logEvent(offsets.size(), "COUNT");
#endif
        for(unsigned int subI = 0; subI < offsets.size(); subI++)
        {
            Location loc;
			bool show_sub = false;
            int pcount = this->processSubWindow(image, offsets[subI], wSize, loc, filterMask, show_sub);

            if(pcount > (int) bestSet.size())
            {
                bestSet = this->model.getCorrespondences();
                bestSubWindow = subI;
                bestLoc = loc;
            }

            if(checkTimeout(timeout, false))
            {
                if(bestSubWindow != -1)
                {
                    break;
                } else {
                    throw DetectionTimeoutException();
                }
            }
        }

        success = bestSubWindow != -1;

        if(success){
            this->model.setCorrespondences(bestSet);
            goodLoc = bestLoc;
        }

    } else {
        #ifdef UMF_DEBUG_TIMING
        dbg->logEvent(1, "COUNT");
#endif
    }

#ifdef UMF_DEBUG_TIMING
    dbg->logEventEnd(logid, "WINX");
#endif
    

    return success;
}

/**
 * Try to detect UMF inside a sub-window of the whole image
 */
template <int NCHAN> template<class T>
int UMFDetector<NCHAN>::processSubWindow(Image<T, NCHAN> *image, Eigen::Vector2i &offset, Eigen::Vector2i &size, Location &loc, ImageGray *mask, bool show)
{
#ifdef UMF_DEBUG_TIMING
    UMFDebug *dbg = UMFDSingleton::Instance();
#endif

    bool success = false;
    //GRID detect

    Eigen::Vector2f offsetF = offset.template cast<float>();
    Eigen::Vector2f sizeF = size.template cast<float>();

    //set the transform scale and other stuff accordingly
    this->gridDetect.setTransformCenter(Eigen::Vector2i(offset[0] + size[0]/2, offset[1] + size[1]/2));
    this->gridDetect.setTransformScale(2.0f/(float) (std::min)((float) size[0], (float) size[1]));

    bool showGrid = true && show;
    if(size[0] >= image->width && size[1] >= image->height)
    {
        success = this->gridDetect.detect(this->edgelDetect.getEdgels(), showGrid);
    }
    else {
        std::vector<Edgel> edgels;
        std::vector<Edgel> &allEdgels = this->edgelDetect.getEdgels();
        for(std::vector<Edgel>::iterator edgelIt = allEdgels.begin(); edgelIt != allEdgels.end(); edgelIt++)
        {
            Eigen::Vector2f diff1 = edgelIt->endPoints[0] - offsetF;
            Eigen::Vector2f diff2 = edgelIt->endPoints[1] - offsetF;
            //either endpoint1 or 2 is inside the box
            if( ((diff1.array() >= 0).all() && (diff1.array() < sizeF.array()).all()) || //endpoint 1
                    ((diff2.array() >= 0).all() && (diff2.array() < sizeF.array()).all())) //endpoint2
            {
                edgels.push_back(*edgelIt);
            }
        }
        success = this->gridDetect.detect(edgels, showGrid);
    }

    if(!success)
    {
#ifdef UMF_DEBUG_TIMING
        dbg->logEvent(0, "X");
#endif
        return -1;
    }


#ifdef UMF_DEBUG_TIMING
    int logid = dbg->logEventStart();
#endif

    const bool showEdgeDirs = false && show;

    this->edgeDirDetect.extract(image, this->gridDetect.getPencil(0), this->gridDetect.getPencil(1), mask, showEdgeDirs);

#ifdef UMF_DEBUG_TIMING
    dbg->logEventEnd(logid, "X");
#endif

    success = this->model.matchModel(image, this->edgeDirDetect.getCols(), this->edgeDirDetect.getRows(),
                                     this->edgeDirDetect.getEdgeDirections(),
                                     this->edgeDirDetect.getExtractionPoints(), loc, true && show);

    return success ? this->model.getCorrespondences().size() : -1;
}


/**
 * @brief load a marker from a string
 * @tparam NCHAN detector using this number of channels
 * @param marker_str the string containing the marker
 * @return whether the marker was successfully loaded
 *
 *The expected marker format:
 * width
 * height
 * kernelSize;mask
 * typeCode[;optionally colors in hex - #aabbcc;#1144dd]
 * data...........
 *
 */
template<int NCHAN>
bool UMFDetector<NCHAN>::loadMarker(const char* markerStr)
{

    std::stringstream dataStream(markerStr, std::stringstream::in);

    if(dataStream.fail())
    {
        return false;
    }

    int width;
    int height;
    int ksize = 2;
    int code;
    int mask;

    char comma = ';';

    dataStream >> width;
    dataStream >> height;
    dataStream >> ksize;
    dataStream >> std::noskipws >> comma >> mask >> std::skipws;

    if(mask != 0)
    {
        std::cerr << "Error - unsupported format with a mask!" << std::endl;
        return false;
    }

    dataStream >> code;

    MarkerType mType; mType.decode(code);

    if(mType.torus)
    {
        std::cerr << "Error - unsupported torus format!" << std::endl;
        return false;
    }

    std::vector< Eigen::Matrix<unsigned char, NCHAN, 1> > colors(mType.range);

    //TODO should somehow handle greenscreen marker's too if only one channel - get the mapping from somewhere
    if(mType.color)
    {
        unsigned long *colorHex = new unsigned long[mType.range];

        const int CHANNELS = 3;
        unsigned char *rgb = new unsigned char[mType.range*CHANNELS];

        for(int i = 0; i < mType.range; i++)
        {
            dataStream >> std::noskipws >> comma >> comma /*hash*/ >> std::hex >> colorHex[i];
            rgb[i*CHANNELS + 0] = (colorHex[i] >> 16) & 255;
            rgb[i*CHANNELS + 1] = (colorHex[i] >> 8) & 255;
            rgb[i*CHANNELS + 2] = (colorHex[i]) & 255;
        }
        delete [] colorHex;

        if(NCHAN == 1)
        {
            for(int i = 0; i < mType.range; i++)
            {
                colors[i](0) = (unsigned char)(0.299f*rgb[i*CHANNELS + 0] + 0.587f*rgb[i*CHANNELS + 1] + 0.114f*rgb[i*CHANNELS + 2]); //convert to grayscale
            }
        }
        else if (NCHAN == 3)
        {
            for(int i = 0; i < mType.range; i++)
            {
                colors[i](0) = rgb[i*CHANNELS + 0];
                colors[i](1) = rgb[i*CHANNELS + 1];
                colors[i](2) = rgb[i*CHANNELS + 2];
            }
        } else {
            //no idea how to map colors
            delete [] rgb;
            return false;
        }

        delete [] rgb;
    } else {
        //We got grayscale on need to read anything
        int step = 255/(mType.range - 1);
        for(int i = 0 ; i < mType.range; i++)
        {
            colors[i].setOnes(); colors[i] *= i*step;
        }
    }

    //read data

    std::vector<unsigned short> data;
    data.reserve(height*width);

    Marker<NCHAN> *marker = new Marker<NCHAN>(height, width, ksize, colors);

    comma = '\0';

    int row = 0;
    while(1)
    {
        if(dataStream.eof() || row == height){
            break;
        }

        for(int i = 0; i < width; i++)
        {
            short val = 0;
            dataStream >> std::skipws >> val >> std::noskipws >> comma;
            data.push_back((unsigned short) val);
        }
        row++;
    }

    if(data.size() != (unsigned)(width*height))
    {
        std::cerr << "Non matching data size." << data.size() << " " << (width*height) << std::endl;
        delete marker;
        marker = NULL;
        return false;
    }

    if(marker->setField(data) != true)
    {
        delete marker;
        marker = NULL;
        return false;
    }

    this->model.addMarker(marker);

    return true;
}

/**
 * @brief load a marker from a string
 * @tparam NCHAN detector using this number of channels
 * @param marker_str the string containing the marker
 * @return whether the marker was successfully loaded
 *
 *The expected marker format:
 * <markers><marker>
 * <width>16</width> <height>12</height>
 *<kernel>3</kernel>
 * <data>...</data>
 * <type>4</type>
 * <torus>0</torus>
 * <colorspace>0</colorspace>
 * <range>3</range>
 * <kernel_type>7</kernel_type>
 * </marker>
 * </markers>
 *
 */
template<int NCHAN>
bool UMFDetector<NCHAN>::loadMarkerXML(const char* markerXML)
{
	rapidxml::xml_document<> doc;
	char comma = '#';
	try{
		char *xmlCopy = new char[strlen(markerXML) + 1];
		strcpy(xmlCopy, markerXML);
		doc.parse<rapidxml::parse_trim_whitespace>(xmlCopy);
	} catch(rapidxml::parse_error e)
	{
		std::cerr << "Error parsing marker xml" << e.what() << std::endl;
		return false;
	}

    int width;
    int height;
    int ksize = 2;
    int code;
    int mask;

	rapidxml::xml_node<> *markers_node = doc.first_node("markers");
	if (!markers_node) {
		doc.clear();
		std::cerr << "Wrong xml format - didn't find markers" << std::endl;
		return false;
	}

	rapidxml::xml_node<> *marker_node = markers_node->first_node("marker");
	if (!marker_node) {
		doc.clear();
		std::cerr << "Wrong xml format - didn't find marker" << std::endl;
		return false;
	}

	rapidxml::xml_node<> *node = marker_node->first_node("width");
	if (!node) {
		doc.clear();
		std::cerr << "Wrong xml format - didn't find width" << std::endl;
		return false;
	}
	width = atoi(node->value());
	node = marker_node->first_node("height");
	if (!node) {
		doc.clear();
		std::cerr << "Wrong xml format - didn't find height" << std::endl;
		return false;
	}
	height = atoi(node->value());
	
	node = marker_node->first_node("kernel");
	if (!node) {
		doc.clear();
		std::cerr << "Wrong xml format - didn't find kernel" << std::endl;
		return false;
	}
	ksize = atoi(node->value());

	//ignore mask
	mask = 0;
	MarkerType mType;

	//color vs grayscale
	node = marker_node->first_node("colorspace");
	if (!node) {
		doc.clear();
		std::cerr << "Wrong xml format - didn't find colorspace" << std::endl;
		return false;
	}
	mType.color = atoi(node->value()) == 1;
	
	//torus vs plane
	node = marker_node->first_node("torus");
	if (!node) {
		doc.clear();
		std::cerr << "Wrong xml format - didn't find torus" << std::endl;
		return false;
	}
	mType.torus = atoi(node->value()) == 1;

	//number of shades
	node = marker_node->first_node("range");
	if (!node) {
		doc.clear();
		std::cerr << "Wrong xml format - didn't find range" << std::endl;
		return false;
	}
	mType.range = atoi(node->value());

    if(mType.torus)
    {
		doc.clear();
        std::cerr << "Error - unsupported torus format!" << std::endl;
        return false;
    }

    std::vector< Eigen::Matrix<unsigned char, NCHAN, 1> > colors(mType.range);

    //TODO should somehow handle greenscreen marker's too if only one channel - get the mapping from somewhere
    if(mType.color)
    {
		node = marker_node->first_node("colors");
		if (!node) {
			doc.clear();
			std::cerr << "Wrong xml format - didn't find colors" << std::endl;
			return false;
		}
		const char *colorStr = node->value();
		std::stringstream dataStream(colorStr, std::stringstream::in);

        unsigned long *colorHex = new unsigned long[mType.range];

        const int CHANNELS = 3;
        unsigned char *rgb = new unsigned char[mType.range*CHANNELS];


        for(int i = 0; i < mType.range; i++)
        {
            dataStream >> std::noskipws >> comma /*hash*/ >> std::hex >> colorHex[i];
            rgb[i*CHANNELS + 0] = (colorHex[i] >> 16) & 255;
            rgb[i*CHANNELS + 1] = (colorHex[i] >> 8) & 255;
            rgb[i*CHANNELS + 2] = (colorHex[i]) & 255;
        }
        delete [] colorHex;

        if(NCHAN == 1)
        {
            for(int i = 0; i < mType.range; i++)
            {
                colors[i](0) = (unsigned char)(0.299f*rgb[i*CHANNELS + 0] + 0.587f*rgb[i*CHANNELS + 1] + 0.114f*rgb[i*CHANNELS + 2]); //convert to grayscale
            }
        }
        else if (NCHAN == 3)
        {
            for(int i = 0; i < mType.range; i++)
            {
                colors[i](0) = rgb[i*CHANNELS + 0];
                colors[i](1) = rgb[i*CHANNELS + 1];
                colors[i](2) = rgb[i*CHANNELS + 2];
            }
        } else {
            //no idea how to map colors
            delete [] rgb;
			doc.clear();
            return false;
        }

        delete [] rgb;
    } else {
        //We got grayscale on need to read anything
        int step = 255/(mType.range - 1);
        for(int i = 0 ; i < mType.range; i++)
        {
            colors[i].setOnes(); colors[i] *= i*step;
        }
    }

    //read data

    std::vector<unsigned short> data;
    data.reserve(height*width);

    Marker<NCHAN> *marker = new Marker<NCHAN>(height, width, ksize, colors);

    comma = '\0';

	node = marker_node->first_node("data");
	if (!node) {
		doc.clear();
		std::cerr << "Wrong xml format - didn't find data" << std::endl;
		return false;
	}
	const char *dataStr = node->value();
	std::stringstream dataStream(dataStr, std::stringstream::in);

    int row = 0;
    while(1)
    {
        if(dataStream.eof() || row == height){
            break;
        }

        for(int i = 0; i < width; i++)
        {
            unsigned char val = 0;
            dataStream >> std::skipws >> val;
            data.push_back((unsigned short) (val - '0'));
        }
        row++;
    }

	doc.clear();

    if(data.size() != (unsigned)(width*height))
    {
        std::cerr << "Non matching data size." << data.size() << " " << (width*height) << std::endl;
        delete marker;
        marker = NULL;
        return false;
    }

    if(marker->setField(data) != true)
    {
        delete marker;
        marker = NULL;
        return false;
    }

    this->model.addMarker(marker);

    return true;
}

template <int NCHAN>
bool UMFDetector<NCHAN>::checkTimeout(float timeout, bool shouldThrow) /* throw (DetectionTimeoutException) */
{
    if(timeout > 0)
    {
        double diff = this->detectionTimer.stop();
        if(diff > timeout)
        {
            if(shouldThrow)
            {
                throw DetectionTimeoutException();
            }
            return true;
        }
    }
    return false;
}

//Generate instances for grayscale and RGB images
template UMF  UMFDetector<1>::UMFDetector(int flags);
template UMF UMFDetector<3>::UMFDetector(int flags);
template UMF UMFDetector<1>::~UMFDetector();
template UMF UMFDetector<3>::~UMFDetector();


template UMF bool UMFDetector<1>::update(ImageGray *img, float timeout);
template UMF bool UMFDetector<3>::update(ImageRGB *img, float timeout);

template UMF bool UMFDetector<1>::detect(ImageGray *img, float timeout);
template UMF bool UMFDetector<3>::detect(ImageRGB *img, float timeout);

template UMF bool UMFDetector<1>::detectPosition(ImageGray *image, std::vector<Eigen::Vector2f> &imgPos, std::vector<Eigen::Vector2f> &modelPos);
template UMF bool UMFDetector<3>::detectPosition(ImageRGB *image, std::vector<Eigen::Vector2f> &imgPos, std::vector<Eigen::Vector2f> &modelPos);

template UMF void UMFDetector<1>::setSubWindowVerticalCount(int count);
template UMF void UMFDetector<3>::setSubWindowVerticalCount(int count);

template UMF int UMFDetector<1>::getSubWindowVerticalCount();
template UMF int UMFDetector<3>::getSubWindowVerticalCount();


template UMF bool UMFDetector<1>::loadMarker(const char* markerStr);
template UMF bool UMFDetector<3>::loadMarker(const char* markerStr);

template UMF bool UMFDetector<1>::loadMarkerXML(const char* markerXML);
template UMF bool UMFDetector<3>::loadMarkerXML(const char* markerXML);
}
