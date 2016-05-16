#include "defines.h"
#include "edgel_detector.h"
#include "util/umfdebug.h"
#include "util/draw.h"
#include "util/corner_detector.h"
#include "util/scanline_tracker.h"

#include <Eigen/Dense>

namespace umf {

EdgelDetector::EdgelDetector()
{
    this->adaptiveThreshold = 15;
	this->scanlineStep = 50;
    this->scanlineWindow = 8;
    this->compareWindow = 4;

    this->edgelMaxSteps = 8;
    this->edgelStep = 5;
    this->edgelThreshold = 10;
	this->edgelFieldValueTolerance = -1;
    this->edgelRadius = 7;
    this->edgelLengthThreshold = 15;
    this->edgelDotThreshold = cosf(3.14f/4); //+-45 degree of freedom

    this->scanTracker = ScanlineTracker(10);
}


template<class fptype, class T, int CHAN>
static void scan(Image<T, CHAN> *input,
                 std::vector<Eigen::Vector2i> &points,
                 ImageGray *mask,
                 const int step,
                 bool vertical,
                 const int bufferSize,
                 const int compareBufferSize,
                 const int adaptiveThreshold)
{
    std::vector< Eigen::Matrix<fptype, CHAN, 1> > buffer(bufferSize);
    std::vector< Eigen::Matrix<fptype, CHAN, 1> > compareBuffer(compareBufferSize);
    unsigned short windowEnd = 0;
    unsigned short compareWindowEnd = 0;
    const fptype adaptiveThresholdSquared = fptype(adaptiveThreshold*adaptiveThreshold);
    const int foundOffset = compareBufferSize/2;

    const fptype inv_size = fptype(1)/(bufferSize);
    const fptype compinv_size = fptype(1)/(compareBufferSize);
    Eigen::Matrix<fptype, CHAN, 1> mean;
    Eigen::Matrix<fptype, CHAN, 1> compareMean;
    mean.setZero();
    compareMean.setZero();

    int x,y;

    int *dirStep, *dirScan;
    int dirStepSize, dirScanSize;

    if(vertical)
    {
        dirStep = &x;
		dirStepSize = input->width - bufferSize;
        dirScan = &y;
        dirScanSize = input->height - bufferSize;
    } else {
        dirStep = &y;
        dirStepSize = input->height - bufferSize;
        dirScan = &x;
        dirScanSize = input->width - bufferSize;
    }

    Eigen::Matrix<fptype, CHAN, 1> curr;
    Eigen::Matrix<fptype, CHAN, 1> diff;
    for(*dirStep=step/2; *dirStep < dirStepSize; (*dirStep) +=step)
    {
        windowEnd = 0;
        mean *= 0;
        //first fill up our buffer - probably clutter - don't really care
        for(*dirScan = 0; *dirScan < bufferSize; (*dirScan)++)
        {
            input->get2Dconv(curr, x, y);

            buffer[windowEnd] = curr;
            windowEnd++;
            mean += curr;
        }
        compareWindowEnd = 0;
        compareMean *= 0;
        for(;*dirScan < bufferSize + compareBufferSize; (*dirScan)++)
        {
            input->get2Dconv(curr, x, y);
            compareBuffer[compareWindowEnd] = curr;
            compareWindowEnd++;
            compareMean += curr;
        }
#ifdef UMF_DEBUG_COUNT_PIXELS
        UMFDSingleton::Instance()->addPixels(bufferSize + compareBufferSize);
#endif

        compareWindowEnd = 0;
        windowEnd = 0;

        diff = mean*inv_size - compareMean*compinv_size;
        bool edgeFound = diff.dot(diff) > adaptiveThresholdSquared;
        int maskedSize = -1;
        Eigen::Vector2i edgeStart(-1, -1);
        //now scan the line
#ifdef UMF_DEBUG_COUNT_PIXELS
        UMFDSingleton::Instance()->addPixels(dirScanSize - bufferSize - compareBufferSize);
#endif
        for(/*dirscan = buffersize+comparebufferSize*/; *dirScan < dirScanSize; (*dirScan)++)
        {

            bool masked = (mask == NULL || (*(mask->get2D(x, y)) == 255));
            if(masked && maskedSize <= 0)
            {

                diff = mean*inv_size - compareMean*compinv_size;
                if( diff.dot(diff) > adaptiveThresholdSquared)
                {
                    if(!edgeFound)
                    {
                        //good we found and edge
                        edgeFound = true;
                        *dirScan -= foundOffset;
                        edgeStart[0] = x;
                        edgeStart[1] = y;
                        *dirScan += foundOffset;
                    }
                } else if (edgeFound)
                {
                    edgeFound = false;

                    Eigen::Vector2i middle = edgeStart;

                    //store the index
                    points.push_back(middle);
                }

            } else {
                edgeFound = false;
            }
            if(!masked)
            {
                maskedSize = bufferSize;
            } else {
                maskedSize--;
            }

            
            input->get2Dconv(curr, x, y);

            mean -= buffer[windowEnd];
            mean += compareBuffer[compareWindowEnd];
            buffer[windowEnd] = compareBuffer[compareWindowEnd];
            compareMean -= compareBuffer[compareWindowEnd];
            compareMean += curr;
            compareBuffer[compareWindowEnd] = curr;
            windowEnd = (windowEnd + 1) % bufferSize;
            compareWindowEnd = (compareWindowEnd + 1) % compareBufferSize;
        }
    }
}

static inline bool IsPower2(int x)
{
	return ( (x > 0) && ((x & (x - 1)) == 0) );
}

//specialization with ignoring the mask completely and fixed point arithmacy with rightly chosen 
//buffer sizes
template<class fptype>
static void scan(ImageGray *input,
                 std::vector<Eigen::Vector2i> &points,
                 ImageGray * /*mask*/,
                 const int step,
                 bool vertical,
                 const int bufferSize,
                 const int compareBufferSize,
                 const int adaptiveThreshold)
{
    assert(IsPower2(bufferSize) && IsPower2(compareBufferSize));

    int compareShift = 0;
    int compareMask = 0;
    int bufferShift = 0;
    int bufferMask = 0;
    while(((compareBufferSize >> compareShift) & 0x01) == 0)
    {
        compareMask |= 1 << compareShift;
        compareShift++;
    }
    
    while(((bufferSize >> bufferShift) & 0x01) == 0)
    {
        bufferMask |= 1 << bufferShift;
        bufferShift++; 
    }

    std::vector< fptype> buffer(bufferSize);
    std::vector< fptype > compareBuffer(compareBufferSize);
    unsigned short windowEnd = 0;
    unsigned short compareWindowEnd = 0;
    const fptype adaptiveThresholdSquared = fptype(adaptiveThreshold*adaptiveThreshold);
    const int foundOffset = compareBufferSize >> 1;

    fptype mean = 0;
    fptype compareMean = 0;

    int x,y;

    int *dirStep, *dirScan;
    int dirStepSize, dirScanSize;

    if(vertical)
    {
        dirStep = &x;
		dirStepSize = input->width - bufferSize;
        dirScan = &y;
        dirScanSize = input->height - bufferSize;
    } else {
        dirStep = &y;
        dirStepSize = input->height - bufferSize;
        dirScan = &x;
        dirScanSize = input->width - bufferSize;
    }

    fptype curr;
    fptype diff;
    for(*dirStep=step/2; *dirStep < dirStepSize; (*dirStep) +=step)
    {
        windowEnd = 0;
        mean *= 0;
        //first fill up our buffer - probably clutter - don't really care
        for(*dirScan = 0; *dirScan < bufferSize; (*dirScan)++)
        {
            curr = (fptype)(*input->get2D(x, y));

            buffer[windowEnd] = curr;
            windowEnd++;
            mean += curr;
        }
        compareWindowEnd = 0;
        compareMean *= 0;
        for(;*dirScan < bufferSize + compareBufferSize; (*dirScan)++)
        {
            curr = (fptype)(*input->get2D(x, y));
            compareBuffer[compareWindowEnd] = curr;
            compareWindowEnd++;
            compareMean += curr;
        }
#ifdef UMF_DEBUG_COUNT_PIXELS
        UMFDSingleton::Instance()->addPixels(bufferSize + compareBufferSize);
#endif

        compareWindowEnd = 0;
        windowEnd = 0;

        diff = (mean >> bufferShift) - (compareMean >> compareShift);
        bool edgeFound = diff*diff > adaptiveThresholdSquared;

        Eigen::Vector2i edgeStart(-1, -1);
        //now scan the line
#ifdef UMF_DEBUG_COUNT_PIXELS
        UMFDSingleton::Instance()->addPixels(dirScanSize - bufferSize - compareBufferSize);
#endif
        for(/*dirscan = buffersize+comparebufferSize*/; *dirScan < dirScanSize; (*dirScan)++)
        {

            diff = (mean >> bufferShift) - (compareMean >> compareShift);
            if( diff*diff > adaptiveThresholdSquared)
            {
                if(!edgeFound)
                {
                    //good we found and edge
                    edgeFound = true;
                    *dirScan -= foundOffset;
                    edgeStart[0] = x;
                    edgeStart[1] = y;
                    *dirScan += foundOffset;
                }
            } else if (edgeFound)
            {
                edgeFound = false;

                Eigen::Vector2i middle = edgeStart;

                //store the index
                points.push_back(middle);
            }
            
            
            curr = (fptype)(*input->get2D(x, y));

            mean -= buffer[windowEnd];
            mean += compareBuffer[compareWindowEnd];
            buffer[windowEnd] = compareBuffer[compareWindowEnd];
            compareMean -= compareBuffer[compareWindowEnd];
            compareMean += curr;
            compareBuffer[compareWindowEnd] = curr;
            windowEnd = (windowEnd + 1) & bufferMask;
            compareWindowEnd = (compareWindowEnd + 1) & compareMask;
        }
    }
}


/**
 * @brief Detect edges along scanlines horizontally and vertically.
 *
 * \param image The input image for detection
 * \param mask Optional parameter (for example for masking out regions - like greenscreen
 *      The size of the image should be the same as for the input image.
 * \param show Show the scanline in the debug output if it is enabled - see image below
 *
 * \tparam T type of the image (unsigned char,float etc.)
 * \tparam NCHAN number of channels
 *
 * Detect along scanlines with \link EdgelDetector::setScanlineStep \endlink shift with adaptive threshod
 * \link EdgelDetector::setAdaptiveThreshold \endlink over a window sized \link EdgelDetector::setScanlineWindow \endlink.
 * \image html 1_scanline.png
 *
 */
template<class T, int NCHAN>
void EdgelDetector::detectEdges(Image<T, NCHAN> *image, ImageGray *mask, bool show)
{
    this->points.clear();
    this->points.reserve(500);
    //scanline detection
    //first vertical pass
    scan< fixp::fixp_8 >(image, this->points, mask, this->scanlineStep, true, this->scanlineWindow, this->compareWindow, this->adaptiveThreshold);
    //then horizontal pass
    scan< fixp::fixp_8 >(image, this->points, mask, this->scanlineStep, false, this->scanlineWindow, this->compareWindow, this->adaptiveThreshold);

#ifdef UMF_DEBUG_DRAW
    if(show)
    {
        UMFDebug *dbg = UMFDSingleton::Instance();
        Renderer *rend = dbg->getRenderer();

        if(rend)
        {
			Eigen::Vector3i lineColor(90, 235, 235);
            int lineWidth = 2;

			for(int row = this->scanlineStep/2; row < rend->getHeight() - this->scanlineWindow; row += this->scanlineStep)
            {
				drawLine(rend, Eigen::Vector2i(0, row), Eigen::Vector2i(rend->getWidth(), row), lineColor, lineWidth);

            }
			for(int col = this->scanlineStep/2; col < rend->getWidth() - this->scanlineWindow; col += this->scanlineStep)
            {
				drawLine(rend, Eigen::Vector2i(col, 0), Eigen::Vector2i(col, rend->getHeight()), lineColor, lineWidth);
            }

            Eigen::Vector3i paintColor(0, 255, 255);
            //Eigen::Vector3i paintColor(255, 255, 0);
            for(std::vector<Eigen::Vector2i>::iterator pit = points.begin(); pit != points.end(); pit++)
            {
                drawCircle(rend, *pit, lineWidth*3, paintColor, -1);
            }
        }
    }
#endif

}

template<class fptype, class T, int CHAN>
static void scan(Image<T, CHAN> *input,
                 std::vector<Eigen::Vector2i> &points,
                 ImageGray *mask,
                 LineIterator<Image<T, CHAN> > &lineIter,
                 const int bufferSize,
                 const int compareBufferSize,
                 const int adaptiveThreshold)
{
    if(lineIter.count < bufferSize + compareBufferSize) //too short line
    {
        return;
    }

    std::vector< Eigen::Matrix<fptype, CHAN, 1> > buffer(bufferSize);
    std::vector< Eigen::Matrix<fptype, CHAN, 1> > compareBuffer(compareBufferSize);
    std::vector< Eigen::Vector2i > locationHistory(compareBufferSize);
    unsigned short windowEnd = 0;
    unsigned short compareWindowEnd = 0;
    const fptype adaptiveThresholdSquared = fptype(adaptiveThreshold*adaptiveThreshold);

    const fptype inv_size = fptype(1)/(bufferSize);
    const fptype compinv_size = fptype(1)/(compareBufferSize);
    Eigen::Matrix<fptype, CHAN, 1> mean;
    Eigen::Matrix<fptype, CHAN, 1> compareMean;
    mean.setZero();
    compareMean.setZero();

    int x,y;

    Eigen::Matrix<fptype, CHAN, 1> curr;
    Eigen::Matrix<fptype, CHAN, 1> diff;

    int counter = 0;

    //first fill up our buffer - probably clutter - don't really care
    for(; counter < bufferSize; ++counter, ++lineIter)
    {
        lineIter.pos(x, y);
        input->get2Dconv(curr, x, y);

        buffer[windowEnd] = curr;
        windowEnd++;
        mean += curr;
    }

    //fill up compare buffer
    for(;counter < bufferSize + compareBufferSize; ++counter, ++lineIter)
    {
        lineIter.pos(x, y);
        input->get2Dconv(curr, x, y);
        compareBuffer[compareWindowEnd] = curr;
        locationHistory[compareWindowEnd][0] = x;
        locationHistory[compareWindowEnd][1] = y;
        compareWindowEnd++;
        compareMean += curr;
    }

#ifdef UMF_DEBUG_COUNT_PIXELS
    UMFDSingleton::Instance()->addPixels(bufferSize + compareBufferSize);
#endif

    compareWindowEnd = 0;
    windowEnd = 0;

    diff = mean*inv_size - compareMean*compinv_size;
    
    fptype currentDiff = diff.dot(diff);
    bool edgeFound = currentDiff > adaptiveThresholdSquared;
    int maskedSize = -1;
    Eigen::Vector2i edgeStart(-1, -1);

    fptype previousDiff = 0;

    //now scan the line
#ifdef UMF_DEBUG_COUNT_PIXELS
    UMFDSingleton::Instance()->addPixels(dirScanSize - bufferSize - compareBufferSize);
#endif
    for(; counter < lineIter.count; ++counter, ++lineIter)
    {
        lineIter.pos(x, y);

        bool masked = (mask == NULL || (*(mask->get2D(x, y)) == 255));
        if(masked && maskedSize <= 0)
        {

            diff = mean*inv_size - compareMean*compinv_size;
            currentDiff = diff.dot(diff);
            if( currentDiff > adaptiveThresholdSquared)
            {
                if(!edgeFound)
                {
                    //good we found and edge
                    edgeFound = true;
                    edgeStart[0] = locationHistory[compareWindowEnd][0];
                    edgeStart[1] = locationHistory[compareWindowEnd][1];
                    
                    previousDiff = currentDiff;
                } else if(currentDiff > previousDiff)
                {
                    edgeStart[0] = locationHistory[compareWindowEnd][0];
                    edgeStart[1] = locationHistory[compareWindowEnd][1];
                    
                    previousDiff = currentDiff;
                }
            } else if (edgeFound)
            {
                edgeFound = false;

                Eigen::Vector2i middle = edgeStart;

                //store the index
                points.push_back(middle);
            }

        } else {
            edgeFound = false;
        }
        if(!masked)
        {
            maskedSize = bufferSize;
        } else {
            maskedSize--;
        }

            
        input->get2Dconv(curr, x, y);
        locationHistory[compareWindowEnd][0] = x;
        locationHistory[compareWindowEnd][1] = y;

        mean -= buffer[windowEnd];
        mean += compareBuffer[compareWindowEnd];
        buffer[windowEnd] = compareBuffer[compareWindowEnd];
        compareMean -= compareBuffer[compareWindowEnd];
        compareMean += curr;
        compareBuffer[compareWindowEnd] = curr;
        windowEnd = (windowEnd + 1) % bufferSize;
        compareWindowEnd = (compareWindowEnd + 1) % compareBufferSize;
    }
}

template<class fptype>
static void scan(ImageGray *input,
                 std::vector<Eigen::Vector2i> &points,
                 ImageGray *mask,
                 LineIterator<ImageGray > &lineIter,
                 const int bufferSize,
                 const int compareBufferSize,
                 const int adaptiveThreshold)
{
    if(lineIter.count < bufferSize + compareBufferSize) //too short line
    {
        return;
    }

    assert(IsPower2(bufferSize) && IsPower2(compareBufferSize));

    int compareShift = 0;
    int compareMask = 0;
    int bufferShift = 0;
    int bufferMask = 0;
    while(((compareBufferSize >> compareShift) & 0x01) == 0)
    {
        compareMask |= 1 << compareShift;
        compareShift++;
    }
    
    while(((bufferSize >> bufferShift) & 0x01) == 0)
    {
        bufferMask |= 1 << bufferShift;
        bufferShift++; 
    }

    std::vector< fptype > buffer(bufferSize);
    std::vector< fptype > compareBuffer(compareBufferSize);
    std::vector< Eigen::Vector2i > locationHistory(compareBufferSize);
    unsigned short windowEnd = 0;
    unsigned short compareWindowEnd = 0;
    const fptype adaptiveThresholdSquared = fptype(adaptiveThreshold*adaptiveThreshold);

    fptype mean = static_cast<fptype>(0);
    fptype compareMean = static_cast<fptype>(0);

    int x,y;

    fptype curr;
    fptype diff;

    int counter = 0;

    //first fill up our buffer - probably clutter - don't really care
    for(; counter < bufferSize; ++counter, ++lineIter)
    {
        lineIter.pos(x, y);
        curr = (fptype)(*input->get2D(x, y));

        buffer[windowEnd] = curr;
        windowEnd++;
        mean += curr;
    }

    //fill up compare buffer
    for(;counter < bufferSize + compareBufferSize; ++counter, ++lineIter)
    {
        lineIter.pos(x, y);
        curr = (fptype)(*input->get2D(x, y));
        compareBuffer[compareWindowEnd] = curr;
        locationHistory[compareWindowEnd][0] = x;
        locationHistory[compareWindowEnd][1] = y;
        compareWindowEnd++;
        compareMean += curr;
    }

#ifdef UMF_DEBUG_COUNT_PIXELS
    UMFDSingleton::Instance()->addPixels(bufferSize + compareBufferSize);
#endif

    compareWindowEnd = 0;
    windowEnd = 0;

    diff = (mean >> bufferShift) - (compareMean >> compareShift);
    diff *= diff;
    bool edgeFound = diff > adaptiveThresholdSquared;
    Eigen::Vector2i edgeStart(-1, -1);

    fptype previousDiff = 0;

    //now scan the line
#ifdef UMF_DEBUG_COUNT_PIXELS
    UMFDSingleton::Instance()->addPixels(dirScanSize - bufferSize - compareBufferSize);
#endif
    for(; counter < lineIter.count; ++counter, ++lineIter)
    {
        lineIter.pos(x, y);

        diff = (mean >> bufferShift) - (compareMean >> compareShift);
        diff *= diff;
        if( diff > adaptiveThresholdSquared)
        {
            if(!edgeFound)
            {
                //good we found and edge
                edgeFound = true;
                edgeStart[0] = locationHistory[compareWindowEnd][0];
                edgeStart[1] = locationHistory[compareWindowEnd][1];
                    
                previousDiff = diff;
            } else if(diff > previousDiff)
            {
                edgeStart[0] = locationHistory[compareWindowEnd][0];
                edgeStart[1] = locationHistory[compareWindowEnd][1];
                    
                previousDiff = diff;
            }
        } else if (edgeFound)
        {
            edgeFound = false;

            Eigen::Vector2i middle = edgeStart;

            //store the index
            points.push_back(middle);
        }

        curr = (fptype)(*input->get2D(x, y));
        locationHistory[compareWindowEnd][0] = x;
        locationHistory[compareWindowEnd][1] = y;

        mean -= buffer[windowEnd];
        mean += compareBuffer[compareWindowEnd];
        buffer[windowEnd] = compareBuffer[compareWindowEnd];
        compareMean -= compareBuffer[compareWindowEnd];
        compareMean += curr;
        compareBuffer[compareWindowEnd] = curr;
        windowEnd = (windowEnd + 1) & bufferMask;
        compareWindowEnd = (compareWindowEnd + 1) & compareMask;
    }
}

template <class T, int NCHAN>
void EdgelDetector::detectEdges(Image<T, NCHAN> *image, std::vector< LineIterator<Image<T,NCHAN> > > &scanlines, ImageGray *mask, bool show)
{
    this->points.clear();
    this->points.reserve(500);
    //scanlines
    for(unsigned int scanIt = 0; scanIt < scanlines.size(); scanIt++)
    {
        LineIterator< Image<T,NCHAN> > curr = scanlines[scanIt];
        scan<fixp::fixp_8>(image, this->points, mask, curr, this->scanlineWindow, this->compareWindow, this->adaptiveThreshold);
    }

#ifdef UMF_DEBUG_DRAW
    if(show)
    {
        UMFDebug *dbg = UMFDSingleton::Instance();
        Renderer *rend = dbg->getRenderer();

        if(rend)
		{
			Eigen::Vector3i lineColor(90, 235, 235);
			int lineWidth = 2;
			for(unsigned int scanIt = 0; scanIt < scanlines.size(); scanIt++)
            {
                const LineIterator< Image<T,NCHAN> > &curr =  scanlines[scanIt];

				drawLine(rend, curr.pointStart, curr.pointEnd, lineColor, lineWidth);
            }


			Eigen::Vector3i paintColor(0, 255, 255);
            //Eigen::Vector3i paintColor(255, 255, 0);
            for(std::vector<Eigen::Vector2i>::iterator pit = points.begin(); pit != points.end(); pit++)
            {
                drawCircle(rend, *pit, lineWidth*2, paintColor, -1);
            }
        }
    }
#endif
}

///////////////////////////////////////////////////////////////////////////////////
////EDGEL search

void debugDrawLine(Eigen::Vector2i p1, Eigen::Vector2i p2, Eigen::Vector3i color)
{
    UMFDebug *dbg = UMFDSingleton::Instance();
    Renderer *rend = dbg->getRenderer();

    if(rend)
    {
        drawLine(rend, p1, p2, color, 1);
    }
}

template<class fptype, class T, int NCHAN>
inline bool checkGoodBinSearchPosition(Image<T, NCHAN> *image, Eigen::Matrix<fptype, 2, 1> searchP1, Eigen::Matrix<fptype, 2, 1> searchP2, Eigen::Matrix<fptype, 2, 1> edgeDirection, float edgelThreshold)
{
    Eigen::Matrix<fptype, NCHAN, 1> p1, p2, p3, p4;
	const fptype MARGIN = 3;
    if(searchP1[0] < MARGIN || searchP2[0] < MARGIN || searchP1[1] < MARGIN || searchP2[1] < MARGIN ||
            searchP1[0] >= image->width - MARGIN || searchP2[0] >= image->width - MARGIN ||
            searchP1[1] >= image->height - MARGIN || searchP2[1] >= image->height - MARGIN)
    {
        return false;
    }

	if (edgelThreshold > 0) {

		image->get2Dconv(p1, static_cast<int>(searchP1[0] + edgeDirection[0] * 2), static_cast<int>(searchP1[1] + edgeDirection[1] * 2));
		image->get2Dconv(p2, static_cast<int>(searchP1[0] - edgeDirection[0] * 2), static_cast<int>(searchP1[1] - edgeDirection[1] * 2));
		image->get2Dconv(p3, static_cast<int>(searchP2[0] + edgeDirection[0] * 2), static_cast<int>(searchP2[1] + edgeDirection[1] * 2));
		image->get2Dconv(p4, static_cast<int>(searchP2[0] - edgeDirection[0] * 2), static_cast<int>(searchP2[1] - edgeDirection[1] * 2));
		return ((p1 - p2).norm() < edgelThreshold) && ((p3 - p4).norm() < edgelThreshold);
	}
	else {
		return true;
	}
}


typedef float EdgelFType;

template<>
inline bool checkGoodBinSearchPosition(ImageGray *image, Eigen::Matrix<EdgelFType, 2, 1> searchP1, Eigen::Matrix<EdgelFType, 2, 1> searchP2, Eigen::Matrix<EdgelFType, 2, 1> edgeDirection, float edgelThreshold)
{
    short p1, p2, p3, p4;
	const EdgelFType MARGIN = 4;
    if(searchP1[0] < MARGIN || searchP2[0] < MARGIN || searchP1[1] < MARGIN || searchP2[1] < MARGIN ||
            searchP1[0] >= image->width - MARGIN || searchP2[0] >= image->width - MARGIN ||
            searchP1[1] >= image->height - MARGIN || searchP2[1] >= image->height - MARGIN)
    {
        return false;
    }

	if (edgelThreshold > 0) {
		//check if the color is roughly the same on each side
		p1 = static_cast<short>(*image->get2D(static_cast<int>(searchP1[0] + edgeDirection[0] * 2.f), static_cast<int>(searchP1[1] + edgeDirection[1] * 2.f)));
		p2 = static_cast<short>(*image->get2D(static_cast<int>(searchP1[0] - edgeDirection[0] * 2.f), static_cast<int>(searchP1[1] - edgeDirection[1] * 2.f)));
		p3 = static_cast<short>(*image->get2D(static_cast<int>(searchP2[0] + edgeDirection[0] * 2.f), static_cast<int>(searchP2[1] + edgeDirection[1] * 2.f)));
		p4 = static_cast<short>(*image->get2D(static_cast<int>(searchP2[0] - edgeDirection[0] * 2.f), static_cast<int>(searchP2[1] - edgeDirection[1] * 2.f)));
		return abs(p1 - p2) < edgelThreshold && abs(p3 - p4) < edgelThreshold;
	}
	else {
		return true;
	}
}

/**
 * @brief Finds a connected edges and creates a continues line
 * @param image The input image
 * @param edgel The found edgel is stored here
 * @param point The seeding point where the search is started
 * @tparam T the type of the image
 * @tparam NCHAN the number of channels
 * @return if the edgel search was successful
 *
 * The method is demonstrated on the image below:
 * \image html 2_edgelgrow.png
 * A good example is the seed point under the mouse. The search is separated into two steps - up (reddish) and down (greenish) lines
 * In both directions the algorithm goes like this:
 *  -# get the edge direction estimate ( \link getSobelResponseRGB \endlink, \link getSobelResponseMax \endlink)
 *  -# based on the edge direction jump by a given number of pixels ( \link setEdgelSearchStep \endlink )
 *  -# perpendicular to the edge direction find and edge ( \link setEdgelSearchRadius \endlink ),
 *     In this step binary search is used between the two endpoints. ( \link setEdgelSearchThreshold \endlink must be the minimum distance between the endPoints)
 *     The lines between the endpoints are the short lines perpendicular to the longer lines in the image.
 *  -# if an edge is found the gradient is tested at the given point, so it is similar to the normal direction ( \link setEdgelDotThreshold \endlink)
 *  -# the normal end line direction is updated if the edge was found and use this as a seed and jump to 1.
 *  -# otherwise stop
 *
 * After in both up and down directions the edges were found the two endpoints are connected,
 * and if the edgel is long enough ( \link setEdgelLengthThreshold \endlink ) true is returned
 * and the edgel parameter is set.
 */
template<class T, int NCHAN>
bool EdgelDetector::findEdgel(Image<T, NCHAN> *image,
               Edgel &edgel,
               const Eigen::Vector2i &point, ImageGray *mask)
{
    typedef Eigen::Matrix<EdgelFType, 2, 1> Vector2fptype;

    EdgelFType h, v;
    if(point[0] <= 0 && point[1] <= 0)
    {
        return false;
    }
    //try using L1 norm
    getSobelResponseRGB(image, point[0], point[1], h, v);

    Vector2fptype normal(v, h);
    normal.normalize();
    Vector2fptype edgeDirection(-normal[1], normal[0]);

    //debugDrawLine(draw, up, down, cvScalar(255, 0, 0), 1);
    int upCount = 0;
    int downCount = 0;
    Vector2fptype step;
    Eigen::Vector2i edgeTmp;
    Eigen::Vector2i edge1 = point;
    Eigen::Vector2i edge2 = point;
    Vector2fptype searchP1;
    Vector2fptype searchP2;
    Vector2fptype next;

    int edgelLength = 0;

    //upwards direction
    for(int stepCount = 0; stepCount < this->edgelMaxSteps; stepCount++)
    {
        step = edgeDirection*(upCount + (EdgelFType)(1))*this->edgelStep;
        next = edge1.template cast<EdgelFType>() + step;

        searchP1 = (next + normal*this->edgelRadius);
        searchP2 = (next - normal*this->edgelRadius);
        bool goodPos = checkGoodBinSearchPosition(image, searchP1, searchP2, edgeDirection, this->edgelFieldValueTolerance);
        if(!goodPos)
        {
			break;
        }
        bool found = binSearchEdge(image, edgeTmp,
                                   Eigen::Vector2i(searchP1(0), searchP1(1)),
                                   Eigen::Vector2i(searchP2(0), searchP2(1)),
                                   this->edgelThreshold, this->edgelDotThreshold);

        if(!found)
        {
            break;

        } else if(mask != NULL && *mask->get2D(edgeTmp[0], edgeTmp[1]) != 255)
        {
            break;
        }
        
        edgelLength += (upCount + 1)*this->edgelStep;

        //update normal and edge direction
        edge1 = edgeTmp;
        edgeDirection = (edge1 - point).template cast<EdgelFType>();
        edgeDirection /= 1.0f*edgelLength;
        //edgeDirection.normalize();
        normal[0] = -edgeDirection[1];
        normal[1] = edgeDirection[0];

        upCount++;
    };

    //downwards
    for(int stepCount = 0; stepCount + upCount < this->edgelMaxSteps; stepCount++)
    {
        step = (downCount + 1.f)*this->edgelStep*edgeDirection;
        Vector2fptype next = edge2.template cast<EdgelFType>() - step;

        searchP1 = (next + normal*this->edgelRadius);
        searchP2 = (next - normal*this->edgelRadius);
        
        bool goodPos = checkGoodBinSearchPosition(image, searchP1, searchP2, edgeDirection, this->edgelFieldValueTolerance);
        if(!goodPos)
        {
            break;
        }
        bool found = binSearchEdge(image, edgeTmp,
                                   Eigen::Vector2i(searchP1(0), searchP1(1)),
                                   Eigen::Vector2i(searchP2(0), searchP2(1)),
                                   this->edgelThreshold, this->edgelDotThreshold);

        if(!found)
        {
            break;
        } else if(mask != NULL && *mask->get2D(edgeTmp[0], edgeTmp[1]) != 255)
        {
            break;
        }
        
        edgelLength += (downCount + 1)*this->edgelStep;

        //update normal and edge direction
        edge2 = edgeTmp;
        edgeDirection = (edge1 - edge2).template cast<EdgelFType>();
        edgeDirection /= 1.0f*edgelLength;
        //edgeDirection.normalize();

        normal[0] = -edgeDirection[1];
        normal[1] = edgeDirection[0];

        downCount++;

    }

    //if the limits are fine
    if(edgelLength >= this->edgelLengthThreshold)
    {
        //we found an edge in both directions, hurray - get the line
        Eigen::Vector2f edgeDirection = (edge1 - edge2).template cast<float>();
        edgel.normal = Eigen::Vector2f(-edgeDirection[1], edgeDirection[0]);
        edgel.normal.normalize();
        edgel.endPoints[0] = edge1.template cast<float>();
        edgel.endPoints[1] = edge2.template cast<float>();
        edgel.line = Eigen::Vector3f(normal[0], normal[1], - normal.dot(edgel.endPoints[0].template cast<EdgelFType>()));
        edgel.score = static_cast<float>(edgelLength);
        return true;
    }

    return false;
}

/**
 * @brief Find edgels using the detected edges as seedpoints
 * @tparam T type of the image
 * @tparam NCHAN the number of channels in the image
 * @param image the input image
 * @param show If debugging is enabled can be used to draw the edgels to debug output
 *
 * The result are edgels \link umf::Edgel \endlink that are further used to detect the grid.
 * For each detected edge the \link findEdgels \endlink is run.
 * \image html 3_edgels.png "The detected edgels for scanline step"
 */
template<class T, int NCHAN>
void EdgelDetector::findEdgels(Image<T, NCHAN> *image, ImageGray* mask, bool show)
{
    this->edgels.clear();

    for(unsigned int i = 0; i != this->points.size(); i++)
    {
        Edgel e;
        bool res = this->findEdgel(image, e, this->points[i], mask);
        if(res)
        {
            this->edgels.push_back(e);
        }
    }

#ifdef UMF_DEBUG
    if(show)
    {
        this->showEdgels();
    }
#endif

}

void EdgelDetector::showEdgels()
{
    UMFDebug *dbg = UMFDSingleton::Instance();
    Renderer *rend = dbg->getRenderer();

    if(rend)
    {
        Eigen::Vector3i lineColor(0, 0, 255);
        //Eigen::Vector3i lineColor(255, 0, 0);
        int lineWidth = 1;

        for(std::vector<Edgel>::iterator it = this->edgels.begin(); it != this->edgels.end(); it++)
        {
            drawLine(rend, Eigen::Vector2i(it->endPoints[0][0], it->endPoints[0][1]),
                    Eigen::Vector2i(it->endPoints[1][0], it->endPoints[1][1]), lineColor, lineWidth);
        }
    }
}

template void EdgelDetector::detectEdges(ImageRGB *image, ImageGray *mask, bool show);
template void EdgelDetector::detectEdges(ImageGray *image, ImageGray *mask, bool show);


template void EdgelDetector::detectEdges(ImageRGB *image, std::vector< LineIterator< ImageRGB > > &scanlines, ImageGray *mask, bool show);
template void EdgelDetector::detectEdges(ImageGray *image, std::vector< LineIterator< ImageGray > > &scanlines, ImageGray *mask, bool show);

template void EdgelDetector::findEdgels(ImageRGB *image, ImageGray *mask, bool show);
template void EdgelDetector::findEdgels(ImageGray *image, ImageGray *mask, bool show);
}
