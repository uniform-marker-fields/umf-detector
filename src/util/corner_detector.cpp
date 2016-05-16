#include "corner_detector.h"
#include "../marker.h"
#include "umfdebug.h"
#include "draw.h"
#include <Eigen/Dense>

#ifdef UMF_USE_OPENCV
#include <opencv/cv.h>
#endif

namespace umf {


	template <typename T> int sgn(T val) {
		return (T(0) < val) - (T(0) >= val);
	}

	/**
	* @brief Get the edge direction in horizontal and vertical direction
	* @param input the input image
	* @param x The x position in the image
	* @param y The y position in the image
	* @param h The gradient size in horizontal direction (signed)
	* @param v The gradient size in vertical direction (signed)
	* @tparam T the type of the image
	* @tparam NCHAN the number of channels in the image
	*
	* This method simply takes the channel with the maximum gradient in vertical and horizontal
	* directions separately.
	*/
	template<class T, int NCHAN>
	void getSobelResponseMax(Image<T, NCHAN> *input, int x, int y, float &h, float &v)
	{
		Eigen::Matrix<float, NCHAN, 1> AB;
		Eigen::Matrix<float, NCHAN, 1> BA;
		Eigen::Matrix<float, NCHAN, 1> BC;
		Eigen::Matrix<float, NCHAN, 1> CB;

		input->get2Dconv(AB, x, (y - 1));
		input->get2Dconv(BA, (x - 1), y);
		input->get2Dconv(BC, (x + 1), y);
		input->get2Dconv(CB, x, (y + 1));

#ifdef UMF_DEBUG_COUNT_PIXELS
		UMFDSingleton::Instance()->addPixels(4);
#endif

		Eigen::Matrix<float, NCHAN, 1>  horiz = (CB - AB);
		Eigen::Matrix<float, NCHAN, 1>  vert = (BC - BA);

		int row = 0;
		int col = 0;
		h = horiz.array().abs().maxCoeff(&row, &col);
		h *= sgn(horiz[col]);

		v = vert.array().abs().maxCoeff(&row, &col);
		v *= sgn(vert[col]);

	}

	//

	/**
	* @brief Get the edge direction in horizontal and vertical direction
	* @param input the input image
	* @param x The x position in the image
	* @param y The y position in the image
	* @param h The gradient size in horizontal direction (signed)
	* @param v The gradient size in vertical direction (signed)
	* @tparam T the type of the image
	* @tparam NCHAN the number of channels in the image
	*
	* This operator is based on Edge Detection of Color Images Using Directional Operators (J. Scharcanski and A. N. Venetsanopoulos)
	* Separates H+ and H- and in vertical direction V+ and V- (each vector based on the number of channels)
	*
	* The resulting horizontal is:
	* h = | ||H+ - H-|| if ||H+|| > ||H-||
	*     | -||H+ - H-|| else
	* Similarly in the vertical direction
	*/
	template<class fptype, class T, int NCHAN>
	void getSobelResponseRGB(Image<T, NCHAN> *input, int x, int y, fptype &h, fptype &v)
	{
		Eigen::Matrix<fptype, NCHAN, 1> AB;
		Eigen::Matrix<fptype, NCHAN, 1> BA;
		Eigen::Matrix<fptype, NCHAN, 1> BC;
		Eigen::Matrix<fptype, NCHAN, 1> CB;

		input->get2DconvScale(AB, x, (y - 1));
		input->get2DconvScale(BA, (x - 1), y);
		input->get2DconvScale(BC, (x + 1), y);
		input->get2DconvScale(CB, x, (y + 1));
#ifdef UMF_DEBUG_COUNT_PIXELS
		UMFDSingleton::Instance()->addPixels(4);
#endif

		Eigen::Matrix<fptype, NCHAN, 1>  horiz = (CB - AB);
		Eigen::Matrix<fptype, NCHAN, 1>  vert = (BC - BA);

		h = sgn(CB.squaredNorm() - AB.squaredNorm())*horiz.norm();
		v = sgn(BC.squaredNorm() - BA.squaredNorm())*vert.norm();
	}

	template<int fixedprecision, class T, int NCHAN>
	void getSobelResponseRGB(Image<T, NCHAN> *input, int x, int y, fixp::fixed_point<fixedprecision> &h, fixp::fixed_point<fixedprecision> &v)
	{
		typedef fixp::fixed_point<fixedprecision> fptype;

		Eigen::Matrix<fptype, NCHAN, 1> AB;
		Eigen::Matrix<fptype, NCHAN, 1> BA;
		Eigen::Matrix<fptype, NCHAN, 1> BC;
		Eigen::Matrix<fptype, NCHAN, 1> CB;

		input->get2DconvScale(AB, x, (y - 1));
		input->get2DconvScale(BA, (x - 1), y);
		input->get2DconvScale(BC, (x + 1), y);
		input->get2DconvScale(CB, x, (y + 1));
#ifdef UMF_DEBUG_COUNT_PIXELS
		UMFDSingleton::Instance()->addPixels(4);
#endif

		Eigen::Matrix<fptype, NCHAN, 1>  horiz = (CB - AB);
		Eigen::Matrix<fptype, NCHAN, 1>  vert = (BC - BA);

		//horiz >>= 4;
		//vert >>= 4;

		h = sgn(CB.squaredNorm() - AB.squaredNorm())*horiz.norm();
		v = sgn(BC.squaredNorm() - BA.squaredNorm())*vert.norm();
	}


	template<>
	void getSobelResponseRGB(ImageGray *input, int x, int y, float &h, float &v)
	{
		short AB, BA, BC, CB;

		AB = static_cast<short>(*input->get2D(x, (y - 1)));
		BA = static_cast<short>(*input->get2D((x - 1), y));
		BC = static_cast<short>(*input->get2D((x + 1), y));
		CB = static_cast<short>(*input->get2D(x, (y + 1)));
		h = static_cast<float>(CB - AB);
		v = static_cast<float>(BC - BA);
	}

	template<>
	void getSobelResponseRGB(ImageGray *input, int x, int y, fixp::fixp_16 &h, fixp::fixp_16 &v)
	{
		//    typedef fixp::fixed_point<P> fptype;
		short AB, BA, BC, CB;

		AB = static_cast<short>(*input->get2D(x, (y - 1)));
		BA = static_cast<short>(*input->get2D((x - 1), y));
		BC = static_cast<short>(*input->get2D((x + 1), y));
		CB = static_cast<short>(*input->get2D(x, (y + 1)));
		h = (static_cast< fixp::fixp_16 >(CB - AB)) >> 4;
		v = (static_cast< fixp::fixp_16 >(BC - BA)) >> 4;
	}


template<class T, int NCHAN>
bool binSearchEdge(Image<T, NCHAN> *image,
                   Eigen::Vector2i &edge,
                   Eigen::Vector2i left,
                   Eigen::Vector2i right,
                   const int searchThreshold,
				   const float angleThreshold)
{

    typedef fixp::fixp_8 fptype;
    
    //test for out of bound coordinates
	const int MARGIN = 1;
    if(left[0] < MARGIN || right[0] < MARGIN || left[1] < MARGIN || right[1] < MARGIN ||
            left[0] >= image->width - MARGIN || right[0] >= image->width - MARGIN ||
            left[1] >= image->height - MARGIN || right[1] >= image->height - MARGIN)
    {
        return false;
    }


    Eigen::Vector2f lineNormal(left[0] - right[0], left[1] - right[1]);
    
    const fptype searchThresholdSquared = fptype(searchThreshold*searchThreshold);
    const fptype fix_2 = fptype(0.5f);

    Eigen::Matrix<fptype, NCHAN, 1> leftVal;
    image->get2Dconv(leftVal, left[0], left[1]);

    Eigen::Matrix<fptype, NCHAN, 1> rightVal;
    image->get2Dconv(rightVal, right[0], right[1]);

#ifdef UMF_DEBUG_COUNT_PIXELS
    int pixelCounter = 2;
#endif

    if((leftVal - rightVal).squaredNorm() < searchThresholdSquared)
    {
        return false;
    }

    Eigen::Matrix<fptype, NCHAN, 1> thresh = (leftVal + rightVal)*fix_2;

    Eigen::Vector2i currMid = (left + right);
    currMid[0] >>= 1;
    currMid[1] >>= 1;

    Eigen::Matrix<fptype, NCHAN, 1> currVal;
    image->get2Dconv(currVal, currMid[0], currMid[1]);
#ifdef UMF_DEBUG_COUNT_PIXELS
    pixelCounter++;
#endif

    //now binary search for a point, that should be our edge pixel
    while( (currVal - thresh).squaredNorm() > fptype(1) && (left - right).squaredNorm() > 2)
    {
        fptype diff1 = (leftVal - currVal).squaredNorm();
        fptype diff2 = (rightVal - currVal).squaredNorm();

        if(diff1 > diff2) //the current value is closer to the right part
        {
            right = currMid;
        }
        else
        {
            left = currMid;
        }


        currMid = (left + right);
        currMid[0] >>= 1;
        currMid[1] >>= 1;
        image->get2Dconv(currVal, currMid[0], currMid[1]);

#ifdef UMF_DEBUG_COUNT_PIXELS
        pixelCounter++;
#endif
    }
#ifdef UMF_DEBUG_COUNT_PIXELS
    UMFDSingleton::Instance()->addPixels(pixelCounter);
#endif

    //now we got the edge point position, save it
    edge = currMid;

    if (angleThreshold > 0) {
        float h, v;
        //try using L1 norm
        getSobelResponseRGB(image, edge[0], edge[1], h, v);

        Eigen::Vector2f normal(v, h);
        normal.normalize();
        lineNormal.normalize();
        if (fabs(normal.dot(lineNormal)) < angleThreshold) {
            return false;
        }
    }

    return true;
}

template<>
bool binSearchEdge(ImageGray *image,
                   Eigen::Vector2i &edge,
                   Eigen::Vector2i left,
                   Eigen::Vector2i right,
                   const int searchThreshold,
				   const float angleThreshold)
{
   
    //test for out of bound coordinates
	const int MARGIN = 1;
	if (left[0] < MARGIN || right[0] < MARGIN || left[1] < MARGIN || right[1] < MARGIN ||
		left[0] >= image->width - MARGIN || right[0] >= image->width - MARGIN ||
		left[1] >= image->height - MARGIN || right[1] >= image->height - MARGIN)
	{
		return false;
	}
    
    short leftVal = static_cast<short>(*image->get2D(left[0], left[1]));

    short rightVal = static_cast<short>(*image->get2D(right[0], right[1]));

    if(leftVal > rightVal)
    {
        std::swap(leftVal, rightVal);
        std::swap(left, right);
    }

	Eigen::Vector2f lineNormal(left[0] - right[0], left[1] - right[1]);

#ifdef UMF_DEBUG_COUNT_PIXELS
    int pixelCounter = 2;
#endif

    if(std::abs(leftVal - rightVal) < searchThreshold)
    {
        return false;
    }

    short thresh = (leftVal + rightVal) >> 1;

    Eigen::Vector2i currMid = (left + right);   
    currMid[0] >>= 1;
    currMid[1] >>= 1;

    short currVal = static_cast<short>(*image->get2D(currMid[0], currMid[1]));
#ifdef UMF_DEBUG_COUNT_PIXELS
    pixelCounter++;
#endif

    //now binary search for a point, that should be our edge pixel
    while( (std::abs)(currVal - thresh) > 1 && (left - right).squaredNorm() > 2)
    {

        if(currVal > thresh) //the current value is closer to the right part
        {
            right = currMid;
        }
        else
        {
            left = currMid;
        }


        currMid = (left + right);
        currMid[0] >>= 1;
        currMid[1] >>= 1;
        currVal = static_cast<short>(*image->get2D(currMid[0], currMid[1]));

#ifdef UMF_DEBUG_COUNT_PIXELS
        pixelCounter++;
#endif
    }
#ifdef UMF_DEBUG_COUNT_PIXELS
    UMFDSingleton::Instance()->addPixels(pixelCounter);
#endif

    //now we got the edge point position, save it
    edge = currMid;

	if (angleThreshold > 0) {
		float h, v;
		//try using L1 norm
		getSobelResponseRGB(image, edge[0], edge[1], h, v);

		Eigen::Vector2f normal(v, h);
		normal.normalize();
		lineNormal.normalize();
		if (fabs(normal.dot(lineNormal)) < angleThreshold) {
			return false;
		}
	}

    return true;
}

template<class T, int NCHAN>
Eigen::Vector2f findMarkerCorner22(Image<T,NCHAN> *srcImg, std::vector<Eigen::Vector2f> &points,
	const int intensityThreshold, const float binsearchDotThreshold, const float angleThreshold)
{
    Eigen::Vector2i edges[4];
    for(int i = 0; i < 4; i++)
    {
        if(!binSearchEdge(srcImg, edges[i], points[i].template cast<int>(), points[(i+1)%4].template cast<int>(), intensityThreshold, binsearchDotThreshold))
        {
            //std::cout << "Error finding 2/2" << std::endl;
            return Eigen::Vector2f(-1, -1);
        }
    }

    Eigen::Vector3f line1 = Eigen::Vector3f(static_cast<float>(edges[0][0]), static_cast<float>(edges[0][1]), 1.0f).cross(Eigen::Vector3f(static_cast<float>(edges[2][0]), static_cast<float>(edges[2][1]), 1.0f));
    Eigen::Vector3f line2 = Eigen::Vector3f(static_cast<float>(edges[1][0]), static_cast<float>(edges[1][1]), 1.0f).cross(Eigen::Vector3f(static_cast<float>(edges[3][0]), static_cast<float>(edges[3][1]), 1.0f));

    Eigen::Vector3f interPoint = line1.cross(line2);

	Eigen::Vector2f line1Norm(line1[0], line1[1]);
	Eigen::Vector2f line2Norm(line2[0], line2[1]);
	line1Norm.normalize();
	line2Norm.normalize();

	Eigen::Vector2f line1Orig = points[2] - points[1];
	Eigen::Vector2f line2Orig = points[1] - points[0];
	line1Orig.normalize();
	line2Orig.normalize();

	if(std::abs(line1Orig.dot(line1Norm)) > angleThreshold ||
            std::abs(line2Orig.dot(line2Norm)) > angleThreshold)
    {
        return Eigen::Vector2f(-1, -1);
    }

    return Eigen::Vector2f(interPoint[0]/interPoint[2], interPoint[1]/interPoint[2]);
}


template<class T, int NCHAN>
Eigen::Vector2f findMarkerCorner13(Image<T,NCHAN> *srcImg, std::vector<Eigen::Vector2f> &points, int index,
	const int intensityThreshold, const float binsearchDotThreshold, const float angleThreshold)
{
    int baseIndexes[2];

    baseIndexes[0] = index;

    baseIndexes[1] = (index + 3) % 4;

    Eigen::Vector2i edges[4];
    for(int pi = 0; pi < 2; pi++)
    {
        int i = baseIndexes[pi];

        Eigen::Vector2f move1 = 0.25*(points[(i+3)%4] - points[i]);
        Eigen::Vector2f move2 = 0.25*(points[(i+2)%4] - points[(i + 1)%4]);

        if(! binSearchEdge(srcImg, edges[pi*2], (points[i] - move1).template cast<int>(), (points[(i+1)%4] - move2).template cast<int>(), intensityThreshold, binsearchDotThreshold))
        {
            //std::cout << "simple not found" << std::endl;
            return Eigen::Vector2f(-1, -1);
        }
        //move our seed points in on one direction
        if(! binSearchEdge(srcImg, edges[pi*2 + 1], (points[i] + move1).template cast<int>(), (points[(i+1)%4] + move2).template cast<int>(), intensityThreshold))
        {
            //std::cout << "Moved not found" << std::endl;
            return Eigen::Vector2f(-1, -1);
        }
    }

    Eigen::Vector3f line1 = Eigen::Vector3f(static_cast<float>(edges[0][0]), static_cast<float>(edges[0][1]), 1.0f).cross(Eigen::Vector3f(static_cast<float>(edges[1][0]), static_cast<float>(edges[1][1]), 1.0f));
    Eigen::Vector3f line2 = Eigen::Vector3f(static_cast<float>(edges[2][0]), static_cast<float>(edges[2][1]), 1.0f).cross(Eigen::Vector3f(static_cast<float>(edges[3][0]), static_cast<float>(edges[3][1]), 1.0f));

    Eigen::Vector3f interPoint = line1.cross(line2);

	Eigen::Vector3f crossLine1 = Eigen::Vector3f(static_cast<float>(points[0][0]), static_cast<float>(points[0][1]), 1.0f).cross(Eigen::Vector3f(static_cast<float>(points[2][0]), static_cast<float>(points[2][1]), 1.0f));
	Eigen::Vector3f crossLine2 = Eigen::Vector3f(static_cast<float>(points[1][0]), static_cast<float>(points[1][1]), 1.0f).cross(Eigen::Vector3f(static_cast<float>(points[3][0]), static_cast<float>(points[3][1]), 1.0f));

	Eigen::Vector3f crossIntersection = crossLine1.cross(crossLine2);

	if(interPoint[2] == 0 || crossIntersection[2] == 0)
	{
		return Eigen::Vector2f(-1, -1);
	}

	//test if we are not too far from the center position
	interPoint *= 1.0f/interPoint[2];
	crossIntersection *= 1.0f/crossIntersection[2];

	Eigen::Vector2f diff = (interPoint - crossIntersection).block(0,0,2,1);
	float diffsq = diff.dot(diff);
	
	Eigen::Vector2f edge1 = (points[1] - points[0]);
	Eigen::Vector2f edge2 = (points[2] - points[1]);

	float edgesq1 = 0.25f*edge1.dot(edge1);
	float edgesq2 = 0.25f*edge2.dot(edge2);
	
	if(diffsq > edgesq1 || diffsq > edgesq2)
	{
		return Eigen::Vector2f(-1, -1);
	}
	//check dot threshold

	Eigen::Vector2f line1Norm(line1[0], line1[1]);
	Eigen::Vector2f line2Norm(line2[0], line2[1]);
	line1Norm.normalize();
	line2Norm.normalize();

	int pindex = baseIndexes[0];
	Eigen::Vector2f line1Orig = points[(pindex+3)%4] - points[pindex];
	pindex = baseIndexes[1];
	Eigen::Vector2f line2Orig = points[(pindex+3)%4] - points[pindex];
	line1Orig.normalize();
	line2Orig.normalize();

	if(std::abs(line1Orig.dot(line1Norm)) > angleThreshold ||
            std::abs(line2Orig.dot(line2Norm)) > angleThreshold)
    {
        return Eigen::Vector2f(-1, -1);
    }

    //test if the point is inside the quad defined by the positions
    for(int i = 0; i < 4; i++){
        Eigen::Vector3f point1(points[i][0], points[i][1], 1.0f);
        Eigen::Vector3f point2(points[(i + 1)%4][0], points[(i + 1)%4][1], 1.0f);
		Eigen::Vector3f interLine = interPoint - point1;
		Eigen::Vector3f origLine = point2 - point1;

        if(origLine.cross(interLine)[2] <= 0) //meaning the point is outside the quad or on the quad connecting line
        {
            return Eigen::Vector2f(-1, -1);
        }
    }
    if(interPoint[2] == 0)
    {
        return Eigen::Vector2f(-1, -1);
    }

    return Eigen::Vector2f(interPoint[0]/interPoint[2], interPoint[1]/interPoint[2]);
}

template<class T, int NCHAN>
Eigen::Vector2f findMarkerCorner(Image<T, NCHAN> *srcImg, std::vector<Eigen::Vector2f> &points, int cornerType,
	const int intensityThreshold, const float binsearchDotThreshold, const float angleThreshold)
{

    if(cornerType == CORNER_TYPE_NONE)
    {
        return Eigen::Vector2f(-1, -1);
    }

    Eigen::Vector2f cornerP;

    if(cornerType == CORNER_TYPE_CROSS)
    {
        cornerP = findMarkerCorner22(srcImg, points, intensityThreshold, binsearchDotThreshold, angleThreshold);
    } else {
        cornerP = findMarkerCorner13(srcImg, points, cornerType - CORNER_TYPE_LEFT_TOP, intensityThreshold, binsearchDotThreshold, angleThreshold);
    }

    return cornerP;
}

template<class T, int NCHAN>
int findCornersSubpixel(Image<T, NCHAN> *srcImg, std::vector<Eigen::Vector2f> &points, const unsigned int windowHalfSize)
{
	int successCount = 0;
#ifdef UMF_USE_OPENCV
	
	CvPoint2D32f *cvpoints = new CvPoint2D32f[points.size()];
	int* point_indexes = new int[points.size()];
	int point_counter = 0;
	for(unsigned int ind = 0; ind < points.size(); ind++)
	{
		if (points[ind][0] < 0 || points[ind][1] < 0 || points[ind][0] >= srcImg->width || points[ind][1] >= srcImg->height) {
			point_indexes[ind] = -1;
			continue;
		}
		cvpoints[point_counter] = cvPoint2D32f(points[ind][0], points[ind][1]);
		point_indexes[ind] = point_counter;
		point_counter++;
	}

	if (point_counter == 0) {
		return 0;
	}

	IplImage *imgCV = cvCreateImageHeader(cvSize(srcImg->width, srcImg->height), IPL_DEPTH_8U, NCHAN);
	imgCV->widthStep = srcImg->widthstep;
	imgCV->imageData = imgCV->imageDataOrigin = srcImg->data;
	
	IplImage *srcGray;
	bool releaseGray = false;
	if(NCHAN == 3)
	{
		srcGray = cvCreateImage(cvSize(srcImg->width, srcImg->height), IPL_DEPTH_8U, 1);
		cvCvtColor(imgCV, srcGray, CV_RGB2GRAY);
		releaseGray = true;
	} else if (NCHAN == 1)
	{
		srcGray = imgCV;
	}
	
	cvFindCornerSubPix(srcGray, cvpoints, point_counter, cvSize(windowHalfSize, windowHalfSize), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 50, 1e-4));

	if(releaseGray)
	{
		cvReleaseImage(&srcGray);
	}
	cvReleaseImageHeader(&imgCV);

	for(unsigned int ind = 0; ind < points.size(); ind++)
	{
		Eigen::Vector2f & cornerP = points[ind];
		CvPoint2D32f &p = cvpoints[point_indexes[ind]];
		if (point_indexes[ind] < 0 || p.x < 0 || p.y < 0 || (p.x > srcImg->width-1) || (p.y > srcImg->height - 1)) {
			cornerP = Eigen::Vector2f(-1, -1);
			continue;
		}
		
		if (p.x == cornerP[0] && p.y == cornerP[1]) {
			//probably a fail, so opencv reset the original, but we have no way to tell
			//cornerP = Eigen::Vector2f(-1, -1);
			//continue;
		}

		cornerP[0] = p.x;
		cornerP[1] = p.y;

		successCount += 1;
	}
#else
	const int CORNER_SEARCH_WINDOW_HALF = windowHalfSize;
	const int CORNER_SEARCH_WINDOW_SIZE = 2*CORNER_SEARCH_WINDOW_HALF + 1;
	const int CORNER_SEARCH_WINDOW_SQR = CORNER_SEARCH_WINDOW_HALF*CORNER_SEARCH_WINDOW_HALF;

	const int SRC_EXTRA_BORDER = windowHalfSize/2;
	const int GRADIENT_BUFFER_SIZE = CORNER_SEARCH_WINDOW_SIZE + 2*SRC_EXTRA_BORDER;
	const int SRC_WINDOW_SIZE = CORNER_SEARCH_WINDOW_SIZE + 2*(1 + SRC_EXTRA_BORDER);
	const int SRC_WINDOW_HALF = SRC_WINDOW_SIZE >> 1;
	const int CORNER_SEARCH_MAX_ITER_COUNT = 20;
	const float CORNER_SEARCH_EPS = 0.01f*0.01f;

	static bool inited = false;
	static float searchMask[CORNER_SEARCH_WINDOW_SIZE*CORNER_SEARCH_WINDOW_SIZE];

	T srcBuffer[SRC_WINDOW_SIZE][SRC_WINDOW_SIZE][NCHAN];
	float gXBuffer[GRADIENT_BUFFER_SIZE][GRADIENT_BUFFER_SIZE][NCHAN];
	float gYBuffer[GRADIENT_BUFFER_SIZE][GRADIENT_BUFFER_SIZE][NCHAN];

	if(!inited)
	{
		inited = true;
		double coeff = 1.0/(CORNER_SEARCH_WINDOW_SQR);
		float maskX[CORNER_SEARCH_WINDOW_SIZE];
		for(int i = -CORNER_SEARCH_WINDOW_HALF, k = 0; i <= CORNER_SEARCH_WINDOW_HALF; i++, k++)
		{
			maskX[k] = (float)exp(-i*i*coeff);
		}
		for(int i = 0; i < CORNER_SEARCH_WINDOW_SIZE; i++)
		{
			for(int j = 0; j < CORNER_SEARCH_WINDOW_SIZE; j++)
			{
				searchMask[i*CORNER_SEARCH_WINDOW_SIZE + j] = maskX[i]*maskX[j];
			}
		}
	}

	std::vector<Eigen::Vector2f> pointsCopy = points;

	Eigen::Vector2f cI;
	/* do optimization loop for all the points */
	for( std::vector<Eigen::Vector2f>::iterator cornerIt = points.begin(); cornerIt != points.end(); cornerIt++ )
    {
        const Eigen::Vector2f &cT = *cornerIt; cI[0] = cT[0], cI[1] = cT[1];
        int iter = 0;
        double err;
		
		Eigen::Vector2f topleft(cI[0] - SRC_WINDOW_HALF, cI[1] - SRC_WINDOW_HALF);

		//extract the subimage at the given position
		if( topleft[0] < 0 || topleft[1] < 0 || (topleft[0] + SRC_WINDOW_SIZE) >= srcImg->width || (topleft[1] + SRC_WINDOW_SIZE) >= srcImg->height)
		{
			//too close to the edge
			cI[0] = -1;
			cI[1] = -1;
			iter = CORNER_SEARCH_MAX_ITER_COUNT;
			continue;
		}
		
		//extract from the image
		for(int y = 0; y < SRC_WINDOW_SIZE; y++)
		{
			T* pt = srcImg->get2D(static_cast<int>(topleft[0]), static_cast<int>(topleft[1] + y));
			for(int x = 0; x < SRC_WINDOW_SIZE; x++)
			{
				for(int c = 0; c < NCHAN; c++, pt++)
				{
					srcBuffer[y][x][c] = *pt;
				}
			}
		}


		float tempBuffer[4][NCHAN];
		int divmask = 0x3;
		//compute gradient x direction
		for(int y = 1; y < SRC_WINDOW_SIZE - 1; y++)
		{
			
			int curr = 0;
			for(int x = 0; x < 2; x++)
			{
				for(int c = 0; c < NCHAN; c++)
				{
					tempBuffer[x][c] = static_cast<float>(srcBuffer[y][x][c])*0.5f;
				}
			}
			for(int x = 2; x < SRC_WINDOW_SIZE; x++)
			{
				int other = (curr + 2) & divmask;
				for(int c = 0; c < NCHAN; c++)
				{
					tempBuffer[other][c] = static_cast<float>(srcBuffer[y][x][c])*0.5f;
					gXBuffer[y - 1][x - 2][c] = -tempBuffer[curr][c] + tempBuffer[other][c];
				}
				curr = (curr + 1) & divmask;
			}
		}

		//compute gradient y direction
		for(int x = 1; x < SRC_WINDOW_SIZE - 1; x++)
		{
			
			int curr = 0;
			for(int y = 0; y < 2; y++)
			{
				for(int c = 0; c < NCHAN; c++)
				{
					tempBuffer[y][c] = static_cast<float>(srcBuffer[y][x][c])*0.5f;
				}
			}
			for(int y = 2; y < SRC_WINDOW_SIZE; y++)
			{
				int other = (curr + 2) & divmask;
				for(int c = 0; c < NCHAN; c++)
				{
					tempBuffer[other][c] = static_cast<float>(srcBuffer[y][x][c])*0.5f;
					gYBuffer[y - 2][x - 1][c] = -tempBuffer[curr][c] + tempBuffer[other][c];
				}
				curr = (curr + 1) & divmask;
			}
		}


        do
        {
            Eigen::Vector2f cI2;
            float a, b, c, bb1, bb2;

            a = b = c = bb1 = bb2 = 0;

			Eigen::Vector2f offset = cI - cT;

			if( offset[0] >= SRC_EXTRA_BORDER || offset[0] <= -SRC_EXTRA_BORDER ||
				offset[1] >= SRC_EXTRA_BORDER || offset[1] <= -SRC_EXTRA_BORDER)
			{
				//diverging, not good
				cI[0] = -1;
				cI[1] = -1;
				iter = CORNER_SEARCH_MAX_ITER_COUNT;
				successCount--;
				continue;
			}

			int offsetX = static_cast<int>( SRC_EXTRA_BORDER + offset[0]);
			int offsetY = static_cast<int>( SRC_EXTRA_BORDER + offset[1]);

			int offsetXCloser = static_cast<int>( SRC_EXTRA_BORDER + offset[0] + 0.5f);
			int offsetYCloser = static_cast<int>( SRC_EXTRA_BORDER + offset[1] + 0.5f);

			float divX = offset[0] - std::floor(offset[0]);
			float divX1 = (1.f - divX);
			float divY = offset[1] - std::floor(offset[1]);
			float divY1 = (1.f - divY);

            /* process gradient */
			for(int i = 0, k = 0; i < CORNER_SEARCH_WINDOW_SIZE; i++ )
            {
				float py = static_cast<float>(i - CORNER_SEARCH_WINDOW_HALF);

				for(int j = 0; j < CORNER_SEARCH_WINDOW_SIZE; j++, k++ )
                {
                    float m = searchMask[k];
					for(int chan = 0; chan < NCHAN; chan++)
					{
						//just linear interpolation hopefully enough
						//float tgx = (gXBuffer[i + offsetY][j + offsetX][chan]*divX1 + gXBuffer[i + offsetY][j + offsetX + 1][chan]*divX)*divY1
						//	+ (gXBuffer[i + offsetY + 1][j + offsetX][chan]*divX1 + gXBuffer[i + offsetY + 1][j + offsetX + 1][chan]*divX)*divY;
						float tgx = (gXBuffer[i + offsetYCloser][j + offsetX][chan]*divX1 + gXBuffer[i + offsetYCloser][j + offsetX + 1][chan]*divX);
						
						//float tgy = (gYBuffer[i + offsetY][j + offsetX][chan]*divY1 + gYBuffer[i + offsetY + 1][j + offsetX][chan]*divY)*divX1
						//	+ (gYBuffer[i + offsetY][j + offsetX + 1][chan]*divY1 + gYBuffer[i + offsetY + 1][j + offsetX + 1][chan]*divY)*divX;
						float tgy = (gYBuffer[i + offsetY][j + offsetXCloser][chan]*divY1 + gYBuffer[i + offsetY + 1][j + offsetXCloser][chan]*divY);

						float gxx = tgx * tgx * m;
						float gxy = tgx * tgy * m;
						float gyy = tgy * tgy * m;
						float px = static_cast<float>(j - CORNER_SEARCH_WINDOW_HALF);

						a += gxx;
						b += gxy;
						c += gyy;

						bb1 += gxx * px + gxy * py;
						bb2 += gxy * px + gyy * py;
					}
                }
            }

            {
				Eigen::Matrix2f A;
				A(0, 0) = a;
				A(0, 1) = A(1, 0) = b;
				A(1, 1) = c;
				Eigen::Matrix2f InvA = A.inverse();

                cI2[0] = (float)(cI[0] + InvA(0, 0)*bb1 + InvA(0, 1)*bb2);
                cI2[1] = (float)(cI[1] + InvA(1, 0)*bb1 + InvA(1, 1)*bb2);
            }

            err = (cI2[0] - cI[0]) * (cI2[0] - cI[0]) + (cI2[1] - cI[1]) * (cI2[1] - cI[1]);
            cI = cI2;
        }
        while( ++iter < CORNER_SEARCH_MAX_ITER_COUNT && err > CORNER_SEARCH_EPS );

		*cornerIt = cI;     /* store result */
		successCount++;
    }

#endif

	return successCount;
}

//INSTANCING

template Eigen::Vector2f findMarkerCorner(ImageGray *srcImg, std::vector<Eigen::Vector2f> &points, int cornerType, const int intensityThreshold, const float binsearchDotThreshold, const float angleThreshold);
template Eigen::Vector2f findMarkerCorner(ImageRGB *srcImg, std::vector<Eigen::Vector2f> &points, int cornerType, const int intensityThreshold, const float binsearchDotThreshold, const float angleThreshold);

template int findCornersSubpixel(ImageGray *srcImg, std::vector<Eigen::Vector2f> &points, const unsigned int windowHalfSize);
template int findCornersSubpixel(ImageRGB *srcImg, std::vector<Eigen::Vector2f> &points, const unsigned int windowHalfSize);

//already implemented for better performance template bool binSearchEdge(ImageGray *image, Eigen::Vector2i &edge, Eigen::Vector2i left, Eigen::Vector2i right, const int searchThreshold); 
template bool binSearchEdge(ImageRGB *image, Eigen::Vector2i &edge, Eigen::Vector2i left, Eigen::Vector2i right, const int searchThreshold, const float angleThreshold);



//template void getSobelResponseRGB(ImageGray *input, int x, int y, float &h, float &v);
template void getSobelResponseRGB(ImageRGB *input, int x, int y, float &h, float &v);
//template void getSobelResponseRGB(ImageGray *input, int x, int y, fixedpoint::fixp_16 &h, fixedpoint::fixp_16 &v);
template void getSobelResponseRGB(ImageRGB *input, int x, int y, fixedpoint::fixp_16 &h, fixedpoint::fixp_16 &v);


}//namespace
