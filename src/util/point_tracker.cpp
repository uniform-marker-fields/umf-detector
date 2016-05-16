#include "point_tracker.h"

#include "corner_detector.h"
#include "umfdebug.h"
#include <Eigen/Dense>
#include "native_x.h"

#ifdef UMF_USE_OPENCV
#include <opencv/cv.h>
#endif

namespace umf {

template<class TP, class TP2, int CHAN, int SUB>
void convertScale(Image<TP, CHAN> *orig, Image<TP2, CHAN> *nimg)
{
	assert((orig->width == nimg->width*SUB) && (orig->height == nimg->height*SUB));
	Eigen::Matrix<float, CHAN, 1> sum;
	Eigen::Matrix<float, CHAN, 1> curr;
	const int subsq = SUB*SUB;
	for(int y = 0; y < nimg->height; y++)
	{
		for(int x = 0; x < nimg->width; x++)
		{
			sum.setZero();
			for(int i = 0; i < SUB; i++)
			{
				for(int j = 0; j < SUB; j++)
				{
					orig->get2Dconv(curr, x*SUB+i, y*SUB+j);
					sum += curr;
				}
			}
			sum *= 1.0f/subsq;
			TP2* pp = (TP2*) nimg->get2D(x, y);
			for(int c = 0; c < CHAN; c++, pp++)
			{
				*pp = static_cast<TP2>(sum[c]);
			}
		}
	}
}

template <>
void convertScale<unsigned char, unsigned char, 1, 2>(ImageGray *orig, ImageGray *nimg)
{
	const int SUB = 2;
	assert((orig->width == nimg->width*SUB) && (orig->height == nimg->height*2));
	unsigned short curr;
	for(int y = 0; y < nimg->height; y++)
	{
		for(int x = 0; x < nimg->width; x++)
		{
			unsigned char *p = orig->get2D(x*SUB, y*SUB);
			curr = static_cast<unsigned short>(*p) + static_cast<unsigned short>(*(p + 1));
			p += orig->widthstep;
			curr += static_cast<unsigned short>(*p) + static_cast<unsigned short>(*(p + 1));
			
			unsigned char* pp = nimg->get2D(x, y);
			
			*pp = static_cast<unsigned char>(curr >> 2);
		}
	}
}

template<class TP>
void smooth3x3(Image<TP, 1> *img, Image<TP, 1> *imgres)
{
	//smooth horizontally
	float tempBuffer[4];
	const unsigned char divmask = 0x3;
	register unsigned char curr = 0;
	//smooth in x direction
	for(int y = 0; y < img->height; y++)
	{
		curr = 0;
		TP *currP = img->get2D(0, y);
        TP *currPr = imgres->get2D(0, y);
        for(int x = 0; x < 2; x++, currP++, currPr++)
		{
			tempBuffer[x] = static_cast<float>(*currP)*0.25f;
		}
        for(int x = 2; x < img->width; x++, currP++, currPr++)
		{
			unsigned char other = (curr + 2) & divmask;
			unsigned char middle = (curr + 1) & divmask;
			tempBuffer[other] = static_cast<float>(*currP)*0.25f;
			float result = tempBuffer[curr] + 2*tempBuffer[middle] + tempBuffer[other];
            *(currPr - 2) = static_cast<TP>(result);
			curr = middle;
		}
	}

    int offset = -imgres->widthstep*2;
	//smooth in y direction
	for(int x = 0; x < img->width; x++)
	{	
		curr = 0;
        char *currPch = (char *) imgres->get2D(x, 0);
        for(int y = 0; y < 2; y++, currPch += imgres->widthstep)
		{
			tempBuffer[y] = static_cast<float>(*((TP *)(currPch)))*0.25f;
		}
        for(int y = 2; y < img->height; y++, currPch += imgres->widthstep)
		{
			unsigned char other = (curr + 2) & divmask;
			unsigned char middle = (curr + 1) & divmask;
			tempBuffer[other]= static_cast<float>(*((TP *)(currPch)))*0.25f;
			
			float result = tempBuffer[curr] + 2*tempBuffer[middle] + tempBuffer[other];

			*((TP *)(currPch + offset)) = static_cast<TP>(result);
			curr = middle;
		}
	}
}


template<>
void smooth3x3(ImageGray *img, ImageGray *imgres)
{
	//smooth horizontally
	short tempBuffer[4];
	const unsigned char divmask = 0x3;
	register unsigned char curr = 0;
	//smooth in x direction
	for (int y = 0; y < img->height; y++)
	{
		curr = 0;
		unsigned char *currP = (unsigned char*) &(img->data[img->widthstep*y]);
		unsigned char *currPr = (unsigned char*) &(imgres->data[img->widthstep*y]);  
		for (int x = 0; x < 2; x++, currP++, currPr++)
		{
			tempBuffer[x] = *currP;
		}
		for (int x = 2; x < img->width; x++, currP++, currPr++)
		{
			unsigned char other = (curr + 2) & divmask;
			unsigned char middle = (curr + 1) & divmask;
			tempBuffer[other] = (*currP);
			short result = (tempBuffer[curr] + 2 * tempBuffer[middle] + tempBuffer[other]) >> 2;
			*(currPr - 2) = static_cast<unsigned char>(result);
			curr = middle;
		}
	}

	int offset = -imgres->widthstep * 2;
	//smooth in y direction
	for (int x = 0; x < img->width; x++)
	{
		curr = 0;
		unsigned char *currPch = (unsigned char*) &(imgres->data[x]);
		for (int y = 0; y < 2; y++, currPch += imgres->widthstep)
		{
			tempBuffer[y] = (*currPch);
		}
		for (int y = 2; y < img->height; y++, currPch += imgres->widthstep)
		{
			unsigned char other = (curr + 2) & divmask;
			unsigned char middle = (curr + 1) & divmask;
			tempBuffer[other] = (*currPch);

			short result = (tempBuffer[curr] + 2 * tempBuffer[middle] + tempBuffer[other]) >> 2;

			*(currPch + offset) = static_cast<unsigned char>(result);
			curr = middle;
		}
	}
}

template<class ITP, class GTP>
void computeGradients3x3(Image<ITP, 1> *img, Image<GTP, 1> *gradX, Image<GTP, 1> *gradY)
{
	//smooth horizontally
	GTP tempBuffer[4];
	const unsigned char divmask = 0x3;
	const GTP quart = GTP(0.25f);
	register unsigned char curr = 0;
	//smooth in x direction
	for(int y = 0; y < img->height; y++)
	{
			
		curr = 0;
		ITP *currP = img->get2D(0, y);
		GTP *currX = gradX->get2D(0, y);
		*currX = static_cast<GTP>(0);
		for(int x = 0; x < 2; x++, currP++)
		{
			tempBuffer[x] = static_cast<GTP>(*currP)*quart;
		}

		currX++;
		for(int x = 2; x < img->width; x++, currP++, currX++)
		{
			unsigned char other = (curr + 2) & divmask;
			tempBuffer[other] = static_cast<GTP>(*currP)*quart;
			*currX = static_cast<GTP>(-tempBuffer[curr] + tempBuffer[other]);
			curr = (curr + 1) & divmask;
		}
		*currX = static_cast<GTP>(0);
	}

	//smooth in y direction
	for(int x = 0; x < img->width; x++)
	{	
		short curr = 0;
		char *currPch = (char *) img->get2D(x, 0);
		GTP *currY = (GTP *) gradY->get2D(0, x);
		*currY = static_cast<GTP>(0);
		currY++;

		for(int y = 0; y < 2; y++, currPch += img->widthstep)
		{
			tempBuffer[y] = static_cast<GTP>(*((ITP *)(currPch)))*quart;
		}

		for(int y = 2; y < img->height; y++, currPch += img->widthstep, currY++)
		{
			int other = (curr + 2) & divmask;
			tempBuffer[other]= static_cast<GTP>(*((ITP *)(currPch)))*quart;
			*currY = static_cast<GTP>(-tempBuffer[curr] + tempBuffer[other]);
			curr = (curr + 1) & divmask;
		}
	}
}


template<class TP, class DTP>
void computeIntensityDifference(Image<TP, 1> *img1, Image<TP, 1> *img2,
								float x1, float y1, float x2, float y2,
								int width, int height, Image<DTP, 1> *diff)
{
	register int hw = width/2, hh = height/2;
	Eigen::Matrix<TP, 1, 1> g1, g2;
	register int i, j;

	/* Compute values */
	for (j = -hh ; j <= hh ; j++)
	{
		DTP *rowP = diff->get2D(0, j+hh);
		for (i = -hw ; i <= hw ; i++) 
		{
			img1->interpolate(g1, x1+i, y1+j);
			img2->interpolate(g2, x2+i, y2+j);
			if(g1[0] != g2[0])
			{
				rowP = rowP;
			}
			*(rowP++) = static_cast<DTP>(g1[0]) - static_cast<DTP>(g2[0]);
		}
	}
}

template<class TP>
void computeGradientSum(Image<TP, 1> *gradx1, Image<TP, 1> *grady1,
						Image<TP, 1> *gradx2, Image<TP, 1> *grady2,
						float x1, float y1, float x2, float y2,
						int width, int height,
						Image<TP, 1> *gradx, Image<TP, 1> *grady)
{
	register int hw = width/2, hh = height/2;
	Eigen::Matrix<TP, 1, 1> g1, g2;
	register int i, j;

	/* Compute values */
	for (j = -hh ; j <= hh ; j++)
	{
		TP *rowPX = gradx->get2D(0, j+hh);
		TP *rowPY = grady->get2D(0, j+hh);
		for (i = -hw ; i <= hw ; i++) 
		{
			gradx1->interpolate(g1, x1+i, y1+j);
			gradx2->interpolate(g2, x2+i, y2+j);
			*rowPX++ = g1[0] + g2[0];
			grady1->interpolate(g1, y1+j, x1+i);
			grady2->interpolate(g2, y2+j, x2+i);
			*rowPY++ = g1[0] + g2[0];
		}
	}
}

template<class TP>
void compute2by2GradientMatrix(Image<TP, 1> *gradx, Image<TP, 1> *grady,
							   int width, int height,
							   float *gxx, float *gxy, float *gyy)
{
	register float gx, gy;

	/* Compute values */
	*gxx = 0.0;  *gxy = 0.0;  *gyy = 0.0;
	for (int y = 0 ; y < height ; y++)
	{
		TP *rowX = gradx->get2D(0, y);
		TP *rowY = grady->get2D(0, y);
		for(int x = 0; x < width; x++)
		{
			gx = *rowX++;
			gy = *rowY++;
			*gxx += gx*gx;
			*gxy += gx*gy;
			*gyy += gy*gy;
		}
	}
}

template<class ITP, class GTP>
void compute2by1ErrorVector(Image<ITP, 1> *imgdiff, Image<GTP, 1> *gradx, Image<GTP, 1> *grady,
							int width, int height, float step_factor, float *ex, float *ey)
{
	register float diff;
	/* Compute values */
	*ex = 0;  *ey = 0;  
	for (int y = 0 ; y < height ; y++)
	{
		GTP *rowX = gradx->get2D(0, y);
		GTP *rowY = grady->get2D(0, y);
		ITP *diffP = imgdiff->get2D(0, y);
		for(int x = 0; x < width; x++)
		{
			diff = *diffP++;
			*ex += diff * (*rowX++);
			*ey += diff * (*rowY++);
		}
	}
	*ex *= step_factor;
	*ey *= step_factor;
}

template<class TP>
float sumAbsWindow(Image<TP, 1> *imgdiff)
{
	register float sum = 0.0f;
	/* Compute values */
	
	for (int y = 0 ; y < imgdiff->height ; y++)
	{
		TP *row = imgdiff->get2D(0, y);
		for(int x = 0; x < imgdiff->width; x++)
		{
			sum = static_cast<float>(abs(*row++));
		}
	}
	return sum;
}


PointTracker::PointTracker()
{
    this->prevGradX = NULL;
    this->prevGradY = NULL;
    this->prevImg = NULL;
	this->windowSize = 9;
	this->displacementThreshold = 0.1f; //minimum displacement
	this->maxIterations = 10;
	this->lightingInsensitive = false;
	this->stepFactor = 1.0f; /* 2.0 comes from equations, 1.0 seems to avoid overshooting */
	this->maxResidue = 10.0f;
	this->minEigenRatio = 100.f;

	this->imgDiff = new Image<IMGDiffType, 1>(this->windowSize, this->windowSize);
	this->wGradX = new Image<GradientType, 1>(this->windowSize, this->windowSize);
	this->wGradY = new Image<GradientType, 1>(this->windowSize, this->windowSize);
}

PointTracker::~PointTracker()
{
    if(this->prevGradX != NULL)
	{
		delete this->prevGradX;
	}

    if(this->prevGradY != NULL)
	{
		delete this->prevGradY;
	}
	
    if(this->prevImg != NULL)
	{
		delete this->prevImg;
	}

    if(this->wGradX != NULL)
	{
		delete this->wGradX;
	}
	
    if(this->wGradY != NULL)
	{
		delete this->wGradY;
	}

    if(this->imgDiff != NULL)
	{
		delete this->imgDiff;
	}

}

void PointTracker::setInitialImage(ImageGray* img)
{
    if(this->prevImg != NULL && (img->width != this->prevImg->width || img->height != this->prevImg->height))
	{
        delete this->prevImg; this->prevImg = NULL;
        delete this->prevGradX; this->prevGradX = NULL;
        delete this->prevGradY; this->prevGradY = NULL;
        delete this->newImg; this->newImg = NULL;
        delete this->subImg; this->subImg = NULL;
        delete this->newGradX; this->newGradX = NULL;
        delete this->newGradY; this->newGradY = NULL;
	}

	int imgWidth = img->width/this->SUBSAMPLE;
	int imgHeight = img->height/this->SUBSAMPLE;

    if(this->prevImg == NULL)
	{
		this->prevImg = new Image<IMGType, 1>(imgWidth, imgHeight);
		this->prevGradX = new Image<GradientType, 1>(imgWidth, imgHeight);
        memset(prevGradX->data, 0, this->prevGradX->widthstep*this->prevGradX->height);
		this->prevGradY = new Image<GradientType, 1>(imgHeight, imgWidth);
        memset(prevGradY->data, 0, this->prevGradY->widthstep*this->prevGradY->height);
		
		this->newImg = new Image<IMGType, 1>(imgWidth, imgHeight);
        this->subImg = new Image<IMGType, 1>(imgWidth, imgHeight);
		this->newGradX = new Image<GradientType, 1>(imgWidth, imgHeight);
        memset(newGradX->data, 0, this->newGradX->widthstep*this->newGradX->height);
		this->newGradY = new Image<GradientType, 1>(imgHeight, imgWidth);
        memset(newGradY->data, 0, this->newGradY->widthstep*this->newGradY->height);
    }
    convertScale<IMGType, IMGType, 1, PointTracker::SUBSAMPLE>(img, this->subImg);
    smooth3x3(this->subImg, this->newImg);
    computeGradients3x3(this->newImg, this->newGradX, this->newGradY);
}

int PointTracker::trackPoints(ImageGray *nextImg, std::vector<Eigen::Vector2f> &points)
{


#ifdef UMF_DEBUG_TIMING
	UMFDebug *dbg = UMFDSingleton::Instance();
	int logid = dbg->logEventStart();
#endif
	
	int successCount = 0;
	
#ifdef UMF_USE_OPENCV
	std::swap(this->newImg, this->prevImg);
    convertScale<IMGType, IMGType, 1, PointTracker::SUBSAMPLE>(nextImg, this->newImg);
	
	IplImage *cvimg = cvCreateImageHeader(cvSize(this->newImg->width, this->newImg->height), IPL_DEPTH_8U, 1);
	cvimg->imageData = cvimg->imageDataOrigin = this->newImg->data;
	cvimg->widthStep = this->newImg->widthstep;

	
	IplImage *cvimgprev = cvCreateImageHeader(cvSize(this->prevImg->width, this->prevImg->height), IPL_DEPTH_8U, 1);
	cvimgprev->imageData = cvimgprev->imageDataOrigin = this->prevImg->data;
	cvimgprev->widthStep = this->prevImg->widthstep;

	int count = points.size();
	CvPoint2D32f * featuresA = new CvPoint2D32f[count]; 
	CvPoint2D32f * featuresB = new CvPoint2D32f[count];
	char * status = new char[count];
	float *err = new float[count];

	const float invSUB = 1.0f / this->SUBSAMPLE;
	for(int i = 0; i < count; i++)
	{
		featuresA[i].x = points[i][0]*invSUB;
		featuresA[i].y = points[i][1]*invSUB;
		assert(featuresA[i].x >= 0 && featuresA[i].y >= 0);
	}

	cvCalcOpticalFlowPyrLK(cvimgprev, cvimg, NULL, NULL, featuresA, featuresB, count,
		cvSize(this->windowSize, this->windowSize), 2, status, err,
		cvTermCriteria( CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 1e-3), 0);


	for(int i = 0; i < count; i++)
	{
		if(status[i] && featuresB[i].x >= 0 && featuresB[i].y >= 0 && featuresB[i].x < this->newImg->width && featuresB[i].y < this->newImg->height)
		{
			points[i][0] = featuresB[i].x*this->SUBSAMPLE;
			points[i][1] = featuresB[i].y*this->SUBSAMPLE;
			successCount++;
		} else {
			points[i][0] = points[i][1] = -1;
		}
	}

	delete [] featuresA;
	delete [] featuresB;
	
	delete [] status;
	delete [] err;

	cvReleaseImageHeader(&cvimg);
	cvReleaseImageHeader(&cvimgprev);
#else

	std::swap(this->newImg, this->prevImg);
	std::swap(this->newGradX, this->prevGradX);
	std::swap(this->newGradY, this->prevGradY);



    convertScale<IMGType, IMGType, 1, this->SUBSAMPLE>(nextImg, this->subImg);
#ifdef UMF_DEBUG_TIMING
	dbg->logEventEnd(logid, "TRT1");
	logid = dbg->logEventStart();
#endif

#ifdef UMF_USE_NATIVE
    umfnative::smooth3x3(this->subImg, this->newImg);
#else
    smooth3x3(this->subImg, this->newImg);
#endif

#ifdef UMF_DEBUG_TIMING
    dbg->logEventEnd(logid, "TRT2");
	logid = dbg->logEventStart();
#endif

#ifdef UMF_USE_NATIVE
    umfnative::computeGradients3x3(this->newImg, this->newGradX, this->newGradY);
#else
    computeGradients3x3(this->newImg, this->newGradX, this->newGradY);
#endif

#ifdef UMF_DEBUG_TIMING
	dbg->logEventEnd(logid, "TRT3");
	logid = dbg->logEventStart();
#endif

	const float invSUB = 1.0f/this->SUBSAMPLE;

	//for each feature try tracking
	for(std::vector<Eigen::Vector2f>::iterator it = points.begin(); it != points.end(); it++)
	{
		float xloc = (*it)[0]*invSUB;
		float yloc = (*it)[1]*invSUB;

		float xlocout = xloc, ylocout = yloc;

		int success = this->trackPoint(this->newImg, this->newGradX, this->newGradY,
			this->prevImg, this->prevGradX, this->prevGradY, xloc, yloc, &xlocout, &ylocout);

		if(success == TRACK_SUCCESS)
		{
			(*it)[0] = xlocout*this->SUBSAMPLE;
			(*it)[1] = ylocout*this->SUBSAMPLE;
			successCount++;
		} else {
			(*it)[0] = -1;
			(*it)[1] = -1;
		}
	}

#ifdef UMF_DEBUG_TIMING
	dbg->logEventEnd(logid, "TRP2");
	logid = dbg->logEventStart();
#endif

#endif

#ifdef UMF_DEBUG_TIMING
	dbg->logEventEnd(logid, "TRS");
#endif

	return successCount;
}

int PointTracker::trackPoint(Image<IMGType, 1>  *nextImg, Image<GradientType, 1>  *nextGradX, Image<GradientType, 1>  *nextGradY,
	Image<IMGType, 1>  *prevImg, Image<GradientType, 1>  *prevGradX, Image<GradientType, 1>  *prevGradY,
	float x1, float y1, float *x2, float *y2)
{
	int width = this->windowSize, height = this->windowSize;
	int hw = width/2;
	int hh = height/2;
	int nc = nextImg->width;
	int nr = nextImg->height;
	const float epsilon = 1e-3f;
	
	float one_plus_eps = epsilon + 1;   /* To prevent rounding errors */
	
	float gxx, gxy, gyy, ex, ey, dx, dy;
	int iteration = 0;
	int status = TRACK_SUCCESS;

	do  {

		/* If out of bounds, exit loop */
		if (  x1-hw < 0.0f || nc-( x1+hw) < one_plus_eps ||
			 *x2-hw < 0.0f || nc-(*x2+hw) < one_plus_eps ||
			  y1-hh < 0.0f || nr-( y1+hh) < one_plus_eps ||
			 *y2-hh < 0.0f || nr-(*y2+hh) < one_plus_eps) {
		  status = TRACK_OOB;
		  break;
		}
		
		if(this->lightingInsensitive)
		{
			assert(false); //not implemented
		} else {
			computeIntensityDifference(prevImg, nextImg, x1, y1, *x2, *y2, width, height, this->imgDiff);
			computeGradientSum(prevGradX, prevGradY, nextGradX, nextGradY, x1, y1, *x2, *y2, width, height, this->wGradX, this->wGradY);
		}

		/* Use these windows to construct matrices */
		compute2by2GradientMatrix(this->wGradX, this->wGradY, width, height, &gxx, &gxy, &gyy);
		compute2by1ErrorVector(this->imgDiff, this->wGradX, this->wGradY, width, height, this->stepFactor, &ex, &ey);

		/* Using matrices, solve equation for new displacement */

		float det = gxx*gyy - gxy*gxy;

		if (det < epsilon)
		{
			status = TRACK_SMALL_DET;
			break;
		}

		Eigen::Matrix2f pm; pm << gxx, gxy, gxy, gyy;
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(pm);
		Eigen::Vector2f eigv = eig.eigenvalues();
		if(eigv[1]/eigv[0] > this->minEigenRatio)
		{
			status = TRACK_NOT_CORNER;
			break;
		}

		dx = (gyy*ex - gxy*ey)/det;
		dy = (gxx*ey - gxy*ex)/det;

		//update position
		*x2 += dx;
		*y2 += dy;
		iteration++;

	} while ((fabs(dx)>=this->displacementThreshold || fabs(dy)>=this->displacementThreshold) && iteration < this->maxIterations);

	/* Check whether window is out of bounds */
	if (*x2-hw < 0.0f || nc-(*x2+hw) < one_plus_eps || 
		*y2-hh < 0.0f || nr-(*y2+hh) < one_plus_eps)
	{
		status = TRACK_OOB;
	}

	/* Check whether residue is too large */
	if (status == TRACK_SUCCESS)  {
		if (this->lightingInsensitive)
		{
			assert(false);
		} else {
			computeIntensityDifference(prevImg, nextImg, x1, y1, *x2, *y2, width, height, this->imgDiff);
		}
		if (sumAbsWindow(this->imgDiff)/(width*height) > this->maxResidue) 
			status = TRACK_RESIDUE;
	}

	if(status == TRACK_SUCCESS && iteration >= this->maxIterations)
	{
		status = TRACK_ITERATION;
	}

	return status;
}

}
