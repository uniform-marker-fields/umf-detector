#include "calibration.h"
#include "distort.h"
#include <fstream>

#ifdef UMF_USE_OPENCV
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#endif

namespace umf
{

Calibration::Calibration(int width, int height, float fovy)
{
    this->imgSize(0) = width;
    this->imgSize(1) = height;
	this->mFovy = fovy;
	this->calibrated = false;

#ifdef UMF_USE_OPENCV
	this->pnp = PnPSolver::GetPnPSolver("OPENCV");
#else 
	this->pnp = PnPSolver::GetPnPSolver("EPNP+LHM");
#endif
}

void Calibration::addImage(CorrespondenceSet &myset)
{
    this->images.push_back(myset);

	if (this->images.size() > 100) {
		this->calibrate();
	}
}

//todo fix aspect ration, fix principal point etc.
bool Calibration::calibrate(bool /*distort*/)
{

	if (this->images.size() < 5) {
		return false;
	}

#ifdef UMF_USE_OPENCV


	int correspondenceCount = 0;
	for (std::vector<CorrespondenceSet>::iterator it = this->images.begin(); it != this->images.end(); it++) {
		correspondenceCount += (*it).size();
	}

	CvMat *modelPoints = cvCreateMat(correspondenceCount, 3, CV_32FC1);
	CvMat *projectedPoints = cvCreateMat(correspondenceCount, 2, CV_32FC1);

	CvMat *pointCounts = cvCreateMat(this->images.size(), 1, CV_32SC1);
	cvSet(pointCounts, cvScalarAll(0));

	unsigned int index = 0;
	unsigned int imageIndex = 0;
	for(std::vector<CorrespondenceSet>::iterator it = this->images.begin(); it != this->images.end(); it++, imageIndex++) {
		pointCounts->data.i[imageIndex] = it->size();
		for (CorrespondenceSet::iterator cit = it->begin(); cit != it->end(); cit++, index++) {
			cvSet2D(modelPoints, index, 0, cvScalar(cit->mx));
			cvSet2D(modelPoints, index, 1, cvScalar(cit->my));
			cvSet2D(modelPoints, index, 2, cvScalar(cit->mz));

			cvSet2D(projectedPoints, index, 0, cvScalar(cit->px));
			cvSet2D(projectedPoints, index, 1, cvScalar(cit->py));
		}
	}

	//set default values for calibration
	CvMat *cameraMatrix = cvCreateMat(3, 3, CV_32FC1);
	cvSet(cameraMatrix, cvScalarAll(0));
	float focal = this->imgSize(1) / (2.0f*tanf(this->mFovy*static_cast<float>(M_PI) / 360.0f));
	cameraMatrix->data.fl[2] = this->imgSize(0) / 2;
	cameraMatrix->data.fl[5] = this->imgSize(1) / 2;
	cameraMatrix->data.fl[0] = focal;
	cameraMatrix->data.fl[4] = focal;
	cameraMatrix->data.fl[8] = 1;
	CvMat *distortionCoeffs = cvCreateMat(8, 1, CV_32FC1);
	cvSet(distortionCoeffs, cvScalarAll(0));

	int FLAGS = CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_FIX_PRINCIPAL_POINT;

	cvCalibrateCamera2(modelPoints, projectedPoints, pointCounts, cvSize(this->imgSize(0), this->imgSize(1)),
		cameraMatrix, distortionCoeffs, NULL, NULL, FLAGS);

	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++) {
			this->cameraMatrix(i, j) = cameraMatrix->data.fl[i * 3 + j];
		}
	}

	for (unsigned int i = 0; i < 8; i++) {
		this->distCoeffs(i) = distortionCoeffs->data.fl[i];
	}
#else

	return false;
	//first compute for each image homography

	//estimate using homography calculations all the parameters

	//use levenberg marquardt to improve

#endif

	this->calibrated = true;

	return true;
}

bool Calibration::saveCalibration(const char* filename) {

	if(!this->calibrated) {
		return false;
	}
	std::fstream calib_info(filename, std::fstream::out);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			calib_info << this->cameraMatrix(i, j) << " ";
		}
		calib_info << std::endl;
	}
	for (int i = 0; i < 8; i++)
	{
		calib_info << this->distCoeffs(i) << " ";
	}
	calib_info << std::endl;
	calib_info.close();

	return true;
}

bool Calibration::isCalibrated() {
	return this->calibrated;
}


}//end of namespace
