#include "pnp_solver_opencv.h"

#ifdef UMF_USE_OPENCV

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>

namespace umf
{
PnPSolverOpenCV::PnPSolverOpenCV():
    PnPSolver()
{
}


bool PnPSolverOpenCV::pnpSolve(CorrespondenceSet correspondences, bool useLast)
{
	if (correspondences.size() < 4)
	{
		return false;
	}
    //create cvmat structures
    CvMat *modelPoints = cvCreateMat(correspondences.size(), 3, CV_32FC1);
    CvMat *projectedPoints = cvCreateMat(correspondences.size(), 2, CV_32FC1);
    CvMat *pointsCount = cvCreateMat(1, 1, CV_32SC1);

    CvMat *cameraMatrixCV = cvCreateMat(3, 3, CV_32FC1);
    cvSet(cameraMatrixCV, cvScalarAll(0));
    cameraMatrixCV->data.fl[2] = static_cast<float> (this->cameraMatrix(0,2));
    cameraMatrixCV->data.fl[5] = static_cast<float> (this->cameraMatrix(1,2));
    cameraMatrixCV->data.fl[0] = static_cast<float> (this->cameraMatrix(0,0));
    cameraMatrixCV->data.fl[4] = static_cast<float> (this->cameraMatrix(1,1));
    cameraMatrixCV->data.fl[8] = 1;

	CvMat *distMatrixCV = cvCreateMat(8, 1, CV_32FC1);
	cvSet(distMatrixCV, cvScalarAll(0));
	for(int i = 0; i < this->distCoeffs.cols()*this->distCoeffs.rows(); i++)
	{
		distMatrixCV->data.fl[i] = this->distCoeffs[i];
	}

    int counter = 0;
    for(CorrespondenceSet::iterator cit = correspondences.begin(); cit != correspondences.end(); cit++, counter++)
    {
        cvSet2D(modelPoints, counter, 0, cvScalar(cit->mx));
        cvSet2D(modelPoints, counter, 1, cvScalar(cit->my));
        cvSet2D(modelPoints, counter, 2, cvScalar(cit->mz));
        cvSet2D(projectedPoints, counter, 0, cvScalar(cit->px));
        cvSet2D(projectedPoints, counter, 1, cvScalar(cit->py));
    }
    cvSet2D(pointsCount, 0, 0, cvScalar(counter));
	
    CvMat *rotationMatrix = cvCreateMat(3, 3, CV_32FC1);
    CvMat *rotationVec = cvCreateMat(1, 3, CV_32FC1);
    cvSet(rotationVec, cvScalarAll(0));
    CvMat *translationVec = cvCreateMat(1, 3, CV_32FC1);
    cvSet(translationVec, cvScalarAll(0));

	if(useLast)
	{
		for(int ri = 0; ri < 3; ri++)
		{
			for(int ci = 0; ci < 3; ci++)
			{
				rotationMatrix->data.fl[ri*3 + ci] = static_cast<float>(this->worldTransformMatrix(ri, ci));
			}
			translationVec->data.fl[ri] = static_cast<float>(this->worldTransformMatrix(ri, 3));
		}
		
		cvRodrigues2(rotationMatrix, rotationVec);
	}

    //we do distortion stuff elsewhere
	cvFindExtrinsicCameraParams2(modelPoints, projectedPoints, cameraMatrixCV, distMatrixCV, rotationVec, translationVec, (useLast) ? 1 : 0);


	//cv::Mat mmodelPoints(modelPoints, true);
	//cv::Mat mprojectedPoints(projectedPoints, true);
	//cv::Mat mcameraMatrixCV(cameraMatrixCV, true);
	//cv::Mat mdistortion(8, 1, CV_32FC1); mdistortion.setTo(0);
	//cv::Mat mrotationVec(rotationVec);
	//cv::Mat mtranslationVec(translationVec);
	//cv::solvePnPRansac(mmodelPoints, mprojectedPoints, mcameraMatrixCV, mdistortion,
	//	mrotationVec, mtranslationVec, (useLast)?1:0,
	//	100, 3.0f, 13, cv::noArray(), 0);

    double rotMat[3][3];
    double transVec[3];

    cvRodrigues2(rotationVec, rotationMatrix);

	

    for(int ri = 0; ri < 3; ri++)
    {
        for(int ci = 0; ci < 3; ci++)
        {
            rotMat[ri][ci] = rotationMatrix->data.fl[ri*3 + ci];
        }
        transVec[ri] = translationVec->data.fl[ri];
    }

	
	if (transVec[2] < 0) //we are behind the marker - this shouldn't happen - invert it!
	{	
		//inverse rotation and translation
		for (int ri = 0; ri < 3; ri++)
		{
			for (int ci = 0; ci < 3; ci++)
			{
				rotationMatrix->data.fl[ri * 3 + ci] = rotMat[ci][ri];
			}
			translationVec->data.fl[ri] = -transVec[ri];
		}

		cvRodrigues2(rotationMatrix, rotationVec);

		cvFindExtrinsicCameraParams2(modelPoints, projectedPoints, cameraMatrixCV, distMatrixCV, rotationVec, translationVec, 1);

		cvRodrigues2(rotationVec, rotationMatrix);

		for (int ri = 0; ri < 3; ri++)
		{
			for (int ci = 0; ci < 3; ci++)
			{
				rotMat[ri][ci] = rotationMatrix->data.fl[ri * 3 + ci];
			}
			transVec[ri] = translationVec->data.fl[ri];
		}
	}

    cvReleaseMat(&rotationMatrix);
    cvReleaseMat(&modelPoints);
    cvReleaseMat(&projectedPoints);
    cvReleaseMat(&pointsCount);
    cvReleaseMat(&rotationVec);
    cvReleaseMat(&translationVec);
    cvReleaseMat(&cameraMatrixCV);

    this->worldTransformMatrix << rotMat[0][0], rotMat[0][1], rotMat[0][2], transVec[0],
            rotMat[1][0], rotMat[1][1], rotMat[1][2], transVec[1],
            rotMat[2][0], rotMat[2][1], rotMat[2][2], transVec[2],
            0.0, 0.0, 0.0, 1.0;

    this->worldPos = Eigen::Vector3d(transVec[0], transVec[1], transVec[2]);
    this->worldQuat = Eigen::Quaterniond(worldTransformMatrix.block<3,3>(0, 0));

    return true;
}

}

#endif //opencv
