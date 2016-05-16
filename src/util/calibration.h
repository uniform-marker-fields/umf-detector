#ifndef _UMF_CALIBRATION_H_
#define _UMF_CALIBRATION_H_

#include "pnp_solver.h"
#include "../defines.h"

namespace umf
{

/**
 * Class helping along with calibration
 * See main.cpp for example usage
 */
class UMF Calibration
{
public:
    Calibration(int width, int height, float fovy);

    /**
     * Add correspondences from an image to the training set.
     * After 100 images calibration is automatically started
     */
    void addImage(CorrespondenceSet &myset);

    /**
     * Start the calibration from the stored set of correspondences
     */
    bool calibrate(bool distort = true);

    /**
     * Save the calibration info into a file
     */
	bool saveCalibration(const char* filename);

    /**
     * Check if the calibration already done
     */
	bool isCalibrated();

private:

    void projectPoints(std::vector<Eigen::Vector3f> &modelPoints,
                       std::vector<Eigen::Vector2f> &imagePoints,
                       bool distort = true);

    std::vector<CorrespondenceSet> images;

	float mFovy;
	bool calibrated;

    Eigen::Vector2i imgSize;
    Eigen::Matrix3d cameraMatrix;
    Eigen::Matrix<double, 8, 1> distCoeffs;
	PnPSolver *pnp;
};

}
#endif // CALIBRATION_H
