#ifndef _UMF_OBJPOSE_H_
#define _UMF_OBJPOSE_H_

#include <Eigen/Core>

namespace umf {
    
/**
 * Used by the LHM PnP solver to recover the object pose.
 */

int objpose( const Eigen::MatrixXd& mP3DNotC, ///<Input: 3xN matrix representing the landmarks in front of the camera
	const Eigen::MatrixXd& mMeas,///<Input: 3xN measurements in the normalised plane or unit sphere
	const int nMaxNumIters,
	const double dTol,
	const double dEpsilon,
	Eigen::Matrix3d& mR,///<Input/Output: initial rotation estimate, this will also contain the estimated rotation
	Eigen::Vector3d& vt,///Output: estimated translation
	int& nNumIterations,
	double& dObjError,
	bool bUseRtForInitialisation = false); //use the initial rotation to estimate t

}
#endif