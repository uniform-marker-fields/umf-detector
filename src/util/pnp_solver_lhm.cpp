#include "pnp_solver_lhm.h"
#include "objpose.h"
#include "robust_planar.h"
#include <float.h>

namespace umf {



PnPSolverLHM::PnPSolverLHM() :
    PnPSolver()
{
}


bool PnPSolverLHM::pnpSolve(CorrespondenceSet correspondences, bool useLast)
{
	const int ITER_COUNT = 25;
	const double UPDATE_THRESH = 5e-3;
	const double UPDATE_EPSILON = 5e-2;
	Eigen::MatrixXd modelPoints(3, correspondences.size());
	Eigen::MatrixXd projPoints(3, correspondences.size());

    double fx = this->cameraMatrix(0,0);
    double fy = this->cameraMatrix(1,1);
    double cx = this->cameraMatrix(0,2);
    double cy = this->cameraMatrix(1,2);

	int counter = 0;
    for(CorrespondenceSet::iterator cit = correspondences.begin(); cit != correspondences.end(); cit++, counter++)
    {
		projPoints(0, counter) =  ((cit->px - cx)/fx);
		projPoints(1, counter) =  ((cit->py - cy)/fy);
		projPoints(2, counter) = 1.0;

		modelPoints(0, counter) = cit->mx;
		modelPoints(1, counter) = cit->my;
		modelPoints(2, counter) = cit->mz;
    }

    Eigen::Matrix3d rotMat;
    Eigen::Vector3d transVec;

	if(useLast)
	{
		rotMat = this->worldTransformMatrix.block<3,3>(0, 0);
		transVec = this->worldTransformMatrix.block<3,1>(0, 3);
	}

	int iterDone = 0;
	double reprojectionError;
	objpose(modelPoints, projPoints, ITER_COUNT, UPDATE_THRESH, UPDATE_EPSILON, rotMat, transVec, iterDone, reprojectionError, useLast);


    this->worldTransformMatrix << rotMat(0, 0), rotMat(0,1), rotMat(0,2), transVec(0),
            rotMat(1, 0), rotMat(1,1), rotMat(1,2), transVec(1),
            rotMat(2, 0), rotMat(2,1), rotMat(2,2), transVec(2),
            0.0, 0.0, 0.0, 1.0;

    this->worldPos = transVec;
    this->worldQuat = Eigen::Quaterniond(worldTransformMatrix.block<3,3>(0, 0));

    return true;
}

/////////////////////////////////////////////////////
// epnp initialize and lhm
PnPSolverEPnPLHM::PnPSolverEPnPLHM() :
    PnPSolver()
{
}


bool PnPSolverEPnPLHM::pnpSolve(CorrespondenceSet correspondences, bool useLast)
{
	const int ITER_COUNT = 200;
	const double UPDATE_THRESH = 1e-5;
	const double UPDATE_EPSILON = 2e-2;
	Eigen::MatrixXd modelPoints(3, correspondences.size());
	Eigen::MatrixXd projPoints(3, correspondences.size());

    double fx = this->cameraMatrix(0,0);
    double fy = this->cameraMatrix(1,1);
    double cx = this->cameraMatrix(0,2);
    double cy = this->cameraMatrix(1,2);

	int counter = 0;
    for(CorrespondenceSet::iterator cit = correspondences.begin(); cit != correspondences.end(); cit++, counter++)
    {
		projPoints(0, counter) =  static_cast<float>((cit->px - cx)/fx);
		projPoints(1, counter) =  static_cast<float>((cit->py - cy)/fy);
		projPoints(2, counter) = 1.0f;

		modelPoints(0, counter) = cit->mx;
		modelPoints(1, counter) = cit->my;
		modelPoints(2, counter) = cit->mz;
    }

    Eigen::Matrix3d rotMat;
    Eigen::Vector3d transVec;

	if(useLast)
	{
 		rotMat = this->worldTransformMatrix.block<3,3>(0, 0);
		transVec = this->worldTransformMatrix.block<3,1>(0, 3);
	} else 
	{
		this->solver.reset_correspondences();
		this->solver.set_maximum_number_of_correspondences(correspondences.size());

		this->solver.set_internal_parameters(cx, cy, fx, fy);


		for(CorrespondenceSet::iterator cit = correspondences.begin(); cit != correspondences.end(); cit++)
		{
			this->solver.add_correspondence(cit->mx, cit->my, cit->mz, cit->px, cit->py);
		}

		double eprotMat[3][3];
		double eptrans[3];
		this->solver.compute_pose(eprotMat, eptrans);

		transVec << eptrans[0], eptrans[1], eptrans[2];
		rotMat << eprotMat[0][0], eprotMat[0][1], eprotMat[0][2],
            eprotMat[1][0], eprotMat[1][1], eprotMat[1][2],
            eprotMat[2][0], eprotMat[2][1], eprotMat[2][2];
	}

	int iterDone = 0;
	double reprojectionError;
	int success = objpose(modelPoints, projPoints, ITER_COUNT, UPDATE_THRESH, UPDATE_EPSILON, rotMat, transVec, iterDone, reprojectionError, true);

	if(success != 0)
	{
		return false;
	}

	if(this->isPlanar && !useLast)
	{
		double *pws = new double[3*counter];
		double *ush = new double[2*counter];

		for(int i = 0; i < counter; i++)
		{
			pws[i*3] = modelPoints(0, i);
			pws[i*3 + 1] = modelPoints(1, i);
			pws[i*3 + 2] = modelPoints(2, i);

			ush[i*2] = projPoints(0, i);
			ush[i*2 + 1] = projPoints(1, i);
		}

		int resultCount;
		PoseSolution *solutions;
		
		double protMat[3][3];
		double ptrans[3];

		for(int r = 0; r < 3; r++)
		{
			for(int c = 0; c < 3; c++)
			{
				protMat[r][c] = rotMat(r,c);
			}
			ptrans[r] = transVec[r];
		}

		get2ndPose(counter, ush, pws, protMat, ptrans, &resultCount, &solutions);

		double bestReprojection = DBL_MAX;
		
		Eigen::Matrix3d bestRotMat = rotMat;
		Eigen::Vector3d bestTransVec = transVec;

		for(int i = 0; i < resultCount; i++)
		{
			const double (&R)[3][3] = solutions[i].R;
			rotMat << R[0][0], R[0][1], R[0][2],
				R[1][0], R[1][1], R[1][2],
				R[2][0], R[2][1], R[2][2];
			transVec << solutions[i].t[0], solutions[i].t[1], solutions[i].t[2];
			success = objpose(modelPoints, projPoints, ITER_COUNT, UPDATE_THRESH, UPDATE_EPSILON, rotMat, transVec, iterDone, reprojectionError, true);

			if(reprojectionError < bestReprojection && success != 0)
			{
				bestReprojection = reprojectionError;
				bestRotMat = rotMat;
				bestTransVec = transVec;
			}
		}

		rotMat = bestRotMat;
		transVec = bestTransVec;
		reprojectionError = bestReprojection;

		delete[] solutions;
		delete[] pws;
		delete[] ush;

		if(resultCount == 0)
		{
			return false;
		}
	}

    this->worldTransformMatrix << rotMat(0, 0), rotMat(0,1), rotMat(0,2), transVec(0),
            rotMat(1, 0), rotMat(1,1), rotMat(1,2), transVec(1),
            rotMat(2, 0), rotMat(2,1), rotMat(2,2), transVec(2),
            0.0, 0.0, 0.0, 1.0;


	this->worldPos = worldTransformMatrix.block<3,1>(0, 3);
    this->worldQuat = Eigen::Quaterniond(worldTransformMatrix.block<3,3>(0, 0));

    return true;
}

}
