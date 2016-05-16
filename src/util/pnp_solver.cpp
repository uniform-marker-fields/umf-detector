#include "pnp_solver.h"
#include "pnp_solver_epnp.h"
#include "pnp_solver_opencv.h"
#include "pnp_solver_lhm.h"
#include <Eigen/Dense>
#include "distort.h"

namespace umf {

PnPSolver::PnPSolver()
{
	this->cameraMatrix.setIdentity();
	this->distCoeffs = Eigen::VectorXd(8);
	this->distCoeffs.setZero();
	this->mvp.setIdentity();

    this->distCoeffs.setZero();
    this->cameraPos.setZero();
    this->worldPos.setZero();

    this->cameraQuat.setIdentity();
    this->worldQuat.setIdentity();
	this->isPlanar = true;
}

PnPSolver *PnPSolver::GetPnPSolver(std::string type)
{
#ifdef UMF_USE_OPENCV
    if (type == "OPENCV") return new PnPSolverOpenCV();
#endif
    if (type == "EPNP") return new PnPSolverEpnp();
	else if(type == "LHM") return new PnPSolverLHM();
	else if(type == "EPNP+LHM") return new PnPSolverEPnPLHM();
    return NULL;
}


bool PnPSolver::computeCameraPose(CorrespondenceSet correspondencesOrig, Eigen::Vector2i imageSize, int flags)
{

    /*
     * Since there are some incompatibilites between OpenGL and other stuff we have to rotate some of stuff
     * invert y -> imgSize.hegiht - y for screen positions
     * modelpoints - replace x and y since in modelling the axis form a right handed system x - down, y - right
     */
    CorrespondenceSet correspondences;
    if(flags & PNP_FLAG_SWAP_Y)
    {
        for(CorrespondenceSet::iterator it = correspondencesOrig.begin(); it != correspondencesOrig.end(); it++)
        {
            correspondences.push_back(Correspondence(it->px, imageSize[1] - it->py, it->my, it->mx, it->mz));
        }
    } else {
        for(CorrespondenceSet::iterator it = correspondencesOrig.begin(); it != correspondencesOrig.end(); it++)
        {
            correspondences.push_back(Correspondence(it->px, it->py, it->my, it->mx, it->mz));
        }
    }

	bool success = this->pnpSolve(correspondences, (flags & PNP_FLAG_USE_LAST) != 0);

    if(!success) {
        return false;
    }

    if(flags & PNP_FLAG_GL_PROJECTION_MV)
    {
        float fx = static_cast<float> (cameraMatrix(0,0));
        float fy = static_cast<float> (cameraMatrix(1,1));
        float cx = static_cast<float> (cameraMatrix(0,2));
        float cy = static_cast<float> (cameraMatrix(1,2));
        float nearPlane = 1.0f;
        float farPlane = 1000.0f;
        float p1 = ( farPlane+nearPlane ) / ( farPlane - nearPlane );
        float p2 = -2.0f * farPlane * nearPlane / ( farPlane - nearPlane );

        Eigen::Matrix4d projection;

        projection << 2*fx/imageSize[0], 0,                 2*cx/imageSize[0] - 1.0, 0,
                      0,                 2*fy/imageSize[1], 2*cy/imageSize[1] - 1.0, 0,
                      0,                 0,                 p1,                      p2,
                      0,                 0,                 1,                       0;

        this->mvp = projection*this->worldTransformMatrix;
    }

    if(flags & PNP_FLAG_COMPUTE_CAMERA)
    {
        if(flags & PNP_FLAG_RIGHT_HANDED)
        {
            Eigen::Matrix4d mirrorZ = Eigen::Matrix4d::Identity(); mirrorZ(2,2) = -1;
            this->cameraTransformMatrix = mirrorZ* this->worldTransformMatrix.inverse() * mirrorZ;

            this->cameraQuat = Eigen::Quaterniond(this->cameraTransformMatrix.block<3,3>(0,0));

            this->cameraPos = this->cameraTransformMatrix.block<3,1>(0,3); //last column
        } else {
			
            Eigen::Matrix4d mirrorZ = Eigen::Matrix4d::Identity(); mirrorZ(2,2) = -1;
            Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
            rotation.block<3,3>(0,0) = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()).matrix();
            this->cameraTransformMatrix = rotation*this->worldTransformMatrix.inverse();
            this->cameraQuat = Eigen::Quaterniond(this->cameraTransformMatrix.block<3, 3>(0, 0));
            this->cameraPos = this->cameraTransformMatrix.block<3,1>(0, 3); 
        }
    }

    return success;
}

}
