#include "pnp_solver_epnp.h"
#include <iostream>

namespace umf {

PnPSolverEpnp::PnPSolverEpnp() :
    PnPSolver()
{
}


bool PnPSolverEpnp::pnpSolve(CorrespondenceSet correspondences, bool /*useLast*/)
{
    this->solver.reset_correspondences();
    this->solver.set_maximum_number_of_correspondences(correspondences.size());

    double fx = this->cameraMatrix(0,0);
    double fy = this->cameraMatrix(1,1);
    double cx = this->cameraMatrix(0,2);
    double cy = this->cameraMatrix(1,2);

    this->solver.set_internal_parameters(cx, cy, fx, fy);


    for(CorrespondenceSet::iterator cit = correspondences.begin(); cit != correspondences.end(); cit++)
    {
        this->solver.add_correspondence(cit->mx, cit->my, cit->mz, cit->px, cit->py);
    }

    double rotMat[3][3];
    double trans[3];
    this->solver.compute_pose(rotMat, trans);

    this->worldTransformMatrix << rotMat[0][0], rotMat[0][1], rotMat[0][2], trans[0],
            rotMat[1][0], rotMat[1][1], rotMat[1][2], trans[1],
            rotMat[2][0], rotMat[2][1], rotMat[2][2], trans[2],
            0.0, 0.0, 0.0, 1.0;

    this->worldPos = Eigen::Vector3d(trans[0], trans[1], trans[2]);
    this->worldQuat = Eigen::Quaterniond(worldTransformMatrix.block<3,3>(0, 0));

    return true;
}

}
