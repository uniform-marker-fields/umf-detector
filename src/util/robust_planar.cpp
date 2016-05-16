#include "robust_planar.h"
#include "Rpoly.h"
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <complex>

/**
 * Copyright (c) I. Szentandrasi 2016
 * Robust pose estimation from Schweighofer and Pinz.
 * The code is based upon the matlab code by pinz and OpenCV implementation
 * from Nghi Ho. License below. (https://github.com/spchuang/AugmentedRealityGo)
 * It has been rewritten to use Eigen library instead.
 *
 *   This is a C++ port using OpenCV of the Robust Pose Estimation from a Planar Target algorithm by Gerald Schweighofer and Axel Pinz.
 *   It is a line by line port of the Matlab code at
 *   http://www.emt.tugraz.at/~pinz/code/
 *   I have no idea what their license is, if any. I'll make my code BSD for now unless I later find they have a more restrictive one.
 *
 *
 *Copyright 2011 Nghia Ho. All rights reserved.
 *Redistribution and use in source and binary forms, with or without modification, are
 *permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice, this list of
 *      conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice, this list
 *      of conditions and the following disclaimer in the documentation and/or other materials
 *      provided with the distribution.
 *THIS SOFTWARE IS PROVIDED BY NGHIA HO ''AS IS'' AND ANY EXPRESS OR IMPLIED
 *WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 *FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL NGHIA HO OR
 *CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *The views and conclusions contained in the software and documentation are those of the
 *authors and should not be interpreted as representing official policies, either expressed
 *or implied, of Nghia Ho.
*/

namespace umf {


typedef struct esolution
{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    double E;
} EPoseSolution;

/**
* normalize each 3d point
*/
void normRv(int count, Eigen::Vector3d *points)
{
    for(int i = 0; i < count; i++)
    {
        points[i].normalize();
    }
}

void getRotationbyVectors(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, Eigen::Matrix3d &Rot)
{
    Eigen::Quaterniond p = Eigen::Quaterniond::FromTwoVectors(v2, v1);
    Rot = p.matrix();
}


void meanv(int count, Eigen::Vector3d *points, Eigen::Vector3d *cent)
{
    *cent << 0, 0, 0;

    for(int i = 0; i < count; i++)
    {
        *cent += points[i];
    }

    (*cent) *= 1.0/count;
}

Eigen::Matrix3d decomposeR(Eigen::Matrix3d &R)
{
    double cl = std::atan2(R(2,1), R(2,0));
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(cl, Eigen::Vector3d(0, 0, 1)).matrix();

    //TODO if fails rest is testing
    return Rz;
}

Eigen::Vector3d eulerIgnoreX(Eigen::Matrix3d &R)
{
    Eigen::Vector3d angles = R.eulerAngles(2, 1, 0);


    while( std::abs(angles(2)) > M_PI*0.5)
    {
        if( angles(2) > 0 )
        {
            angles(2) -= M_PI;
            angles(1) = M_PI - angles(1);
            angles(0) -= M_PI;
        } else {
            angles(2) += M_PI;
            angles(1) = 3*M_PI - angles(1);
            angles(0) += M_PI;
        }
    }

    return angles;
}

EPoseSolution *getRfor2ndPose_V_Exact(int *solution_count,
                                     int corr_count,
                                     Eigen::Vector3d *projected, 
                                     Eigen::Vector3d *model,
                                     Eigen::Matrix3d &Rp,
                                     Eigen::Vector3d &/*tp*/)
{
    Eigen::Matrix3d RzN = decomposeR(Rp);

    Eigen::Vector3d *model_ = new Eigen::Vector3d[corr_count];

    Eigen::Matrix3d R_ = Rp*RzN;

    //change the model by Rz
    for(int i = 0; i < corr_count; i++)
    {
        model_[i] = RzN.transpose()*model[i];
    }

    //project into Image with only Ry
    Eigen::Vector3d angles = eulerIgnoreX(R_);

    //Eigen::Matrix3d Ry = Eigen::AngleAxisd(angles[1], Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(angles[0], Eigen::Vector3d::UnitZ()).matrix();

    //generate Vis And G
 
    Eigen::Matrix3d G = Eigen::Matrix3d::Zero();

    Eigen::Matrix3d *Vs = new Eigen::Matrix3d[corr_count];
    for(int i = 0; i < corr_count; i++)
    {
        Vs[i] = (projected[i]*projected[i].transpose())*1.0/(projected[i].dot(projected[i]));
        G += Vs[i];
    }

    G = (Eigen::Matrix3d::Identity() - G*1.0/corr_count).inverse()*(1.0/corr_count);

    //generate opt_t
    Eigen::Matrix3d opt_t = Eigen::Matrix3d::Zero();
    for(int i = 0; i < corr_count; i++)
    {
        Eigen::Matrix3d vmat;
        double px = model_[i][0];
        double py = model_[i][1];
        double pz = model_[i][2];
        vmat << -px,  2*pz, px, 
                 py,     0, py,
                -pz, -2*px, pz;
        opt_t += (Vs[i] - Eigen::Matrix3d::Identity())*Rz*vmat;
    }

    opt_t = G*opt_t;

    //Estimate the error function
    Eigen::VectorXd E_2(5, 1);
    E_2.setZero();

    for(int i = 0; i < corr_count; i++)
    {
        double px = model_[i][0];
        double py = model_[i][1];
        double pz = model_[i][2];

        Eigen::Matrix3d Rpi;
        Rpi << -px,  2*pz, px,
                py,     0, py,
               -pz, -2*px, pz;

        Eigen::Matrix3d Err = (Eigen::Matrix3d::Identity() - Vs[i])*(Rz*Rpi + opt_t);

        Eigen::Vector3d e2 = Err.block<3,1>(0, 0);
        Eigen::Vector3d e1 = Err.block<3,1>(0, 1);
        Eigen::Vector3d e0 = Err.block<3,1>(0, 2);
        Eigen::VectorXd p(5, 1);
        p << e2.dot(e2), 2*e1.dot(e2), 2*e0.dot(e2) + e1.dot(e1), 2*e0.dot(e1), e0.dot(e0);
        E_2 = E_2 + p;
    }

    double e4 = E_2(0);
    double e3 = E_2(1);
    double e2 = E_2(2);
    double e1 = E_2(3);
    double e0 = E_2(4);

    double a4 = -e3;
    double a3 = 4*e4 - 2*e2;
    double a2 = -3*e1 + 3*e3;
    double a1 = -4*e0 + 2*e2;
    double a0 = e1;

    
    double coeffs[5];
    coeffs[0] = a4;
    coeffs[1] = a3;
    coeffs[2] = a2;
    coeffs[3] = a1;
    coeffs[4] = a0;

    //printf("coeffs = %f %f %f %f\n", coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
    int degrees = 4;
    double zero_real[4];
    double zero_imag[4];

    rpoly_ak1(coeffs, &degrees, zero_real, zero_imag);

    std::vector< double > atSol;
    std::vector< double > alSol;
    std::vector< Eigen::Vector3d > t_opt;

    for(int i=0; i < 4; i++) {
        double _at = zero_real[i];

        //check valid solution
        double p1 = pow(1.0 + _at*_at, 3.0);

        if(fabs(p1) < 0.1 || zero_imag[i] != 0) {
            continue;
        }

        double sa = (2.f*_at) / (1.f +_at*_at);
        double ca = (1.f - _at*_at) / (1.f + _at*_at);

        double al = std::atan2(sa,ca);

        //check the second derivate
        double tMaxMin = (4*a4*_at*_at*_at + 3*a3*_at*_at + 2*a2*_at + a1);

        if(tMaxMin > 0) {
            alSol.push_back(al);
            atSol.push_back(_at);
        }

    }

    std::vector< Eigen::Vector3d > tsolutions(atSol.size());
    //get solutions
    for(unsigned int i = 0; i < atSol.size(); i++)
    {
        Eigen::Matrix3d R = Rz*Eigen::AngleAxisd(alSol[i], Eigen::Vector3d::UnitY());
        Eigen::Vector3d t_opt = Eigen::Vector3d::Zero();

        for(int pointI = 0; pointI < corr_count; pointI++)
        {
            t_opt += (Vs[pointI] - Eigen::Matrix3d::Identity())*R*model_[pointI];
        }
        tsolutions[i] = G*t_opt;
    }
    const int solutionCount = atSol.size();
    EPoseSolution *solutions = new EPoseSolution[solutionCount];

    for(int i = 0; i < solutionCount; i++)
    {
        Eigen::Matrix3d rot = Rz*Eigen::AngleAxisd(alSol[i], Eigen::Vector3d::UnitY())*RzN.transpose();
        double E = 0;

        for(int pi = 0; pi < corr_count; pi++)
        {
            Eigen::Vector3d perr = (Eigen::Matrix3d::Identity() - Vs[pi])*(rot*model[pi] + tsolutions[i]);
            E += perr.dot(perr);
        }

        //copy results
        solutions[i].E = E;
        solutions[i].R = rot;
        solutions[i].t = tsolutions[i];
    }

    delete [] Vs;
    delete [] model_;
    *solution_count = solutionCount;
    return solutions;
}

/* get the second pose for our problem
*/
int get2ndPose(int corr_count,
               double *projected,
               double *model,
               double rotation[3][3],
               double translation[3],
               int *results, PoseSolution **solutions)
{
    Eigen::Vector3d *normalized_points = new Eigen::Vector3d[corr_count];
    Eigen::Vector3d *model_points = new Eigen::Vector3d[corr_count];
    Eigen::Matrix3d RotOrig;
    RotOrig << rotation[0][0], rotation[0][1], rotation[0][2],
        rotation[1][0], rotation[1][1], rotation[1][2],
        rotation[2][0], rotation[2][1], rotation[2][2];

    Eigen::Vector3d transOrig(translation[0], translation[1], translation[2]);
    
    //make a copy of projected points
    for(int i = 0; i < corr_count; i++)
    {
        normalized_points[i] << projected[i*2 + 0], projected[i*2 + 1], 1.0;
        model_points[i] << model[i*3 + 0], model[i*3 + 1], model[i*3 + 2];
    }

    normRv(corr_count, normalized_points);
    Eigen::Vector3d centroid;
    meanv(corr_count, normalized_points, &centroid);
    centroid.normalize();
    
    Eigen::Matrix3d Rim;
    getRotationbyVectors(Eigen::Vector3d(0, 0, 1), centroid, Rim);
   
    //transform all points
    for(int i = 0; i < corr_count; i++)
    {
        normalized_points[i] = Rim*normalized_points[i];
    }

    Eigen::Matrix3d Rp = Rim*RotOrig;
    Eigen::Vector3d tp = Rim*transOrig;

    int solution_count = 0;

    EPoseSolution *esolutions = getRfor2ndPose_V_Exact(&solution_count, corr_count, normalized_points, model_points, Rp, tp);

    //denormalize the poses
    *solutions = new PoseSolution[solution_count];
    for(int i = 0; i < solution_count; i++)
    {
        esolutions[i].R = Rim.transpose()*esolutions[i].R;
        esolutions[i].t = Rim.transpose()*esolutions[i].t;
        (*solutions)[i].E = esolutions[i].E;

        for(int row = 0; row < 3; row++)
        {
            (*solutions)[i].t[row] = esolutions[i].t[row];
            for(int col = 0; col < 3; col++)
            {
                (*solutions)[i].R[row][col] = esolutions[i].R(row, col);
            }
        }
    }

    delete [] esolutions;
    delete [] normalized_points;
    delete [] model_points;

    *results = solution_count;
    return solution_count;
}

}
