// Copyright (c) 2009, V. Lepetit, EPFL
// Rewritten to use with Eigen with Robust Planar support
//  instead of OpenCV:
// Copyright (c) 2016, I. Szentandrasi
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met: 

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer. 
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution. 

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// The views and conclusions contained in the software and documentation are those
// of the authors and should not be interpreted as representing official policies, 
//   either expressed or implied, of the FreeBSD Project.


#include <iostream>
using namespace std;

#include "epnp.h"
#include "robust_planar.h"
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <Eigen/QR>

epnp::epnp(void)
{
    maximum_number_of_correspondences = 0;
    number_of_correspondences = 0;

    pws = 0;
    us = 0;
    ush = 0;
    alphas = 0;
    pcs = 0;

    planar = false;
    reprojection_error_threshold = 20.0;
    gauss_newton_threshold = 1e-5;
}

epnp::~epnp()
{
    delete [] pws;
    delete [] us;
    delete [] ush;
    delete [] alphas;
    delete [] pcs;
}

void epnp::set_internal_parameters(double uc, double vc, double fu, double fv)
{
    this->uc = uc;
    this->vc = vc;
    this->fu = fu;
    this->fv = fv;
}

void epnp::set_maximum_number_of_correspondences(int n)
{
    if (maximum_number_of_correspondences < n) {
        if (pws != 0) delete [] pws;
        if (us != 0) delete [] us;
        if (alphas != 0) delete [] alphas;
        if (pcs != 0) delete [] pcs;

        maximum_number_of_correspondences = n;
        pws = new double[3 * maximum_number_of_correspondences];
        us = new double[2 * maximum_number_of_correspondences];
        ush = new double[2 * maximum_number_of_correspondences];
        alphas = new double[4 * maximum_number_of_correspondences];
        pcs = new double[3 * maximum_number_of_correspondences];
    }
}

void epnp::reset_correspondences(void)
{
    number_of_correspondences = 0;
}

void epnp::add_correspondence(double X, double Y, double Z, double u, double v)
{
    pws[3 * number_of_correspondences    ] = X;
    pws[3 * number_of_correspondences + 1] = Y;
    pws[3 * number_of_correspondences + 2] = Z;

    us[2 * number_of_correspondences    ] = u;
    us[2 * number_of_correspondences + 1] = v;

    ush[2 * number_of_correspondences    ] = (u - uc)/fu;
    ush[2 * number_of_correspondences + 1] = (v - vc)/fv;

    number_of_correspondences++;
}

void epnp::choose_control_points(void)
{

    //stupid version TODO remove
    //cws[3][0] = cws[3][1] = cws[3][2] = 0;
    //cws[0][1] = cws[0][2] = 0; cws[0][0] = 1;
    //cws[1][0] = cws[1][2] = 0; cws[1][1] = 1;
    //cws[2][0] = cws[2][1] = 0; cws[2][2] = 1;
    //return;

    // Take C0 as the reference points centroid:
    cws[0][0] = cws[0][1] = cws[0][2] = 0;
    for(int i = 0; i < number_of_correspondences; i++)
        for(int j = 0; j < 3; j++)
            cws[0][j] += pws[3 * i + j];

    for(int j = 0; j < 3; j++)
        cws[0][j] /= number_of_correspondences;

    // Take C1, C2, and C3 from PCA on the reference points:

    //move the points based on the mean 
    RMatrixX3d PW0(number_of_correspondences, 3);

    for(int i = 0; i < number_of_correspondences; i++)
    {
        for(int j = 0 ; j < 3; j++)
        {
            PW0(i,j) = pws[3*i + j] - cws[0][j];
        }
    }

    //compute covariance matrix of the sample points relative to the mean
    RMatrix3d PW0tPW0 = PW0.transpose()*PW0;

    //compute SVD decomposition of the covariance matrix
    Eigen::JacobiSVD<RMatrix3d> eigSVD(PW0tPW0, Eigen::ComputeFullU);
    //standard deviation of the covariance matrix in the directions of the eigen vectors
    RVector3d DC = (eigSVD.singularValues().array() / number_of_correspondences ).sqrt();
    //get the eigen vectors as row matrix
    RMatrix3d UCt = eigSVD.matrixU().transpose();

    //descending order
    if(DC[0] > DC[2]*1000)
    {
        //2D case - one direction is too small
        DC[2] = 0.01;
        planar = false;
    } else {
        planar = false;
    }

    //distribute the control points based on the standard deviation around the mean of the points
    for(int i = 1; i < 4; i++) {
        int eigIndex = i - 1;
        double k = DC[eigIndex];
        for(int j = 0 ; j < 3; j ++)
        {
            cws[i][j] = cws[0][j] + k * UCt(eigIndex,j);
        }
    }

}

void epnp::compute_barycentric_coordinates(void)
{
    RMatrix3d CC;
    RMatrix3d CC_inv;

    for(int i = 0; i < 3; i++)
    {
        for(int j = 1; j < 4; j++)
        {
            CC(i,j-1) = cws[j][i] - cws[0][i];
        }
    }

    CC_inv = CC.inverse();

    for(int i = 0; i < number_of_correspondences; i++)
    {
        double * pi = pws + 3 * i;
        double * a = alphas + 4 * i;

        for(int j = 0; j < 3; j++)
        {
            a[1 + j] = CC_inv(j,0) * (pi[0] - cws[0][0]) +
                    CC_inv(j, 1) * (pi[1] - cws[0][1]) +
                    CC_inv(j, 2) * (pi[2] - cws[0][2]);
        }
        a[0] = 1.0f - a[1] - a[2] - a[3];
    }
}

void epnp::fill_M(RMatrixXd &M,
                  const int row, const double * as, const double u, const double v)
{
    for(int i = 0; i < 4; i++) {
        //M.block<1,3>(row, 3*i) = RVector3d(as[i] * fu, 0.0, as[i] * (uc - u));
        M(row, 3*i    ) = as[i] * fu;
        M(row, 3*i + 1) = 0.0;
        M(row, 3*i + 2) = as[i] * (uc - u);

        M(row + 1, 3*i    ) = 0.0;
        M(row + 1, 3*i + 1) = as[i] * fv;
        M(row + 1, 3*i + 2) = as[i] * (vc - v);
    }
}

void epnp::compute_ccs(const double * betas, const double * ut)
{
    for(int i = 0; i < 4; i++)
        ccs[i][0] = ccs[i][1] = ccs[i][2] = 0.0f;

    for(int i = 0; i < 4; i++) {
        //only the first few smallest eigenvalues are interesting
        const double * v = ut + 12 * (11 - i);
        for(int j = 0; j < 4; j++)
            for(int k = 0; k < 3; k++)
                ccs[j][k] += betas[i] * v[3 * j + k];
    }
}

void epnp::compute_pcs(void)
{
    for(int i = 0; i < number_of_correspondences; i++) {
        double * a = alphas + 4 * i;
        double * pc = pcs + 3 * i;

        for(int j = 0; j < 3; j++)
            pc[j] = a[0] * ccs[0][j] + a[1] * ccs[1][j] + a[2] * ccs[2][j] + a[3] * ccs[3][j];
    }
}

double epnp::compute_pose(double R[3][3], double t[3])
{
    //distribute the control points according to data
    choose_control_points();
    //compute the barycentric coordinates for all world points
    compute_barycentric_coordinates();

    RMatrixXd M(2 * number_of_correspondences, 12);

    //fill the 2n x 12 matrix with values
    // sum_1^4 a_{ij} * f_u * x_j^c + a_{ij} * (u_c - u_i) * z_j^c = 0
    // sum_1^4 a_{ij} * f_v * y_j^c + a_{ij} * (v_c - v_i) * z_j^c = 0
    //the two equations and just linearized so that
    // x_1^c, y_1^c, z_1^c, x_2^c... etc for the 4 control points
    for(int i = 0; i < number_of_correspondences; i++)
    {
        fill_M(M, 2*i, alphas + 4 * i, us[2 * i], us[2 * i + 1]);
    }

    RMatrix12d MtM; //covariance matrix of M (camera coordinates
    RVector12d D; //12 singular values
    RMatrix12d Ut; //left singular vectors of MtM

    //compute covariance matrix
    MtM = M.transpose()*M;

    ////solve the equation Mx = 0 -> the solution is in the null-space of the MtM
    // Eigen::JacobiSVD<RMatrix12d> eigSVD(MtM, Eigen::ComputeFullU);
    //D = eigSVD.singularValues();
    ////the M*M eigenvectors correspont to the right singular values of M, so we are cool :)
    //Ut = eigSVD.matrixU().transpose();

    
    Eigen::SelfAdjointEigenSolver<RMatrix12d> eig(MtM);
    D = eig.eigenvalues().reverse();
    Ut = eig.eigenvectors().transpose().colwise().reverse();
    
    //now the solution lies somewhere as linear combination of the null eigenvectors
    //based on the dimensionality we might have 4 cases, just to be sure we compute
    //all of them and choose the best one
    RMatrix6x10d L_6x10(6, 10);
    RVector6d Rho(6, 1);

    //ok so we are now looking for betas in the extension
    //X = \sum^dimension beta_i * v_i, where v_i are the null vectors

    compute_L_6x10(Ut.data(), L_6x10.data()); //differences between the eigenvalues
    compute_rho(Rho.data()); //the squared distances between control points in world system coordinates

    double Betas[4][4], rep_errors[4];
    double Rs[4][3][3], ts[4][3];

    find_betas_approx_1(L_6x10, Rho, Betas[1]);
    gauss_newton(L_6x10, Rho, Betas[1]);
    rep_errors[1] = compute_R_and_t(Ut.data(), Betas[1], Rs[1], ts[1]);

    find_betas_approx_2(L_6x10, Rho, Betas[2]);
    gauss_newton(L_6x10, Rho, Betas[2]);
    rep_errors[2] = compute_R_and_t(Ut.data(), Betas[2], Rs[2], ts[2]);

    find_betas_approx_3(L_6x10, Rho, Betas[3]);
    gauss_newton(L_6x10, Rho, Betas[3]);
    rep_errors[3] = compute_R_and_t(Ut.data(), Betas[3], Rs[3], ts[3]);

    //only run this, if there is a major problem - not too precise - maybe wrong somewhere
    if(reprojection_error_threshold < std::min(std::min(rep_errors[0], rep_errors[1]), rep_errors[2]))
    {
        find_betas_approx_4_m(L_6x10, Ut.data(), Rho, Betas[0]);
        //find_betas_approx_4(L_6x10, Rho, Betas[0]);
        gauss_newton(L_6x10, Rho, Betas[0]);
        rep_errors[0] = compute_R_and_t(Ut.data(), Betas[0], Rs[0], ts[0]);
    } else {
        rep_errors[0] = reprojection_error_threshold;
    }

    int N = 1;
    if (rep_errors[2] < rep_errors[1]) N = 2;
    if (rep_errors[3] < rep_errors[N]) N = 3;
    if (rep_errors[0] < rep_errors[N]) N = 0;

    gauss_newton(L_6x10, Rho, Betas[N]);
    rep_errors[N] = compute_R_and_t(Ut.data(), Betas[N], Rs[N], ts[N]);

    if(this->planar)
    {
        int count_solutions = 0;
        umf::PoseSolution *solutions;
        umf::get2ndPose(this->number_of_correspondences, this->ush, this->pws,
                   Rs[N], ts[N], //initial pose
                   &count_solutions, &solutions);
        umf::PoseSolution sol2[2];
        sol2[0] = solutions[0];
        sol2[1] = solutions[1];


        double min_error = 1e6;
        int min_index = -1;

        for(int itsol = 0; itsol < count_solutions; itsol++)
        {
            double sol_betas[4];
            //transform to get the control points
            compute_ccs_from_R_t(solutions[itsol].R, solutions[itsol].t);

            //compute betas
            compute_betas_from_ccs(Ut.data(), sol_betas);

            //gauss newton
            gauss_newton(L_6x10, Rho, sol_betas);
            //compute_R_and_T
            solutions[itsol].E = compute_R_and_t(Ut.data(), sol_betas, solutions[itsol].R, solutions[itsol].t);

            if(solutions[itsol].E < min_error)
            {
                min_index = itsol;
                min_error = solutions[itsol].E;
            }
        }

        sol2[0] = solutions[0];
        sol2[1] = solutions[1];

        if(min_index >= 0)
        {
            copy_R_and_t(solutions[min_index].R, solutions[min_index].t, R, t);

        } else {
            copy_R_and_t(Rs[N], ts[N], R, t);
            min_error = rep_errors[N];
        }

        delete [] solutions;

        return min_error;
    } else {

        copy_R_and_t(Rs[N], ts[N], R, t);

        return rep_errors[N];
    }
}


void epnp::compute_ccs_from_R_t(const double R[3][3], const double t[3])
{
    for(int i = 0; i < 4; i++)
    {
        double *cw = this->cws[i];
        double *cc = this->ccs[i];
        for(int i = 0; i < 3; i++)
        {
            cc[i] = R[i][0]*cw[0] + R[i][1]*cw[1] + R[i][2]*cw[2] + t[i];
        }
    }
}

void epnp::compute_betas_from_ccs(const double * ut, double current_betas[4])
{
    const double * v[4];

    //the four smallest eigen Values
    v[0] = ut + 12 * 11;
    v[1] = ut + 12 * 10;
    v[2] = ut + 12 *  9;
    v[3] = ut + 12 *  8;

    //the eigen values should be independent, since they were created from self-adjoint, right?
    //compute the dot products
    RVector12d v0 = RVector12d::Zero();
    RVector12d v1 = RVector12d::Zero();
    RVector12d v2 = RVector12d::Zero();
    RVector12d v3 = RVector12d::Zero();

    RVector12d x;

    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            int pi = i*3 + j;

            v0[pi] = v[0][pi];
            v1[pi] = v[1][pi];
            v2[pi] = v[2][pi];
            v3[pi] = v[3][pi];

            x[pi] = this->ccs[i][j];
        }
    }

    current_betas[0] = v0.dot(x);
    current_betas[1] = v1.dot(x);
    current_betas[2] = v2.dot(x);
    current_betas[3] = v3.dot(x);
}

void epnp::copy_R_and_t(const double R_src[3][3], const double t_src[3],
double R_dst[3][3], double t_dst[3])
{
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++)
            R_dst[i][j] = R_src[i][j];
        t_dst[i] = t_src[i];
    }
}

double epnp::dist2(const double * p1, const double * p2)
{
    return
            (p1[0] - p2[0]) * (p1[0] - p2[0]) +
            (p1[1] - p2[1]) * (p1[1] - p2[1]) +
            (p1[2] - p2[2]) * (p1[2] - p2[2]);
}

double epnp::dot(const double * v1, const double * v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

double epnp::reprojection_error(const double R[3][3], const double t[3])
{
    double sum2 = 0.0;

    for(int i = 0; i < number_of_correspondences; i++) {
        double * pw = pws + 3 * i;
        double Xc = dot(R[0], pw) + t[0];
        double Yc = dot(R[1], pw) + t[1];
        double inv_Zc = 1.0 / (dot(R[2], pw) + t[2]);
        double ue = uc + fu * Xc * inv_Zc;
        double ve = vc + fv * Yc * inv_Zc;
        double u = us[2 * i], v = us[2 * i + 1];

        sum2 += sqrt( (u - ue) * (u - ue) + (v - ve) * (v - ve) );
    }

    return sum2 / number_of_correspondences;
}

void epnp::estimate_R_and_t(double R[3][3], double t[3])
{
    double pc0[3], pw0[3];

    pc0[0] = pc0[1] = pc0[2] = 0.0;
    pw0[0] = pw0[1] = pw0[2] = 0.0;

    for(int i = 0; i < number_of_correspondences; i++) {
        const double * pc = pcs + 3 * i;
        const double * pw = pws + 3 * i;

        for(int j = 0; j < 3; j++) {
            pc0[j] += pc[j];
            pw0[j] += pw[j];
        }
    }
    for(int j = 0; j < 3; j++) {
        pc0[j] /= number_of_correspondences;
        pw0[j] /= number_of_correspondences;
    }

    RMatrix3d ABt;
    RVector3d ABt_D;
    RMatrix3d ABt_U;
    RMatrix3d ABt_V;

    ABt.setZero();

    for(int i = 0; i < number_of_correspondences; i++) {
        double * pc = pcs + 3 * i;
        double * pw = pws + 3 * i;

        for(int j = 0; j < 3; j++) {
            ABt(j, 0) += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
            ABt(j, 1) += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
            ABt(j, 2) += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
        }
    }

    Eigen::JacobiSVD<RMatrix3d> svd(ABt, Eigen::ComputeFullU | Eigen::ComputeFullV);

    ABt_D = svd.singularValues();
    ABt_U = svd.matrixU();
    ABt_V = svd.matrixV();

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            R[i][j] = ABt_U.row(i).dot(ABt_V.row(j)); //V * U^T
        }
    }
    const double det =
            R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
            R[0][2] * R[1][1] * R[2][0] - R[0][1] * R[1][0] * R[2][2] - R[0][0] * R[1][2] * R[2][1];

    //multiply the last column!!! alternatively we cant take the third column of V and multiply by -1 and compute R again
    if (det < 0) {
        R[0][2] = -R[0][2];
        R[1][2] = -R[1][2];
        R[2][2] = -R[2][2];
    }

    t[0] = pc0[0] - dot(R[0], pw0);
    t[1] = pc0[1] - dot(R[1], pw0);
    t[2] = pc0[2] - dot(R[2], pw0);

}

void epnp::print_pose(const double R[3][3], const double t[3])
{
    cout << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << t[0] << endl;
    cout << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << t[1] << endl;
    cout << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << t[2] << endl;
}

void epnp::solve_for_sign(void)
{
    if (pcs[2] < 0.0) {
        for(int i = 0; i < 4; i++)
            for(int j = 0; j < 3; j++)
                ccs[i][j] = -ccs[i][j];

        for(int i = 0; i < number_of_correspondences; i++) {
            pcs[3 * i    ] = -pcs[3 * i];
            pcs[3 * i + 1] = -pcs[3 * i + 1];
            pcs[3 * i + 2] = -pcs[3 * i + 2];
        }
    }
}

double epnp::compute_R_and_t(const double * ut, const double * betas,
                             double R[3][3], double t[3])
{
    compute_ccs(betas, ut);
    compute_pcs();

    solve_for_sign();

    estimate_R_and_t(R, t);

    return reprojection_error(R, t);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


void epnp::find_betas_approx_1(const RMatrix6x10d &L_6x10, const RVector6d &Rho,
                               double * betas)
{
    double counter = 0;
    double denom = 0;

    for(int i = 0; i < 6; i++)
    {
        counter += std::sqrt(Rho[i] * L_6x10(i, 0));
        denom += L_6x10(i, 0);
    }

    betas[0] = counter/denom;

    betas[1] = 0.;
    betas[2] = 0.;
    betas[3] = 0.;
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_1 = [B11 B12     B13         B14]

void epnp::find_betas_approx_1_old(const RMatrix6x10d &L_6x10, const RVector6d &Rho,
                               double * betas)
{
    RMatrix6x4d L_6x4(6,4);
    RVector4d B4;
    B4.setZero();

    for(int i = 0; i < 6; i++) {
        L_6x4(i,0) = L_6x10(i,0);
        L_6x4(i,1) = L_6x10(i,1);
        L_6x4(i,2) = L_6x10(i,3);
        L_6x4(i,3) = L_6x10(i,6);
    }

    B4 = L_6x4.fullPivLu().solve(Rho);

    if (B4[0] < 0) {
        betas[0] = sqrt(-B4[0]);
        betas[1] = -B4[1] / betas[0];
        betas[2] = -B4[2] / betas[0];
        betas[3] = -B4[3] / betas[0];
    } else {
        betas[0] = sqrt(B4[0]);
        betas[1] = B4[1] / betas[0];
        betas[2] = B4[2] / betas[0];
        betas[3] = B4[3] / betas[0];
    }
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_2 = [B11 B12 B22                            ]

void epnp::find_betas_approx_2(const RMatrix6x10d &L_6x10, const RVector6d &Rho,
                               double * betas)
{
    RMatrix6x3d L_6x3(6,3);
    RVector3d B3;

    for(int i = 0; i < 6; i++)
    {
        L_6x3.row(i) = L_6x10.block<1,3>(i,0);
    }

    B3 = L_6x3.fullPivLu().solve(Rho);
    
    //changed to match matlab code
    betas[0] = std::sqrt(std::abs(B3[0]))*sgn(B3[1])*sgn(B3[2]);
    betas[1] = std::sqrt(std::abs(B3[2]));

    /*
    if (B3[0] < 0) {
        betas[0] = sqrt(-B3[0]);
        betas[1] = (B3[2] < 0) ? sqrt(-B3[2]) : 0.0;
    } else {
        betas[0] = sqrt(B3[0]);
        betas[1] = (B3[2] > 0) ? sqrt(B3[2]) : 0.0;
    }

    if (B3[1] < 0) betas[0] = -betas[0];
    */

    betas[2] = 0.0;
    betas[3] = 0.0;
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_3 = [B11 B12 B22 B13 B23                    ]

void epnp::find_betas_approx_3_old(const RMatrix6x10d &L_6x10, const RVector6d &Rho,
                               double * betas)
{
    RMatrix6x5d L_6x5 (6,5);
    RVector5d B5(5,1);

    for(int i = 0; i < 6; i++)
    {
        L_6x5.row(i) = L_6x10.block<1,5>(i,0);
    }

    B5 = L_6x5.fullPivLu().solve(Rho);

    if (B5[0] < 0) {
        betas[0] = sqrt(-B5[0]);
        betas[1] = (B5[2] < 0) ? sqrt(-B5[2]) : 0.0;
    } else {
        betas[0] = sqrt(B5[0]);
        betas[1] = (B5[2] > 0) ? sqrt(B5[2]) : 0.0;
    }
    if (B5[1] < 0) betas[0] = -betas[0];
    betas[2] = B5[3] / betas[0];
    betas[3] = 0.0;
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_3 = [B11 B12 B22 B13 B23 B33                ]
void epnp::find_betas_approx_3(const RMatrix6x10d &L_6x10, const RVector6d &Rho,
                               double * betas)
{
    RMatrix6d L_6x6(6, 6);
    RVector6d B6(6, 1);

    for(int i = 0; i < 6; i++)
    {
        L_6x6.row(i) = L_6x10.block<1,6>(i, 0);
    }

    B6 = L_6x6.fullPivLu().solve(Rho);

    betas[0] = std::sqrt(std::abs(B6[0]))*sgn(B6[3])*sgn(B6[5]);
    betas[1] = std::sqrt(std::abs(B6[2]))*sgn(B6[4])*sgn(B6[5]);
    betas[2] = std::sqrt(std::abs(B6[5]));
    betas[3] = 0.0;
}

/*
inline double VGet4(const RMatrixXd &V, int row, int col)
{
    return V(row, V.cols() - 4 + col);
}

void extractXaXb4(const RMatrixXd &V, const RVector10d& X0, int a, int b, RVector10d &result)
{
    result[0] = X0(a)*X0(b);
    result[1] = X0(a)*VGet4(V,b,0) + VGet4(V,a,0)*X0(b);

    result[2] = VGet4(V,a,0)*VGet4(V,b,0);
    result[3] = X0(a)*VGet4(V,b,1) + VGet4(V,a,1)*X0(b);
    result[4] = VGet4(V,a,0)*VGet4(V,b,1) + VGet4(V,a,1)*VGet4(V,b,0);
    result[5] = VGet4(V,a,1)*VGet4(V,b,1);

    result[6] = X0(a)*VGet4(V,b,2) + VGet4(V,a,2)*X0(b);
    result[7] = VGet4(V,a,0)*VGet4(V,b,2) + VGet4(V,a,2)*VGet4(V,b,0);
    result[8] = VGet4(V,a,1)*VGet4(V,b,2) + VGet4(V,a,2)*VGet4(V,b,1);
    result[9] = VGet4(V,a,2)*VGet4(V,b,2);
}


void epnp::find_betas_approx_4(const RMatrix6x10d &L_6x10, const RVector6d &Rho, double *betas)
{
    //first extract null space
    Eigen::JacobiSVD<RMatrix6x10d> svd(L_6x10, Eigen::ComputeFullV);

    //get the right singular vectors -> null space
    RMatrixXd V = svd.matrixV();

    std::cout << V << std::endl;

    //compute one possible solution for our case
    RVector10d X0 = L_6x10.fullPivLu().solve(Rho);

    //compute additional constraints to reduce the number of possible solutions
    int rowAA = 0;
    RMatrixXd AA(10,9);
    RVectorXd yy(10, 1);
    RVectorXd alphas(9, 1);
    const int ndof = 4;

    Eigen::Matrix4i idx;

    // betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
    idx << 0, 1, 3, 6,
           1, 2, 4, 7,
           3, 4, 5, 8,
           6, 7, 8, 9;


    RVector10d XiiXjk;
    RVector10d XikXji;

    for(int i = 0; i < ndof; i++)
    {
        for(int j = i+1; j < ndof; j++)
        {
            for(int k = j; k < ndof; k++, rowAA++)
            {
                //get quatratic forms :)
                extractXaXb4(V, X0, idx(i,i), idx(j,k), XiiXjk);
                extractXaXb4(V, X0, idx(i,k), idx(j,i), XikXji);
                //x_ii * x_jk = x_ik*x_ji
                AA.block<1,9>(rowAA, 0) = (XikXji - XiiXjk).block<9,1>(1, 0);
                yy(rowAA) = XiiXjk(0) - XikXji(0);
            }
        }
    }

    alphas = AA.fullPivLu().solve(yy);

    //compute the final solution
    for(int i = 0; i < 10; i++) //X0 size
    {
        for(int j = 0; j < 3; j++) //3 columns
        {
            X0(i) += alphas(j)*VGet4(V, i, j);
        }
    }

    betas[0] = std::sqrt(std::abs(X0(0)))*sgn(X0(6));
    betas[1] = std::sqrt(std::abs(X0(2)))*sgn(X0(7));
    betas[2] = std::sqrt(std::abs(X0(5)))*sgn(X0(8));
    betas[3] = std::sqrt(std::abs(X0(9)));
}
*/

void epnp::find_betas_approx_4_m(const RMatrix6x10d &L_6x10, const double *ut, const RVector6d &Rho,
                               double * betas)
{
    RMatrix9x11d D(9,11);
    D.setZero();
    //[B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
    D.block<6,10>(0, 0) = L_6x10;
    D.block<6, 1>(0, 10) = -Rho;

    const double * v[4];

    //the four smallest eigen Values
    v[0] = ut + 12 * 11;
    v[1] = ut + 12 * 10;
    v[2] = ut + 12 *  9;
    v[3] = ut + 12 *  8;


    //we do comparisons now between 
    //1 - 4, 2 - 4; 1 - 4, 3 - 4; 2 - 4, 3 - 4
    double dv[4][3][6];

    for(int i = 0; i < 4; i++)
    {
        int a = 0;
        int b = 1;
        int c = 3;
        for(int j = 0; j < 3; j++)
        {
            dv[i][j][0] = v[i][3*a + 0] - v[i][3*c + 0];
            dv[i][j][1] = v[i][3*a + 1] - v[i][3*c + 1];
            dv[i][j][2] = v[i][3*a + 2] - v[i][3*c + 2];
            
            dv[i][j][3] = v[i][3*b + 0] - v[i][3*c + 0];
            dv[i][j][4] = v[i][3*b + 1] - v[i][3*c + 1];
            dv[i][j][5] = v[i][3*b + 2] - v[i][3*c + 2];

            b++;
            if (b == 3)
            {
                a = 1;
                b = 2;
            }
        }
    }

    for(int ci = 0; ci < 3; ci++)
    {
        D(6 + ci, 0) = dot(dv[0][ci], &(dv[0][ci][3]));
        D(6 + ci, 1) = dot(dv[1][ci], &(dv[0][ci][3])) + dot(dv[0][ci], &(dv[1][ci][3]));
        D(6 + ci, 2) = dot(dv[1][ci], &(dv[1][ci][3]));
        D(6 + ci, 3) = dot(dv[2][ci], &(dv[0][ci][3])) +dot(dv[0][ci], &(dv[2][ci][3]));
        D(6 + ci, 4) = dot(dv[2][ci], &(dv[1][ci][3])) +dot(dv[1][ci], &(dv[2][ci][3]));
        D(6 + ci, 5) = dot(dv[2][ci], &(dv[2][ci][3]));
        D(6 + ci, 6) = dot(dv[3][ci], &(dv[0][ci][3])) +dot(dv[0][ci], &(dv[3][ci][3]));
        D(6 + ci, 7) = dot(dv[3][ci], &(dv[1][ci][3])) +dot(dv[1][ci], &(dv[3][ci][3]));
        D(6 + ci, 8) = dot(dv[3][ci], &(dv[2][ci][3])) +dot(dv[2][ci], &(dv[3][ci][3]));
        D(6 + ci, 9) = dot(dv[3][ci], &(dv[3][ci][3]));
    }

    //get the null space of S
    RMatrixXd V = D.fullPivLu().kernel();

    //build the second set of equations based on the constraints between permutations
    const int N = V.cols();
    const int n = 4;

    Eigen::Matrix4i idx;

    // betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
    idx << 0, 1, 3, 6,
           1, 2, 4, 7,
           3, 4, 5, 8,
           6, 7, 8, 9;

    //generate first set of equations Bii.Bjj = Bij.Bji n*(n-1)/2 eqs
    int nrowsK = n*(n-1)/2 + n*(n-1)*n/2;
    int ncolsK = N*(N+1)/2;

    RMatrixXd K(nrowsK, ncolsK);
    K.setZero();

    int t = 0;
    //first part Bii.Bjj = Bij.Bij
    for(int i = 0; i < n; i++)
    {
        for(int j = i+1; j < n; j++)
        {
            int offset = 0;
            for(int a = 0; a < N; a++)
            {

                K(t, offset) = V(idx(i, i), a)*V(idx(j, j), a) - V(idx(i, j), a)*V(idx(i, j), a);
                ++offset;
                
                for(int b = a + 1; b < N; b++, offset++)
                {
                    K(t, offset) = V(idx(i, i), a)*V(idx(j, j), b) - V(idx(i, j), a)*V(idx(i, j), b) +
                                   V(idx(i, i), b)*V(idx(j, j), a) - V(idx(i, j), b)*V(idx(i, j), a);
                }
            }
            t++;
        }
    }

    //second part Bij*Bik = Bii*Bjk
    for(int k = 0; k < n; k++)
    {
        for(int j = k; j < n; j++)
        {
            for(int i = 0; i < n; i++)
            {
                if(i !=j && i != k)
                {
                    int offset = 0;
                    for(int a = 0; a < N; a++)
                    {

                        K(t, offset) = V(idx(i, j), a)*V(idx(i, k), a) - V(idx(i, i), a)*V(idx(j, k), a);
                        ++offset;

                        for(int b = a + 1; b < N; b++, offset++)
                        {
                            K(t, offset) = V(idx(i, j), a)*V(idx(i, k), b) - V(idx(i, i), a)*V(idx(j, k), b) +
                                           V(idx(i, j), b)*V(idx(i, k), a) - V(idx(i, i), b)*V(idx(j, k), a);
                        }
                    }
                    t++;
                }
            }
        }
    }

    //now K contains the new matrix for the constraint linear equations; linearization done
    
    //should be constraint quite well resulting in one smallest eigenvector :)
    Eigen::SelfAdjointEigenSolver<RMatrixXd> es(K.transpose()*K);
    Eigen::MatrixXd eigVectors = es.eigenvectors();
    Eigen::VectorXd lambdas_ = eigVectors.col(0);

    //std::cout << lambdas_ << std::endl;
    
    RVector5d lambda;
    lambda(0) = std::sqrt(std::abs(lambdas_(0)));
    lambda(1) = std::sqrt(std::abs(lambdas_( 5)))*sgn(lambdas_(1))*sgn(lambdas_(0));
    lambda(2) = std::sqrt(std::abs(lambdas_( 9)))*sgn(lambdas_(2))*sgn(lambdas_(0));
    lambda(3) = std::sqrt(std::abs(lambdas_(12)))*sgn(lambdas_(3))*sgn(lambdas_(0));
    lambda(4) = std::sqrt(std::abs(lambdas_(14)))*sgn(lambdas_(4))*sgn(lambdas_(0));

    RVectorXd vbeta = lambda(0)*V.col(0) + lambda(1)*V.col(1) + lambda(2)*V.col(2) + lambda(3)*V.col(3) + lambda(4)*V.col(4);
    
    std::cout << V << std::endl;
    std::cout << "Vbeta: \n" << vbeta << std::endl;

    //[B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
    //matlab has it the other way around
    betas[0] = std::sqrt(std::abs(vbeta(0)))*sgn(vbeta(6));
    betas[1] = std::sqrt(std::abs(vbeta(2)))*sgn(vbeta(7));
    betas[2] = std::sqrt(std::abs(vbeta(5)))*sgn(vbeta(8));
    betas[3] = std::sqrt(std::abs(vbeta(9)));
}

void epnp::compute_L_6x10(const double * ut, double * l_6x10)
{
    const double * v[4];

    //the four smallest eigen Values
    v[0] = ut + 12 * 11;
    v[1] = ut + 12 * 10;
    v[2] = ut + 12 *  9;
    v[3] = ut + 12 *  8;

    double dv[4][6][3];

    //for all four null eigen vectors compute the constraints between points
    for(int i = 0; i < 4; i++) {
        int a = 0, b = 1;
        for(int j = 0; j < 6; j++) {
            dv[i][j][0] = v[i][3 * a    ] - v[i][3 * b];
            dv[i][j][1] = v[i][3 * a + 1] - v[i][3 * b + 1];
            dv[i][j][2] = v[i][3 * a + 2] - v[i][3 * b + 2];

            b++;
            if (b > 3) {
                a++;
                b = a + 1;
            }
        }
    }

    //store the length's of the differences
    //the six we iterate through are the distances parts of the null vectors
    // 01, 02, 03, 12, 13, 23
    //the 10 we fill are the dot products of combinations from different null vectors
    // V11, 2*V12, V22, 2*V13, 2*V23, V33, 2*V14, 2*V24, 2*V34, V44
    for(int i = 0; i < 6; i++) {
        double * row = l_6x10 + 10 * i;

        row[0] =        dot(dv[0][i], dv[0][i]);
        row[1] = 2.0f * dot(dv[0][i], dv[1][i]);
        row[2] =        dot(dv[1][i], dv[1][i]);
        row[3] = 2.0f * dot(dv[0][i], dv[2][i]);
        row[4] = 2.0f * dot(dv[1][i], dv[2][i]);
        row[5] =        dot(dv[2][i], dv[2][i]);
        row[6] = 2.0f * dot(dv[0][i], dv[3][i]);
        row[7] = 2.0f * dot(dv[1][i], dv[3][i]);
        row[8] = 2.0f * dot(dv[2][i], dv[3][i]);
        row[9] =        dot(dv[3][i], dv[3][i]);
    }
}

void epnp::compute_rho(double * rho)
{
    rho[0] = dist2(cws[0], cws[1]);
    rho[1] = dist2(cws[0], cws[2]);
    rho[2] = dist2(cws[0], cws[3]);
    rho[3] = dist2(cws[1], cws[2]);
    rho[4] = dist2(cws[1], cws[3]);
    rho[5] = dist2(cws[2], cws[3]);
}

void epnp::compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
                                        double betas[4], RMatrix6x4d &A, RVector6d &b)
{
    for(int i = 0; i < 6; i++) {
        const double * rowL = l_6x10 + i * 10;
        double * rowA = A.data() + i * 4;

        rowA[0] = 2 * rowL[0] * betas[0] +     rowL[1] * betas[1] +     rowL[3] * betas[2] +     rowL[6] * betas[3];
        rowA[1] =     rowL[1] * betas[0] + 2 * rowL[2] * betas[1] +     rowL[4] * betas[2] +     rowL[7] * betas[3];
        rowA[2] =     rowL[3] * betas[0] +     rowL[4] * betas[1] + 2 * rowL[5] * betas[2] +     rowL[8] * betas[3];
        rowA[3] =     rowL[6] * betas[0] +     rowL[7] * betas[1] +     rowL[8] * betas[2] + 2 * rowL[9] * betas[3];

        b(i) = rho[i] -
                (
                    rowL[0] * betas[0] * betas[0] +
                rowL[1] * betas[0] * betas[1] +
                rowL[2] * betas[1] * betas[1] +
                rowL[3] * betas[0] * betas[2] +
                rowL[4] * betas[1] * betas[2] +
                rowL[5] * betas[2] * betas[2] +
                rowL[6] * betas[0] * betas[3] +
                rowL[7] * betas[1] * betas[3] +
                rowL[8] * betas[2] * betas[3] +
                rowL[9] * betas[3] * betas[3]
                );
    }
}

static inline bool local_isnan(double x) { return x != x; }
static inline bool local_isinf(double x) { return !local_isnan(x) && local_isnan(x - x); }

void epnp::gauss_newton(const RMatrix6x10d &L_6x10, const RVector6d &Rho,
                        double betas[4])
{
    const int iterations_number = 10;

    RMatrix6x4d A(6,4);
    RVector6d B(6, 1);
    RVector4d X;

    for(int k = 0; k < iterations_number; k++)
    {
        compute_A_and_b_gauss_newton(L_6x10.data(), Rho.data(), betas, A, B);
        //based on the precision needed, we can change this
        //X = A.fullPivLu().solve(B);
        X = A.colPivHouseholderQr().solve(B);
        //qr_solve(A, B, X);
        if(local_isnan(X[0]) || local_isinf(X[3]) || X.squaredNorm() < this->gauss_newton_threshold)
        {
            break;
        }
        for(int i = 0; i < 4; i++)
        {
            betas[i] += X[i];
        }
    }
}

void epnp::qr_solve(RMatrix6x4d &A, RVector6d &b, RVector4d &X)
{
    static int max_nr = 0;
    static double * A1, * A2;

    const int nr = A.rows();
    const int nc = A.cols();

    if (max_nr != 0 && max_nr < nr) {
        delete [] A1;
        delete [] A2;
    }
    if (max_nr < nr) {
        max_nr = nr;
        A1 = new double[nr];
        A2 = new double[nr];
    }

    double * pA = A.data(), * ppAkk = pA;
    for(int k = 0; k < nc; k++) {
        double * ppAik = ppAkk, eta = fabs(*ppAik);
        for(int i = k + 1; i < nr; i++) {
            double elt = fabs(*ppAik);
            if (eta < elt) eta = elt;
            ppAik += nc;
        }

        if (eta == 0) {
            A1[k] = A2[k] = 0.0;
            cerr << "God damnit, A is singular, this shouldn't happen." << endl;
            return;
        } else {
            double * ppAik = ppAkk, sum = 0.0, inv_eta = 1. / eta;
            for(int i = k; i < nr; i++) {
                *ppAik *= inv_eta;
                sum += *ppAik * *ppAik;
                ppAik += nc;
            }
            double sigma = sqrt(sum);
            if (*ppAkk < 0)
                sigma = -sigma;
            *ppAkk += sigma;
            A1[k] = sigma * *ppAkk;
            A2[k] = -eta * sigma;
            for(int j = k + 1; j < nc; j++) {
                double * ppAik = ppAkk, sum = 0;
                for(int i = k; i < nr; i++) {
                    sum += *ppAik * ppAik[j - k];
                    ppAik += nc;
                }
                double tau = sum / A1[k];
                ppAik = ppAkk;
                for(int i = k; i < nr; i++) {
                    ppAik[j - k] -= tau * *ppAik;
                    ppAik += nc;
                }
            }
        }
        ppAkk += nc + 1;
    }

    // b <- Qt b
    double * ppAjj = pA, * pb = b.data();
    for(int j = 0; j < nc; j++) {
        double * ppAij = ppAjj, tau = 0;
        for(int i = j; i < nr; i++)	{
            tau += *ppAij * pb[i];
            ppAij += nc;
        }
        tau /= A1[j];
        ppAij = ppAjj;
        for(int i = j; i < nr; i++) {
            pb[i] -= tau * *ppAij;
            ppAij += nc;
        }
        ppAjj += nc + 1;
    }

    // X = R-1 b
    double * pX = X.data();
    pX[nc - 1] = pb[nc - 1] / A2[nc - 1];
    for(int i = nc - 2; i >= 0; i--) {
        double * ppAij = pA + i * nc + (i + 1), sum = 0;

        for(int j = i + 1; j < nc; j++) {
            sum += *ppAij * pX[j];
            ppAij++;
        }
        pX[i] = (pb[i] - sum) / A2[i];
    }
}



void epnp::relative_error(double & rot_err, double & transl_err,
                          const double Rtrue[3][3], const double ttrue[3],
const double Rest[3][3],  const double test[3])
{
    double qtrue[4], qest[4];

    mat_to_quat(Rtrue, qtrue);
    mat_to_quat(Rest, qest);

    double rot_err1 = sqrt((qtrue[0] - qest[0]) * (qtrue[0] - qest[0]) +
            (qtrue[1] - qest[1]) * (qtrue[1] - qest[1]) +
            (qtrue[2] - qest[2]) * (qtrue[2] - qest[2]) +
            (qtrue[3] - qest[3]) * (qtrue[3] - qest[3]) ) /
            sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

    double rot_err2 = sqrt((qtrue[0] + qest[0]) * (qtrue[0] + qest[0]) +
            (qtrue[1] + qest[1]) * (qtrue[1] + qest[1]) +
            (qtrue[2] + qest[2]) * (qtrue[2] + qest[2]) +
            (qtrue[3] + qest[3]) * (qtrue[3] + qest[3]) ) /
            sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

    rot_err = min(rot_err1, rot_err2);

    transl_err =
            sqrt((ttrue[0] - test[0]) * (ttrue[0] - test[0]) +
            (ttrue[1] - test[1]) * (ttrue[1] - test[1]) +
            (ttrue[2] - test[2]) * (ttrue[2] - test[2])) /
            sqrt(ttrue[0] * ttrue[0] + ttrue[1] * ttrue[1] + ttrue[2] * ttrue[2]);
}

void epnp::mat_to_quat(const double R[3][3], double q[4])
{
    double tr = R[0][0] + R[1][1] + R[2][2];
    double n4;

    if (tr > 0.0f) {
        q[0] = R[1][2] - R[2][1];
        q[1] = R[2][0] - R[0][2];
        q[2] = R[0][1] - R[1][0];
        q[3] = tr + 1.0f;
        n4 = q[3];
    } else if ( (R[0][0] > R[1][1]) && (R[0][0] > R[2][2]) ) {
        q[0] = 1.0f + R[0][0] - R[1][1] - R[2][2];
        q[1] = R[1][0] + R[0][1];
        q[2] = R[2][0] + R[0][2];
        q[3] = R[1][2] - R[2][1];
        n4 = q[0];
    } else if (R[1][1] > R[2][2]) {
        q[0] = R[1][0] + R[0][1];
        q[1] = 1.0f + R[1][1] - R[0][0] - R[2][2];
        q[2] = R[2][1] + R[1][2];
        q[3] = R[2][0] - R[0][2];
        n4 = q[1];
    } else {
        q[0] = R[2][0] + R[0][2];
        q[1] = R[2][1] + R[1][2];
        q[2] = 1.0f + R[2][2] - R[0][0] - R[1][1];
        q[3] = R[0][1] - R[1][0];
        n4 = q[2];
    }
    double scale = 0.5f / double(sqrt(n4));

    q[0] *= scale;
    q[1] *= scale;
    q[2] *= scale;
    q[3] *= scale;
}
