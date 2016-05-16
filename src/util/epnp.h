#ifndef epnp_h
#define epnp_h

#include <Eigen/Core>

//EPNP based on 2009, V. Lepetit, EPFL. Rewritten to use Eigen library instead of OpenCV
//additionally since we are tracking only planar, robust planar detection is also included

//since eigen defaults to column major, it's a problem -> have to define matrices

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RMatrixXd;
typedef Eigen::Matrix<double, Eigen::Dynamic,              4, Eigen::RowMajor> RMatrixX4d;
typedef Eigen::Matrix<double, Eigen::Dynamic,              3, Eigen::RowMajor> RMatrixX3d;
typedef Eigen::Matrix<double,              3,              3, Eigen::RowMajor> RMatrix3d;
typedef Eigen::Matrix<double,             12,             12, Eigen::RowMajor> RMatrix12d;
typedef Eigen::Matrix<double,              6,             10, Eigen::RowMajor> RMatrix6x10d;
typedef Eigen::Matrix<double,              9,             11, Eigen::RowMajor> RMatrix9x11d;
typedef Eigen::Matrix<double,              6,              6, Eigen::RowMajor> RMatrix6d;
typedef Eigen::Matrix<double,              6,              5, Eigen::RowMajor> RMatrix6x5d;
typedef Eigen::Matrix<double,              6,              4, Eigen::RowMajor> RMatrix6x4d;
typedef Eigen::Matrix<double,              6,              3, Eigen::RowMajor> RMatrix6x3d;
//for vector we don't care, just define them anyways
typedef Eigen::Matrix<double, Eigen::Dynamic,              1> RVectorXd;
typedef Eigen::Matrix<double,             12,              1> RVector12d;
typedef Eigen::Matrix<double,             10,              1> RVector10d;
typedef Eigen::Matrix<double,              6,              1> RVector6d;
typedef Eigen::Matrix<double,              5,              1> RVector5d;
typedef Eigen::Matrix<double,              4,              1> RVector4d;
typedef Eigen::Matrix<double,              3,              1> RVector3d;


class epnp {
public:
    epnp(void);
    ~epnp();

    void set_internal_parameters(const double uc, const double vc,
                                 const double fu, const double fv);

    void set_maximum_number_of_correspondences(const int n);
    void reset_correspondences(void);
    void add_correspondence(const double X, const double Y, const double Z,
                            const double u, const double v);

    double compute_pose(double R[3][3], double T[3]);

    void relative_error(double & rot_err, double & transl_err,
                        const double Rtrue[3][3], const double ttrue[3],
    const double Rest[3][3],  const double test[3]);

    void print_pose(const double R[3][3], const double t[3]);
    double reprojection_error(const double R[3][3], const double t[3]);

//private:
    void choose_control_points(void);
    void compute_barycentric_coordinates(void);
    void fill_M(RMatrixXd &M, const int row, const double * alphas, const double u, const double v);
    void compute_ccs(const double * betas, const double * ut);
    void compute_pcs(void);

    void solve_for_sign(void);
    
    void find_betas_approx_1(const RMatrix6x10d &L_6x10, const RVector6d &Rho, double * betas);
    void find_betas_approx_1_old(const RMatrix6x10d &L_6x10, const RVector6d &Rho, double * betas);
    void find_betas_approx_2(const RMatrix6x10d &L_6x10, const RVector6d &Rho, double * betas);
    void find_betas_approx_3(const RMatrix6x10d &L_6x10, const RVector6d &Rho, double * betas);
    void find_betas_approx_3_old(const RMatrix6x10d &L_6x10, const RVector6d &Rho, double * betas);
    //void find_betas_approx_4(const RMatrix6x10d &L_6x10, const RVector6d &Rho, double *betas);
    void find_betas_approx_4_m(const RMatrix6x10d &L_6x10, const double * ut, const RVector6d &Rho, double *betas);
    void qr_solve(RMatrix6x4d &A, RVector6d &b, RVector4d &X);

    double dot(const double * v1, const double * v2);
    double dist2(const double * p1, const double * p2);

    void compute_rho(double * rho);
    void compute_L_6x10(const double * ut, double * l_6x10);

    void gauss_newton(const RMatrix6x10d &L_6x10, const RVector6d &Rho, double current_betas[4]);
    void compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
                                      double cb[4], RMatrix6x4d &A, RVector6d &b);

    double compute_R_and_t(const double * ut, const double * betas,
                           double R[3][3], double t[3]);

    void estimate_R_and_t(double R[3][3], double t[3]);

    void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],
    double R_src[3][3], double t_src[3]);

    void mat_to_quat(const double R[3][3], double q[4]);


    void compute_ccs_from_R_t(const double R[3][3], const double t[3]);

    void compute_betas_from_ccs(const double * ut, double current_betas[4]);


    double uc, vc, fu, fv; //camera parameters u-center, v-center, focal-u, focal-v

    double * pws; //world points
    double * alphas; //world points in barycentric coordinates
    double * us; //the 2d positions of points
    double * ush; //in homogenic coordinate system
    double * pcs;
    int maximum_number_of_correspondences;
    int number_of_correspondences;

    double cws[4][3]; //control points in world system
    double ccs[4][3];
    double cws_determinant;

    bool planar;
    double reprojection_error_threshold;
    double gauss_newton_threshold;
};

#endif
