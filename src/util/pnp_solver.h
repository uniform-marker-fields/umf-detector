#ifndef __UMF_PNP_SOLVER_H
#define __UMF_PNP_SOLVER_H

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "../defines.h"


namespace umf {

class UMF Correspondence
{
public:
    Correspondence(float projectX, float projectY, float modelX, float modelY, float modelZ, float score = 0.f)
    {
        this->px = projectX;
        this->py = projectY;
        this->mx = modelX;
        this->my = modelY;
        this->mz = modelZ;
		this->score = score;
		this->reprojE = 0;
    }

    float px;
    float py;
    float mx;
    float my;
    float mz;
	float score;
	float reprojE;

    inline bool operator<(const Correspondence &other) const
    {
        return this->mx < other.mx ||
                (this->mx == other.mx && (this->my < other.my ||
                 (this->my == other.my && this->mz < other.mz)));
    }
};

typedef std::vector<Correspondence> CorrespondenceSet;

inline bool corrSortByScore(const Correspondence &p1, const Correspondence &p2)
{
	return p1.score > p2.score;
}

inline bool corrSortByReprjE(const Correspondence &p1, const Correspondence &p2)
{
	return p1.reprojE < p2.reprojE;
}


enum PnPFlags {
	PNP_FLAG_COMPUTE_CAMERA = 0x01, //compute the full camera pose (position, rotation)
	PNP_FLAG_GL_PROJECTION_MV = 0x02, //compute the OpenGL projection modelview matrix
	PNP_FLAG_SWAP_Y = 0x04, //swap the Y direction (in OpenGL and the bottom left corner is 0, 0 in 2D)
	PNP_FLAG_RIGHT_HANDED = 0x08, //use Right Handed coordinate system
	PNP_FLAG_USE_LAST = 0x10, // use the last cached camera pose as initial guess
	PNP_FLAG_FILTER_REPR = 0x20, //filter points based on reprojection error
	PNP_FLAG_LOOK_Z_POSITIVE = 0x40 //If set, the camera is looking down the positive Z axis 
};

class UMF PnPSolver
{
public:
    PnPSolver();
    virtual ~PnPSolver() {}

	void setIsPlanar(bool planar) { this->isPlanar = planar; }
	bool getIsPlanar() { return this->isPlanar; }

    void addCorrespondences(CorrespondenceSet correspondences);
    bool computeCameraPose(CorrespondenceSet correspondencesOrig, Eigen::Vector2i imageSize, int flags = PNP_FLAG_COMPUTE_CAMERA);

    void setCameraMatrix( Eigen::Matrix3d &cameraMatrix) { this->cameraMatrix = cameraMatrix; }
    Eigen::Matrix3d &getCameraMatrix() { return this->cameraMatrix; }

    void setDistortionCoeffs( Eigen::VectorXd &distCoeffs ) {this->distCoeffs = distCoeffs; }
    Eigen::VectorXd &getDistortionCoeffs() {return this->distCoeffs; }

    static PnPSolver *GetPnPSolver(std::string type = "EPNP");

    /**
     * @brief getWorldTransformMatrix - this is only rotation and transformation
     * @return
     */
    Eigen::Matrix4d &getWorldTransformMatrix() { return this->worldTransformMatrix; }

    /**
     * @brief getCameraTransformMatrix - this is only rotation and transformation of the camera - inverse of the #getWorldTransformMatrix
     * @return
     *
     * the #computeCameraPose has to be called with flag #PNP_FLAG_COMPUTE_CAMERA
     */
    Eigen::Matrix4d &getCameraTransformMatrix() { return this->cameraTransformMatrix; }


    /**
     * @brief getMVP get model view projection matrix for openGL based on the focal lengthes and
     * @return mvp
     *
     * the #computeCameraPose has to be called with flag #PNP_FLAG_GL_PROJECTION_MV
     */
    Eigen::Matrix4d &getMVP() { return this->mvp; }


    /**
     * @brief getCameraPos - get the camera position in 3D as if the world was fixed
     * @return
     *
     * The #computeCameraPose has to be called with flag #PNP_FLAG_COMPUTE_CAMERA
     */
    Eigen::Vector3d &getCameraPos() { return this->cameraPos; }

    /**
     * @brief getWorldPos - get the world translation relative to the camera
     * @return
     */
    Eigen::Vector3d &getWorldPos() { return this->worldPos; }

    /**
     * @brief getWorldQuaternion the world rotation only quaternion
     * @return
     */
    Eigen::Quaterniond &getWorldQuaternion() { return this->worldQuat; }

    /**
     * @brief getCameraQuaternion - get the world camera rotation quaternion
     * @return
     *
     * The #computeCameraPose has to be called with flag #PNP_FLAG_COMPUTE_CAMERA
     */
    Eigen::Quaterniond &getCameraQuaternion() { return this->cameraQuat; }

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    virtual bool pnpSolve(CorrespondenceSet correspondences, bool usePrev = false) = 0;

    Eigen::Matrix3d cameraMatrix;
    Eigen::VectorXd distCoeffs;

    Eigen::Matrix4d mvp;
    Eigen::Matrix4d worldTransformMatrix;
    Eigen::Matrix4d cameraTransformMatrix;
    Eigen::Vector3d cameraPos;
    Eigen::Vector3d worldPos;

    Eigen::Quaterniond cameraQuat;
    Eigen::Quaterniond worldQuat;

	bool isPlanar;

};



}
#endif // PNP_SOLVER_H
