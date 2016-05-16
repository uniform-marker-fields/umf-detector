#ifndef __UMF_MODEL_H
#define __UMF_MODEL_H

#include <set>
#include <vector>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include "util/pnp_solver.h"
#include "util/image.h"
#include "util/mask_tracker.h"
#include "marker.h"

namespace umf {



template <int NCHAN>
class UMF Model
{
public:
    Model();
    ~Model();

    /**
     * Match the extracted image information against the Marker edges and save the location into loc
     * and save the correct edge corners internally in correspondences (access through getCorrespondences)
     *
     * @param img - the input image (used for corner search)
     * @param subH - width of the extracted marker region (in number of fields)
     * @param subH - height of the extracted marker region (in number of fields)
     * @param edgeDir - the extracted edge directions
     * @param extractionPoints - the estimated module centers of the marker in image pixels
     * @param loc - output - the location of the left top corner of the extracted marker region
     * @param show - if the output should be drawn to the user for debugging
     * @return - if the extracted region was successfully matched with a marker position
     */
    template<class T, class fptype>
    bool matchModel(Image<T, NCHAN> *img, unsigned short subW, unsigned short subH,
                    std::vector< typename Marker<NCHAN>::DirectionType > &edgeDir,
                    std::vector< Eigen::Matrix<fptype, 2, 1> > &extractionPoints,
                    Location &loc, bool show = false);

    /**
     * Compute the camera position and rotation based on the detected correspondences
     * 
     * @param img - the input image used to extract positions for refinement
     * @param subW, subH ignored
     * @param refine - if more points should be find to refine camera position (iteratively search for points in the neighborhood)
     * @param show - show the results using the global drawing singleton
     * @param mask - optional during refinement process mask out regions where no point should be found
     * @return if successfully computed camera pose
     */
    template<class T>
    bool computeCameraPosition(Image<T, NCHAN> *img, unsigned short subW, unsigned short subH,
                               bool refine = true,
                               bool show = false, ImageGray* mask = NULL);
                               
    /**
     * Compute the sum of reprojection error in image space for all correspondences
     * @param imgSize is required for precise reprojection
     */
	float computeReprojectionErrors(const Eigen::Vector2i &imgSize);

    /**
     * Scale all reprojection error based on image diameter size
     */
	void scaleReprojectionByImageSize(const Eigen::Vector2i &imgSize);
	float getTileSize(const Eigen::Vector2i &imgSize);
    /**
     * scale all reprojection errors based on detected marker module size
     */
	void scaleReprojectionByTileSize(const Eigen::Vector2i &imgSize);
	
    /**
     * Compute 2D homography between the image and marker
     */
    bool computeHomography(bool show = false);

    /**
     * Project arbitrary 3D points into image space based on the computed camera pose
     *
     * @param modelPoints - 3D points
     * @param imagePoints - the resulting vector containing the matching 2D coordinates in image space
     * @param imageSize - the size of the image
     * @param distort - if points should be also distorted based on the camera calibration
     * @param swap - if the x and y axis should be swapped for the model points, since marker space and image
     *             space does not match 
     */
    void projectPoints(std::vector<Eigen::Vector3f> &modelPoints,
                       std::vector<Eigen::Vector2f> &imagePoints,
                       Eigen::Vector2i imageSize,
                       bool distort = true,
                       bool swap = false);

	MaskTracker &getMaskTracker() {return this->maskTracker; }
	void updateMaskTracker(Eigen::Vector2i imgSize);

    /**
     * @brief wether to find the corners in the image, or just rely on the detected field centers
     * @param useCorner flag to use corner search
     */
    void setUseCornerSearch(bool useCorner) { this->useCornerSearch = useCorner; }
    bool getUseCornerSearch() {return this->useCornerSearch; }

	/**
	 * @brief if camera pose should use subpixel search
	 * @param useSubpix flag to use subpixel search
	 */

	void setUseSubPixel(bool useSubpix) { this->useSubPixel = useSubpix; }
	bool getUseSubPixel() { return this->useSubPixel; }

    /**
     * @brief getCorrespondeces get the correspondences
     * @return
     */
    CorrespondenceSet &getCorrespondences() {return this->correspondences; }
    void setCorrespondences(CorrespondenceSet &p) {this->correspondences = p;}

    void addMarker(Marker<NCHAN> *marker);
    Marker<NCHAN> *getMarker(){ return this->marker; }

	void setCameraProperties(Eigen::Matrix3d &matrix, Eigen::VectorXd &distCoeffs);
	const Eigen::Matrix3d &getCameraMatrix() const;

    /**
     * Get the camera's position and rotation quaternion (w,x,y,z) relative to
     * the marker at position 0,0,0
     */
	int getCameraPosRot(double cameraPos[3], double rotationQuat[4]);
    /**
     * Get the world's position and rotation quaternion (w,x,y,z) relative to
     * the camera at position 0,0,0 and looking down the z axis
     */
    int getWorldPosRot(double cameraPos[3], double rotationQuat[4]);

    /**
     * Get the 2D homography between the marker and the image
     */
    Eigen::Matrix3d &getHomography() { return this->homography; }

    /**
     * @brief set flags for the pnp calculations see util/pnp_solver.h for flags
     */
    void setPnPFlags(int flags) { this->pnpFlags = flags; }
    int getPnPFlags() { return this->pnpFlags; }

	/*parameters for corner thresholds*/

    /**
     * get/set the angle threshold as cos(angle). During corner search the edgeline should be parallel
     * with the line joining module centers.  The angle between the edgeline normal and the line joining
     * module centers is used. For example if 30degree difference is allowed, that it should be cos(90-30)
     */
	float getCornerAngleThreshold() { return this->cornerAngleThreshold; }
	void setCornerAngleThreshold(float threshold) { this->cornerAngleThreshold = threshold; }

    /**
     * The minimum intensity difference between neighbouring module centers
     */
	int getCornerIntensityThreshold() { return this->cornerIntensityThreshold; }
	void setCornerIntensityThreshold(int threshold) { this->cornerIntensityThreshold = threshold; }

    /**
     * A conservative threshold - during binary searching between two modules for the edge, the edge's sobel response
     * should not be larger than this.
     */
	void setCornerBinSearchDotThreshold(int threshold) { this->binsearchDotThreshold = threshold; }

private:

    template <class fptype>
    void showCorners(std::vector<unsigned char> &corners, std::vector< Eigen::Matrix<fptype, 2, 1> > &extractionPoints, unsigned short subW, unsigned short subH);
    void showCorrespondences(bool refine = false);
    void showBox();

    template<class T, class fptype>
    bool matchPoints(Image<T, NCHAN> *img,
                     Location &loc,
                     std::vector< Eigen::Matrix<fptype, 2, 1> > &extractionPoints,
                     std::vector<unsigned char> &corners,
                     unsigned short subW, unsigned short subH);

    template<class T>
    int getNewPositions(Image<T, NCHAN> *img,
                        const Eigen::MatrixXi &mask);

    void filterCorrespondences(ImageGray* mask);


    CorrespondenceSet correspondences;
    Marker<NCHAN> *marker;
	MaskTracker maskTracker;
    bool useCornerSearch;
	bool useSubPixel;

	float reprFilterTileSizeCutoffMin;
	float reprFilterTileSizeCutoffMax;
	float movementTrackMax;
	float cornerAngleThreshold;
	float binsearchDotThreshold;
	int cornerIntensityThreshold;

    int pnpFlags;
    PnPSolver *pnp;
    Eigen::Matrix3d homography;
};

}

#endif // MODEL_H
