#ifndef __UMF_GRID_DETECTOR_H
#define __UMF_GRID_DETECTOR_H

#include "edgel_detector.h"
#include <vector>
#include <Eigen/Core>

namespace umf
{

class UMF GridDetector
{
public:
    GridDetector();

    /**
     * @brief Set the size of the histogram used to group lines for vanishing point detection
     */
    void setHistogramSize(int size) { this->histogramSize = size; this->directionCounts.resize(size); this->directions.resize(size);}
    int getHistogramSize() { return this->histogramSize; }

    /**
     * @brief setTransformScale
     * @param scale set the scaling of the line c parameter (the offset from origin)
     */
    void setTransformScale(float scale) { this->transformScale = scale; }
    float getTransformScale(){ return this->transformScale; }

    /**
     * @brief sets the number of iterations used when finding the vanishing point
     * @param iterations number of iterations of choosing two vectors
     */
    void setRANSACIterations(int iterations) { this->ransacIterations = iterations; }
    int getRANSACIterations() { return this->ransacIterations; }

    /**
     * @brief sets the threshold between the normalized vanishing point and normalized lines
     * @param threshold
     */
    void setRANSACDotThreshold( float threshold ) { this->ransacDotThreshold = threshold; }
    float getRANSACDotThreshold() {return this->ransacDotThreshold; }

    /**
     * @brief setEigenSoundnessThreshold threshold for eigen vector decomposition
     * @param threshold
     * The soundness for the eigen decomposition (used for getting the normal of the hyperplane)
     * is computed as soundness = smallest_eigenvalue / next_eigenvalue
     * The smaller is the soundness the better.
     */
    void setEigenSoundnessThreshold(float threshold) { this->vanishingSoundnessThreshold = threshold;}
    float getEigenSoundnessThreshold() { return this->vanishingSoundnessThreshold; }

    /**
     * @brief setTransformCenter sets the [0,0] after transformation
     * @param center the point in image that becomes the origin in our transformed system
     */
    void setTransformCenter(Eigen::Vector2i center) { this->transformCenter = center; }
    Eigen::Vector2i &getTransformCenter() { return this->transformCenter; }

    /**
     * @brief setReplaceEdgels whether to replace edgels by connecting vanishing point and edge points
     * @param replace flag
     */
    void setReplaceEdgels(bool replace = true) { this->replaceEdgels = replace; }
    bool getReplaceEdgels() { return this->replaceEdgels; }


    /**
     * @brief Set the number of lines used as marker grid hypothesis around the 0th line.
     * This basically defines how large a region should be extracted to identify the marker location
     */
    void setGenerateLineCount(int count) { this->generateLineCountHalf = count/2;}
    int getGenerateLineCount() { return 2*this->generateLineCountHalf;}

    /**
     * @brief set the threshold for the number of lines the mean-shift based method is used
     * @param threshold
     *
     * There are two possible methods for estimating the step between cluster of lines in
     * the grid. One is a simple median distance based and the second one is using mean-shift
     * of the sampled invert k's. The latter one should be more robust against noise in the image
     * the first one should be better for small number of precise lines
     */
    void setEstimateClusterThreshold(unsigned int threshold) { this->estimateClusterThreshold = threshold; }
    unsigned int getEstimateClusterThreshold() { return this->estimateClusterThreshold; }

    /**
     * @brief sets the minimum distance between clusters expressed by k parameter
     * @param threshold
     *
     * If the k parameter is calculated for each line separately with the index set to 1:
     * l_i = k*l_0 + h
     * The the inverse of the 1/k will be linear, so we can use clustering linearly and
     * then linear regression. Default value for the threshold is 5e-2 (worked so far)
     */
    void setInvKStepThreshold(float threshold) { this->invKStepThreshold = threshold; }
    float getInvKStepThreshold() { return this->invKStepThreshold; }

    bool detect(std::vector<Edgel> &edgels, bool show = false);
    bool detectIndexed(std::vector<Edgel> &edgels, std::vector<int> &indexed, bool show = false);

    /**
     * @brief getPencils get pencils
     * @param index 0 or 1
     * @return reference to the pencil of lines
     */
    std::vector<Eigen::Vector3f>& getPencil(int index) { return this->pencils[index]; }

    void transformEdgels();
    void transformEdgelsBack();

    void transformPencils();
    void transformPencilsBack();

    /**
     * @brief Generate a fan of lines in each directionCounts
     * @param indexOffset By this amount should be the lines shifted (if you want module centers, should be 0.5)
     */
    bool generatePencils(float extraIndexOffset = 0.0f);

private:

    void showGroups(bool filtered = false);
    void showPencils();

    void transformLine(Eigen::Vector3f &line);
    void transformEdgel(Edgel &edgel);

    void transformLineBack(Eigen::Vector3f &line);
    void transformEdgelBack(Edgel &edgel);

    void copyEdgels2Pencils();

    //private parts of the detection of the grid

    bool separateTwoGroups(std::vector<Edgel> &edgels, bool show = false);
    bool separateTwoGroupsIndexed(std::vector<Edgel> &edgels, std::vector<int> &indexed, bool show = false);

    /**
     * @brief normalizeEdgels normalize all edgel lines as vec3
     * This is required by findVanishing, where we need normalized vectors
     */
    void normalizeEdgels();

    bool findVanish(std::vector<Edgel> &group, Eigen::Vector3f &vanishing, bool replace = false);

    bool detectMesh();
    bool detectPencil(Eigen::Vector3f &vanishing, std::vector<Eigen::Vector3f> &pencil,
                      Eigen::Vector3f &line0, float &indexOffset, float &parameterK);



    std::vector<Edgel> groups[2];
    Eigen::Vector3f vanishing[2];
    Eigen::Vector3f horizon;
    std::vector<Eigen::Vector3f> pencils[2];

    std::vector< std::vector< std::vector<Edgel>::iterator > > directions;
    std::vector<int> directionCounts;

    bool replaceEdgels;
    unsigned int replaceEdgelsThreshold;
    int histogramSize;
    int ransacIterations;
    float ransacDotThreshold;
    float vanishingSoundnessThreshold;
    float transformScale;
    unsigned int estimateClusterThreshold;
    float invKStepThreshold;
    int generateLineCountHalf;
    Eigen::Vector2i transformCenter;

    //parameters needed to generate lines
    Eigen::Vector3f line0[2];
    float indexOffset[2];
    float parameterK[2];
};

}

#endif // GRID_DETECTOR_H
