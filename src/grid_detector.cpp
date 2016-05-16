#include "grid_detector.h"
#include "defines.h"
#include "util/draw.h"
#include "util/umfdebug.h"
#include "util/grid_util.h"
#include "util/prosac.h"
#include <iostream>
#include <algorithm>
#include <Eigen/Eigenvalues>

namespace umf {

GridDetector::GridDetector()
{
    this->vanishing[0] = Eigen::Vector3f(0, 0, 0);
    this->vanishing[1] = Eigen::Vector3f(0, 0, 0);
    this->horizon = Eigen::Vector3f(0, 0, 0);
    this->pencils[0].clear();
    this->pencils[1].clear();
    this->groups[0].clear();
    this->groups[1].clear();
    this->histogramSize = 12;
    this->transformScale = 1.0f/240;
    this->transformCenter = Eigen::Vector2i(320, 240);
    this->generateLineCountHalf = 5;

    this->ransacIterations = 100;
    this->ransacDotThreshold = 5e-2f;
    this->vanishingSoundnessThreshold = 0.2f;
    this->replaceEdgels = true;
    this->replaceEdgelsThreshold = 1;
    this->estimateClusterThreshold = 50; //if replaced edgels this means 25 lines
    this->invKStepThreshold = 0.05f; //this is just a rough estimate

    this->directionCounts.resize(this->histogramSize);
    this->directions.resize(this->histogramSize);
}

bool GridDetector::detect(std::vector<Edgel> &edgels, bool show)
{
    bool success = false;

    const bool showTwoGroups = show && false;
    const bool showFilteredLines = show && false;
    const bool showGrid = show && true;

    ////////////////////////////////////////////////////////////////////////////////
    //GROUP
    success = this->separateTwoGroups(edgels, showTwoGroups);

    if(!success)
    {
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////////
    //VANISH
    //finding vanishing point requires the group to be first transformed and normalized
    this->transformEdgels();
    this->normalizeEdgels();

    success = this->findVanish(this->groups[0], this->vanishing[0], showFilteredLines)
            && this->findVanish(this->groups[1], this->vanishing[1], showFilteredLines);


    if(!success)
    {
        return false;
    }

    //extract the lines from edgels
    this->copyEdgels2Pencils();
    //now we can do anything we want with the groups. they are not needed any more

#ifdef UMF_DEBUG_DRAW
    if(showFilteredLines)
    {
        this->transformEdgelsBack();
        this->showGroups(true);
    }
#endif

    ////////////////////////////////////////////////////////////////////////////////
    //GRID
    success = this->detectMesh();
    //something went wrong with the grid detection
    if(!success)
    {
        return false;
    }

#ifdef UMF_DEBUG_DRAW
    if(showGrid)
    {
        //generate pencils going through the corners
        bool p = this->generatePencils(0.5);
        if(p)
        {
            this->transformPencilsBack();
            this->showPencils();
        }
    }
#endif

    //everything is fine, generate our pencils of lines
    success = this->generatePencils();

    if(success)
    {
        //and finally transform it back to normal positions
        this->transformPencilsBack();
    }

    return success;
}

bool GridDetector::detectIndexed(std::vector<Edgel> &edgels, std::vector<int> &indexed, bool show)
{
    //if this is going to be used most of the time, then this should be rewritten
    //so we do not create a subvector each time
    std::vector<Edgel> subPart(indexed.size());
    for(unsigned int i = 0; i < indexed.size(); i++)
    {
        subPart[i] = edgels[indexed[i]];
    }

    return this->detect(subPart, show);
}


inline int getHistogramIndex(float b, float a, const int HIST_SIZE)
{
    double angle = std::atan2(b, a)/M_PI + 1.0001;
    //std::cout << "Angle: " << (angle - 1.0001)*180;
    return ((int)std::floor(angle*HIST_SIZE))%HIST_SIZE;
}

void getMainTwoBins(std::vector<int> &origHistogram, int maxBins[2][3])
{

    const int HIST_SIZE = origHistogram.size();
    std::vector<int> histogram(HIST_SIZE);
    //smooth over
    for(int i = 0; i < HIST_SIZE; i++)
    {
        histogram[i] = origHistogram[(i - 1 + HIST_SIZE)%HIST_SIZE] + origHistogram[i] + origHistogram[(i + 1)%HIST_SIZE];
    }

    maxBins[0][1] = std::max_element(histogram.begin(), histogram.end()) - histogram.begin();
    //left neighborhood
    maxBins[0][0] = (maxBins[0][1] + (HIST_SIZE-1))%HIST_SIZE;
    //right neighborhood
    maxBins[0][2] = (maxBins[0][1] + 1)%HIST_SIZE;

    //zero bin and neighborhood bins
    origHistogram[maxBins[0][0]] = 0;
    origHistogram[maxBins[0][1]] = 0;
    origHistogram[maxBins[0][2]] = 0;
    
    for(int i = 0; i < HIST_SIZE; i++)
    {
        histogram[i] = origHistogram[(i - 1 + HIST_SIZE)%HIST_SIZE] + origHistogram[i] + origHistogram[(i + 1)%HIST_SIZE];
    }

    //same for second max bin + neighborhood
    maxBins[1][1] = std::max_element(histogram.begin(), histogram.end()) - histogram.begin();
    maxBins[1][0] = (maxBins[1][1] + (HIST_SIZE-1))%HIST_SIZE;
    maxBins[1][2] = (maxBins[1][1] + 1)%HIST_SIZE;
}

template<class T>
inline T circ_diff(T a, T b, T period)
{
    return (a > b) ? (std::min)( a - b, b + period - a) : (std::min)( b - a, a + period - b);
}

float vonMises(float mean, float variance, float period, float sample)
{
    //we don't need normalization here
    return expf(cosf(circ_diff(sample, mean, period)*static_cast<float>(M_PI)/period)/variance);
}


void getMainTwoBinsKMeans(std::vector<int> &origHistogram, int maxBins[2][3])
{
    const int HIST_SIZE = origHistogram.size();

    //detect one maximum
    float mean1 = static_cast<float>(std::max_element(origHistogram.begin(), origHistogram.end()) - origHistogram.begin());
    float mean2 = static_cast<float>((static_cast<int>(mean1) + HIST_SIZE/2)%HIST_SIZE);
    
    std::vector<unsigned char> responsible(HIST_SIZE, 0);

    const int KM_ITER = 4;
    const float ALLOWED_DIFF = HIST_SIZE/6.0f + 1e-2f;

    for(int emi = 0; emi < KM_ITER; emi++)
    {   
        //******************************************
        //Assignment
        for(int i = 0; i < HIST_SIZE; i++)
        {
            float diff1 = circ_diff<float>(mean1, static_cast<float>(i), static_cast<float>(HIST_SIZE));
            float diff2 = circ_diff<float>(mean2, static_cast<float>(i), static_cast<float>(HIST_SIZE));
            if(diff1 > diff2)
            {
                responsible[i] = (diff2 < ALLOWED_DIFF) ? 2 : 0;
            } else {
                responsible[i] = (diff1 < ALLOWED_DIFF) ? 1 : 0;
            }
        }

        //**********************************************
        //update
        float sin1sum = 0, sin2sum = 0;
        float cos1sum = 0, cos2sum = 0;
        for(int i = 0; i < HIST_SIZE; i++)
        {
            float sini = sinf(2*static_cast<float>(M_PI)*i/HIST_SIZE);
            float cosi = cosf(2*static_cast<float>(M_PI)*i/HIST_SIZE);
            sin1sum += (responsible[i] & 1)*origHistogram[i]*sini;
            cos1sum += (responsible[i] & 1)*origHistogram[i]*cosi;
            
            sin2sum += ((responsible[i] & 2) >> 1)*origHistogram[i]*sini;
            cos2sum += ((responsible[i] & 2) >> 1)*origHistogram[i]*cosi;
        }
        mean1 = atan2(sin1sum, cos1sum)*0.5f/static_cast<float>(M_PI);
        mean2 = atan2(sin2sum, cos2sum)*0.5f/static_cast<float>(M_PI);

        mean1 = HIST_SIZE*((mean1 < 0) ? (1.0f + mean1) : mean1);
        mean2 = HIST_SIZE*((mean2 < 0) ? (1.0f + mean2) : mean2);

    }

    maxBins[0][1] = static_cast<int>(mean1) % HIST_SIZE;
    //left neighborhood
    maxBins[0][0] = (maxBins[0][1] + (HIST_SIZE-1))%HIST_SIZE;
    //right neighborhood
    maxBins[0][2] = (maxBins[0][1] + 1)%HIST_SIZE;

    maxBins[1][1] = static_cast<int>(mean2) % HIST_SIZE;
    maxBins[1][0] = (maxBins[1][1] + (HIST_SIZE-1))%HIST_SIZE;
    maxBins[1][2] = (maxBins[1][1] + 1)%HIST_SIZE;
}

/**
 * @brief separateTwoGroups separate the edgels into two groups based on a rough histogram
 * @param edgels all the edgels
 * @param show Optionally show the results in the debug output if debugging is enabled
 * @return the success (one or both directions contain zero elements
 *
 * The image below demonstrates the two groups for the edgels extracted by \link EdgelDetector::findEdgels \endlink
 * \image html 4_groups.png
 *
 * The algorithm takes all the lines and creates a rough histogram with low number of bins (default 12 o 18)
 * (set using \link setHistogramSize \endlink). In this histogram two main groups are detected each of width 3.
 * If the groups are overlapping the function will return false.
 * The corresponding edgels are then stored into two groups. In case one the groups is empty, the function again returns false.
 */
bool GridDetector::separateTwoGroups(std::vector<Edgel> &edgels, bool show)
{

    this->groups[0].clear();
    this->groups[1].clear();

    for(int i = 0; i < this->histogramSize; i++)
    {
        this->directionCounts[i] = 0;
        this->directions[i].clear();
    }

    //create set of lines for the four directions
    std::vector< std::vector< std::vector<Edgel>::iterator > > directions(this->histogramSize);
    std::vector<int> directionCounts(this->histogramSize, 0);

    for(std::vector<Edgel>::iterator edgelIt = edgels.begin(); edgelIt != edgels.end(); edgelIt++)
    {

        int hist_index = getHistogramIndex(edgelIt->line[1], edgelIt->line[0], this->histogramSize);
        //std::cout << " Hi: " << hist_index << std::endl;

        directions[hist_index].push_back(edgelIt);
        directionCounts[hist_index]++;
    }

    int maxBins[2][3];
    getMainTwoBinsKMeans(directionCounts, maxBins);
    //getMainTwoBins(directionCounts, maxBins);

    //bins are too close to each other
    if(circ_diff(maxBins[0][1], maxBins[1][1], this->histogramSize) < this->histogramSize/4)
    {
        return false;
    }

    //group1
    this->groups[0].clear();
    this->groups[0].reserve(directions[maxBins[0][0]].size() +  directions[maxBins[0][1]].size() + directions[maxBins[0][1]].size());

    for(int i = 0; i < 3; i++)
    {
        for(std::vector< std::vector<Edgel>::iterator >::iterator dit = directions[maxBins[0][i]].begin();
            dit != directions[maxBins[0][i]].end(); dit++)
        {
            this->groups[0].push_back(**dit);
        }
    }

    //group2
    this->groups[1].clear();
    this->groups[1].reserve(directions[maxBins[1][0]].size() +  directions[maxBins[1][1]].size() + directions[maxBins[1][1]].size());
    for(int i = 0; i < 3; i++)
    {
        for(std::vector< std::vector<Edgel>::iterator >::iterator dit = directions[maxBins[1][i]].begin();
            dit != directions[maxBins[1][i]].end(); dit++)
        {
            this->groups[1].push_back(**dit);
        }
    }


#ifdef UMF_DEBUG_DRAW
    if(show)
    {
        this->showGroups();
    }
#endif

    return !(this->groups[0].empty() || this->groups[1].empty());
}


bool sortByScore(const Edgel &e1, const Edgel &e2)
{
    return e1.score > e2.score;
}


class ProsacModelVanishing: public ProsacModel
{
public:
    ProsacModelVanishing(std::vector<Edgel> *edgels, float soundnessThreshold = 0.1f, float dotThreshold = 5e-2): 
        edgels(edgels), soundnessThreshold(soundnessThreshold), dotThreshold(dotThreshold)
    {
        //we need to sort our lines based on score to get better results
        std::sort(edgels->begin(), edgels->end(), sortByScore);
    }

    virtual unsigned int findSupport(std::vector<int> &samples, std::vector<bool> &inliers)
    {
        
        Eigen::Vector3f ivanishing;
        //do eigen decomposition to get vanishing point in case we compute from more than 2 
        if(samples.size() > 2)
        {
            covMat.setZero();
            for(std::vector<int>::iterator it = samples.begin(); it != samples.end(); it++)
            {
                covMat += ((*edgels)[*it].line * (*edgels)[*it].line.adjoint()) * (*edgels)[*it].score;
            }
            eig.compute(covMat);

            //compute the soundness -> lines not too close to each other
            float soundness = eig.eigenvalues().coeff(0)/eig.eigenvalues().coeff(1);
            if(soundness > soundnessThreshold)
            {
                return 0;
            }

            //get vanishing point
            ivanishing = eig.eigenvectors().col(0);
        } else {
            ivanishing = (*edgels)[samples[0]].line.cross((*edgels)[samples[1]].line);
        }
        
        std::fill(inliers.begin(), inliers.end(), true);

        //Eigen::Vector3f ivanishing = (*edgels)[samples[0]].line.cross((*edgels)[samples[1]].line);
        //ivanishing.normalize();

        //std::cout << "Soundness: " << soundness << " vanishing: " << ivanishing.transpose() << std::endl;

        int inlierCount = 0;
        unsigned int edgelCount = (*(this->edgels)).size();
        //get number of inliers
        for(unsigned int i = 0; i < edgelCount; i++)
        {
            Edgel &ecurr = (*(this->edgels))[i];
            Eigen::Vector3f p1 = ivanishing.cross(Eigen::Vector3f(ecurr.endPoints[0][0], ecurr.endPoints[0][1], 1.f));
            Eigen::Vector2f p1norm(-p1[1], p1[0]);
            p1norm.normalize();
            float dp = std::abs(p1norm.dot(ecurr.normal));
            //float dp = fabs(ivanishing.dot(ecurr.line));
            if(dp < this->dotThreshold)
            {
                inlierCount++;
                //inliers[i] = true;
            } else {
                inliers[i] = false;
            }
        }

        return inlierCount;
        
    }
    virtual unsigned int getDataCount()
    {
        return this->edgels->size();
    }
private:
    std::vector<Edgel> *edgels;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig;
    Eigen::Matrix3f covMat;
    float soundnessThreshold;
    float dotThreshold;
};

/**
 * @brief Finds a vanishing point to the group of edgels
 * @param group groups of edgels for which the vanishing point is found. The lines in the edgels should be normalized first (as vec3s, not just the normal).
 * @param vanishing the found vanishing point
 * @param replace if the function should replace the group with the filtered lines
 * @return if the vanishing point could be found
 *
 * This function uses ransac to filter out outliers in the group using \link setRANSACIterations \endlink
 * iterations and the threshold for the dot product between the lines and the vanishing point (the closer to 0 the more precise)
 * is set by \link setRANSACDotThreshold \endlink.
 *
 * The RANSAC is done like this:
 *  -# choose randomly two line
 *  -# get the vanishing point by crossing the two lines and normalize it
 *  -# loop through all lines
 *      -# error = dot(line, vanishing)
 *      -# if ( error < thrshold ) accumulate error, and inliers
 *      .
 *  -# compute the score for the group of lines: score = inliers*(threshold - avg_error)*100
 *  -# if so far best store it
 *
 * After RANSAC the best subgroup is used to compute a covariance matrix with mean at the origin
 * ( we are searching for a hyperplane going through the origin). Eigen decomposition is done
 * using eigen - closed form for our 3x3 covariance matrix. After that the smallest eigenvector is
 * chosen. To filter out bad decompositions the \link setEigenSoundnessThreshold \endlink soundness threshold
 * is used (details see that function).
 */
bool GridDetector::findVanish(std::vector<Edgel> &group, Eigen::Vector3f &vanishing, bool replace)
{
    bool FILTER_PROSAC = true;

    vanishing = Eigen::Vector3f(0, 0, 0);

    int maxLineCount = group.size();
    typedef std::vector< std::vector<Edgel>::iterator > EdgelItList;
    EdgelItList bestSolution;
    int bestSolutionInliers = -1;

    //too few lines
    if(maxLineCount < 2)
    {
        return false;
    }

    
    if(FILTER_PROSAC)
    {
        const int PROSAC_SAMPLE_COUNT = 2;
        const float PROSAC_DOT_THRESHOLD = cosf(static_cast<float>(M_PI/2 - M_PI/64));
        const float PROSAC_SOUNDNESS = 1e-1f;
        srand(1);
        if(group.size() > PROSAC_SAMPLE_COUNT + 1)
        {
            ProsacModelVanishing prosModel(&group, PROSAC_SOUNDNESS, PROSAC_DOT_THRESHOLD);
            Prosac pros(PROSAC_SAMPLE_COUNT, 0.8f, 0.99f, 0.4f);
            std::vector<bool> inliers(group.size(), false);
            bestSolutionInliers = pros.run(&prosModel, inliers);
            bestSolution.resize(bestSolutionInliers);
            int inct = 0;
            for(unsigned int it = 0; it < group.size(); it++)
            {
                if(inliers[it])
                {
                    bestSolution[inct] = group.begin() + it;
                    ++inct;
                }
            }
            bestSolutionInliers = inct;
            bestSolution.resize(inct);
        }
    } else {
        
        double bestSolutionScore = -1;
        
        //RANSAC
        srand(1);
        for(int iteration = 0; iteration < this->ransacIterations; iteration++)
        {
            //choose two lines at random

            Eigen::Vector3f &line0 = group[rand()%maxLineCount].line;
            Eigen::Vector3f &line1 = group[rand()%maxLineCount].line;

            //get vanishing point
            Eigen::Vector3f ivanishing = line0.cross(line1);
            ivanishing.normalize();

            int inliers = 0;
            float inlierErrorSum = 0;

            EdgelItList currentLines;

            for(std::vector<Edgel>::iterator edgelIt = group.begin(); edgelIt != group.end(); edgelIt++)
            {
                Eigen::Vector3f &edgel = edgelIt->line;

                float dp = std::abs(edgel.dot(ivanishing));
                if(dp < this->ransacDotThreshold)
                {
                    inliers++;
                    inlierErrorSum += dp;
                    currentLines.push_back(edgelIt);
                }
            }

            //funny scoring somehow balance between the number of outliers
            //and the error these lines represen
            double score = inliers*100.0*(this->ransacDotThreshold - inlierErrorSum/inliers);

            //if better than current
            if(score > bestSolutionScore)
            {
                bestSolutionInliers = inliers;
                bestSolutionScore = score;
                //store them as results
                bestSolution = currentLines;
            }
        }

        if(this->ransacIterations < 0)
        {
            for(std::vector<Edgel>::iterator edgelIt = group.begin(); edgelIt != group.end(); edgelIt++)
            {
                bestSolutionInliers++;
                bestSolution.push_back(edgelIt);
            }
        }
    }

    if(bestSolutionInliers < 2)
    {
        if(replace)
        {
            group.clear();
        }
        return false;
    }

    // compute the covariance matrix - the mean is at zero
    // and we have real number so it's pretty simple
    // possible optimization is possible if we don't sum matrices,
    // but do a loop 3x3 and do a loop over all lines there
    //for now enough this way - less issues
    //scale with the length of the edgel
    Eigen::Matrix3f covMat;
    covMat.setZero();
    for(EdgelItList::iterator it = bestSolution.begin(); it != bestSolution.end(); it++)
    {
        covMat += ((**it).line * (**it).line.adjoint()) * (**it).score;
    }

    // now we just have to pick the eigen vector with smallest eigen value
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(covMat);
    //the eigenvectors are sorted in increasing order
    vanishing = eig.eigenvectors().col(0);
    //the soundness of the eigen decomposition - the smaller the better
    float soundness = eig.eigenvalues().coeff(0)/eig.eigenvalues().coeff(1);

    //if we want to replace the group with the best solution from ransac
    if(replace)
    {
        std::vector<Edgel> result;

        for(EdgelItList::iterator it = bestSolution.begin(); it != bestSolution.end(); it++)
        {
            result.push_back(**it);
        }
        group = result;
    }

    if(soundness > this->vanishingSoundnessThreshold)
    {
        return false;
    }

    return true;
}

/**
 * @brief detectMesh Computes the horizon and cals \link detectPencil \endlink for both group of lines
 * @return the success of detecting the pencils
 *
 * The result for our example after group separation \link separateTwoGroups \endlink,
 * vanishing point detection (and also ransac filtering) by \link findVanish \endlink and replacing lines by optionally connecting
 * with the vanishing point \link copyEdgels2Pencils \endlink the result is shown below:
 *
 *  \image html 5_mesh.png "scanline every 200 pixels"
 */
bool GridDetector::detectMesh()
{
    this->horizon = this->vanishing[0].cross(this->vanishing[1]);
    this->horizon.normalize();

    bool success = this->detectPencil(this->vanishing[0], this->pencils[0], this->line0[0], this->indexOffset[0], this->parameterK[0]);

    if(!success)
    {
        return false;
    }

    //problematic passing array elements, if they are not objects
    success = this->detectPencil(this->vanishing[1], this->pencils[1], this->line0[1], this->indexOffset[1], this->parameterK[1]);

    return success;
}

/**
 * @brief detectPencil detect a pencil of lines
 * @param vanishing the theoretical vanishing for the pencil of lines
 * @param pencil the cluster of lines used to detect the pencil
 * @param[out] line0 The line0, relative to which the parameters are calculated is stored here
 * @param[out] indexOffset The index offset of the grid relative to line0 is set if successful
 * @param[out] parameterK The k parameter is stored here
 * @return whether the pencil was successfully detected
 *
 * The equation again which is used to determine the three parameters:
 * l_i = line_0 * k + (i + indexOffset) * h;
 * where l_i are the lines of the grid.
 *
 * This is where the magic happens.
 *  -# choose the line0 as the center of the coordinate system ( [0;0] this corresponds to the transform center of the lines )
 *  -# for each line calculate the k parameter and store it's inverse (the inverse should be linear)
 *  -# somehow estimate the step (see \link  setEstimateClusterThreshold \endlink )
 *      -# option 1 - use simple differences between and take the median
 *      -# option 2 - use sampling and meanshift of the differences - bit strange, but works well
 *      .
 *  -# the estimating algorithm with the estimated step also returns a starting position
 *  -# use this starting position as seed
 *  -# find all lines with corresponding 1/k with threshold (0.5/estimatedK) from the seed
 *  -# set the seed += estimatedK and repeat the last two steps
 *  -# there is a limit for the clusters (currently MAX_CLUSTER_COUNT = 40) - TODO probably should parametrize this too
 *  -# the previous steps create a mapping between cluster indexes and ks
 *  -# run linear regression on these points - returns the slope and offset of the line
 *  -# the slope of the line is the parameterK we are searching for
 *  -# the offset + round(the average index) is the offset of our middle line for generation
 *  -# add 0.5 to the offset so we have the location for the field centers in the marker
 */
bool GridDetector::detectPencil(Eigen::Vector3f &vanishing, std::vector<Eigen::Vector3f> &pencil,
                                Eigen::Vector3f &line0, float &indexOffset, float &parameterK)
{
    if(pencil.empty())
    {
        return false;
    }

    //consider that everythin is normalized, and the normal center is 0, 0
    Eigen::Vector3f ecenter(0.f, 0.0f, 1.0f);

    //now for each line we can calculate it's k based on on the line connecting the reference line and the center
    line0 = ecenter.cross(vanishing);
    line0.normalize();

    //other calculations using inverse product
    Eigen::Vector3f ln = line0.cross(horizon);
    float nsqrinv = 1.0f/ln.dot(ln);

    Eigen::Vector3f l0q = horizon.cross(ln)*nsqrinv;
    Eigen::Vector3f lhq = ln.cross(line0)*nsqrinv;

    //now create a list of all inverted k-s and store it somewhere
    std::vector<float> invks;
    invks.reserve(pencil.size());
    for(unsigned int i = 0; i != pencil.size(); i++)
    {
        float invk = pencil[i].dot(lhq)/pencil[i].dot(l0q);
        invks.push_back(invk);
    }
    /********************************************************************/
    /********************************************************************/

    //now sort all k inverts-s
    std::sort(invks.begin(), invks.end());

    //////////////////////////////////////////////////////////////////////////////
    //estimate the step between clusters of lines

    float estimatedStep = 0;
    float startOffset = 0;
    bool success = false;
    if(invks.size() > this->estimateClusterThreshold)
    {
        success = estimateKCluster(invks, this->invKStepThreshold, estimatedStep, startOffset);
    } else {
        success = estimateKSimple(invks, this->invKStepThreshold,  estimatedStep, startOffset);
    }

    if(!success)
    {
        pencil.clear();
        return false;
    }


    ///////////////////////////////////////////////////////////////////////////////////
    //create relation mapping ks to indexes

    float clusterDiffThreshold = estimatedStep/2;

    //now create groups and assign them indexes

    const int MAX_CLUSTER_COUNT = 40;
    const int MAX_CLUSTER_COUNT_HALF = MAX_CLUSTER_COUNT/2;

    std::vector<float> G0;
    getKGroupSorted(invks, G0, startOffset, clusterDiffThreshold);
    if(G0.empty())
    {
        //this probably means to large deviation was found, that we are unable to detect
        pencil.clear();
        return false;
    }
    //assign index weights this will be important later when we try to find the
    //mean for the indexes, so we generate lines close to the mean

    std::vector<float> indexWeights(MAX_CLUSTER_COUNT, 0);
    //median
    float g0 = G0[G0.size()/2];

    //we must use pointers, otherwise it's problematic being eigen fixed size arrays
    std::vector<Eigen::Vector2f> indexes;
    addClusterMapping(indexes, G0, 0);

    //now that we have a starting position, just move along to the next and create our
    //function for linear regression

    float nextStart = g0 + estimatedStep;
    float maxKinv = invks.back();
    for(int index = 1; index < MAX_CLUSTER_COUNT_HALF && nextStart < maxKinv; index++)
    {
        std::vector<float> Gi;
        getKGroupSorted(invks, Gi, nextStart, clusterDiffThreshold);
        indexWeights[MAX_CLUSTER_COUNT_HALF +index] = static_cast<float>(Gi.size());
        if(Gi.empty())
        {
            //we skipped one probably
            nextStart += estimatedStep;
        } else {
            //everything is fine, calculate the next start by getting the mean of the current, plus the diffThreshold
            float gi = Gi[Gi.size()/2];
            addClusterMapping(indexes, Gi, index);
            nextStart = gi + estimatedStep;
        }
    }

    nextStart = g0 - estimatedStep;
    float minKinv = invks.front();
    for(int index = -1; index > -MAX_CLUSTER_COUNT_HALF && nextStart > minKinv; index--)
    {
        std::vector<float> Gi;
        getKGroupSorted(invks, Gi, nextStart, clusterDiffThreshold);
        indexWeights[MAX_CLUSTER_COUNT_HALF +index] = static_cast<float>(Gi.size());
        if(Gi.empty())
        {
            //we skipped one probably
            nextStart -= estimatedStep;
        } else {
            //everything is fine, calculate the next start by getting the mean of the current, plus the diffThreshold
            float gi = Gi[Gi.size()/2];
            addClusterMapping(indexes, Gi, index);
            nextStart = gi - estimatedStep;
        }
    }
    /////////////////////////////////////////////////////////////////////////////////
    // use linear regression to get  coefficients

    //now we should have a nice mapping for each k to its indexes

    Eigen::Vector3f line = fitLine(indexes);

    Eigen::Vector2f coeffs;
    coeffs[0] = - line[0]/line[1];
    coeffs[1] = - line[2]/line[1];

    //great, we have our line matching best our indexes based on k
    /*****************************************************************************/
    /****************************************************************************/
    //first get k
    parameterK = coeffs[0]; //actually this gives us our k - how beautiful :)

    //now get index offset
    //simple least squares with weighted clusters
    float avgIndexSum = 0;
    float weightSum = 0;
    for(int i = 1; i < MAX_CLUSTER_COUNT; i++)
    {
        avgIndexSum += i*indexWeights[i];
        weightSum += indexWeights[i];
    }
    //round the average
    float avgIndex = floor(avgIndexSum/weightSum - MAX_CLUSTER_COUNT_HALF + 0.5f);
    /* - offset of the zero's cluster + offset of the mean + offset of the field centers*/
    indexOffset = - coeffs[1] + avgIndex + 0.5f;

    return true;
}

/**
 * @brief Generate pencils of lines based on the stored parameters
 * @param extraIndexOffset extra offset
 *
 * The is that indexOffset goes through the center of the fields. If somebody want lines going through
 * the corners, the extra offset should be set to 0.5.
 * First the function clears the stored pencils and replaces them with our lines
 *
 * The function also test if the grid is a correct perspectively deformed rectangle by using
 * the dot product between the first and last lines in the pencils (if for some reason
 * the dot product is negative, it means the line direction suddenly got inverted -> wrong
 * vanishing point)
 */
bool GridDetector::generatePencils(float extraIndexOffset)
{
    bool success = true;
    for(int pi = 0; pi < 2; pi++)
    {
        this->pencils[pi].clear();

        for(int i = -this->generateLineCountHalf; i < this->generateLineCountHalf; i++)
        {
            //if(i == 0) continue;
            Eigen::Vector3f newline = this->line0[pi]*this->parameterK[pi] + (i + this->indexOffset[pi] + extraIndexOffset)*this->horizon;
            newline.normalize();
            //std::cout << "Line " << i << ": " << newline.a << "; " << newline.b << "; " << newline.c << std::endl;
            this->pencils[pi].push_back(newline);
        }

        success = success && (pencils[pi].front().block(0,0,2,1).dot(pencils[pi].back().block(0,0,2,1)) > 0);
    }
    return success;
}


void GridDetector::showGroups(bool filtered)
{
    UMFDebug *dbg = UMFDSingleton::Instance();
    Renderer *rend = dbg->getRenderer();

    if(rend == NULL)
    {
        return;
    }

    
    int lineWidth = 1;
    Eigen::Vector3i lineColor1(255, 98, 51);
    Eigen::Vector3i lineColor2(100, 100, 255);
    
	if(filtered) {
        lineWidth = 1;
        lineColor1 = Eigen::Vector3i(0, 40, 255);
        lineColor2 = Eigen::Vector3i(255, 40, 0);
    }


    for(std::vector<Edgel>::iterator eIt = this->groups[0].begin(); eIt != this->groups[0].end(); eIt++)
    {
        drawLineEq(rend, eIt->line, lineColor1, lineWidth);
    }

    for(std::vector<Edgel>::iterator eIt = this->groups[1].begin(); eIt != this->groups[1].end(); eIt++)
    {
        drawLineEq(rend, eIt->line, lineColor2, lineWidth);
    }

}

void GridDetector::showPencils()
{
    UMFDebug *dbg = UMFDSingleton::Instance();
    Renderer *rend = dbg->getRenderer();

    if(rend == NULL)
    {
        return;
    }


    Eigen::Vector3i lineColor1(255, 40, 0);
    //Eigen::Vector3i lineColor2(100, 100, 255);
	Eigen::Vector3i lineColor2(0, 40, 255);
    int lineWidth = 2;

    for(unsigned int i = 0; i < this->pencils[0].size(); i++)
    {
        drawLineEq(rend, this->pencils[0][i], lineColor1, lineWidth);
    }

    for(unsigned int i = 0; i < this->pencils[1].size(); i++)
    {
        drawLineEq(rend, this->pencils[1][i], lineColor2, lineWidth);
    }

}

/**
 * @brief separateTwoGroupsIndexed same as \link separateTwoGroups \endlink just with prefiltered indexes
 * @param edgels
 * @param indexed
 * @param show
 * @return success
 */
bool GridDetector::separateTwoGroupsIndexed(std::vector<Edgel> &edgels, std::vector<int> &indexed, bool show)
{
    //if this is going to be used most of the time, then this should be rewritten
    //so we do not create a subvector each time
    std::vector<Edgel> subPart(indexed.size());
    for(unsigned int i = 0; i < indexed.size(); i++)
    {
        subPart[i] = edgels[indexed[i]];
    }
    return this->separateTwoGroups(subPart, show);
}



void GridDetector::transformLine(Eigen::Vector3f &line)
{
    line[2] = (line[2] + (line[0]*this->transformCenter[0] + line[1]*this->transformCenter[1]))*this->transformScale;
}

void GridDetector::transformEdgel(Edgel &edgel)
{
    this->transformLine(edgel.line);
    edgel.endPoints[0][0] = (edgel.endPoints[0][0] - this->transformCenter[0])*this->transformScale;
    edgel.endPoints[0][1] = (edgel.endPoints[0][1] - this->transformCenter[1])*this->transformScale;
    edgel.endPoints[1][0] = (edgel.endPoints[1][0] - this->transformCenter[0])*this->transformScale;
    edgel.endPoints[1][1] = (edgel.endPoints[1][1] - this->transformCenter[1])*this->transformScale;
    edgel.score *= this->transformScale;
}

void GridDetector::transformLineBack(Eigen::Vector3f &line)
{
    float scale = 1.0f/this->transformScale;
    line[2] = line[2]*scale - line[0]*this->transformCenter[0] - line[1]*this->transformCenter[1];
}

void GridDetector::transformEdgelBack(Edgel &edgel)
{
    this->transformLineBack(edgel.line);
    float scale = 1.0f/this->transformScale;
    edgel.endPoints[0][0] = edgel.endPoints[0][0]*scale + this->transformCenter[0];
    edgel.endPoints[0][1] = edgel.endPoints[0][1]*scale + this->transformCenter[1];
    edgel.endPoints[1][0] = edgel.endPoints[1][0]*scale + this->transformCenter[0];
    edgel.endPoints[1][1] = edgel.endPoints[1][1]*scale + this->transformCenter[1];
    edgel.score *= scale;
}

void GridDetector::transformEdgels()
{
    for(std::vector<Edgel>::iterator eIt = this->groups[0].begin(); eIt != this->groups[0].end(); eIt++)
    {
        this->transformEdgel(*eIt);
    }
    for(std::vector<Edgel>::iterator eIt = this->groups[1].begin(); eIt != this->groups[1].end(); eIt++)
    {
        this->transformEdgel(*eIt);
    }
}

void GridDetector::transformEdgelsBack()
{
    for(std::vector<Edgel>::iterator eIt = this->groups[0].begin(); eIt != this->groups[0].end(); eIt++)
    {
        this->transformEdgelBack(*eIt);
    }
    for(std::vector<Edgel>::iterator eIt = this->groups[1].begin(); eIt != this->groups[1].end(); eIt++)
    {
        this->transformEdgelBack(*eIt);
    }
}

void GridDetector::transformPencils()
{
    for(unsigned int i = 0; i < this->pencils[0].size(); i++)
    {
        this->transformLine(this->pencils[0][i]);
    }

    for(unsigned int i = 0; i < this->pencils[1].size(); i++)
    {
        this->transformLine(this->pencils[0][i]);
    }
}


void GridDetector::transformPencilsBack()
{
    for(unsigned int i = 0; i < this->pencils[0].size(); i++)
    {
        this->transformLineBack(this->pencils[0][i]);
    }

    for(unsigned int i = 0; i < this->pencils[1].size(); i++)
    {
        this->transformLineBack(this->pencils[1][i]);
    }
}

void GridDetector::normalizeEdgels()
{
    for(std::vector<Edgel>::iterator eIt = this->groups[0].begin(); eIt != this->groups[0].end(); eIt++)
    {
        eIt->line.normalize();
    }
    for(std::vector<Edgel>::iterator eIt = this->groups[1].begin(); eIt != this->groups[1].end(); eIt++)
    {
        eIt->line.normalize();
    }
}

void getEdgelLines(std::vector<Edgel> &edgels, std::vector<Eigen::Vector3f> &pencil, Eigen::Vector3f &vanishing)
{
    pencil.clear();
    for(std::vector<Edgel>::iterator eIt = edgels.begin(); eIt != edgels.end(); eIt++)
    {
        Eigen::Vector3f p1 = vanishing.cross(Eigen::Vector3f(eIt->endPoints[0][0], eIt->endPoints[0][1], 1));
        Eigen::Vector3f p2 = vanishing.cross(Eigen::Vector3f(eIt->endPoints[1][0], eIt->endPoints[1][1], 1));
        p1.normalize();
        p2.normalize();
        pencil.push_back(p1);
        pencil.push_back(p2);
    }
}


/**
 * @brief copyEdgels2Pencils Extract lines from the edgels and store them in the pencils
 * They are further processed then by \link detectMesh \endlink.
 * If the \link setReplaceEdgels \endlink is set, then for each edgel two lines are added
 * by connecting the vanishnig point with the endpoints of the edgel
 */
void GridDetector::copyEdgels2Pencils()
{
    this->pencils[0].clear();
    this->pencils[1].clear();
    if(this->replaceEdgels && this->groups[0].size() < this->replaceEdgelsThreshold)
    {
        getEdgelLines(this->groups[0], this->pencils[0], this->vanishing[0]);
    } else {

        for(std::vector<Edgel>::iterator eIt = this->groups[0].begin(); eIt != this->groups[0].end(); eIt++)
        {
            Eigen::Vector3f p1 = this->vanishing[0].cross(
				Eigen::Vector3f((eIt->endPoints[0][0] + eIt->endPoints[1][0])*0.5f, (eIt->endPoints[0][1] + eIt->endPoints[1][1])*0.5f, 1.f));
            p1.normalize();
            this->pencils[0].push_back(p1);
        }
    }

    if(this->replaceEdgels && this->groups[1].size() < this->replaceEdgelsThreshold)
    {
        getEdgelLines(this->groups[1], this->pencils[1], this->vanishing[1]);
    } else {
        
        for(std::vector<Edgel>::iterator eIt = this->groups[1].begin(); eIt != this->groups[1].end(); eIt++)
        {
            Eigen::Vector3f p1 = this->vanishing[1].cross(
				Eigen::Vector3f((eIt->endPoints[0][0] + eIt->endPoints[1][0])*0.5f, (eIt->endPoints[0][1] + eIt->endPoints[1][1])*0.5f, 1.f));
            p1.normalize();
            this->pencils[1].push_back(p1);
        }
    }
}

}
