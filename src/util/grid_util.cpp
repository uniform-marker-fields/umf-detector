#include "grid_util.h"
#include <Eigen/Eigenvalues>

namespace umf {


template<class T>
bool cmpr_abs(T i, T j)
{
    return std::abs(i)<std::abs(j);
}

void getMainIndexes(Eigen::Vector3f line0, Eigen::Vector3f line1, int &index1, int &index2)
{
    //find two maximal directions
    Eigen::Vector3f templ1 = line0.cross(line1);

    float *maxval = std::max_element(&(templ1[0]), &(templ1[0])+3 , cmpr_abs<float>);

    int index3 = maxval - &(templ1[0]);
    std::vector<int> indexes(3);
    indexes[0] = 0;
    indexes[1] = 1;
    indexes[2] = 2;
    indexes.erase(indexes.begin() + index3);
    index1 = indexes[0];
    index2 = indexes[1];
}

/**
 * @brief linear regression of a 2D function
 * @param points
 * @return the line represented as ax + by + c -> Vector3f (a, b, c)
 */
Eigen::Vector3f fitLine(std::vector<Eigen::Vector2f> &points)
{
    int numPoints = points.size();

    // compute the mean of the data
    Eigen::Vector2f mean; mean.setZero();
    for(int i = 0; i < numPoints; ++i)
        mean += (points[i]);
    mean /= static_cast<float> (numPoints);

    // compute the covariance matrix
    Eigen::Matrix2f covMat; covMat.setZero();
    for(int i = 0; i < numPoints; ++i)
    {
        Eigen::Vector2f diff = ((points[i]) - mean);
        covMat += diff * diff.adjoint();
    }

    // now we just have to pick the eigen vector with smallest eigen value for normal
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(covMat);
    Eigen::Vector3f result;
    result[0] = eig.eigenvectors().col(0)[0];
    result[1] = eig.eigenvectors().col(0)[1];

    // let's compute the constant coefficient such that the
    // plane pass trough the mean point:
    result[2] = - (result[0]* mean[0] + result[1]*mean[1]);

    return result;
}

/**
 * @brief getKGroup get a group of values that are closer then threshold from value
 * @param ks list of all values
 * @param result the resulting group of values
 * @param value reference value
 * @param threshold the threshold of the distance
 */
void getKGroup(std::vector<float> &ks, std::vector<float> &result, float value, float threshold)
{
    result.clear();
    for(std::vector<float>::iterator kIt = ks.begin(); kIt != ks.end(); kIt++)
    {
        if(std::abs(*kIt - value) < threshold)
        {
            result.push_back(*kIt);
        }
    }
}


/**
 * @brief getKGroup get a group of values that are closer then threshold from value.
 * @param ks sorted! list of all values
 * @param result the resulting group of values
 * @param value reference value
 * @param threshold the threshold of the distance
 *
 * First does binary search then goes left and right to find good values inside threshold.
 */
void getKGroupSorted(std::vector<float> &ks, std::vector<float> &result, float value, float threshold)
{
    result.clear();
    //binary search for index
    int kleft = 0;
    int kright = ks.size() - 1;
    int mid = (kleft + kright)/2;
    while(mid != kright && mid != kleft)
    {
        if(std::abs(ks[mid] - value) < threshold)
        {
            break;
        }

        if(ks[mid] > value)
        {
            kright = mid;
        } else {
            kleft = mid;
        }

        mid = (kright + kleft)/2;
    }

    for(int i = mid + 1; i < (int) ks.size() && ks[i] < (value + threshold); i++)
    {
        result.push_back(ks[i]);
    }

    for(int i = mid; i >= 0 && ks[i] > (value - threshold); i--)
    {
        result.push_back(ks[i]);
    }
}

/**
 * @brief assignIndex - fill our relation function with data - for each k set index
 * @param indexes - reference to our relation function
 * @param ks - values of the step coefficient for the mesh, that should have the same index
 * @param index - the index that is assigned to the values of the step coefficient
 */
void addClusterMapping(std::vector<Eigen::Vector2f> &indexes, std::vector<float> &ks, int index)
{
    for(std::vector<float>::iterator kIt = ks.begin(); kIt != ks.end(); kIt++)
    {
        indexes.push_back(Eigen::Vector2f(*kIt, (float)index));
    }
}



typedef struct
{
    float value;
    int pos;
} SortedK;

bool sortKByValue(const SortedK &k1, const SortedK &k2)
{
    return k1.value < k2.value;
}

/**
 * @brief estimateKSimple - For small number of lines estimate the step coefficient
 * @param ks the set of values
 * @param kDiffThreshold the minimum difference between clusters
 * @param estimatedK the estimated step int the inverse k values
 * @param startK the starting position for assigning indexes to k (this represents 0th cluster)
 * @return the success of the operation
 */
bool estimateKSimple(std::vector<float> &ks, const float kDiffThreshold, float &estimatedK, float &startK)
{
    std::vector<SortedK> diffs;
    //no get the diferrences between pairs and store positions and values for maximum for each group

    float prevValue = ks.front();
    for(std::vector<float>::iterator kIt = ks.begin() + 1; kIt != ks.end(); kIt++)
    {
        if(*kIt - prevValue > kDiffThreshold)
        {
            SortedK newK;
            newK.value = *kIt - prevValue;
            newK.pos = kIt - ks.begin() - 1;
            diffs.push_back(newK);
        }
        prevValue = *kIt;
    }

    if(diffs.empty())
    {
        return false;
    }

    //now sort the diffks to get
    std::sort(diffs.begin(), diffs.end(), sortKByValue);
    //get the median from the values
    SortedK diffMean = diffs[diffs.size()/2];

    estimatedK = diffMean.value;
    startK = ks[diffMean.pos];

    return true;
}



typedef struct
{
    int diff;
    int mean;
} MSDiff;


bool sortByDiff(const MSDiff &k1, const MSDiff &k2)
{
    return k1.diff < k2.diff;
}

/**
 * @brief estimateKCluster - Estimate K using simple clustering (mean-shift)
 * @param ks the set of values
 * @param kDiffThreshold the minimum distance between clusters of lines
 * @param estimatedK the resulting estimate
 * @param startK the resulting starting position for cluster search (cluster with index 0)
 * @return success of the operation
 */
bool estimateKCluster(std::vector<float> &ks, const float kDiffThreshold, float &estimatedK, float &startK)
{
    //compute sum and squared to get standard deviation
    float ksum = 0;
    float ksquareSum = 0;
    for(std::vector<float>::iterator kIt = ks.begin(); kIt != ks.end(); kIt++)
    {
        ksum += *kIt;
        ksquareSum += (*kIt)*(*kIt);
    };

    //get initial guess for step and bias using correlation
    float avg = ksum/ks.size();
    float dev = 2*sqrt(std::abs(ksquareSum/ks.size() - avg*avg));

    //the buffer size for the clustering
    const int MEAN_SHIFT_BUFFER_SIZE = 512;
    const int MEAN_SHIFT_BUFFER_SIZE_HALF = MEAN_SHIFT_BUFFER_SIZE/2;

    std::vector<int> buffer(MEAN_SHIFT_BUFFER_SIZE, 0);
    for(std::vector<float>::iterator kIt = ks.begin(); kIt != ks.end(); kIt++)
    {
        int pt = static_cast<int>((*kIt - avg)*MEAN_SHIFT_BUFFER_SIZE_HALF/dev) + MEAN_SHIFT_BUFFER_SIZE_HALF;
        if(pt >= 0 && pt < MEAN_SHIFT_BUFFER_SIZE)
        {
            buffer[pt]++;
        }
    }

    float bufferAvg = 0;
    //float bufferGood = 0;
    for(int i = 0; i < MEAN_SHIFT_BUFFER_SIZE; i++)
    {
        bufferAvg += static_cast<float>(buffer[i]*buffer[i]);
        //bufferGood += (int) (buffer[i] > 0);
    }
	bufferAvg = (std::max)(std::sqrt(bufferAvg/MEAN_SHIFT_BUFFER_SIZE), 1.f);

    for(int i = 0; i < MEAN_SHIFT_BUFFER_SIZE; i++)
    {
        buffer[i] *= (int)(buffer[i] > bufferAvg);
    }

    const int windowSize = static_cast<int>(kDiffThreshold*MEAN_SHIFT_BUFFER_SIZE_HALF/dev);
    const int windowSizeHalf = windowSize/2;
    const int MAX_ITER = windowSize*3;

    std::vector<int> means;
    means.reserve(MEAN_SHIFT_BUFFER_SIZE);
    for(int i = 0; i < MEAN_SHIFT_BUFFER_SIZE; i++)
    {
        if(buffer[i] > 0)
        {
            //run meanshift
            int prevPos = -1;
            int currentPos = i;
            int iterCount = 0;
            while((iterCount++) < MAX_ITER && prevPos != currentPos)
            {
                prevPos = currentPos;
                int weightedSum = 0;
                int weightSum = 0;

                for(int cp = std::max(currentPos - windowSizeHalf,0); cp < std::min(currentPos + windowSizeHalf, MEAN_SHIFT_BUFFER_SIZE); cp++)
                {
                    weightSum += (buffer[cp]);
                    weightedSum += (buffer[cp]*cp);
                }

                if(weightSum == 0)
                {
                    currentPos = -1;
                    break;
                }

                int newPos = (weightedSum/weightSum);
                currentPos = newPos;
            }
            if(currentPos >= 0)
            {
                if(!means.empty() && currentPos == *means.rbegin())
                {
                    continue; //skip if the same as the previous
                }
                means.push_back(currentPos);
            }
        }
    }

    //now do the same as previously
    //maybe unnecessary, just to be sure
    std::sort(means.begin(), means.end());
    //get diffs
    //std::vector<int> diffs;
    std::vector<MSDiff> diffs;

    unsigned int meancount = means.size();
    if(meancount < 2)
    {
        return false;
    }

    for(unsigned int mi = 0; mi < meancount - 1; mi++)
    {
        int cdiff = means[mi+1] - means[mi];
        if(cdiff > windowSizeHalf)
        {
            MSDiff pdiff;
            pdiff.diff = cdiff;
            pdiff.mean = means[mi];
            diffs.push_back(pdiff);
        }
    }

    if(diffs.empty())
    {
        return false;
    }

    std::sort(diffs.begin(), diffs.end(), sortByDiff);
    MSDiff dMed = diffs[(diffs.size())/2];

    startK = (dMed.mean - MEAN_SHIFT_BUFFER_SIZE_HALF)*dev/MEAN_SHIFT_BUFFER_SIZE_HALF + avg;
    estimatedK = dMed.diff*dev/MEAN_SHIFT_BUFFER_SIZE_HALF;

    return true;
}


}
