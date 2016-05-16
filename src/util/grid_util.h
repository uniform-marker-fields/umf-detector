#ifndef __UMF_GRID_UTIL_H
#define __UMF_GRID_UTIL_H

#include <Eigen/Core>
#include <vector>
#include "prosac.h"

namespace umf {

//see the .cpp document for more documentation
    
    
/**
 * @brief getMainIndexes - get the two main axes which we use to get the indexes and corresponding k-s
 * @param line0 - two lines
 * @param line1
 * @param index1 - returned first index
 * @param index2 - returned second index
 */
void getMainIndexes(Eigen::Vector3f line0, Eigen::Vector3f line1, int &index1, int &index2);


/**
 * @brief lineGetK get the k for a given line with a given index
 * @param i - the index of the line
 * @param ehorizont - the horizon
 * @param eline0 the zeroth line - fix
 * @param elinei line for which we need the coefficient
 * @param index1 first axis of the most change
 * @param index2 second axis of largest change in values
 * @return the k
 *
 * this is the k from the expression:
 *  l_1 = k*l_0 + i*h
 */
inline float lineGetK(float i, Eigen::Vector3f &ehorizont, Eigen::Vector3f &eline0, Eigen::Vector3f &elinei, int index1, int index2)
{
    return i*(ehorizont[index1]*elinei[index2] - ehorizont[index2]*elinei[index1])/(elinei[index1]*eline0[index2] - elinei[index2]*eline0[index1]);
}

inline float lineGetKPsI(float i, Eigen::Vector3f &horizont, Eigen::Vector3f &line0, Eigen::Vector3f &linei)
{
    Eigen::Vector3f ln = line0.cross(horizont);
    float nsqrinv = 1.0f/ln.dot(ln);

    Eigen::Vector3f l0q = horizont.cross(ln)*nsqrinv;
    Eigen::Vector3f lhq = ln.cross(line0)*nsqrinv;

    float ki = linei.dot(l0q)/linei.dot(lhq);
    return ki*i;
}

Eigen::Vector3f fitLine(std::vector<Eigen::Vector2f> &points);


void getKGroup(std::vector<float> &ks, std::vector<float> &result, float value, float threshold);

void getKGroupSorted(std::vector<float> &ks, std::vector<float> &result, float value, float threshold);

void addClusterMapping(std::vector<Eigen::Vector2f> &indexes, std::vector<float> &ks, int index);

bool estimateKSimple(std::vector<float> &ks, const float kDiffThreshold, float &estimatedK, float &startK);

bool estimateKCluster(std::vector<float> &ks, const float kDiffThreshold, float &estimatedK, float &startK);

}

#endif // GRID_UTIL_H
