#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include <Eigen/Core>

//per element median filter
//very unoptimal
template<class T, int N>
class MedianFilter
{
public:
    MedianFilter(unsigned int elements = 3)
    {
        maxCount = elements;
    }

    Eigen::Matrix<T,N,1> filter(Eigen::Matrix<T,N,1> newData){
        elements.push_back(newData);
        if(elements.size() > maxCount)
        {
            elements.erase(elements.begin());
        }
        std::vector<T> pp[N];
        int medianPos = elements.size()/2;
        for(unsigned int eI = 0; eI < elements.size(); eI++)
        {
            for(int i = 0; i < N; i++)
            {
                pp[i].push_back(elements[eI][i]);
            }
        }
        for(int i = 0; i < N; i++)
        {
            std::sort(pp[i].begin(), pp[i].end());
        }

        Eigen::Matrix<T, N, 1> result;
        for(int i = 0; i < N; i++)
        {
            result[i] = pp[i][medianPos];
        }
        return result;
    }
private:
    std::vector< Eigen::Matrix<T,N,1> > elements;
    unsigned int maxCount;
};


#endif // MEDIAN_FILTER_H
