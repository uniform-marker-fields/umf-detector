#ifndef __UMF_DECISIONTREE_H__
#define __UMF_DECISIONTREE_H__
#include "defines.h"
#include <Eigen/Core>
#include <vector>

namespace umf {

enum {
    DTREE_NODE = 0,
    DTREE_LEAF,
    DTREE_TYPE_COUNT
};

template<int B, int N>
struct Pow {
    // recursive call and recombination.
    enum{ value = B*Pow<B, N-1>::value };
};

template< int B >
struct Pow<B, 0> {
    // ''N == 0'' condition of termination.
    enum{ value = 1 };
};

template< int NCHAN >
struct UMFNodeCount {
    enum { value = Pow<3, NCHAN>::value };
};

template<int NCHAN>
struct UMFNode
{
    unsigned char type;
    UMFNode<NCHAN> *parent;
    union node_data
    {
        UMFNode<NCHAN> * next[ UMFNodeCount<NCHAN>::value ]; //3^channels tree
        Location loc;
    } data;

    static int nodeCount;
};

template<int NCHAN>
class DecisionTree
{
public:
    DecisionTree();
    ~DecisionTree();

    bool addPath(Location loc, std::vector< Eigen::Matrix<EdgeType,NCHAN,1> > &values);

    void simplify(int maxSimplifyCount = 0);


    Location getLocation(std::vector<Eigen::Matrix<EdgeType, NCHAN, 1> > &values) const;
    Location getLocation(std::vector< unsigned short > &values) const;

private:
    UMFNode<NCHAN> *root;
    int counter;

    inline unsigned short getNodeIndex(Eigen::Matrix<EdgeType,NCHAN,1> &p) const;

    UMFNode<NCHAN> *pathGetNext(UMFNode<NCHAN> *, Eigen::Matrix<EdgeType,NCHAN,1> &nextM) const;
    UMFNode<NCHAN> *pathGetNext(UMFNode<NCHAN> *, unsigned short &index) const;

    //returns the next node in depth first pass
    UMFNode<NCHAN> *dfNext(UMFNode<NCHAN> *);
    UMFNode<NCHAN> *df(UMFNode<NCHAN> *current);

    int getChildNodeCount(UMFNode<NCHAN> *);
};

template<>
inline unsigned short DecisionTree<1>::getNodeIndex(Eigen::Matrix<EdgeType,1,1> &p) const
{
    return p[0];
}

template<>
inline unsigned short DecisionTree<3>::getNodeIndex(Eigen::Matrix<EdgeType,3,1> &p) const
{
    return p[0] + p[1]*UMFNodeCount<1>::value + p[2]*UMFNodeCount<2>::value;
}

template<int NCHAN>
inline unsigned short DecisionTree<NCHAN>::getNodeIndex(Eigen::Matrix<EdgeType,NCHAN,1> &p) const
{
    unsigned short index = 0;
    int pow3 = 1;
    for(int i = 0; i < NCHAN; i++)
    {
        index += p[i]*pow3;
        pow3 *= UMFNodeCount<1>::value;
    }
    return index;
}


}

#endif // DECISIONTREE_H
