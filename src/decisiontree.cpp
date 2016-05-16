#include "decisiontree.h"

#include <vector>
#include <string>

namespace umf {

template<int NCHAN>
DecisionTree<NCHAN>::DecisionTree()
{
    this->root = new UMFNode<NCHAN>;
    for(int i = 0; i < UMFNodeCount<NCHAN>::value; i++)
    {
        this->root->data.next[i] = NULL;
    }

    this->root->type = DTREE_NODE;
    this->root->parent = NULL;
    counter = 0;
}

template<int NCHAN>
DecisionTree<NCHAN>::~DecisionTree()
{
    //depth first path
    UMFNode<NCHAN> *currentNode = this->df(this->root);
    while(currentNode)
    {
        UMFNode<NCHAN> *nextNode = this->dfNext(currentNode);
        delete currentNode;
        currentNode = nextNode;
    }
}

template<int NCHAN>
bool DecisionTree<NCHAN>::addPath(Location loc, std::vector<Eigen::Matrix<EdgeType, NCHAN, 1> > &values)
{
    UMFNode<NCHAN> *parent = this->root;

    unsigned int index = 0;
    for(; index+1 < values.size(); index++)
    {
        unsigned short chi = this->getNodeIndex(values[index]);
        if(parent->data.next[chi] == NULL) //if the child doesn't exist
        {
            UMFNode<NCHAN> *current = new UMFNode<NCHAN>;
            parent->data.next[chi] = current;
            current->parent = parent;
            current->type = DTREE_NODE;

            for(int i = 0; i < UMFNodeCount<NCHAN>::value; i++)
            {
                current->data.next[i] = NULL;
            }
        }
        parent = parent->data.next[chi];
    }
    unsigned short last = this->getNodeIndex(values[index]);
    if(parent->data.next[last] != NULL)
    {
        return false;
    }

    UMFNode<NCHAN> *leaf = new UMFNode<NCHAN>;
    leaf->type = DTREE_LEAF;
    leaf->data.loc = loc;
    leaf->parent = parent;
    parent->data.next[last] = leaf;
    return true;
}

template<int NCHAN>
void DecisionTree<NCHAN>::simplify(int maxCut)
{
    UMFNode<NCHAN> *leaf = this->df(this->root);

    while(leaf != NULL && leaf->type == DTREE_LEAF)
    {
        //std::cout << leaf->data.loc.c << " " << leaf->data.loc.r << " " << leaf->data.loc.rotation << std::endl;

        //go up the tree until we find a parent that has more than one child
        UMFNode<NCHAN> *parent = leaf->parent;
        bool searchFurther = true;
        int skipped = 0;
        while(searchFurther)
        {
            searchFurther = false;
            int nonNULLCount = 0;
            for(int i = 0; i < UMFNodeCount<NCHAN>::value; i++)
            {
                if(parent->data.next[i] != NULL)
                {
                    nonNULLCount++;
                }
            }

            //search further if it's still the only non-null child
            if(nonNULLCount < 2 && parent->parent != NULL)
            {
                skipped++;
                searchFurther = skipped < maxCut; //don't continue if tree would become too shallow
            }

            if(searchFurther)
            {
                parent = parent->parent;
            }
        }

        //now check if we have the same parent or not
        if(leaf->parent != parent)
        {
            //juppijee, we can cut down this branch
            //fere everthing in between
            UMFNode<NCHAN> *nextParent = leaf->parent;
            int chosenChild = 0;
            while(nextParent != parent)
            {
                UMFNode<NCHAN> *pnextParent = nextParent->parent;
                if(pnextParent == parent)
                {
                    for(; chosenChild < UMFNodeCount<NCHAN>::value; chosenChild++)
                    {
                        if(parent->data.next[chosenChild] == nextParent)
                        {
                            break;
                        }
                    }
                }
                counter++;
                delete nextParent;
                nextParent = pnextParent;
            }

            parent->data.next[chosenChild] = leaf;
            leaf->parent = parent;

        }

        //get next leaf
        leaf = this->dfNext(leaf);
        while(leaf != NULL && leaf->type == DTREE_NODE)
        {
            leaf = this->dfNext(leaf);
        }
    }

}

template<int NCHAN>
UMFNode<NCHAN>* DecisionTree<NCHAN>::pathGetNext(UMFNode<NCHAN>* node, unsigned short &nextIndex) const
{
    if(node == NULL)
    {
        return this->root->data.next[nextIndex];
    }

    if(node->type == DTREE_LEAF)
    {
        return NULL;
    }

    return node->data.next[nextIndex];
}

template<int NCHAN>
UMFNode<NCHAN>* DecisionTree<NCHAN>::pathGetNext(UMFNode<NCHAN>* node, Eigen::Matrix<EdgeType,NCHAN,1> &nextM) const
{
    unsigned short next = this->getNodeIndex(nextM);
    return this->pathGetNext(node, next);
}

template<int NCHAN>
UMFNode<NCHAN> *DecisionTree<NCHAN>::dfNext(UMFNode<NCHAN> *current)
{

    UMFNode<NCHAN> *parent = current->parent;

    if(parent == NULL)
    {
        return NULL;
    }

    //check siblings
    int index = UMFNodeCount<NCHAN>::value;
    int nextindex = UMFNodeCount<NCHAN>::value;
    for(int i = 0; i < UMFNodeCount<NCHAN>::value; i++)
    {
        if(parent->data.next[i] == current)
        {
            index = i;
        }
        else if(parent->data.next[i] != NULL && index < i)
        {
            nextindex = i;
            break;
        }
    }

    if(nextindex == UMFNodeCount<NCHAN>::value)
    {
        return parent;
    }

    //else go deep on the sibling we found
    return this->df(parent->data.next[nextindex]);
}

template<int NCHAN>
UMFNode<NCHAN> *DecisionTree<NCHAN>::df(UMFNode<NCHAN> *current)
{
    while(current->type != DTREE_LEAF)
    {
        int chosen_child = 0;
        for(; chosen_child < UMFNodeCount<NCHAN>::value; chosen_child++)
        {
            if(current->data.next[chosen_child] != NULL)
            {
                break;
            }
        }
        if(chosen_child == UMFNodeCount<NCHAN>::value) //no not null child was found, return ourselves
        {
            break;
        }
        current = current->data.next[chosen_child];
    }

    return current;
}

template<int NCHAN>
Location DecisionTree<NCHAN>::getLocation(std::vector< Eigen::Matrix<EdgeType,NCHAN,1> > &values) const
{
    UMFNode<NCHAN> *next = NULL;

    for(unsigned int index = 0; index < values.size(); index++)
    {
        unsigned short chi = this->getNodeIndex(values[index]);
        if(chi > UMFNodeCount<NCHAN>::value - 1)
        {
            return Location::invalid;
        }
        next = this->pathGetNext(next, chi);
        if(next == NULL)
        {
            return Location::invalid;
        }
        else if(next->type == DTREE_LEAF)
        {
            return next->data.loc;
        }
    }

    return Location::invalid;
}

template<int NCHAN>
Location DecisionTree<NCHAN>::getLocation(std::vector< unsigned short > &values) const
{
    UMFNode<NCHAN> *next = NULL;

    for(unsigned int index = 0; index < values.size(); index++)
    {
        unsigned short chi = values[index];
        if(chi > UMFNodeCount<NCHAN>::value - 1)
        {
            return Location::invalid;
        }
        next = this->pathGetNext(next, chi);
        if(next == NULL)
        {
            return Location::invalid;
        }
        else if(next->type == DTREE_LEAF)
        {
            return next->data.loc;
        }
    }

    return Location::invalid;
}

template<int NCHAN>
struct StackVal
{
    StackVal(): posX(0), posY(0), node(NULL) {}

    StackVal(float _posX, float _posY,UMFNode<NCHAN>* _node):
        posX(_posX), posY(_posY), node(_node){}

    float posX;
    float posY;
    UMFNode<NCHAN> *node;
};

template<int NCHAN>
int DecisionTree<NCHAN>::getChildNodeCount(UMFNode<NCHAN>* node)
{
    int leafCounter = 0;
    if(node == NULL)
    {
        return 1;
    }

    std::vector< StackVal<NCHAN> > bfstack;
    bfstack.push_back(StackVal<NCHAN>(0, 0, node));
    while(!bfstack.empty())
    {
        UMFNode<NCHAN> *currentNode = bfstack.back().node;
        bfstack.pop_back();
        if(currentNode->type == DTREE_LEAF)
        {
            leafCounter++;
        } else if(currentNode->type == DTREE_NODE)
        {
            for(int i = 0; i < 3; i++)
            {
                if(currentNode->data.next[i] == NULL)
                {
                    leafCounter++;
                } else {
                    bfstack.push_back(StackVal<NCHAN>(0,0, currentNode->data.next[i]));
                }
            }
        }
    }

    return leafCounter;
}

template DecisionTree<1>::DecisionTree();
template DecisionTree<3>::DecisionTree();

template DecisionTree<1>::~DecisionTree();
template DecisionTree<3>::~DecisionTree();

template void DecisionTree<1>::simplify(int);
template void DecisionTree<3>::simplify(int);

template bool DecisionTree<1>::addPath(Location loc, std::vector< Eigen::Matrix<EdgeType, 1, 1> >& values);
template bool DecisionTree<3>::addPath(Location loc, std::vector< Eigen::Matrix<EdgeType, 3, 1> >& values);

template Location DecisionTree<1>::getLocation(std::vector< Eigen::Matrix<EdgeType,1,1> > &values) const;
template Location DecisionTree<3>::getLocation(std::vector< Eigen::Matrix<EdgeType,3,1> > &values) const;
}

