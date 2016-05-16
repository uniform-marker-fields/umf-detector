#ifndef _UMF_LINE_ITERATOR_H__
#define _UMF_LINE_ITERATOR_H__

#include <Eigen/Core>
#include "image.h"

namespace umf {
/*!
   Line iterator class

   The class is used to iterate over all the pixels on the raster line
   segment connecting two specified points.
*/

    
bool clipLine( const Eigen::Vector2i& img_size, Eigen::Vector2i& pt1, Eigen::Vector2i& pt2, int border = 2);

template<class IMG_TYPE>
class LineIterator
{
public:
    //! intializes the iterator
    LineIterator( const IMG_TYPE* img, const Eigen::Vector2i& pt1, const Eigen::Vector2i& pt2,
                  int connectivity=4, bool leftToRight=false );
    LineIterator( const LineIterator& other);

    //! returns pointer to the current pixel
    inline unsigned char* operator *();
    //! prefix increment operator (++it). shifts iterator to the next pixel
    inline LineIterator& operator ++();
    //! postfix increment operator (it++). shifts iterator to the next pixel
    inline LineIterator operator ++(int);
    //! returns coordinates of the current pixel
    inline Eigen::Vector2i pos() const;
    inline void pos(int &x, int &y) const;

    unsigned char* ptr;
    const unsigned char* ptr0;
    int step, elemSize;
    int err, count;
    int minusDelta, plusDelta;
    int minusStep, plusStep;
    int posX, posY;
    int *stepDir, *jumpDir;
    int stepSign, jumpSign;
	Eigen::Vector2i pointStart;
	Eigen::Vector2i pointEnd;
};

template<class IMG_TYPE>
unsigned char* LineIterator<IMG_TYPE>::operator *() { return ptr; }

template<class IMG_TYPE>
LineIterator<IMG_TYPE>& LineIterator<IMG_TYPE>::operator ++()
{
    int mask = err < 0 ? -1 : 0;
    ptr += minusStep + (plusStep & mask);
    (*stepDir) += stepSign & (-1 - mask);
    (*jumpDir) += jumpSign & mask;
    
    err += minusDelta + (plusDelta & mask);
    return *this;
}

template<class IMG_TYPE>
LineIterator<IMG_TYPE> LineIterator<IMG_TYPE>::operator ++(int)
{
    LineIterator it = *this;
    ++(*this);
    return it;
}

template<class IMG_TYPE>
Eigen::Vector2i LineIterator<IMG_TYPE>::pos() const
{
    Eigen::Vector2i p;
    p(1) = (int)((ptr - ptr0)/step);
    p(0) = (int)(((ptr - ptr0) - p(1)*step)/elemSize);
    return p;
}

template<class IMG_TYPE>
void LineIterator<IMG_TYPE>::pos(int &x, int &y) const
{
    //y = (int)((ptr - ptr0)/step);
    //x = (int)(((ptr - ptr0) - y*step)/elemSize);
    x = this->posX;
    y = this->posY;
}

}


#endif