#include "line_iterator.h"

namespace umf {




bool clipLine( const Eigen::Vector2i& img_size, Eigen::Vector2i& pt1, Eigen::Vector2i& pt2, int border)
{
    long int x1, y1, x2, y2;
    int c1, c2;
    long int right = img_size(0)-1 - border, bottom = img_size(1)-1 - border;

    if( img_size(0) <= 0 || img_size(1) <= 0 )
        return false;

    x1 = pt1(0); y1 = pt1(1); x2 = pt2(0); y2 = pt2(1);
    c1 = (x1 < border) + (x1 > right) * 2 + (y1 < border) * 4 + (y1 > bottom) * 8;
    c2 = (x2 < border) + (x2 > right) * 2 + (y2 < border) * 4 + (y2 > bottom) * 8;

    if( (c1 & c2) == 0 && (c1 | c2) != 0 )
    {
        long int a;
        if( c1 & 12 )
        {
            a = c1 < 8 ? border : bottom;
            x1 +=  (a - y1) * (x2 - x1) / (y2 - y1);
            y1 = a;
            c1 = (x1 < border) + (x1 > right) * 2;
        }
        if( c2 & 12 )
        {
            a = c2 < 8 ? border : bottom;
            x2 += (a - y2) * (x2 - x1) / (y2 - y1);
            y2 = a;
            c2 = (x2 < border) + (x2 > right) * 2;
        }
        if( (c1 & c2) == 0 && (c1 | c2) != 0 )
        {
            if( c1 )
            {
                a = c1 == 1 ? border : right;
                y1 += (a - x1) * (y2 - y1) / (x2 - x1);
                x1 = a;
                c1 = 0;
            }
            if( c2 )
            {
                a = c2 == 1 ? border : right;
                y2 += (a - x2) * (y2 - y1) / (x2 - x1);
                x2 = a;
                c2 = 0;
            }
        }

        assert( (c1 & c2) != 0 || (x1 | y1 | x2 | y2) >= 0 );

        pt1(0) = (int)x1;
        pt1(1) = (int)y1;
        pt2(0) = (int)x2;
        pt2(1) = (int)y2;
    }

    return (c1 | c2) == 0;
}

/*
   Initializes line iterator.
   Returns number of points on the line or negative number if error.
*/
template<class IMG_TYPE>
LineIterator<IMG_TYPE>::LineIterator(const IMG_TYPE* img, const Eigen::Vector2i& pt1, const Eigen::Vector2i& pt2,
                           int connectivity, bool left_to_right)
{
    count = -1;

    assert( connectivity == 8 || connectivity == 4 );
	this->pointStart = pt1;
	this->pointEnd = pt2;

    if( (unsigned)pt1(0) >= (unsigned)(img->width) ||
        (unsigned)pt2(0) >= (unsigned)(img->width) ||
        (unsigned)pt1(1) >= (unsigned)(img->height) ||
        (unsigned)pt2(1) >= (unsigned)(img->height) )
    {
        if( !clipLine( Eigen::Vector2i(img->width, img->height), this->pointStart, this->pointEnd ) )
        {
            ptr = (unsigned char*) img->data;
            err = plusDelta = minusDelta = plusStep = minusStep = count = 0;
            return;
        }
    }

    int bt_pix0 = (int) img->elemSize(), bt_pix = bt_pix0;
    size_t istep = img->widthstep;

    int dx = this->pointEnd(0) - this->pointStart(0);
    int dy = this->pointEnd(1) - this->pointStart(1);
    int s = dx < 0 ? -1 : 0;

    if( left_to_right )
    {
        dx = (dx ^ s) - s;
        dy = (dy ^ s) - s;
		this->pointStart(0) ^= (this->pointStart(0) ^ this->pointEnd(0)) & s;
		this->pointStart(1) ^= (this->pointStart(1) ^ this->pointEnd(1)) & s;
    }
    else
    {
        dx = (dx ^ s) - s;
        bt_pix = (bt_pix ^ s) - s;
    }

    ptr = (unsigned char*)(img->data + this->pointStart(1) * istep + this->pointStart(0) * bt_pix0);
    posX = this->pointStart(0);
    posY = this->pointStart(1);

    s = dy < 0 ? -1 : 0;
    dy = (dy ^ s) - s;
    istep = (istep ^ s) - s;

    s = dy > dx ? -1 : 0;

    /* conditional swaps */
    dx ^= dy & s;
    dy ^= dx & s;
    dx ^= dy & s;

    bt_pix ^= istep & s;
    istep ^= bt_pix & s;
    bt_pix ^= istep & s;

    if( connectivity == 8 )
    {
        assert( dx >= 0 && dy >= 0 );

        err = dx - (dy + dy);
        plusDelta = dx + dx;
        minusDelta = -(dy + dy);
        plusStep = (int)istep;
        minusStep = bt_pix;
        count = dx + 1;
    }
    else /* connectivity == 4 */
    {
        assert( dx >= 0 && dy >= 0 );

        err = 0;
        plusDelta = (dx + dx) + (dy + dy);
        minusDelta = -(dy + dy);
        plusStep = (int)istep - bt_pix;
        minusStep = bt_pix;
        count = dx + dy + 1;
    }

    int maskedIncrement = minusStep + plusStep;
    if(maskedIncrement == 1 || maskedIncrement == -1)
    {
        stepDir = &posY;
        stepSign = (minusStep > 0)? 1 : -1;
        jumpDir = &posX;
        jumpSign = maskedIncrement;
    } else {
        stepDir = &posX;
        jumpDir = &posY;
        stepSign = (minusStep > 0)? 1: -1;
        jumpSign = (maskedIncrement > 0)? 1: -1;
    }

    this->ptr0 = (unsigned char*) img->data;
    this->step = (int)img->widthstep;
    this->elemSize = bt_pix0;
}

template<class IMG_TYPE>
LineIterator<IMG_TYPE>::LineIterator( const LineIterator<IMG_TYPE> &other)
{
    this->ptr = other.ptr;
    this->ptr0 = other.ptr0;
    this->step = other.step;
    this->elemSize = other.elemSize;
    this->err = other.err;
    this->count = other.count;
    this->minusDelta = other.minusDelta;
    this->plusDelta = other.plusDelta;
    this->minusStep = other.minusStep;
    this->plusStep = other.plusStep;
    this->jumpSign = other.jumpSign;
    this->stepSign = other.stepSign;
    this->posX = other.posX;
    this->posY = other.posY;
    this->jumpDir = (other.jumpDir == &other.posX) ? &this->posX : &this->posY;
    this->stepDir = (other.stepDir == &other.posX) ? &this->posX : &this->posY;

	this->pointStart = other.pointStart;
	this->pointEnd = other.pointEnd;

}

template LineIterator<ImageGray>::LineIterator(const LineIterator<ImageGray> &other);
template LineIterator<ImageRGB>::LineIterator(const LineIterator<ImageRGB> &other);
template LineIterator<ImageRGB>::LineIterator( const ImageRGB* img, const Eigen::Vector2i& pt1, const Eigen::Vector2i& pt2, int connectivity, bool leftToRight );
template LineIterator<ImageGray>::LineIterator( const ImageGray* img, const Eigen::Vector2i& pt1, const Eigen::Vector2i& pt2, int connectivity, bool leftToRight );

}