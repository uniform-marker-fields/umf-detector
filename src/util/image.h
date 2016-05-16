#ifndef __UMF_IMAGE_H_
#define __UMF_IMAGE_H_

#include <Eigen/Core>
#include "fixed_class.h"
#include "../defines.h"

namespace fixp {
    using namespace fixedpoint;
}

namespace umf {

// The internal image structure and sampling methods

struct Edgel
{
    Eigen::Vector3f line;
    Eigen::Vector2f endPoints[2];
    Eigen::Vector2f normal;
    float score;
};

template<class T, int N>
class UMF Image
{
public:
    Image(int width, int height, bool allocate = true, int widthstep = -1);
    ~Image();

    inline T *get2D(int x, int y, int channel=0);
    inline const T *get2D(int x, int y, int channel=0) const;
    Eigen::Matrix<T, N, 1> get2De(int x, int y);

	inline void get2Der(Eigen::Matrix<T, N, 1> &res, int x, int y);
	inline void get2Der(Eigen::Matrix<T, N, 1> &res, int x, int y) const;

    template<class TP>
    inline void get2Dconv(Eigen::Matrix<TP, N, 1> &res, int x, int y);

    template< int precision>
    inline void get2Dconv(Eigen::Matrix< fixp::fixed_point<precision>, N, 1> &res, int x, int y);

	template<class TP>
	inline void interpolate(Eigen::Matrix<TP, N, 1> &res, float x, float y);

    template<class TP>
    inline void get2DconvScale(Eigen::Matrix<TP, N, 1> &res, int x, int y);

	template<class TP>
	inline void convert(Image<TP, N> *otherimg);

    template< int precision>
    inline void get2DconvScale(Eigen::Matrix< fixp::fixed_point<precision>, N, 1> &res, int x, int y);

    inline size_t elemSize() const { return sizeof(T)*N; }

    char *data;
    int width;
    int height;
    int channels;
    int widthstep;

private:
    bool allocated;

};



typedef Image<unsigned char, 3> ImageRGB;
typedef Image<unsigned char, 1> ImageGray;
typedef Image<float, 1> ImageFloat;
typedef Image<short, 1> ImageShort;

template<class T, int NCHAN>
T *Image<T, NCHAN>::get2D(int x, int y, int channel)
{
    return (T*) &(this->data[this->widthstep*y + x*NCHAN*sizeof(T) + channel*sizeof(T)]);
}

template<class T, int NCHAN>
const T *Image<T, NCHAN>::get2D(int x, int y, int channel) const
{
    return (T*) &(this->data[this->widthstep*y + x*NCHAN*sizeof(T) + channel*sizeof(T)]);
}

template<class T, int N>
template<class TP>
inline void Image<T, N>::convert(Image<TP, N> *otherimg)
{
	assert(otherimg->width == this->width && this->height == otherimg->height);
	for(int y = 0; y < this->height; y++)
	{
		T* currOrig = this->get2D(0, y);
		TP* currOther = otherimg->get2D(0, y);
		for(int x = 0; x < this->width*N; x++)
		{
			*(currOther++) = static_cast<TP>(*(currOrig++));
		}
	}
}

/*
template<>
void ImageRGB::get2Der(Eigen::Matrix<unsigned char, 3, 1> &retVal, int x, int y)
{
    unsigned char *p = this->get2D(x, y);
	retVal[0] = p[0];
	retVal[1] = p[1];
	retVal[2] = p[2];
}
*/

template<class T, int N>
void Image<T, N>::get2Der(Eigen::Matrix<T, N, 1> &retVal, int x, int y)
{
    T *p = this->get2D(x, y);
    for(int i = 0; i < N; i++){
        retVal(i) = p[i];
    }
}


template<class T, int N>
void Image<T, N>::get2Der(Eigen::Matrix<T, N, 1> &retVal, int x, int y) const
{
    const T *p = this->get2D(x, y);
    for(int i = 0; i < N; i++){
        retVal(i) = p[i];
    }
}


template<class T, int N>
template<class TP>
void Image<T, N>::get2Dconv(Eigen::Matrix<TP, N, 1> &res, int x, int y)
{
    T *p = this->get2D(x, y);
    for(int i = 0; i < N; i++)
    {
        res[i] = static_cast<TP>(p[i]);
    }
}

template<class T, int N>
template<int precision>
void Image<T, N>::get2Dconv(Eigen::Matrix< fixp::fixed_point<precision>, N, 1> &res, int x, int y)
{
    T *p = this->get2D(x, y);
    for(int i = 0; i < N; i++)
    {
        res[i] = fixp::fixed_point<precision>(p[i]);
    }
}

template<class T, int N>
template<class TP>
void Image<T, N>::get2DconvScale(Eigen::Matrix<TP, N, 1> &res, int x, int y)
{
    this->get2Dconv(res, x, y);
}


template<class T, int N>
template<class TP>
inline void Image<T, N>::interpolate(Eigen::Matrix<TP, N, 1> &res, float x, float y)
{
	int xt = static_cast<int>(x);  /* coordinates of top-left corner */
	int yt = static_cast<int>(y);
	float ax = x - xt;
	float ay = y - yt;

	Eigen::Matrix<float, N, 1> p1, p2, p3, p4;
	this->get2Dconv(p1, xt, yt);
	this->get2Dconv(p2, xt+1, yt);
	this->get2Dconv(p3, xt, yt+1);
	this->get2Dconv(p4, xt+1, yt+1);

	res = ((1 - ax)*((1 - ay)*p1 + ay*p3) + ax*( (1 - ay)*p2 + ay*p4)).template cast<TP>();
}

UMF void convertToGrayscale(ImageRGB *rgb, ImageGray* gray);

UMF void convertToYCbCr(ImageRGB *rgb, ImageGray* gray);

UMF void subSample2(ImageGray *input, ImageGray* output);

UMF void subSample4(ImageGray *input, ImageGray* output);


/*****************************************************************************/
//special stuff

template<>
template<class TP>
void ImageGray::get2Dconv(Eigen::Matrix<TP, 1, 1> &res, int x, int y)
{
    unsigned char* p = this->get2D(x,y);
    res[0] = static_cast<TP>(p[0]);
}

template<>
template<class TP>
void ImageRGB::get2Dconv(Eigen::Matrix<TP, 3, 1> &res, int x, int y)
{
    unsigned char* p = this->get2D(x,y);
    res[0] = static_cast<TP>(p[0]);
    res[1] = static_cast<TP>(p[1]);
    res[2] = static_cast<TP>(p[2]);
}

template<>
template<int precision>
void ImageGray::get2Dconv(Eigen::Matrix< fixp::fixed_point<precision>, 1, 1> &res, int x, int y)
{
    unsigned char *p = this->get2D(x, y);
    res[0].intValue = static_cast<int32_t>(p[0]) << precision;
}

template<>
template<int precision>
void ImageRGB::get2Dconv(Eigen::Matrix< fixp::fixed_point<precision>, 3, 1> &res, int x, int y)
{
    unsigned char *p = this->get2D(x, y);
    
    res[0].intValue = static_cast<int32_t>(p[0]) << precision;
    res[1].intValue = static_cast<int32_t>(p[1]) << precision;
    res[2].intValue = static_cast<int32_t>(p[2]) << precision;
}

//scale 0 - 255 to 0 - 1
template<>
template<int precision>
void ImageGray::get2DconvScale(Eigen::Matrix< fixp::fixed_point<precision>, 1, 1> &res, int x, int y)
{
    unsigned char *p = this->get2D(x, y);
    res[0].intValue = static_cast<int32_t>(p[0]) << (precision - 4);
}

template<>
template<int precision>
void ImageRGB::get2DconvScale(Eigen::Matrix< fixp::fixed_point<precision>, 3, 1> &res, int x, int y)
{
    unsigned char *p = this->get2D(x, y);
    
    res[0].intValue = static_cast<int32_t>(p[0]) << (precision - 4);
    res[1].intValue = static_cast<int32_t>(p[1]) << (precision - 4);
    res[2].intValue = static_cast<int32_t>(p[2]) << (precision - 4);
}



}



#endif
