#include "image.h"
#include "../defines.h"

namespace umf {

template<class T, int NCHAN>
Image<T, NCHAN>::Image(int width, int height, bool allocate, int widthstep)
    :width(width), height(height), channels(NCHAN), allocated(allocate)

{
    if(widthstep == -1)
    {
        this->widthstep = width*NCHAN*sizeof(T);
    } else {
        this->widthstep = widthstep;
    }


    if(allocate)
    {
        this->data = new char[this->widthstep*this->height];
    } else {
        this->data = NULL;
    }
}


template<class T, int NCHAN>
Image<T, NCHAN>::~Image()
{
    if(this->allocated)
    {
        delete [] this->data;
    }
}



template<class T, int N>
Eigen::Matrix<T, N, 1> Image<T, N>::get2De(int x, int y)
{
    Eigen::Matrix<T, N, 1> retVal;
    T *p = this->get2D(x, y);
    for(int i = 0; i < N; i++){
        retVal(i) = p[i];
    }
    return retVal;
}



template<>
Eigen::Matrix<unsigned char, 3, 1> Image<unsigned char, 3>::get2De(int x, int y)
{
    unsigned char *p = this->get2D(x, y);
    return Eigen::Matrix<unsigned char, 3, 1>(p[0], p[1], p[2]);
}



void convertToGrayscale(ImageRGB *rgb, ImageGray* gray)
{
	if(rgb->width != gray->width || rgb->height != gray->height)
	{
		return;
	}
    for(int row = 0; row < rgb->height; row++)
	{
        for(int col = 0; col < rgb->width; col++)
		{
			unsigned char* p = (unsigned char*) &(rgb->data[row*rgb->widthstep + col*rgb->channels]);
			gray->data[row*gray->widthstep + col] = (unsigned char) (0.299f * p[0]  + 0.587f * p[1] + 0.114f*p[2]);
		}
	}
}

void convertToYCbCr(ImageRGB *rgb, ImageGray* gray)
{
	if(rgb->width*3 != gray->width || rgb->height != gray->height)
	{
		return;
	}
    for(int row = 0; row < rgb->height; row++)
	{
        for(int col = 0; col < rgb->width; col++)
		{
			unsigned char* p = (unsigned char*) &(rgb->data[row*rgb->widthstep + col*rgb->channels]);
			int y = (int) (0.299f * p[0]  + 0.587f * p[1] + 0.114f*p[2]);
			gray->data[row*gray->widthstep + 3*col] = (unsigned char) y;
			gray->data[row*gray->widthstep + 3*col + 2] = (unsigned char) ((p[0] - y)*0.713f + 128); //Cr
			gray->data[row*gray->widthstep + 3*col + 1] = (unsigned char) ((p[2] - y)*0.564f + 128); //Cb
		}
	}
}


void subSample2(ImageGray *input, ImageGray *output)
{
    for (int y = 0; y < output->height; y++)
	{
		unsigned char* orow = output->get2D(0, y);
		unsigned char* irow1 = input->get2D(0, y * 2);
		unsigned char* irow2 = input->get2D(0, y * 2 + 1);

        for (int x = 0; x < output->width; x++, orow++, irow1 += 2, irow2 += 2)
		{
			*(orow) = static_cast<uint8_t>((static_cast<uint16_t>(*irow1) + static_cast<uint16_t>(*(irow1 + 1)) + static_cast<uint16_t>(*(irow2)) + static_cast<uint16_t>(*(irow2 + 1))) >> 2);
		}
	}
}

void subSample4(ImageGray *input, ImageGray *output)
{
    for (int y = 0; y < output->height; y++)
	{
		unsigned char* orow = output->get2D(0, y);
		unsigned char* irow1 = input->get2D(0, y * 4);
		unsigned char* irow2 = input->get2D(0, y * 4 + 1);
		unsigned char* irow3 = input->get2D(0, y * 4 + 2);
		unsigned char* irow4 = input->get2D(0, y * 4 + 3);

        for (int x = 0; x < output->width; x++, orow++, irow1 += 4, irow2 += 4, irow3 += 4, irow4 += 4)
		{
			*(orow) = static_cast<uint8_t>((static_cast<uint16_t>(*irow1) + static_cast<uint16_t>(*(irow1 + 1)) + static_cast<uint16_t>(*(irow1 + 2)) + static_cast<uint16_t>(*(irow1 + 3)) +
				static_cast<uint16_t>(*irow2) + static_cast<uint16_t>(*(irow2 + 1)) + static_cast<uint16_t>(*(irow2 + 2)) + static_cast<uint16_t>(*(irow2 + 3)) + 
				static_cast<uint16_t>(*irow3) + static_cast<uint16_t>(*(irow3 + 1)) + static_cast<uint16_t>(*(irow3 + 2)) + static_cast<uint16_t>(*(irow3 + 3)) + 
				static_cast<uint16_t>(*irow4) + static_cast<uint16_t>(*(irow4 + 1)) + static_cast<uint16_t>(*(irow4 + 2)) + static_cast<uint16_t>(*(irow4 + 3))) >> 4);
		}
	}
}

template UMF ImageRGB::Image(int width, int height, bool allocate, int widthstep);
template UMF ImageGray::Image(int width, int height, bool allocate, int widthstep);
template UMF ImageRGB::~Image();
template UMF ImageGray::~Image();


template UMF ImageFloat::Image(int width, int height, bool allocate, int widthstep);
template UMF ImageFloat::~Image();
template UMF ImageShort::Image(int width, int height, bool allocate, int widthstep);
template UMF ImageShort::~Image();


//template Eigen::Matrix<unsigned char, 3, 1> ImageRGB::get2De(int x, int y);
template Eigen::Matrix<unsigned char, 1, 1> ImageGray::get2De(int x, int y);

}
