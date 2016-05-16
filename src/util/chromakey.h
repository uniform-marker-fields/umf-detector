#ifndef __UMF_CHROMAKEY_H
#define __UMF_CHROMAKEY_H

#include "image.h"
#include "../defines.h"
#include "../model.h"
#include "../marker.h"
#include <Eigen/Core>


namespace e {
	using namespace Eigen;
}

namespace umf {

/**
 * general version to compute chrome mask
 */
template<class T, int NCHAN>
void getChromeMask(Image<T, NCHAN> *srcRGB, ImageGray *mask);

/**
 * Get the alphamask from YCbCr image using chromakeying. This is the low-quality masking and
 * mapping algorithm used as input for the detector. It only generates a mask and grayscale mapping
 * of colors.
 *
 * The key color is fixed. GMM mapping is in experimental phase. Will be published later.
 *
 * @param the input YCbCr image stored overlapped Y Cb Cr channels, neither is subsampled
 * @param mask The output binary mask containing the masked out background
 * @param map The output grayscale map used further for the detector
 */
UMF void getChromeMaskYCbCr(ImageGray *srcYCbCr, ImageGray *mask, ImageGray *map = NULL);

/**
 * Get the alphamask from NV21 encoded image using chromakeying. This is the low-quality masking and
 * mapping algorithm used as input for the detector. It only generates a mask and grayscale mapping
 * of colors.
 *
 * The key color is fixed. GMM mapping is in experimental phase. Will be published later.
 *
 * @param the input NV21 The channels should not be overlapped. First the full res Y channel,followed by subspampled Cb, Cr channels
 * @param mask The output binary mask containing the masked out background
 * @param map The output grayscale map used further for the detector
 */
void getChromeMaskNV21(ImageGray *srcYUV, ImageGray *mask, ImageGray *map = NULL);
}
#endif // CHROMAKEY_H
