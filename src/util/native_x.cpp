
#include "native_x.h"

#ifdef UMF_USE_NATIVE

#include <native/convertYCbCr_x.h>
#include <native/getChromeMaskYCbCr_map_x.h>
#include <native/getChromeMaskYCbCr_mask_x.h>
#include <native/getChromeMaskNV21_tuple.h>
#include <native/smooth3x3_ui8.h>
#include <native/computeGradients3x3_X.h>
#include <native/computeGradients3x3_Y.h>
#include <native/halide_subsample2.h>
#include <native/halide_subsample4.h>
#include "image.h"
#include <stdint.h>
namespace umfnative
{

#ifndef BUFFER_T_DEFINED
#define BUFFER_T_DEFINED
#include <stdint.h>
typedef struct buffer_t {
    uint64_t dev;
    uint8_t* host;
    int32_t extent[4];
    int32_t stride[4];
    int32_t min[4];
    int32_t elem_size;
    bool host_dirty;
    bool dev_dirty;
} buffer_t;
#endif

extern "C" void halide_copy_to_host(void *user_context, buffer_t* buf);

template<typename T>
class XImage {
    struct Contents {
        Contents(buffer_t b, uint8_t* a, bool own = true) : buf(b), ref_count(1), alloc(a), owning(own) {}
        buffer_t buf;
        bool owning;
        int ref_count;
        uint8_t *alloc;
        ~Contents() {
            if(owning)
            {
                delete[] alloc;
            }
        }
    };

    Contents *contents;

    void initialize(int w, int h, int c, uint8_t *ptr = NULL) {
        buffer_t buf;
        if(c > 1)
        {
            buf.extent[0] = c;
            buf.extent[1] = w;
            buf.extent[2] = h;
            buf.extent[3] = 1;
            buf.stride[0] = 1;
            buf.stride[1] = c;
            buf.stride[2] = w*c;
            buf.stride[3] = 0;
        } else {
            buf.extent[0] = w;
            buf.extent[1] = h;
            buf.extent[2] = 1;
            buf.extent[3] = 1;
            buf.stride[0] = 1;
            buf.stride[1] = w;
            buf.stride[2] = 0;
            buf.stride[3] = 0;
        }
        buf.min[0] = 0;
        buf.min[1] = 0;
        buf.min[2] = 0;
        buf.min[3] = 0;
        buf.elem_size = sizeof(T);

        bool own = false;
        if(ptr == NULL)
        {
            own = true;
            ptr = new uint8_t[sizeof(T)*w*h*c+32];
            buf.host = ptr;
            while ((size_t)buf.host & 0x1f) buf.host++;
        } else {
            buf.host = ptr;
        }
        buf.host_dirty = false;
        buf.dev_dirty = false;
        buf.dev = 0;
        contents = new Contents(buf, ptr, own);
    }

public:
    XImage() : contents(NULL) {
    }

    XImage(int w, int h = 1, int c = 1) {
        initialize(w, h, c);
    }

    XImage(uint8_t* data, int w, int h = 1, int c = 1)
    {
        initialize(w, h, c, data);
    }

    XImage(const XImage &other) : contents(other.contents) {
        if (contents) {
            contents->ref_count++;
        }
    }

    ~XImage() {
        if (contents) {
            contents->ref_count--;
            if (contents->ref_count == 0) {
                delete contents;
                contents = NULL;
            }
        }
    }

    XImage &operator=(const XImage &other) {
        Contents *p = other.contents;
        if (p) {
            p->ref_count++;
        }
        if (contents) {
            contents->ref_count--;
            if (contents->ref_count == 0) {
                delete contents;
                contents = NULL;
            }
        }
        contents = p;
        return *this;
    }

    T *data() {return (T*)contents->buf.host;}

    void set_host_dirty(bool dirty = true) {
        // If you use data directly, you must also call this so that
        // gpu-side code knows that it needs to copy stuff over.
        contents->buf.host_dirty = dirty;
    }

    void copy_to_host() {
        if (contents->buf.dev_dirty) {
            halide_copy_to_host(NULL, &contents->buf);
            contents->buf.dev_dirty = false;
        }
    }

    XImage(T vals[]) {
        initialize(sizeof(vals)/sizeof(T), 1, 1);
        for (int i = 0; i < sizeof(vals); i++) (*this)(i, 0, 0) = vals[i];
    }

    void copy(T* vals, int width, int height) {
        assert(contents && "Copying into an uninitialized image");
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < height; x++) {
                (*this)(x, y, 0) = vals[y * width + x];
            }
        }
    }

    /** Make sure you've called copy_to_host before you start
     * accessing pixels directly. */
    T &operator()(int x, int y = 0, int c = 0) {
        T *ptr = (T *)contents->buf.host;
        int w = contents->buf.extent[0];
        int h = contents->buf.extent[1];
        return ptr[(c*h + y)*w + x];
    }

    /** Make sure you've called copy_to_host before you start
     * accessing pixels directly */
    const T &operator()(int x, int y = 0, int c = 0) const {
        const T *ptr = (const T *)contents->buf.host;
        return ptr[c*contents->buf.stride[2] + y*contents->buf.stride[1] + x*contents->buf.stride[0]];
    }

    operator buffer_t *() const {
        return &(contents->buf);
    }

    int width() const {
        return contents->buf.extent[0];
    }

    int height() const {
        return contents->buf.extent[1];
    }

    int channels() const {
        return contents->buf.extent[2];
    }

    int stride(int dim) const {
        return contents->buf.stride[dim];
    }

    int size(int dim) const {
        return contents->buf.extent[dim];
    }

    inline void setSize(int dim, size_t size)
    {
        contents->buf.extent[dim] = size;
    }

    inline void setMin(int dim, size_t size)
    {
        contents->buf.min[dim] = size;
    }

    inline void setData(uint8_t* palloc)
    {
        contents->buf.host = palloc;
        contents->alloc = palloc;
    }

};

void convertToYCbCr(umf::ImageRGB *rgb, umf::ImageGray *yuv)
{
    XImage<uint8_t> xrgb((uint8_t*) rgb->data, rgb->width, rgb->height, 3);
    XImage<uint8_t> xyuv((uint8_t*) yuv->data, rgb->width, rgb->height, 3);

    convertYCbCr_x(xrgb, xyuv);
}

void getChromeMaskYCbCr(umf::ImageGray* yuv, umf::ImageGray* mask, umf::ImageGray* map)
{
    XImage<uint8_t> xyuv((uint8_t*) yuv->data, yuv->width/3, yuv->height, 3);
    XImage<uint8_t> xmask((uint8_t*) mask->data, mask->width, mask->height);
    XImage<uint8_t> xmap((uint8_t*) map->data, map->width, map->height);

    getChromeMaskYCbCr_map_x(xyuv, xmap);
    getChromeMaskYCbCr_mask_x(xyuv, xmask);
}

void getChromeMaskNV21(umf::ImageGray* yuv, umf::ImageGray* mask, umf::ImageGray* map)
{
    XImage<uint8_t> xyuv((uint8_t*) yuv->data, yuv->width, yuv->height);
    XImage<uint8_t> xmask((uint8_t*) mask->data, mask->width, mask->height);
    XImage<uint8_t> xmap((uint8_t*) map->data, map->width, map->height);

    //getChromeMaskNV21_map_x(xyuv, xmap);
    //getChromeMaskNV21_mask_x(xyuv, xmask);
    getChromeMaskNV21_tuple(xyuv, xmap, xmask);
}

void smooth3x3(umf::ImageGray *gray, umf::ImageGray *result)
{
    XImage<uint8_t> xgray((uint8_t*) gray->data, gray->width, gray->height);
    XImage<uint8_t> xresult((uint8_t*) result->data, result->width, result->height);

    smooth3x3_ui8(xgray, xresult);
}

void computeGradients3x3(umf::ImageGray *input, umf::ImageFloat *gradX, umf::ImageFloat *gradY)
{
    XImage<uint8_t> xinput((uint8_t*) input->data, input->width, input->height);
    XImage<float> xgradX((uint8_t*) gradX->data, gradX->width, gradX->height);
    xgradX.setSize(0, gradX->width - 2); xgradX.setSize(1, gradX->height - 2);
    xgradX.setMin(0, 1); xgradX.setMin(1, 1);
    xgradX.setData((uint8_t*)gradX->data + gradX->widthstep + gradX->elemSize());

    XImage<float> xgradY((uint8_t*) gradY->data, gradY->width, gradY->height);
    xgradY.setSize(0, gradY->width - 2); xgradY.setSize(1, gradX->height - 2);
    xgradY.setMin(0, 1); xgradY.setMin(1, 1);
    xgradY.setData((uint8_t*)gradY->data + gradY->widthstep + gradY->elemSize());

    computeGradients3x3_X(xinput, xgradX);
    computeGradients3x3_Y(xinput, xgradY);
}


void h_subsample2( unsigned char *img, int ncols, int nrows, unsigned char *img2)
{
    XImage<uint8_t> xinput((uint8_t*) img, ncols, nrows);
    XImage<uint8_t> xoutput((uint8_t*) img2, ncols/2, nrows/2);
    halide_subsample2(xinput, xoutput);
}

void h_subsample4( unsigned char *img, int ncols, int nrows, unsigned char *img2)
{
    XImage<uint8_t> xinput((uint8_t*) img, ncols, nrows);
    XImage<uint8_t> xoutput((uint8_t*) img2, ncols/4, nrows/4);
    halide_subsample4(xinput, xoutput);
}



}

#endif

