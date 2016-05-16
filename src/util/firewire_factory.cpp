#include "firewire_factory.h"
#include "../defines.h"

#ifdef HAVE_FIREWIRE_LIB

#include <stdint.h>
#include <dc1394/dc1394.h>
#include <stdlib.h>
#include <inttypes.h>
#include <iostream>

#ifndef _WIN32
#include <unistd.h>
#endif

namespace umf {

static const char* cameraModeMapping[DC1394_VIDEO_MODE_NUM] = {
    "DC1394_VIDEO_MODE_160x120_YUV444",     //DC1394_VIDEO_MODE_160x120_YUV444= 64",
    "DC1394_VIDEO_MODE_320x240_YUV422",    //DC1394_VIDEO_MODE_320x240_YUV422",
    "DC1394_VIDEO_MODE_640x480_YUV411",    //DC1394_VIDEO_MODE_640x480_YUV411",
    "DC1394_VIDEO_MODE_640x480_YUV422",    //DC1394_VIDEO_MODE_640x480_YUV422",
    "DC1394_VIDEO_MODE_640x480_RGB8",      //DC1394_VIDEO_MODE_640x480_RGB8",
    "DC1394_VIDEO_MODE_640x480_MONO8",     //DC1394_VIDEO_MODE_640x480_MONO8",
    "DC1394_VIDEO_MODE_640x480_MONO16",    //DC1394_VIDEO_MODE_640x480_MONO16",
    "DC1394_VIDEO_MODE_800x600_YUV422",    //DC1394_VIDEO_MODE_800x600_YUV422",
    "DC1394_VIDEO_MODE_800x600_RGB8",      //DC1394_VIDEO_MODE_800x600_RGB8",
    "DC1394_VIDEO_MODE_800x600_MONO8",     //DC1394_VIDEO_MODE_800x600_MONO8",
    "DC1394_VIDEO_MODE_1024x768_YUV422",   //DC1394_VIDEO_MODE_1024x768_YUV422",
    "DC1394_VIDEO_MODE_1024x768_RGB8",     //DC1394_VIDEO_MODE_1024x768_RGB8",
    "DC1394_VIDEO_MODE_1024x768_MONO8",    //DC1394_VIDEO_MODE_1024x768_MONO8",
    "DC1394_VIDEO_MODE_800x600_MONO16",    //DC1394_VIDEO_MODE_800x600_MONO16",
    "DC1394_VIDEO_MODE_1024x768_MONO16",   //DC1394_VIDEO_MODE_1024x768_MONO16",
    "DC1394_VIDEO_MODE_1280x960_YUV422",   //DC1394_VIDEO_MODE_1280x960_YUV422",
    "DC1394_VIDEO_MODE_1280x960_RGB8",     //DC1394_VIDEO_MODE_1280x960_RGB8",
    "DC1394_VIDEO_MODE_1280x960_MONO8",    //DC1394_VIDEO_MODE_1280x960_MONO8",
    "DC1394_VIDEO_MODE_1600x1200_YUV422",  //DC1394_VIDEO_MODE_1600x1200_YUV422",
    "DC1394_VIDEO_MODE_1600x1200_RGB8",    //DC1394_VIDEO_MODE_1600x1200_RGB8",
    "DC1394_VIDEO_MODE_1600x1200_MONO8",   //DC1394_VIDEO_MODE_1600x1200_MONO8",
    "DC1394_VIDEO_MODE_1280x960_MONO16",   //DC1394_VIDEO_MODE_1280x960_MONO16",
    "DC1394_VIDEO_MODE_1600x1200_MONO16",  //DC1394_VIDEO_MODE_1600x1200_MONO16",
    "DC1394_VIDEO_MODE_EXIF",              //DC1394_VIDEO_MODE_EXIF",
    "DC1394_VIDEO_MODE_FORMAT7_0",         //DC1394_VIDEO_MODE_FORMAT7_0",
    "DC1394_VIDEO_MODE_FORMAT7_1",         //DC1394_VIDEO_MODE_FORMAT7_1",
    "DC1394_VIDEO_MODE_FORMAT7_2",         //DC1394_VIDEO_MODE_FORMAT7_2",
    "DC1394_VIDEO_MODE_FORMAT7_3",         //DC1394_VIDEO_MODE_FORMAT7_3",
    "DC1394_VIDEO_MODE_FORMAT7_4",         //DC1394_VIDEO_MODE_FORMAT7_4",
    "DC1394_VIDEO_MODE_FORMAT7_5",         //DC1394_VIDEO_MODE_FORMAT7_5",
    "DC1394_VIDEO_MODE_FORMAT7_6",         //DC1394_VIDEO_MODE_FORMAT7_6",
    "DC1394_VIDEO_MODE_FORMAT7_7"          //DC1394_VIDEO_MODE_FORMAT7_7
};

static const char* framerateMapping[DC1394_FRAMERATE_NUM] = {
    "DC1394_FRAMERATE_1_875",    //DC1394_FRAMERATE_1_875= 32",
    "DC1394_FRAMERATE_3_75",     //DC1394_FRAMERATE_3_75",
    "DC1394_FRAMERATE_7_5",      //DC1394_FRAMERATE_7_5",
    "DC1394_FRAMERATE_15",       //DC1394_FRAMERATE_15",
    "DC1394_FRAMERATE_30",       //DC1394_FRAMERATE_30",
    "DC1394_FRAMERATE_60",       //DC1394_FRAMERATE_60",
    "DC1394_FRAMERATE_120",      //DC1394_FRAMERATE_120",
    "DC1394_FRAMERATE_240"        //DC1394_FRAMERATE_240
};

/**
 * @brief FirewireFactory::init using libdc1394
 * @return
 *
 * docs see here: http://damien.douxchamps.net/ieee1394/libdc1394/
 */
int FirewireFactory::init(void *) {

    dc1394camera_list_t * list;
    dc1394error_t err;

    d = dc1394_new ();
    if (!d)
    {
        std::cerr << "Cannot create dc1394 instance" << std::endl;
        return EXIT_FAILURE;
    }

    err=dc1394_camera_enumerate (d, &list);
    DC1394_WRN(err, "Failed to enumarete cameras");
    if(err!=DC1394_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (list->num == 0) {
        dc1394_log_error("No cameras found");
        return EXIT_FAILURE;
    }

    this->camera = dc1394_camera_new(d, list->ids[0].guid);
    if (!camera) {
        dc1394_log_error("Failed to initialize camera with guid %llx", list->ids[0].guid);
        return EXIT_FAILURE;
    }
    dc1394_camera_free_list (list);

    std::cout << "Using camera with GUID " <<  camera->guid << std::endl;

    /*-----------------------------------------------------------------------
     *  setup capture
     *-----------------------------------------------------------------------*/

    err=dc1394_video_set_iso_speed(camera, DC1394_ISO_SPEED_400);
    DC1394_WRN(err,"Could not set iso speed");
    if(err != DC1394_SUCCESS)
    {
        this->release();
        return EXIT_FAILURE;
    }

    { //print available video modes
        dc1394video_modes_t p;
        dc1394_video_get_supported_modes(camera, &p);
        std::cout << "Available modes: " << std::endl;
        for(unsigned int i = 0; i < p.num; i++)
        {
            std::cout << "\t" << cameraModeMapping[p.modes[i]- DC1394_VIDEO_MODE_MIN] << std::endl;
        }
    }

    this->videoMode = DC1394_VIDEO_MODE_640x480_RGB8;
    err=dc1394_video_set_mode(camera, videoMode);
    DC1394_WRN(err,"Could not set video mode");
    if(err != DC1394_SUCCESS)
    {
        dc1394video_modes_t p;
        dc1394_video_get_supported_modes(camera, &p);
        std::cout << "Available modes: " << std::endl;
        for(unsigned int i = 0; i < p.num; i++)
        {
            std::cout << "\t" << cameraModeMapping[p.modes[i]- DC1394_VIDEO_MODE_MIN] << std::endl;
        }
        this->release();
        return EXIT_FAILURE;
    }

    this->dc1394Framerate = DC1394_FRAMERATE_15;
    err=dc1394_video_set_framerate(camera, this->dc1394Framerate);
    DC1394_WRN(err, "Could not set framerate\n");
    if(err != DC1394_SUCCESS)
    {
        dc1394framerates_t p;
        dc1394_video_get_supported_framerates(camera, videoMode, &p);
        std::cout << "Available frame rates for this video Mode: " << cameraModeMapping[this->videoMode - DC1394_VIDEO_MODE_MIN] << std::endl;
        for(unsigned int i = 0; i < p.num; i++)
        {
            std::cout << "\t" << framerateMapping[ p.framerates[i] - DC1394_FRAMERATE_MIN] << std::endl;
        }
        this->release();
        return EXIT_FAILURE;
    }

    err=dc1394_capture_setup(camera,4, DC1394_CAPTURE_FLAGS_DEFAULT);
    DC1394_WRN(err, "Could not setup camera-\nmake sure that the video mode and framerate are\nsupported by your camera\n");
    if(err != DC1394_SUCCESS)
    {
        this->release();
        return EXIT_FAILURE;
    }

    /*-----------------------------------------------------------------------
     *  have the camera start sending us data
     *-----------------------------------------------------------------------*/
    err=dc1394_video_set_transmission(this->camera, DC1394_ON);
    DC1394_WRN(err,"Could not start camera iso transmission\n");
    if(err != DC1394_SUCCESS)
    {
        this->release();
        return EXIT_FAILURE;
    }

    uint32_t width;
    uint32_t height;

    dc1394_get_image_size_from_video_mode(this->camera, this->videoMode, &width, &height);
    this->width = width;
    this->height = height;
    dc1394_get_color_coding_from_video_mode(this->camera, this->videoMode, &this->colorCoding);
    dc1394_get_color_coding_bit_size(this->colorCoding, &this->bits);
    dc1394_framerate_as_float(this->dc1394Framerate, &this->frameCount);

    this->prevFrame = NULL;
    this->convert = false;
    this->channels = 3;

    return EXIT_SUCCESS;
}

void FirewireFactory::release()
{
    dc1394_video_set_transmission(this->camera, DC1394_OFF);
    dc1394_capture_stop(this->camera);
    dc1394_camera_free(this->camera);
    dc1394_free (d);
    this->camera = NULL;
    this->d = NULL;
}

int FirewireFactory::getImage(Image<unsigned char, 3> *img, bool internalBuffer)
{

    if(convert)
    {
        if(this->prevFrame)
        {
            delete [] this->prevFrame->image;
            delete this->prevFrame;
            this->prevFrame = NULL;
        }
    } else {
        if(this->prevFrame)
        {
            dc1394_capture_enqueue(this->camera, this->prevFrame);
        }
    }

    dc1394video_frame_t *frame=NULL;
    dc1394error_t err;

    /*-----------------------------------------------------------------------
     *  capture one frame
     *-----------------------------------------------------------------------*/

    err=dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame);
    DC1394_WRN(err, "Could not capture a frame\n");

    if(err != DC1394_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if(this->colorCoding != DC1394_COLOR_CODING_RGB8)
    {
        this->prevFrame = new dc1394video_frame_t;
        this->prevFrame->stride = this->width*this->channels;
        this->prevFrame->image = new unsigned char[this->prevFrame->stride*this->height];
        err = dc1394_convert_to_RGB8(frame->image,
                               this->prevFrame->image,
                               this->width,
                               this->height,
                               frame->yuv_byte_order,
                               this->colorCoding,
                               this->bits);
        if( err != DC1394_SUCCESS)
        {
            return EXIT_FAILURE;
        }
        this->convert = true;
        dc1394_capture_enqueue(this->camera, frame);
    } else {
        this->convert = false;
        this->prevFrame = frame;
    }

    if(img->width != this->width)
    {
        img->width = this->width;
        img->height = this->height;
        img->channels = this->channels;
    }

    //not sure if it's leaking frames:/

    img->data = (char *) this->prevFrame->image;
    img->widthstep = this->prevFrame->stride;

    return EXIT_SUCCESS;
}

}

#endif
