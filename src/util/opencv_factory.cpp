#include "opencv_factory.h"
#include "../defines.h"

#ifdef UMF_USE_OPENCV_MINIMUM
namespace umf
{

int OpenCVImageFactory::init(void *data)
{
    this->inputFile = false;
    this->inputImage = false;

    if(data == NULL)
    {
        this->cam = cvCaptureFromCAM(0);
    } else {
        CVImageInitStruct *p = (CVImageInitStruct *) data;
        this->inputFile = p->file;

        if(this->inputFile)
        {
            this->cam = cvCaptureFromFile(p->filename.c_str());
        } else {
            this->cam = cvCaptureFromCAM(p->cameraIndex);
        }
    }

    if(this->cam == NULL)
    {
        return EXIT_FAILURE;
    }

	if(!this->inputFile)
	{

	}

    this->frameCount = static_cast<float> (cvGetCaptureProperty(cam, CV_CAP_PROP_FRAME_COUNT));
    if(frameCount == 1)
    {
        inputImage = true;
		imageRead = false;
    }

    if(frameCount < 0)
    {
        //this means it is a webcam -> set it to 15 just in case
        this->frameCount = 15;
    }

    IplImage *frame = cvQueryFrame(cam);
    this->prevImg = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
    cvCvtColor(frame, this->prevImg, CV_BGR2RGB);

    this->width = this->prevImg->width;
    this->height = this->prevImg->height;
    this->channels = this->prevImg->nChannels;

    //if we are processing file skip back to the beginning

    if(this->inputFile && !this->inputImage)
    {
        cvReleaseCapture(&this->cam);
        this->cam = cvCaptureFromFile(((CVImageInitStruct *) data)->filename.c_str());
    }

    return EXIT_SUCCESS;
}

int OpenCVImageFactory::getImage(ImageRGB *img, bool internalBuffer)
{
    if(img == NULL || this->channels != 3)
    {
        return EXIT_FAILURE;
    }

    if(img->width != this->width)
    {
        img->width = this->width;
        img->height = this->height;
        img->channels = this->channels;
    }

    if(this->inputImage)
    {
        img->data = this->prevImg->imageData;
        img->widthstep = this->prevImg->widthStep;
		if (this->imageRead) {
			return EXIT_FAILURE;
		}
		this->imageRead = true;
    } else {
        IplImage *frame = cvQueryFrame(this->cam);

        if(frame){
            if(internalBuffer)
            {
                cvCvtColor(frame, this->prevImg, CV_BGR2RGB);

                img->data = this->prevImg->imageData;
                img->widthstep = this->prevImg->widthStep;
            } else {
                IplImage* cvHead = cvCreateImageHeader(cvGetSize(frame), frame->depth, frame->nChannels);
                cvHead->imageData = img->data;
                cvHead->widthStep = img->widthstep;
                cvCvtColor(frame, cvHead, CV_BGR2RGB);
                cvReleaseImageHeader(&cvHead);
            }
        } else {
            img = NULL;
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}


void OpenCVImageFactory::release()
{
    if(this->prevImg)
    {
        cvReleaseImage(&this->prevImg);
    }
    cvReleaseCapture(&this->cam);
}


}
#endif
