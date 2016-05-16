#include "draw.h"
#include "image.h"
#include "../defines.h"

#ifdef UMF_USE_OPENCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "renderer.h"
#endif

#ifdef DRAW_GL
#include "gl_util.h"
#endif

namespace umf {

void drawLine(Renderer *r, Eigen::Vector2i startP, Eigen::Vector2i endP, Eigen::Vector3i lineColor, int width)
{
#ifdef DRAW_CV
    
	IplImage *imgCV = ((OpenCVRenderer *) r)->getCVImg();
	if(imgCV == NULL)
	{
		return;
	}

    cvLine(imgCV, cvPoint(startP[0], startP[1]), cvPoint(endP[0], endP[1]), CV_RGB(lineColor[0], lineColor[1], lineColor[2]), width, CV_AA);
#endif
#ifdef DRAW_GL
#ifdef UMF_ANDROID
#else
	int imgwidth = r->getWidth()/2;
	int imgheight = r->getHeight()/2;
	
	glEnable(GL_DEPTH_TEST);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glEnable(GL_LINE_SMOOTH);
	glLineWidth(width);

	glColor4f(lineColor[0]/255.0f, lineColor[1]/255.0f, lineColor[2]/255.0f, 1.0f);
	const GLfloat line[] = {
		startP[0]*1.0f/imgwidth - 1.0f, 1.0f - startP[1]*1.0f/imgheight, //point A
		endP[0]*1.0f/imgwidth - 1.0f, 1.0f - endP[1]*1.0f/imgheight, //point B
		};

	glVertexPointer(2, GL_FLOAT, 0, line);
	glEnableClientState(GL_VERTEX_ARRAY);
	glDrawArrays(GL_LINES, 0, 2);

	glDisableClientState(GL_VERTEX_ARRAY);
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	
	glFlush();
#endif
	
#endif
}

void drawLineEq(Renderer *r, Eigen::Vector3f line, Eigen::Vector3i drawColor, int width)
{
#ifdef DRAW_CV
	IplImage *imgCV = ((OpenCVRenderer *) r)->getCVImg();
	if(imgCV == NULL)
	{
		return;
	}

    int draw_width = width;

    if(std::abs(line[0]) > std::abs(line[1]))
    {
        //check with top and bottom since the normal is closer to the x axis
        cvLine(imgCV, cvPoint((int) (-line[2]/line[0]), 0),
                cvPoint((int) (-(line[1]*imgCV->height + line[2])/line[0]), imgCV->height),
                CV_RGB(drawColor[0], drawColor[1], drawColor[2]), draw_width, CV_AA);
    } else {
        //check with left and right borders
        cvLine(imgCV, cvPoint(0, (int) (-line[2]/line[1])),
                cvPoint(imgCV->width, (int) (-(line[0]*imgCV->width + line[2])/line[1])),
                CV_RGB(drawColor[0], drawColor[1], drawColor[2]), draw_width, CV_AA);
    }

#endif

}

void drawCircle(Renderer *r, Eigen::Vector2i center, int radius, Eigen::Vector3i lineColor, int width)
{

#ifdef DRAW_CV
    
	IplImage *imgCV = ((OpenCVRenderer *) r)->getCVImg();
	if(imgCV == NULL)
	{
		return;
	}

    cvCircle(imgCV, cvPoint(center[0], center[1]), radius, cvScalar(lineColor[0], lineColor[1], lineColor[2]), width, CV_AA);
#endif

#ifdef DRAW_GL
#ifdef UMF_ANDROID
#else
	int imgwidth = r->getWidth()/2;
	int imgheight = r->getHeight()/2;
	
	glEnable(GL_DEPTH_TEST);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glEnable(GL_POINT_SMOOTH);
	glPointSize(radius);
	glColor4f(lineColor[0]/255.0f, lineColor[1]/255.0f, lineColor[2]/255.0f, 1.0f);
	
	const GLfloat points[] = {
		center[0]*1.0f/imgwidth - 1.0f, 1.0f - center[1]*1.0f/imgheight, 
		};

	glVertexPointer(2, GL_FLOAT, 0, points);
	glEnableClientState(GL_VERTEX_ARRAY);
	glDrawArrays(GL_POINTS, 0, 1);
	glDisableClientState(GL_VERTEX_ARRAY);

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glFlush();
#endif
	
#endif

}



void drawArrow(Renderer *r, Eigen::Vector2i startP, Eigen::Vector2i endP, Eigen::Vector3i lineColor, int width)
{
    Eigen::Vector2i direction = endP - startP;
    Eigen::Vector2i normal(-direction[1], direction[0]);
    if((normal[0]*normal[0] + normal[1]*normal[1]) > 400)
    {
        normal /= 8;
        direction /= 4;
    } else {
        normal /= 4;
        direction /= 2;
    }

    //main line
    drawLine(r, startP, endP, lineColor, width);

    //arrow's
    drawLine(r, endP, endP - direction + normal, lineColor, width);
    drawLine(r, endP, endP - direction - normal, lineColor, width);
}


void drawEquals(Renderer *r, Eigen::Vector2i startP, Eigen::Vector2i endP, Eigen::Vector3i lineColor, int width)
{
    int diffRatio = 4;
    Eigen::Vector2i direction = endP - startP;
    Eigen::Vector2i normal(-direction[1], direction[0]);

    if((normal[0]*normal[0] + normal[1]*normal[1]) < 100)
    {
        normal[0] += normal[0]*width/diffRatio;
        normal[1] += normal[1]*width/diffRatio;
    }

    normal /= diffRatio;

    //two lines
    drawLine(r, startP, endP, lineColor, width);
    drawLine(r, startP + normal, endP + normal, lineColor, width);
}

#ifndef isnan
#define isnan(x) ((x) != (x))
#endif


#ifndef NAN
    static const unsigned long __nan[2] = {0xffffffff, 0x7fffffff};
    #define NAN (*(const float *) __nan)
#endif

/**
 * @brief rgb2hsv converts rgb to hsv
 * @param in rgb color [percent, percent, percent]
 * @return hsv color [degrees <0,360>, percent, percent]
 */
Eigen::Vector3f rgb2hsv(Eigen::Vector3f in)
{
    Eigen::Vector3f out;
    float min, max, delta;

    min = in[0] < in[1] ? in[0] : in[1];
    min = min  < in[2] ? min  : in[2];

    max = in[0] > in[1] ? in[0] : in[1];
    max = max  > in[2] ? max  : in[2];

    out[2] = max;                                // v
    delta = max - min;
    if( max > 0.0 ) {
        out[1] = (delta / max);                  // s
    } else {
        // r = g = b = 0                        // s = 0, v is undefined
        out[1] = 0.0;
        out[0] = NAN;                            // its now undefined
        return out;
    }
    if( in[0] >= max )                           // > is bogus, just keeps compilor happy
        out[0] = ( in[1] - in[2] ) / delta;        // between yellow & magenta
    else
    if( in[1] >= max )
        out[0] = 2.0f + ( in[2] - in[0] ) / delta;  // between cyan & yellow
    else
        out[0] = 4.0f + ( in[0] - in[1] ) / delta;  // between magenta & cyan

    out[0] *= 60.0f;                              // degrees

    if( out[0] < 0.0 )
        out[0] += 360.0f;

    return out;
}

/**
 * @brief rgb2hsv converts hsv to rgb
 * @param in hsv color [degrees <0,360>, percent, percent]
 * @return rgb color [percent, percent, percent]
 */
Eigen::Vector3f hsv2rgb(Eigen::Vector3f in)
{
    float hh, p, q, t, ff;
    long i;
    Eigen::Vector3f out;

    if(in[1] <= 0.0) {       // < is bogus, just shuts up warnings
        if(isnan(in[0])) {   // in[0] == NAN
            out[0] = in[2];
            out[1] = in[2];
            out[2] = in[2];
            return out;
        }
        // error - should never happen
        out[0] = 0.0;
        out[1] = 0.0;
        out[2] = 0.0;
        return out;
    }
    hh = in[0];
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in[2] * (1.0f - in[1]);
    q = in[2] * (1.0f - (in[1] * ff));
    t = in[2] * (1.0f - (in[1] * (1.0f - ff)));

    switch(i) {
    case 0:
        out[0] = in[2];
        out[1] = t;
        out[2] = p;
        break;
    case 1:
        out[0] = q;
        out[1] = in[2];
        out[2] = p;
        break;
    case 2:
        out[0] = p;
        out[1] = in[2];
        out[2] = t;
        break;

    case 3:
        out[0] = p;
        out[1] = q;
        out[2] = in[2];
        break;
    case 4:
        out[0] = t;
        out[1] = p;
        out[2] = in[2];
        break;
    case 5:
    default:
        out[0] = in[2];
        out[1] = p;
        out[2] = q;
        break;
    }
    return out;
}


}
