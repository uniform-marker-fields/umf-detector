//#include <vld.h>

#ifndef UMF
#if defined(__WIN32__) || defined(_WIN32)
#define UMF __declspec( dllimport )
#else
#define UMF
#endif
#endif

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string>
#include <argtable2.h>
#include <fstream>
#include <Eigen/Geometry>
#include "defines.h"

#include "umf.h"
#include "util/image.h"
#include "util/stream_factory.h"
#include "util/opencv_factory.h"
#include "util/firewire_factory.h"
#include "util/umfdebug.h"
#include "util/renderer.h"
#include "util/homography2d.h"
#include "util/chromakey.h"
#include "util/calibration.h"

#include "chroma.hpp"
#include "util/native_x.h"

using namespace umf;

#ifdef __OPENCV_OLD_CV_H__


#ifdef _DEBUG
#define CV_DEBUG_SUFFIX "d"
#else 
#define CV_DEBUG_SUFFIX
#endif

#ifdef CV_MAJOR_VERSION
//if compiled in releas mode remove the "d" suffix
#define CV_SHORTVERSION CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION) CV_DEBUG_SUFFIX
#pragma comment(lib, "opencv_core" CV_SHORTVERSION ".lib")
#pragma comment(lib, "opencv_highgui" CV_SHORTVERSION ".lib")
#pragma comment(lib, "opencv_imgproc" CV_SHORTVERSION ".lib")
#pragma comment(lib, "opencv_features2d" CV_SHORTVERSION ".lib")
#pragma comment(lib, "opencv_calib3d" CV_SHORTVERSION ".lib")
#pragma comment(lib, "opencv_video" CV_SHORTVERSION ".lib")
#else
#pragma comment(lib, "opencv_core220.lib")
#pragma comment(lib, "opencv_highgui220.lib")
#pragma comment(lib, "opencv_imgproc220.lib")
#pragma comment(lib, "opencv_features2d220.lib")
#endif

#else
#pragma comment(lib, "cxcore210.lib")
#pragma comment(lib, "cv210.lib")
#pragma comment(lib, "highgui210.lib")
#endif

#pragma comment(lib, "argtable2.lib")
#pragma comment(lib, "glew32.lib")
#pragma comment(lib, "glfw3dll.lib")
#pragma comment(lib, "glu32.lib")
#pragma comment(lib, "OpenGL32.lib")

#pragma comment(lib, "UMFDetector.lib")
#pragma comment(lib, "chroma.lib")


#include <stdio.h>
#include <stdlib.h>

#ifndef M_PI_4
#define M_PI_4 0.78539816339744830961566084581988
#endif


class SuccessLogger {
public:
    SuccessLogger(float fps = 20) {
        this->current = 1;
        this->onepfps = 1000.0f/fps;
        this->detectSum = 0;
    }
    
    void detectStart()
    {
        this->timer.start();
    }

    void addFailure()
    {
        this->detectSum += (std::max)(0.0, this->timer.stop() - this->onepfps);
    }
    
    void addSuccess(int frameId)
    {
        this->detectSum += this->timer.stop();
        int diff = frameId - current - 1;
        //frameDiffs.push_back(diff*onepfps + detectSum);
        frameDiffs.push_back(diff);
        this->detectSum = 0;
        this->current = frameId;
    }

    void store(std::string outDir, std::string filename_short);

private:
    std::vector<int> frameDiffs;
    Timer timer;
    float onepfps;
    int current;
    double detectSum;
};

void SuccessLogger::store(std::string outDir, std::string filename_short)
{
    std::string diff_name = outDir + filename_short + std::string(".diffs.txt");
    std::sort(this->frameDiffs.begin(), this->frameDiffs.end());
    std::fstream successOut(diff_name.c_str(), std::fstream::out);
    double fullSize = this->frameDiffs.size();
    int index = 0;
    for(std::vector<int>::iterator it = this->frameDiffs.begin(); it != this->frameDiffs.end(); it++, index++)
    {
        successOut << *it << " " << index/fullSize << std::endl;
    }
    successOut.close();

    std::fstream successOutPlot((diff_name + std::string(".p")).c_str(), std::fstream::out);

    successOutPlot << "set key right bottom \n";
    successOutPlot << "set auto\n";
    successOutPlot << "\n";


    successOutPlot << "set style line 1 lt 1 lw 1 lc rgb \"red\"\n";
    successOutPlot << "set style line 2 lt 2 lw 1 lc rgb \"green\"\n";
    successOutPlot << "set style line 3 lt 3 lw 1 lc rgb \"blue\"\n";
    successOutPlot << "set style line 4 lt 4 lw 1 lc rgb \"purple\"\n";
    successOutPlot << "set style line 5 lt 5 lw 1 lc rgb \"cyan\"\n";
    successOutPlot << "set style line 6 lt 6 lw 1 lc rgb \"orange\"\n";

    successOutPlot << "set lmargin 5.0\n";
    successOutPlot << "set bmargin 2.5\n";
    successOutPlot << "set rmargin 0.5\n";
    successOutPlot << "set tmargin 0.5\n";

    successOutPlot << "\n";
    successOutPlot << "set xrange[0:20]\n";
    successOutPlot << "set yrange[0: 1]\n";
    successOutPlot << "set xtics 5\n";

    successOutPlot << "\n";
    successOutPlot << "# Make some suitable labels.\n";
    //successOutPlot << "set notitle\n";
    successOutPlot << "set xlabel \"Missed frame count\" offset 0.0, 0.5\n";
    successOutPlot << "set ylabel \"Probability distribution\" offset 2.7, 0.0\n";
    successOutPlot << "set terminal postscript eps enhanced color \"Times-Roman\" 25\n";
    successOutPlot << "set output '| epstopdf --filter --outfile=" << (diff_name + std::string(".pdf")) << "'\n";


    successOutPlot << " plot '" << diff_name << "' using 1:2 title '"<< filename_short << "' with lines\n";

    successOutPlot << "\n";
    successOutPlot.close();
}



char *loadFile(const char* filename)
{
    FILE *f = fopen(filename, "r");
    if(!f)
    {
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *str = new char[fsize + 1];
    fread(str, fsize, 1, f);
    fclose(f);

    str[fsize] = '\0';
    return str;
}


void loadCalibInfo(const char* cameraInfo, Eigen::Matrix3d &cameraMatrix, Eigen::VectorXd &distCoeffs)
{
    std::fstream dataFile(cameraInfo, std::fstream::in);

    if(dataFile.fail())
    {
        return;
    }

    cameraMatrix.setZero();
    distCoeffs.setZero();

    int row = 0;
    while(1)
    {
        if(dataFile.eof()){
            break;
        }

        if(row < 3)
        {

            for(int i = 0; i < 3; i++)
            {
                dataFile >> cameraMatrix(row, i);
            }
            row++;
        }
        else
        {
            for(int i = 0; i < 8; i++)
            {
                dataFile >> distCoeffs[i];
            }
        }
    }
}

int mainCV(int argc, char* argv[])
{
	struct arg_str *arg_alg = arg_str0("aA", "alg", "algorithm", "Choose the algorithm [sog (default), chroma]");
    struct arg_file  *arg_marker    = arg_file1("mM", NULL, "marker", "the file containing the marker information");
    struct arg_file  *arg_ivideo = arg_file0("iI", NULL, "input_video", "the input video or image used for processing");
    struct arg_file  *arg_calib = arg_file0("cC", NULL, "calib", "calibration data for opencv");
	struct arg_lit  *arg_maxprec = arg_lit0("xX", "max", "if maximum precision should be the goal");
	struct arg_lit  *arg_pos = arg_lit0("pP", "positions", "if set, for each frame the position is output instead of success rate");
    struct arg_lit  *arg_help    = arg_lit0("h","help", "print this help and exit");
    struct arg_str *arg_path = arg_str0("dD", "dir", "output_path", "the relative directory to store from the current location without the / symbol");
    struct arg_end  *end     = arg_end(20);
    void* argtable[] = {arg_marker, arg_ivideo, arg_calib, arg_help, arg_path, arg_alg, arg_maxprec, arg_pos, end};
    const char* progname = "detect";

    /* verify the argtable[] entries were allocated sucessfully */
    if (arg_nullcheck(argtable) != 0)
    {
        /* NULL entries were detected, some allocations must have failed */
        printf("%s: insufficient memory\n",progname);
        return 1;
    }

    /* Parse the command line as defined by argtable[] */
    int nerrors = arg_parse(argc,argv,argtable);

    /* special case: '--help' takes precedence over error reporting */
    if (arg_help->count > 0)
    {
        printf("Usage: %s", progname);
        arg_print_syntax(stdout,argtable,"\n");
        printf("This program demonstrates detection with UMF markers\n");
        arg_print_glossary(stdout,argtable,"  %-25s %s\n");
        return 0;
    }


    /* If the parser returned any errors then display them and exit */
    if (nerrors > 0)
    {
        /* Display the error details contained in the arg_end struct.*/
        arg_print_errors(stdout,end,progname);
        printf("Try '%s --help' for more information.\n",progname);

        return -1;
    }

    std::string outpath(".");
    if(arg_path->count > 0)
    {
        outpath = std::string(arg_path->sval[0]);
    }

	bool is_chroma = false;
	if (arg_alg->count > 0) {
		std::string algorithm = std::string(arg_alg->sval[0]);
		if (algorithm.compare("chroma") == 0) {
			is_chroma = true;
		}
	}

	bool max_precision = false;
	if (arg_maxprec->count > 0) {
		max_precision = true;
	}

	bool print_positions = false;
	if (arg_pos->count > 0) {
		print_positions = true;
	}


    ImageFactory *factory = StreamFactory::GetImageFactory(std::string("OPENCV"));
    if(factory == NULL)
    {
        std::cout << "Unable to create factory" << std::endl;
        return 1;
    }

    std::string filename_short = "webcam0";
    CVImageInitStruct sCVIni;
    sCVIni.cameraIndex = 1;
    sCVIni.file = (arg_ivideo->count > 0);
    if(sCVIni.file)
    {
        sCVIni.filename = std::string(arg_ivideo->filename[0]);

        int pos = sCVIni.filename.rfind('/');
        if(pos == std::string::npos)
        {
            filename_short = sCVIni.filename;
        } else {
            filename_short = sCVIni.filename.substr(pos+1);
        }
    }

	if (factory->init((void*)&sCVIni) == EXIT_FAILURE) {
		std::cout << "unable to open video file or webcam device " << sCVIni.filename  << std::endl;
		return 1;
	}


    CvVideoWriter * write = NULL;

    float fps = factory->getFrameCount();
    if(fps > 200 || fps == 0) fps = 30;

	const bool writeVideo = true;
	const bool writeFrames = false;
	const bool calibrate = false;
    if(writeVideo)
    {
		char videobuf[1024];
		sprintf(videobuf, "%s/output.avi", outpath.c_str());
        write = cvCreateVideoWriter(videobuf,
                                    CV_FOURCC('M', 'P', '4', '2'),
                                    fps, cvSize(factory->getWidth(), factory->getHeight()));
    }


    //create an RGB detector
	int detector_flags = /*UMF_FLAG_ITER_REFINE |*/ UMF_FLAG_SUBWINDOWS | UMF_FLAG_SUBPIXEL;// | UMF_FLAG_TRACK_POS;
	if (is_chroma) {
		detector_flags |= UMF_FLAG_CHROMAKEY;
	}
	if (max_precision) {
		detector_flags |= UMF_FLAG_MAX_PRECISION;
	}


    UMFDetector<1> *detector = new UMFDetector<1>(detector_flags); 
    detector->setTrackingFlags(0/*UMF_TRACK_MARKER | UMF_TRACK_SCANLINES | UMF_TRACK_CORNERS*/);

	if (factory->getHeight() > 719) {
		detector->setSubWindowVerticalCount(3);
	}


	float fovy = 49.f;// 47.43f;
    //float fov = 28.05f*static_cast<float>(M_PI)/180.0f;
    //float fov = 30.0f*static_cast<float>(M_PI)/180.0f;
	float fov = fovy*static_cast<float>(M_PI) / 180.0f;
    Eigen::Matrix3d cameraMatrix;
    Eigen::Vector2f imgSize(factory->getWidth(), factory->getHeight());
    float focal = imgSize[1]/(2.0f*tanf(fov/2.0f));
    cameraMatrix << focal, 0, imgSize[0]/2,
            0, focal, imgSize[1]/2,
            0, 0, 1;

    Eigen::VectorXd distCoeffs(8);
    distCoeffs << 0, 0, 0, 0, 0, 0, 0, 0;

    if(arg_calib->count > 0)
    {
        loadCalibInfo(arg_calib->filename[0], cameraMatrix, distCoeffs);
    }

	Calibration* calibCV = new Calibration(imgSize(0), imgSize(1), fovy);

    detector->model.setCameraProperties(cameraMatrix, distCoeffs);
	detector->model.setPnPFlags(PNP_FLAG_COMPUTE_CAMERA | PNP_FLAG_GL_PROJECTION_MV | PNP_FLAG_SWAP_Y | PNP_FLAG_RIGHT_HANDED | PNP_FLAG_FILTER_REPR /* | PNP_FLAG_LOOK_Z_POSITIVE*/);

    if(arg_marker->count > 0)
    {
        const char *fileStr = loadFile(arg_marker->filename[0]);
        if(fileStr)
        {
			if (strcmp((arg_marker->filename[0] + (strlen(arg_marker->filename[0]) - 3)), "xml") == 0) {
				detector->loadMarkerXML(fileStr);
			}
			else {
				detector->loadMarker(fileStr);
			}
            delete [] fileStr;
        } else {
            delete detector;
            return -1;
        }
    } else {
        delete detector;
        return -1;
    }


	if (max_precision) {
		detector->tracker.setSubSampling(Tracker::SUBSAMPLE_1);
		detector->tracker.setMaxPointCount(1000);
		detector->tracker.setMinPointCount(50);
	}
	else {
		detector->tracker.setMaxPointCount(50);
		detector->tracker.setMinPointCount(25);
	}

    UMFDebug *dbg = UMFDSingleton::Instance();

    ImageRGB *img = new ImageRGB(factory->getWidth(), factory->getHeight(), true);
    int frameCounter = 0;
    int successCount = 0;
    std::vector<Eigen::Vector2f> modelPoss;

    //GLRenderer *renderer = GLRendererSingleton::Instance();
    //renderer->init(factory->getWidth(), factory->getHeight());

    OpenCVRenderer *cvrenderer = OpenCVRendererSingleton::Instance();
    cvrenderer->init(factory->getWidth(), factory->getHeight(), is_chroma);

    float directions[2] = {static_cast<float>(M_PI_4), static_cast<float>(3*M_PI_4)};
    detector->edgelDetect.getOrientationFilter().update(directions);

    SuccessLogger slogger(20);

    double cameraPos[3];
    double rotationQuat[4];
    Eigen::Vector3d angles;


    ImageGray *imgGray;
    if((detector->getFlags() & UMF_FLAG_CHROMAKEY) == 0)
    {
        imgGray = new ImageGray(factory->getWidth(), factory->getHeight(), true, factory->getWidth());
    } else {
        imgGray = new ImageGray(factory->getWidth()*3, factory->getHeight());
    }

    while(factory->getImage(img, false) == EXIT_SUCCESS)
    {
        frameCounter++;
        
        cvrenderer->setImageRGB(img);
#ifdef DRAW_GL
        renderer->clear();
        renderer->setImageRGB(img);
        dbg->setRenderer(renderer);
#else
        dbg->setRenderer(cvrenderer);
#endif

        if((detector->getFlags() & UMF_FLAG_CHROMAKEY) == 0)
        {
            convertToGrayscale(img, imgGray);
        } else {

#ifndef UMF_USE_NATIVE
            convertToYCbCr(img, imgGray);
#else
            umfnative::convertToYCbCr(img, imgGray);
#endif
        }

        bool success = false;

		slogger.detectStart();
        try{
            success = detector->update(imgGray, -1.f);
        } catch(DetectionTimeoutException &)
        {
            std::cout << "Timed out" << std::endl;
        }
		if (success)
		{
			//Eigen::Matrix3d pp = detector->model.getHomography();
			//std::cout << "H:\n" << pp << std::endl;
			detector->model.getCameraPosRot(cameraPos, rotationQuat);
			Eigen::Quaterniond p(rotationQuat[0], rotationQuat[1], rotationQuat[2], rotationQuat[3]);
			angles = p.toRotationMatrix().eulerAngles(0, 1, 2);
		}

		if (print_positions) {
			std::cout << frameCounter << " " << cameraPos[0] << " " << cameraPos[1] << " " << cameraPos[2]
				<< " " << rotationQuat[0] << " " << rotationQuat[1] << " " << rotationQuat[2] << " " << rotationQuat[3] << std::endl;
		}

        //std::cout << "Camera: " << detector->model.getCameraMatrix()(0, 0) << " " << detector->model.getCameraMatrix()(1, 1) << " " << detector->model.getCameraMatrix()(0, 2) << " "<< detector->model.getCameraMatrix()(1, 2) << std::endl;

#ifdef UMF_DEBUG_TIMING
		std::vector< std::pair<double, std::string> > timing2;
		dbg->getUniqLog(timing2);
#endif
        //Eigen::Vector2f imgPos(img->width/2, img->height/2);
        //Eigen::Vector2f modelPos;

        //std::vector<Eigen::Vector2f> vimgPos; vimgPos.push_back(imgPos);
        //std::vector<Eigen::Vector2f> vmodelPos; vmodelPos.push_back(modelPos);
        //bool success = detector->detect(imgGray);

        //bool success = detector->detectPosition(imgGray, vimgPos, vmodelPos);
        successCount += (int) success;
        
        if(success)
        {
            slogger.addSuccess(frameCounter);


			if (calibrate && !calibCV->isCalibrated() && detector->model.getCorrespondences().size() > 50/* && (frameCounter % 5) == 0*/)
			{
				calibCV->addImage(detector->model.getCorrespondences());
				if (calibCV->isCalibrated()) {
					char calibbuf[1024];
					sprintf(calibbuf, "%s/camera_calib.txt", outpath.c_str());
					calibCV->saveCalibration(calibbuf);
				}
			}
			/*
            modelPoss.push_back(vmodelPos[0]*6);
            modelPoss.back()[1] -= 300;
            if(modelPoss.size() > 10)
            {
                modelPoss.erase(modelPoss.begin());
            }
			*/
        } else {
            slogger.addFailure();
        }
        

        //convert back to bgr, so we can show it

        /*
        for(unsigned int i = 0; i+1 < modelPoss.size(); i++)
        {
            cvLine(cvimg, cvPoint(modelPoss[i][0], modelPoss[i][1]), cvPoint(modelPoss[i+1][0], modelPoss[i+1][1]), cvScalar(i*20+50, i*10, i*10), 2);
        }
        */

        cvrenderer->update();


#if DRAW_GL
        renderer->update();
#endif
		int key = -1;

#ifdef UMF_DEBUG
        key = cvWaitKey(0);
#else
		key = cvWaitKey(2);
#endif

        //int key = 'a';
        if((char) key == 'q')
        {
            break;
        } else if((char) key == 's')
        {
            cvSaveImage("test/test.png", cvrenderer->getCVImg());
        }

        if(writeFrames)
        {
            char buf[1024];
            int p[3] = { CV_IMWRITE_PNG_COMPRESSION, 9, 0 };
            sprintf(buf, "%s/frame_%d.png", outpath.c_str(), frameCounter);

            if((detector->getFlags() & UMF_FLAG_CHROMAKEY) != 0)
            {

                ImageGray *pmask = new ImageGray(factory->getWidth(), factory->getHeight(), true, factory->getWidth());
                ImageGray *pmap = new ImageGray(factory->getWidth(), factory->getHeight(), true, factory->getWidth());
#ifndef UMF_USE_NATIVE
            	umf::getChromeMaskYCbCr(imgGray, pmask, pmap);
#else
                umfnative::getChromeMaskYCbCr(imgGray, pmask, pmap);
#endif
                IplImage* cpimg = cvCreateImageHeader(cvSize(pmask->width, pmask->height), IPL_DEPTH_8U, 1);
                cpimg->imageData = cpimg->imageDataOrigin = pmask->data;
                cpimg->widthStep = pmask->widthstep;

                IplImage* cpycbcr = cvCreateImageHeader(cvSize(imgGray->width/3, imgGray->height), IPL_DEPTH_8U, 3);
                cpycbcr->imageData = cpycbcr->imageDataOrigin = imgGray->data;
                cpycbcr->widthStep = imgGray->widthstep;

                //chroma::getChromeMask(cpycbcr, cpimg);

				cvSaveImage(buf, cpimg, p);

                cpimg->imageData = cpimg->imageDataOrigin = pmap->data;
                cpimg->widthStep = pmap->widthstep;

                sprintf(buf, "%s/frame_debug_%d.png", outpath.c_str(), frameCounter);
                cvSaveImage(buf, cpimg, p);

                sprintf(buf, "%s/frame_cvout_%d.png", outpath.c_str(), frameCounter);
                cvSaveImage(buf, cvrenderer->getCVImg(), p);

                cvReleaseImageHeader(&cpimg);

                sprintf(buf, "%s/frame_info_%d.txt", outpath.c_str(), frameCounter);
                FILE* frameInfoTXT = fopen(buf, "w");
                fprintf(frameInfoTXT, "Pos: %.2f %.2f %.2f\n", cameraPos[0], cameraPos[1], cameraPos[2]);
                fprintf(frameInfoTXT, "Angles: %.4f %.4f %.4f\n", angles[0], angles[1], angles[2]);
				Eigen::Quaterniond p(rotationQuat[0], rotationQuat[1], rotationQuat[2], rotationQuat[3]);
				p.normalize();
				fprintf(frameInfoTXT, "quaternion: %.4f %.4f %.4f %.4f\n", p.w(), p.x(), p.y(), p.z());

                fclose(frameInfoTXT);

                delete pmask;
            } else {
                IplImage* cpimg = cvCreateImageHeader(cvSize(img->width, img->height), IPL_DEPTH_8U, 3);
                cpimg->imageData = cpimg->imageDataOrigin = img->data;
                cpimg->widthStep = img->widthstep;
                cvCvtColor(cpimg, cpimg, CV_RGB2BGR);

                cvSaveImage(buf, cpimg, p);

				sprintf(buf, "%s/frame_cvout_%d.png", outpath.c_str(), frameCounter);
				cvSaveImage(buf, cvrenderer->getCVImg(), p);

                cvReleaseImageHeader(&cpimg);
            }
        }


        if(write)
        {

            cvWriteFrame(write, cvrenderer->getCVImg());
        }

    }


	if (calibrate && !calibCV->isCalibrated())
	{
		std::cout << "Calibrating hurray" << std::endl;
		calibCV->calibrate(); 
		char calibbuf[1024];
		sprintf(calibbuf, "%s/camera_calib.txt", outpath.c_str());
		calibCV->saveCalibration(calibbuf);
	}

	if (!print_positions) {

		//char outbuf[255];
		//sprintf(outbuf, "%s/", outpath.c_str());
		//slogger.store(outbuf, filename_short);
		std::cout << successCount*100.0 / frameCounter << ";" << frameCounter << ";" << successCount << std::endl;
		//std::cout << "FrameCount: " << frameCounter << " SuccessCount: " << successCount << " Percentual: " << successCount*100.0/frameCounter << std::endl;
	}


#ifdef UMF_DEBUG_TIMING
    std::vector< std::pair<double, std::string> > timing;
    dbg->getUniqLog(timing);

	std::cout << "Timing: " << timing.size() << std::endl;
    for(std::vector< std::pair<double, std::string> >::iterator it = timing.begin(); it != timing.end(); it++)
    {
        std::cout << it->second << ":" << it->first << " ";
    }
    std::cout << std::endl;
#endif
    if(write)
    {
        cvReleaseVideoWriter(&write);
    }
    delete detector;
    factory->release();
    delete factory;
    UMFDSingleton::Release();
    //char p = 0;
    //std::cin >> p;
#ifdef DRAW_GL
    renderer->destroy();
    GLRendererSingleton::Release();
#endif

    cvrenderer->destroy();
    OpenCVRendererSingleton::Release();
    return 0;
}

int mainFW(int argc, char* argv[])
{
    struct arg_file  *arg_marker    = arg_file1("mM", NULL, "marker", "the file containing the marker information");
    struct arg_file  *arg_ivideo = arg_file0("iI", NULL, "input_video", "the input video or image used for processing -ignored for now");
    struct arg_lit  *arg_help    = arg_lit0("h","help", "print this help and exit");
    struct arg_end  *end     = arg_end(20);
    void* argtable[] = {arg_marker, arg_ivideo, arg_help, end};
    const char* progname = "detect";

    /* verify the argtable[] entries were allocated sucessfully */
    if (arg_nullcheck(argtable) != 0)
    {
        /* NULL entries were detected, some allocations must have failed */
        printf("%s: insufficient memory\n",progname);
        return 1;
    }

    /* Parse the command line as defined by argtable[] */
    int nerrors = arg_parse(argc,argv,argtable);

    /* special case: '--help' takes precedence over error reporting */
    if (arg_help->count > 0)
    {
        printf("Usage: %s", progname);
        arg_print_syntax(stdout,argtable,"\n");
        printf("This program demonstrates the use of the argtable2 library\n");
        printf("for parsing command line arguments. Argtable accepts integers\n");
        printf("in decimal (123), hexadecimal (0xff), octal (0o123) and binary\n");
        printf("(0b101101) formats. Suffixes KB, MB and GB are also accepted.\n");
        arg_print_glossary(stdout,argtable,"  %-25s %s\n");
        return 0;
    }


    /* If the parser returned any errors then display them and exit */
    if (nerrors > 0)
    {
        /* Display the error details contained in the arg_end struct.*/
        arg_print_errors(stdout,end,progname);
        printf("Try '%s --help' for more information.\n",progname);

        return -1;
    }


    ImageFactory *factory = StreamFactory::GetImageFactory(std::string("IEEE1394"));
    if(factory == NULL)
    {
        return 1;
    }

    if(factory->init(NULL) != EXIT_SUCCESS)
    {
        return -1;
    }

    //create an RGB detector
    UMFDetector<3> *detector = new UMFDetector<3>(UMF_FLAG_SUBWINDOWS);
    UMFDebug *dbg = UMFDSingleton::Instance();

    dbg->setRenderer(NULL);
    
    ImageRGB *img = new ImageRGB(-1, -1, false, -1);
    while(factory->getImage(img) == EXIT_SUCCESS)
    {
        ImageRGB *imgCopy = new ImageRGB(img->width, img->height, true, img->widthstep);
        memcpy(imgCopy->data, img->data, img->widthstep*img->height);
        detector->detect(img);

        IplImage *cvimg = cvCreateImageHeader(cvSize(img->width, img->height), IPL_DEPTH_8U, img->channels);
        cvimg->widthStep = imgCopy->widthstep;
        cvimg->imageData = cvimg->imageDataOrigin = imgCopy->data;

        cvCvtColor(cvimg, cvimg, CV_RGB2BGR);
        cvShowImage("test", cvimg);
        int key = cvWaitKey(20);
        if((char) key == 'q')
        {
            cvReleaseImageHeader(&cvimg);
            delete imgCopy;
            break;
        }

        cvReleaseImageHeader(&cvimg);
        delete imgCopy;
    }

    factory->release();

#ifdef UMF_DEBUG_TIMING
    std::vector< std::pair<double, std::string> > timing;
    dbg->getUniqLog(timing);
    for(std::vector< std::pair<double, std::string> >::iterator it = timing.begin(); it != timing.end(); it++)
    {
        std::cout << it->second << ":" << it->first << " ";
    }
    std::cout << std::endl;
#endif

    delete detector;
    return 0;
}

int main(int argc, char* argv[])
{
    return mainCV(argc, argv);
    //return mainFW(argc, argv);
    return 0;
}



