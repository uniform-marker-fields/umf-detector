#include "model.h"
#include "util/umfdebug.h"
#include "util/draw.h"
#include "util/corner_detector.h"
#include "util/distort.h"
#include "util/pnp_solver.h"
#include "edge_dir_detector.h"
#include <algorithm>

#ifdef UMF_USE_OPENCV
#include <opencv2/calib3d/calib3d.hpp>
#endif

namespace umf {

template<int NCHAN>
Model<NCHAN>::Model()
{
    this->marker = NULL;
    this->useCornerSearch = true;
    this->useSubPixel = true;

	this->cornerIntensityThreshold = 4; //the intensity difference between fields
	this->cornerAngleThreshold = cosf(70.f * M_PI / 180.f); //allow for 20 degree difference, should be 90 degrees
	this->binsearchDotThreshold = cosf(3.14f / 4); //+-45 degree of freedom when searching for corner
	this->reprFilterTileSizeCutoffMin = 0.2;
	this->reprFilterTileSizeCutoffMax = 0.5;
	this->movementTrackMax = 4; //maximum movement on-screen for tiles

#ifdef UMF_USE_OPENCV
    this->pnp = PnPSolver::GetPnPSolver("OPENCV");
#else 
    this->pnp = PnPSolver::GetPnPSolver("EPNP+LHM");
#endif

    Eigen::Matrix3d cameraMatrix;
    float focal = 700;
    Eigen::Vector2f imgSize(640,480);
    cameraMatrix << focal, 0, imgSize[0]/2,
            0, focal, imgSize[1]/2,
            0, 0, 1;

    Eigen::VectorXd distCoeffs(8);
    distCoeffs << 0, 0, 0, 0, 0, 0, 0, 0;

    //desire hd
    focal = 1200;
    imgSize[0] = 1280;
    imgSize[1] = 720;

    cameraMatrix << focal, 0, imgSize[0]/2,
            0, focal, imgSize[1]/2,
            0, 0, 1;
    /*
    //ADAMOVA kamera <- odstranit ;)
    cameraMatrix << 1939.05, 0, 959.5,
            0, 1939.05, 539.5,
            0, 0, 1;
    distCoeffs << -0.228015, 0.360624, 0.0133008, 0.00272655, -0.169941, 0, 0, 0;
    */
    this->pnp->setCameraMatrix(cameraMatrix);
    this->pnp->setDistortionCoeffs(distCoeffs);

    this->pnpFlags = PNP_FLAG_COMPUTE_CAMERA | PNP_FLAG_GL_PROJECTION_MV | PNP_FLAG_SWAP_Y | PNP_FLAG_RIGHT_HANDED | PNP_FLAG_FILTER_REPR;
}


template<int NCHAN>
Model<NCHAN>::~Model()
{
    if(this->marker)
    {
        delete this->marker;
    }
}


template<int NCHAN>
void Model<NCHAN>::setCameraProperties(Eigen::Matrix3d &camMatrix, Eigen::VectorXd &distCoeffs)
{
    this->pnp->setCameraMatrix(camMatrix);
    this->pnp->setDistortionCoeffs(distCoeffs);
}

template<int NCHAN>
const Eigen::Matrix3d &Model<NCHAN>::getCameraMatrix() const
{
	return this->pnp->getCameraMatrix();
}

template<int NCHAN>
void Model<NCHAN>::addMarker(Marker<NCHAN> *marker)
{
    if(this->marker)
    {
        delete this->marker;
        this->marker = NULL;
    }
    this->marker = marker;
}


static inline int clamp(int x, int a, int b)
{
    return x < a ? a : (x > b ? b : x);
}

template<int NCHAN> template<class T, class fptype>
bool Model<NCHAN>::matchModel(Image<T, NCHAN> *img, unsigned short subW, unsigned short subH,
                              std::vector< typename Marker<NCHAN>::DirectionType > &edgeDir,
                              std::vector< Eigen::Matrix<fptype, 2, 1> > &extractionPoints, Location &loc,
                              bool show)
{

    if(this->marker == NULL)
    {
        return false;
    }

    bool showCorners = false && show;
    bool showCorrespondences = false && show;

    //TODO maybe make a list of the corners instead of a map, that way we can add the position, too
    std::vector<unsigned char> corners;
    loc = this->marker->getLocation(subW, subH, edgeDir, corners);

    bool success = !(loc == Location::invalid);

    if(!success)
    {
        return success;
    }

#ifdef UMF_DEBUG_DRAW
    if(showCorners)
    {
        this->showCorners(corners, extractionPoints, subW, subH);
    }//show
#endif

    //now we have our location and good corners -> find the corresponding corner points in the image, and match them to 3d points
    success = this->matchPoints(img, loc, extractionPoints, corners, subW, subH);

    if(!success)
    {
        return success;
    }

#ifdef UMF_DEBUG_DRAW
    if(showCorrespondences)
    {
        this->showCorrespondences();
    }//show
#endif

    return success;
}

template<int NCHAN>
bool Model<NCHAN>::computeHomography(bool show)
{

#ifdef UMF_DEBUG_DRAW
	if(show)
	{
		this->showCorrespondences(true);
	}
#endif
#ifdef UMF_USE_OPENCV
    cv::Mat srcPoints(this->correspondences.size(), 1, CV_32FC2);
    cv::Mat dstPoints(this->correspondences.size(), 1, CV_32FC2);
    for(unsigned int i = 0; i < this->correspondences.size(); i++)
    {
        srcPoints.at<cv::Point2f>(i) = cv::Point2f(this->correspondences[i].mx, this->correspondences[i].my);
        dstPoints.at<cv::Point2f>(i) = cv::Point2f(this->correspondences[i].px, this->correspondences[i].py);
    }
    cv::Mat HCV = cv::findHomography(srcPoints, dstPoints);

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            this->homography(i, j) = HCV.at<double>(i, j);
        }
    }
    
    //this->homography << 0, 0, 0,  0, 0, 0,  0, 0, 0;
    return true;
#else
    return false;
#endif
}

void dilateMatrix(Eigen::MatrixXi &intmatrix, int size = 2)
{
	int hsize = size/2;
	for(int r = 0; r < intmatrix.rows(); r++)
	{
		for(int c = 0; c < intmatrix.cols(); c++)
		{
			for(int i = -hsize; i <= hsize; i++)
			{
				int cr = r + i;
				if( cr <= 0 || cr >= intmatrix.rows())
				{
					continue;
				}
				for(int j = -hsize; j <= hsize; j++)
				{
					int cc = c+j;
					if( cc < 0 || cc >= intmatrix.cols())
					{
						continue;
					}

					if((intmatrix(cr, cc) & 1) != 0)
					{
						intmatrix(r, c) |= 2;
						i = hsize + 1;
						j = hsize + 1;
						break;
					}
				}
			}
		}
	}
	for(int r = 0; r < intmatrix.rows(); r++)
	{
		for(int c = 0; c < intmatrix.cols(); c++)
		{
			intmatrix(r, c) >>= 1;
		}
	}
}

template<int NCHAN>
void Model<NCHAN>::filterCorrespondences(ImageGray *mask)
{
    int i = this->correspondences.size() - 1;
    int erased = 0;
    for(; i >= 0; i--)
    {
        if(*(mask->get2D(this->correspondences[i].px, this->correspondences[i].py)) != 255)
        {
            this->correspondences.erase(this->correspondences.begin() + i);
            erased++;
        }
    }
}

template<int NCHAN> template<class T>
bool Model<NCHAN>::computeCameraPosition(Image<T, NCHAN> *img, unsigned short subW, unsigned short subH, bool refine, bool show, ImageGray *fmask)
{
    if(fmask != NULL)
    {
        this->filterCorrespondences(fmask);
    }

	if (this->getPnPFlags() & PNP_FLAG_USE_LAST) {
		//check if the reprojection is not too far
		float reprojectionAvg = this->computeReprojectionErrors(Eigen::Vector2i(img->width, img->height));
		reprojectionAvg /= this->getTileSize(Eigen::Vector2i(img->width, img->height));
		if (reprojectionAvg > this->movementTrackMax) {
			this->setPnPFlags(this->getPnPFlags() & (~PNP_FLAG_USE_LAST));
		}
	}

	bool success = this->pnp->computeCameraPose(this->correspondences, Eigen::Vector2i(img->width, img->height), this->pnpFlags);

    if(refine && success)
    {
        Eigen::MatrixXi visited(this->marker->h, this->marker->w);
        Eigen::MatrixXi mask(this->marker->h, this->marker->w);
        
        visited.setZero();
		
		for(CorrespondenceSet::iterator it = this->correspondences.begin(); it != this->correspondences.end(); it++)
		{
			visited(static_cast<int>(it->my) - 1, static_cast<int>(it->mx) - 1) = 1;
		}

        int stepWidth = 2;
        int iterationCount = 5;
        for(int iterCount = 0; iterCount < iterationCount; iterCount++)
        {
            mask = visited;
			dilateMatrix(visited, stepWidth*2);
            mask = visited - mask;

            int count = this->getNewPositions(img, mask);

            if(count == 0)
            {
                break;
            }
        }

        if(fmask != NULL)
        {
            this->filterCorrespondences(fmask);
        }

		success = this->pnp->computeCameraPose(this->correspondences, Eigen::Vector2i(img->width, img->height), this->pnpFlags | PNP_FLAG_USE_LAST);
    }

#ifdef UMF_DEBUG_DRAW
	if (show)
	{
		//this->computeReprojectionErrors(Eigen::Vector2i(img->width, img->height));
		//this->scaleReprojectionByTileSize(Eigen::Vector2i(img->width, img->height));
		//this->showCorrespondences(false);
	}
#endif

	if(success && this->pnpFlags & PNP_FLAG_FILTER_REPR)
	{
		const float filterperc = 1.5f;
		this->computeReprojectionErrors(Eigen::Vector2i(img->width, img->height));
		this->scaleReprojectionByTileSize(Eigen::Vector2i(img->width, img->height));
		std::sort(this->correspondences.begin(), this->correspondences.end(), corrSortByReprjE);
		float med = this->correspondences[this->correspondences.size() / 2].reprojE;
        float filterthresh = (std::min)((std::max)(filterperc*med, reprFilterTileSizeCutoffMin), reprFilterTileSizeCutoffMax);
		for(int i = 0; i < this->correspondences.size(); i++)
		{
			if (this->correspondences[i].reprojE > filterthresh) {
				this->correspondences.resize(i, Correspondence(0, 0, 0, 0, 0));
				break;
			}
		}
		success = this->pnp->computeCameraPose(this->correspondences, Eigen::Vector2i(img->width, img->height), this->pnpFlags | PNP_FLAG_USE_LAST);
	}
	
#ifdef UMF_DEBUG_DRAW
    if(success && show)
    {
		this->computeReprojectionErrors(Eigen::Vector2i(img->width, img->height));
		this->scaleReprojectionByTileSize(Eigen::Vector2i(img->width, img->height));
        this->showCorrespondences(true);
    }
#endif


#ifdef UMF_DEBUG_DRAW
    if(success && show)
    {
        this->showBox();
    }
#endif

    return success;
}


template<int NCHAN> template<class T>
int Model<NCHAN>::getNewPositions(Image<T,NCHAN> *img, const Eigen::MatrixXi &mask)
{
    int counter = 0;
    std::vector<Eigen::Vector3f> modelPoints(4);
    std::vector<Eigen::Vector2f> projPoints(4);
    std::vector<Location> locations;

    std::vector<Eigen::Vector2f> projCorners;
    std::vector<Eigen::Vector3f> modelCorners;
	std::vector<float> scores;

    for(int row = 0; row < mask.rows(); row++)
    {
        for(int col = 0; col < mask.cols(); col++)
        {
            if(mask(row,col) == 0){
                continue;
            }
            counter++;

            Location p; p.c = col; p.r = row; p.rotation = LOC_ROT_0;
            Eigen::Vector2f cornerLoc;
            typename Marker<NCHAN>::DirectionType edges[4];
			float currentScore;

            int type = this->marker->getCornerType(p, cornerLoc, edges, &currentScore);

            if(type == CORNER_TYPE_NONE)
            {
                continue;
            }

            modelPoints[0] = Eigen::Vector3f(cornerLoc[0], cornerLoc[1], 0.0f);
            modelPoints[1] = Eigen::Vector3f(cornerLoc[0] + 1.f, cornerLoc[1], 0.0f);
            modelPoints[2] = Eigen::Vector3f(cornerLoc[0] + 1.f, cornerLoc[1] + 1.f, 0.0f);
            modelPoints[3] = Eigen::Vector3f(cornerLoc[0], cornerLoc[1] + 1.f, 0.0f);

            //we want the position in the 3d model - corners are +0.5f moved
            cornerLoc[0] += 0.5f;
            cornerLoc[1] += 0.5f;

            this->projectPoints(modelPoints, projPoints, Eigen::Vector2i(img->width, img->height), true, true);

            Eigen::Vector2f corner = findMarkerCorner(img, projPoints, type, this->cornerIntensityThreshold, this->binsearchDotThreshold, this->cornerAngleThreshold);
            if((corner.array() >= 0.f).all())
            {
                if(this->useSubPixel)
                {
                    projCorners.push_back(corner);
                    modelCorners.push_back(Eigen::Vector3f(cornerLoc[0], cornerLoc[1], 0));
					scores.push_back(currentScore);
                } else {
                    this->correspondences.push_back(Correspondence(corner[0], corner[1], cornerLoc[0], cornerLoc[1], 0, currentScore));
                }
            }
        }
    }


    //if subpixel search is on
    if(this->useSubPixel)
    {
        counter = findCornersSubpixel(img, projCorners);

        if(counter > 0)
        {
            for(unsigned int ind = 0; ind < projCorners.size(); ind++)
            {
                if((projCorners[ind].array() >= 0.f).all())
                {
                    this->correspondences.push_back(Correspondence(projCorners[ind][0], projCorners[ind][1],
						modelCorners[ind][0], modelCorners[ind][1], modelCorners[ind][2], scores[ind]));
                }
            }
        }
    }
    return counter;
}

template <int NCHAN>
void Model<NCHAN>::showBox()
{
    UMFDebug *dbg = UMFDSingleton::Instance();
    Renderer *rend = dbg->getRenderer();
    if(rend != NULL)
    {
        std::vector<Eigen::Vector2f> points;
        std::vector<Eigen::Vector3f> modelPoints(8);

        int w = 4;
        float wf = static_cast<float>(w);

        modelPoints[0] = Eigen::Vector3f(0, 0, 0);
        modelPoints[1] = Eigen::Vector3f(0, wf, 0);
        modelPoints[2] = Eigen::Vector3f(wf, wf, 0);
        modelPoints[3] = Eigen::Vector3f(wf, 0, 0);
        modelPoints[4] = Eigen::Vector3f(0, 0, -wf);
        modelPoints[5] = Eigen::Vector3f(0, wf, -wf);
        modelPoints[6] = Eigen::Vector3f(wf, wf, -wf);
        modelPoints[7] = Eigen::Vector3f(wf, 0, -wf);

		this->projectPoints(modelPoints, points, Eigen::Vector2i(rend->getWidth(), rend->getHeight()), true, false);

        Eigen::Vector3i lineColor(255, 199, 95);
        Eigen::Vector3i lineColorY(0, 255, 0);
        Eigen::Vector3i lineColorX(255, 0, 0);
        Eigen::Vector3i lineColorZ(0, 0, 255);


        drawLine(rend, points[0].template cast<int>(), points[1].template cast<int>(), lineColorY, 4 ); //y
        drawLine(rend, points[0].template cast<int>(), points[3].template cast<int>(), lineColorX, 4 ); //x
        drawLine(rend, points[0].template cast<int>(), points[4].template cast<int>(), lineColorZ, 4); //Z

        for(int i = 1; i < 3; i++)
        {
            drawLine(rend, points[i].template cast<int>(), points[(i+1)%4].template cast<int>(), lineColor, 4 ); //floor
        }
        for(int i = 1; i < 4; i++)
        {
            drawLine(rend, points[i].template cast<int>(), points[i + 4].template cast<int>(), lineColor, 4); //sides
        }
        for(int i = 0; i < 4; i++)
        {
            drawLine(rend, points[4+ i].template cast<int>(), points[4 + (i+1)%4].template cast<int>(), lineColor, 4);//roof
        }

    }
}

template <int NCHAN>
void Model<NCHAN>::showCorrespondences(bool refine)
{
    UMFDebug *dbg = UMFDSingleton::Instance();
    Renderer *rend = dbg->getRenderer();
    if(rend != NULL)
    {
        Eigen::Vector3i lineColor(0, 40, 255);
        int linewidth = 3;

        if(refine)
        {
            lineColor = Eigen::Vector3i(50, 0, 200);
            linewidth = 2;
        }
        for(CorrespondenceSet::iterator it = this->correspondences.begin(); it != this->correspondences.end(); it++)
        {
			drawCircle(rend, Eigen::Vector2i(it->px, it->py), linewidth*2, (lineColor.template cast<float>()*0.3/it->reprojE).template cast<int>(), 2);
        }
    }
}

template <int NCHAN> template<class fptype>
void Model<NCHAN>::showCorners(std::vector<unsigned char> &corners, std::vector< Eigen::Matrix<fptype, 2, 1> > &extractionPoints, unsigned short subW, unsigned short subH)
{
    UMFDebug *dbg = UMFDSingleton::Instance();
    Renderer *rend = dbg->getRenderer();
    if(rend != NULL)
    {
        int rows = subH - 1;
        int cols = subW - 1;
        for(int row = 0; row < rows; row++)
        {
            for(int col = 0; col < cols; col++)
            {
                int pindex = row*(cols+1) + col;
                unsigned char type = corners[row*cols + col];
                Eigen::Vector3i color(255, 0, 0);
                switch(type)
                {
                case CORNER_TYPE_CROSS:
                    color = Eigen::Vector3i(0, 255, 255);
                    break;
                case CORNER_TYPE_NONE:
                    color = Eigen::Vector3i(0, 0, 255);
                    break;
                case CORNER_TYPE_LEFT_TOP:
                    color = Eigen::Vector3i(255, 255, 0);
                    break;
                case CORNER_TYPE_LEFT_BOTTOM:
                    color = Eigen::Vector3i(255, 0, 0);
                    break;
                case CORNER_TYPE_RIGHT_BOTTOM:
                    color = Eigen::Vector3i(255, 0, 0);
                    break;
                case CORNER_TYPE_RIGHT_TOP:
                    color = Eigen::Vector3i(255, 255, 0);
                    break;
                default:
                    color = Eigen::Vector3i((type - CORNER_TYPE_LEFT_TOP)*63, 200, 0);
                }

                if(type != CORNER_TYPE_NONE)
                {
                    drawCircle(rend,
                               ((extractionPoints[pindex] + extractionPoints[pindex+1] + extractionPoints[pindex + cols + 1] + extractionPoints[pindex + cols + 2])*0.25).template cast<int>(),
                            8, color, 1);
                }
            }//cols
        }//rows
    }//imgDbg
}


template<int NCHAN> template<class T, class fptype>
bool Model<NCHAN>::matchPoints(Image<T, NCHAN> *img, Location &loc,
                               std::vector< Eigen::Matrix<fptype, 2, 1> > &extractionPoints,
                               std::vector<unsigned char> &corners,
                               unsigned short subW,
                               unsigned short subH)
{
    this->correspondences.clear();

    //////////////////////////////////////////////////////////////////////////
    // using corner search
    if(useCornerSearch)
    {
        std::vector<Eigen::Vector2f> projCorners;
        std::vector<Eigen::Vector3f> modelCorners;
		std::vector<float> scores;
        for(int row = 0; row < subW - 1; row++)
        {
            for(int col = 0; col < subH - 1; col++)
            {
                int cornerIndex = row*(subW - 1) + col;
                if(corners[cornerIndex] == CORNER_TYPE_NONE)
                {
                    continue;
                }

                //get model pos
                Location currLoc = loc;
                currLoc.c += col;
                currLoc.r += row;
                changeBackLocation(currLoc, this->marker->w, this->marker->h, 2, 2);

                //now we got the actual position - in order for the model's 0,0 to be at the marker's edge
                Eigen::Vector2f cornerLoc(currLoc.c + 1.0f, currLoc.r + 1.0f);

                //great, now we have the model position, find the more precise image position

                std::vector<Eigen::Vector2f> subPos(4);

                //get the four points around the corner
                for(int subRow = 0; subRow < 2; subRow++)
                {
                    for(int subCol = 0; subCol < 2; subCol++)
                    {
                        int sid = (row + subRow)*subW + col + subCol;
                        subPos[subRow*2 + subCol][0] = extractionPoints[sid][0];
                        subPos[subRow*2 + subCol][1] = extractionPoints[sid][1];
                    }
                }

                std::swap(subPos[2], subPos[3]);

                Eigen::Vector2f corner = findMarkerCorner(img, subPos, corners[cornerIndex], this->cornerIntensityThreshold, this->binsearchDotThreshold, this->cornerAngleThreshold);
                if((corner.array() >= 0).all())
                {
					float score = this->marker->getCornerScore(currLoc);
                    if(this->useSubPixel)
                    {
                        projCorners.push_back(corner);
                        modelCorners.push_back(Eigen::Vector3f(cornerLoc[0], cornerLoc[1], 0));
						scores.push_back(score);
                    } else {
                        correspondences.push_back(Correspondence(corner[0], corner[1], cornerLoc[0], cornerLoc[1], 0, score));
                    }
                }
            }

        }

        //if subpixel search is on
        if(this->useSubPixel)
        {
            int count = findCornersSubpixel(img, projCorners);

            if(count > 0)
            {
                for(unsigned int ind = 0; ind < projCorners.size(); ind++)
                {
                    if(projCorners[ind][0] >= 0)
                    {
                        correspondences.push_back(Correspondence(projCorners[ind][0], projCorners[ind][1], modelCorners[ind][0], modelCorners[ind][1], modelCorners[ind][2], scores[ind]));
                    }
                }
            }
        }
    } else {
        ////////////////////////////////////////////////////////////////////////////////////
        // not using corner search, just get the good points

        Eigen::Vector2f imagePoint;
        for(int row = 0; row < subW - 1; row++)
        {
            for(int col = 0; col < subH - 1; col++)
            {
                bool addPoint = false;
                //test if some of the corners around are good points and it's OK to add this
                for(int subRow = 0; subRow < 2; subRow++)
                {
                    for(int subCol = 0; subCol < 2; subCol++)
                    {
                        int y = (std::max)((std::min)(row - subRow,subH - 1), 0);
                        int x = (std::max)((std::min)(col - subCol,subW - 1), 0);

                        int cornerIndex = y*(subW - 1) + x;
                        if(corners[cornerIndex] != CORNER_TYPE_NONE)
                        {
                            addPoint = true;
                            subCol = subRow = 3;
                            break;
                        }
                    }
                }

                if(!addPoint)
                {
                    continue;
                }

                //get model pos
                Location currLoc = loc;
                currLoc.c += col;
                currLoc.r += row;
                changeBackLocation(currLoc, this->marker->w, this->marker->h);

                const Eigen::Matrix<fptype, 2, 1> &imagePoint = extractionPoints[row*subW + col];
                //shift by 0.5f so that the origin is at the marker's edge, not the field center
                this->correspondences.push_back(Correspondence(imagePoint[0], imagePoint[1], currLoc.c + 0.5f, currLoc.r + 0.5f, 0.0f,
					this->marker->getCornerScore(currLoc)));
            }
        }
    }

    return this->correspondences.size() > 4; //the minimum we need for camera localization
}

template<int NCHAN>
void Model<NCHAN>::projectPoints(std::vector<Eigen::Vector3f> &modelPoints, std::vector<Eigen::Vector2f> &imagePoints, Eigen::Vector2i imageSize, bool distort, bool swap)
{
    std::vector<Eigen::Vector2f> points;
    Eigen::Matrix4d &mv = this->pnp->getWorldTransformMatrix();
    Eigen::Matrix3d &camera = this->pnp->getCameraMatrix();
    Eigen::Vector4d currentPos;
    Eigen::Vector3d projected;
    double k = 1;
    for(unsigned int i = 0; i < modelPoints.size(); i++)
    {
        //switch x and y, cos we do that
        Eigen::Vector4d pos3d(modelPoints[i][0], modelPoints[i][1], modelPoints[i][2], 1.0);
        if(swap)
        {
            //when we want the mask coordinate system, not the real 3d coordinate system, we have to do this
            std::swap(pos3d[0], pos3d[1]);
        }
        currentPos = mv*pos3d;

        if(distort) //the distortion will compute the multiplication with the camer matrix, so we only need the transformed point
        {
            //the current position probably is still going to be normalized, since we don't do any projection in the model view matrix
            //but just in case:
            currentPos *= 1.0/currentPos[3];

            //now that we have this if i get this right we just do 1/z in projection, so (the camera matrix has 1 at the diagonal for the z point
            k = currentPos[2] ? 1./currentPos[2] : 1;
            points.push_back(Eigen::Vector2f(currentPos[0]*k, currentPos[1]*k));
        } else {
            //if we are going to just project it move everything to the imagePoints at once
            projected = camera*Eigen::Vector3d(currentPos[0], currentPos[1], currentPos[2]);

            k = projected[2] ? 1./projected[2] : 1;

            imagePoints.push_back(Eigen::Vector2f(projected[0]*k, projected[1]*k));

        }

    }

    //now we do some kind of distorting based on opencl. It will also scale the points based on camera matrix
    if(distort)
    {
        distortPoints(points, imagePoints, camera.template cast<float>(), this->pnp->getDistortionCoeffs().template cast<float>());
    }

    //transform back the y
    if(this->pnpFlags & PNP_FLAG_SWAP_Y)
    {
        for(unsigned int i = 0; i < imagePoints.size(); i++)
        {
            imagePoints[i](1) = imageSize[1] - imagePoints[i](1);
        }
    }
}

template<int NCHAN>
int Model<NCHAN>::getCameraPosRot(double cameraPos[3], double rotationQuat[4])
{
    Eigen::Vector3d ecameraPos = this->pnp->getCameraPos();
    Eigen::Quaterniond ecameraQuat = this->pnp->getCameraQuaternion();

    cameraPos[0] = ecameraPos[0];
    cameraPos[1] = ecameraPos[1];
    cameraPos[2] = ecameraPos[2];

	if (this->pnpFlags & PNP_FLAG_LOOK_Z_POSITIVE) {
		Eigen::Quaterniond rotate180(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
		ecameraQuat = ecameraQuat*rotate180;
	}

    rotationQuat[1] = ecameraQuat.x();
    rotationQuat[2] = ecameraQuat.y();
    rotationQuat[3] = ecameraQuat.z();
    rotationQuat[0] = ecameraQuat.w();
    return EXIT_SUCCESS;
}

template<int NCHAN>
int Model<NCHAN>::getWorldPosRot(double cameraPos[3], double rotationQuat[4])
{
    Eigen::Vector3d eWorldPos = this->pnp->getWorldPos();
    Eigen::Quaterniond eWorldQuat = this->pnp->getWorldQuaternion();

    cameraPos[0] = eWorldPos[0];
    cameraPos[1] = eWorldPos[1];
    cameraPos[2] = eWorldPos[2];

    rotationQuat[1] = eWorldQuat.x();
    rotationQuat[2] = eWorldQuat.y();
    rotationQuat[3] = eWorldQuat.z();
    rotationQuat[0] = eWorldQuat.w();
    return EXIT_SUCCESS;
}

template<int NCHAN>
void Model<NCHAN>::updateMaskTracker(Eigen::Vector2i imgSize)
{
    std::vector<Eigen::Vector3f> modelPoints;
    modelPoints.push_back(Eigen::Vector3f(-1, -1, 0));
    modelPoints.push_back(Eigen::Vector3f(this->marker->w + 1, -1, 0));
    modelPoints.push_back(Eigen::Vector3f(this->marker->w + 1, this->marker->h + 1, 0));
    modelPoints.push_back(Eigen::Vector3f(-1, this->marker->h + 1, 0));
    std::vector<Eigen::Vector2f> projPoints;
    this->projectPoints(modelPoints, projPoints, imgSize, false, true);

    this->maskTracker.update(projPoints);

}

template<int NCHAN>
float Model<NCHAN>::computeReprojectionErrors(const Eigen::Vector2i &imgSize)
{
	std::vector<Eigen::Vector2f> points; points.reserve(this->correspondences.size());
	std::vector<Eigen::Vector3f> modelPoints; modelPoints.reserve(this->correspondences.size());

	for (CorrespondenceSet::iterator it = this->correspondences.begin(); it != this->correspondences.end(); it++)
	{
		modelPoints.push_back(Eigen::Vector3f(it->mx, it->my, it->mz));
	}

	this->projectPoints(modelPoints, points, imgSize, true, true);

	int counter = 0;
	float avg = 0;

	for (CorrespondenceSet::iterator it = this->correspondences.begin(); it != this->correspondences.end(); it++, counter++)
	{
		float diffx = it->px - points[counter][0];
	    float diffy = it->py - points[counter][1];
		it->reprojE = std::sqrt(diffx*diffx + diffy*diffy);
		avg += it->reprojE;
	}

	return avg / counter;
}

template<int NCHAN>
void Model<NCHAN>::scaleReprojectionByImageSize(const Eigen::Vector2i &imgSize)
{
	float imgdiam = 1.0f / std::sqrt(static_cast<float>(imgSize.dot(imgSize)));
	for (CorrespondenceSet::iterator it = this->correspondences.begin(); it != this->correspondences.end(); it++)
	{
		it->reprojE *= imgdiam;
	}
}

template<int NCHAN>
float Model<NCHAN>::getTileSize(const Eigen::Vector2i &imgSize) {

	std::vector<Eigen::Vector2f> points(4);
	std::vector<Eigen::Vector3f> modelPoints(4);

	CorrespondenceSet::iterator bestCorrespondence = this->correspondences.begin();
	modelPoints[0] = Eigen::Vector3f(bestCorrespondence->mx, bestCorrespondence->my, bestCorrespondence->mz);
	modelPoints[1] = Eigen::Vector3f(bestCorrespondence->mx + 1, bestCorrespondence->my, bestCorrespondence->mz);
	modelPoints[2] = Eigen::Vector3f(bestCorrespondence->mx, bestCorrespondence->my + 1, bestCorrespondence->mz);
	modelPoints[3] = Eigen::Vector3f(bestCorrespondence->mx + 1, bestCorrespondence->my + 1, bestCorrespondence->mz);

	this->projectPoints(modelPoints, points, imgSize, true, true);

	Eigen::Vector2f diagonal1 = points[3] - points[0];
	Eigen::Vector2f diagonal2 = points[2] - points[1];
	const float sqrt2 = std::sqrt(2);
	float maxWidth = (std::max)(diagonal1.norm(), diagonal2.norm()) / sqrt2;
	return maxWidth;
}

template<int NCHAN>
void Model<NCHAN>::scaleReprojectionByTileSize(const Eigen::Vector2i &imgSize) {
	float maxWidthInvert = 1 / this->getTileSize(imgSize);

	for (CorrespondenceSet::iterator it = this->correspondences.begin(); it != this->correspondences.end(); it++)
	{
		it->reprojE *= maxWidthInvert;
	}
}


//INSTANCING
template Model<1>::Model();
template Model<3>::Model();


template Model<1>::~Model();
template Model<3>::~Model();

template void Model<1>::addMarker(Marker<1> *marker);
template void Model<3>::addMarker(Marker<3> *marker);

template int Model<1>::getCameraPosRot(double pos[3], double quat[4]);
template int Model<3>::getCameraPosRot(double pos[3], double quat[4]);
template int Model<1>::getWorldPosRot(double pos[3], double quat[4]);
template int Model<3>::getWorldPosRot(double pos[3], double quat[4]);

template void Model<1>::setCameraProperties(Eigen::Matrix3d &cameraMatrix, Eigen::VectorXd &distCoeffs);
template void Model<3>::setCameraProperties(Eigen::Matrix3d &cameraMatrix, Eigen::VectorXd &distCoeffs);

template const Eigen::Matrix3d & Model<1>::getCameraMatrix() const;
template const Eigen::Matrix3d & Model<3>::getCameraMatrix() const;

template bool Model<1>::matchModel(ImageGray *img, unsigned short subW, unsigned short subH,
                                   std::vector< typename Marker<1>::DirectionType > &edgeDir,
                                   std::vector< Eigen::Matrix<EDGEDIR_FPTYPE, 2, 1> > &extractionPoints, Location &loc, bool show);
template bool Model<3>::matchModel(ImageRGB *img, unsigned short subW, unsigned short subH,
                                   std::vector< typename Marker<3>::DirectionType > &edgeDir,
                                   std::vector< Eigen::Matrix<EDGEDIR_FPTYPE, 2, 1> > &extractionPoints, Location &loc, bool show);

template bool Model<1>::computeHomography(bool show);
template bool Model<3>::computeHomography(bool show);
template bool Model<1>::computeCameraPosition(ImageGray *img, unsigned short subW, unsigned short subH, bool refine, bool show, ImageGray* mask);
template bool Model<3>::computeCameraPosition(ImageRGB *img, unsigned short subW, unsigned short subH, bool refine, bool show, ImageGray* mask);

template void Model<1>::updateMaskTracker(Eigen::Vector2i imgSize);
template void Model<3>::updateMaskTracker(Eigen::Vector2i imgSize);


template void Model<1>::projectPoints(std::vector<Eigen::Vector3f> &modelPoints, std::vector<Eigen::Vector2f> &imagePoints, Eigen::Vector2i imageSize, bool distort, bool swap);
template void Model<3>::projectPoints(std::vector<Eigen::Vector3f> &modelPoints, std::vector<Eigen::Vector2f> &imagePoints, Eigen::Vector2i imageSize, bool distort, bool swap);

template float Model<1>::computeReprojectionErrors(const Eigen::Vector2i &imgSize);
template float Model<3>::computeReprojectionErrors(const Eigen::Vector2i &imgSize);


template float Model<1>::getTileSize(const Eigen::Vector2i &imgSize);
template float Model<3>::getTileSize(const Eigen::Vector2i &imgSize);
} //namespace
