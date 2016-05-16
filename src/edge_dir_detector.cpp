#include <Eigen/Dense>
#include "util/exceptions.h"
#include "edge_dir_detector.h"
#include "util/umfdebug.h"
#include "util/draw.h"

namespace umf {


#ifndef isnan
#define isnan(x) ((x) != (x))
#endif

/**
 * @brief check if the point is inside the image with size with x height
 * @param point
 * @param width
 * @param height
 * @return  if a point represents a real pixel
 */
template <class fptype>
static inline bool checkBounds(Eigen::Matrix<fptype, 2, 1> &point, int width, int height)
{
    return point[0] < static_cast<fptype>(width) && point[1] < static_cast<fptype>(height) && !isnan(point[0]) && !isnan(point[1]) && point[0] >= static_cast<fptype>(0) && point[1] >= static_cast<fptype>(0);
}

template <class fptype>
inline void assignFastSamples(Eigen::Matrix<fptype, 2, 1> origin,
                          Eigen::Matrix<fptype, 2, 1> direction,
                          Eigen::Matrix<fptype, 2, 1> otherDirection,
                          std::vector< Eigen::Matrix<fptype, 2, 1> > &samples)
{
    Eigen::Matrix<fptype, 2, 1> normalOffset = otherDirection*static_cast<fptype>(0.25f);

    Eigen::Matrix<fptype, 2, 1> directionOffset = direction*static_cast<fptype>(0.25f) + origin;

    samples[0] = origin;
    samples[1] = normalOffset + directionOffset;

    samples[2] = -normalOffset + directionOffset;
    samples[3] = directionOffset;
}

template <>
inline void assignFastSamples(Eigen::Matrix<fixp::fixp_8, 2, 1> origin,
                          Eigen::Matrix<fixp::fixp_8, 2, 1> direction,
                          Eigen::Matrix<fixp::fixp_8, 2, 1> otherDirection,
                          std::vector< Eigen::Matrix<fixp::fixp_8, 2, 1> > &samples)
{
    Eigen::Matrix<fixp::fixp_8, 2, 1> normalOffset;
    normalOffset[0] = otherDirection[0] >> 2;
    normalOffset[1] = otherDirection[1] >> 2;

    Eigen::Matrix<fixp::fixp_8, 2, 1> directionOffset;
    directionOffset[0] = (direction[0] >> 2) + origin[0];
    directionOffset[1] = (direction[1] >> 2) + origin[1];

    samples[0] = origin;
    samples[1] = normalOffset + directionOffset;

    samples[2] = -normalOffset + directionOffset;
    samples[3] = directionOffset;
}

template <class fptype>
static inline void assignSamples(Eigen::Matrix<fptype, 2, 1> origin,
                          Eigen::Matrix<fptype, 2, 1> direction,
                          Eigen::Matrix<fptype, 2, 1> otherDirection,
                          std::vector< Eigen::Matrix<fptype, 2, 1> > &samples)
{
    Eigen::Matrix<fptype, 2, 1> normalOffset = otherDirection*static_cast<fptype>(0.15f);
    Eigen::Matrix<fptype, 2, 1> normalOffset2 = otherDirection*static_cast<fptype>(0.3f);

    Eigen::Matrix<fptype, 2, 1> directionOffset1 = direction*static_cast<fptype>(0.45f);
    Eigen::Matrix<fptype, 2, 1> directionOffset2 = direction*static_cast<fptype>(0.4f);
    Eigen::Matrix<fptype, 2, 1> directionOffset3 = direction*static_cast<fptype>(0.3f);

    samples[0] = -normalOffset + origin + directionOffset1;
    samples[1] = normalOffset + origin + directionOffset1;

    samples[2] = -normalOffset2 + origin + directionOffset2;
    samples[3] =                origin + directionOffset2;
    samples[4] = normalOffset2 + origin + directionOffset2;

    samples[5] = -normalOffset + origin + directionOffset3;
    samples[6] = normalOffset + origin + directionOffset3;
    samples[7] = origin + direction*static_cast<fptype>(0.2f);
}


template <class fptype>
static void assignCenteredSamples(Eigen::Matrix<fptype, 2, 1> origin,
                                  Eigen::Matrix<fptype, 2, 1> direction,
                                  Eigen::Matrix<fptype, 2, 1> otherDirection,
                                  std::vector< Eigen::Matrix<fptype, 2, 1> > &samples)
{

    assignSamples(origin, direction, otherDirection, samples);

    samples[8] = origin + otherDirection*static_cast<fptype>(0.1f) + direction*static_cast<fptype>(0.1f);
    samples[9] = origin - otherDirection*static_cast<fptype>(0.1f) + direction*static_cast<fptype>(0.1f);
    samples[10] = origin - otherDirection*static_cast<fptype>(0.1f) - direction*static_cast<fptype>(0.1f);
    samples[11] = origin - otherDirection*static_cast<fptype>(0.1f) + direction*static_cast<fptype>(0.1f);

    samples[12] = origin + otherDirection*static_cast<fptype>(0.25f);
    samples[13] = origin + direction*static_cast<fptype>(0.25f);
    samples[14] = origin - otherDirection*static_cast<fptype>(0.25f);
    samples[15] = origin - direction*static_cast<fptype>(0.25f);
}

template <class T>
static inline T clamp(T x, T a, T b)
{
    return x < a ? a : (x > b ? b : x);
}

/**
 * @brief generate sampling points
 * @tparam NCHAN the number of channels for edge detection - no role here
 * @param p1 first point
 * @param p2 second point
 * @param otherDirection the direction in the detected grid perpendicular to the direction between p1, p2
 * @param samples1 The sampling points in the image for the first point
 * @param samples2 The sampling point in the image for the second point
 */
template <int NCHAN, class fptype>
void EdgeDirDetector<NCHAN, fptype>::getSamplingPoints(const Eigen::Matrix<fptype, 2, 1> &p1,
                                               const Eigen::Matrix<fptype, 2, 1> &p2,
                                               const Eigen::Matrix<fptype, 2, 1> &otherDirection,
                                               std::vector< Eigen::Matrix<fptype, 2, 1> > &samples1,
                                               std::vector< Eigen::Matrix<fptype, 2, 1> > &samples2)
{
    Eigen::Matrix<fptype, 2, 1> p12p2 = p2 - p1;
    Eigen::Matrix<fptype, 2, 1> p22p1 = p1 - p2;
    if(samples1.size() <= EDGE_DIR_SAMPLE_COUNT_FAST)
    {
        samples1.resize(EDGE_DIR_SAMPLE_COUNT_FAST);
        samples2.resize(EDGE_DIR_SAMPLE_COUNT_FAST);
        
        assignFastSamples(p1, p12p2, otherDirection, samples1);
        assignFastSamples(p2, p22p1, otherDirection, samples2);
    }
    else if(samples1.size() <= EDGE_DIR_SAMPLE_COUNT_SIMPLE)
    {
        samples1.resize(EDGE_DIR_SAMPLE_COUNT_SIMPLE);
        samples2.resize(EDGE_DIR_SAMPLE_COUNT_SIMPLE);

        assignSamples(p1, p12p2, otherDirection, samples1);
        assignSamples(p2, p22p1, otherDirection, samples2);
    } else if(samples1.size() <= EDGE_DIR_SAMPLE_COUNT_CENTERED){

        samples1.resize(EDGE_DIR_SAMPLE_COUNT_CENTERED);
        samples2.resize(EDGE_DIR_SAMPLE_COUNT_CENTERED);

        assignCenteredSamples(p1, p12p2, otherDirection, samples1);
        assignCenteredSamples(p2, p22p1, otherDirection, samples2);
    } else {
        throw UMFException();
    }
}


template <int NCHAN, class fptype>
EdgeDirDetector<NCHAN, fptype>::EdgeDirDetector()
{
    int defaultrgb[3] = {3, 1, 1};

    for(int i = 0; i < NCHAN; i++)
    {
        this->fieldDiffThreshold[i] = defaultrgb[i];
    }

    //this->sampleCount = EDGE_DIR_SAMPLE_COUNT_CENTERED;
    //this->scoreDecider = EDGE_DIR_SCORE_DECIDER_CENTERED;
    //this->sampleCount = EDGE_DIR_SAMPLE_COUNT_SIMPLE;
    //this->scoreDecider = EDGE_DIR_SCORE_DECIDER_SIMPLE;
    this->sampleCount = EDGE_DIR_SAMPLE_COUNT_FAST;
    this->scoreDecider = EDGE_DIR_SCORE_DECIDER_FAST;
}

template<int NCHAN, class fptype> template<class T>
Eigen::Matrix<int, NCHAN, 1> EdgeDirDetector<NCHAN, fptype>::getScore(const Image<T, NCHAN> *img,
                                                                      const std::vector< Eigen::Matrix<fptype, 2, 1> > &samples1,
                                                                      const std::vector< Eigen::Matrix<fptype, 2, 1> > &samples2,
                                                                      bool show)
{
    Eigen::Matrix<int, NCHAN, 1> score;
    score.setZero();

#ifdef UMF_DEBUG_COUNT_PIXELS
    UMFDebug *dbg = UMFDSingleton::Instance();
#endif

    Eigen::Matrix<T,NCHAN,1> sampleValue1;
    Eigen::Matrix<T,NCHAN,1> sampleValue2;
    
    Eigen::Array<int,NCHAN,1> diff;

    fptype maxX = static_cast<fptype>(img->width - 1);
    fptype maxY = static_cast<fptype>(img->height - 1);

    for(unsigned int i = 0; i < this->sampleCount; i++)
    {
        img->get2Der(sampleValue1, static_cast<int>(clamp(samples1[i][0], static_cast<fptype>(0), maxX)), static_cast<int>(clamp(samples1[i][1], static_cast<fptype>(0), maxY)));
        img->get2Der(sampleValue2, static_cast<int>(clamp(samples2[i][0], static_cast<fptype>(0), maxX)), static_cast<int>(clamp(samples2[i][1], static_cast<fptype>(0), maxY)));
#ifdef UMF_DEBUG_COUNT_PIXELS
        dbg->addPixels(2);
#endif
        diff = (sampleValue1.template cast<int>() - sampleValue2.template cast<int>()).array();
        score += (diff > this->fieldDiffThreshold).template cast<int>().matrix();
        score -= (diff < -this->fieldDiffThreshold).template cast<int>().matrix();

        if((score.array().abs() > this->scoreDecider).all())
        {
            break;
        }
    }

    return score;
}

template<> template<>
Eigen::Matrix<int, 1, 1> EdgeDirDetector<1, EDGEDIR_FPTYPE>::getScore(const ImageGray *img,
                                                      const std::vector<EdgeDirDetector::Vector2fp> &samples1,
                                                      const std::vector<EdgeDirDetector::Vector2fp> &samples2,
                                                      bool show)
{
    typedef EDGEDIR_FPTYPE fptype;
    Eigen::Matrix<int, 1, 1> score;
    int iscore = 0;

#ifdef UMF_DEBUG_COUNT_PIXELS
    UMFDebug *dbg = UMFDSingleton::Instance();
#endif
    
    short sampleValue1;
    short sampleValue2;
    
    short diff;
    short threshold = this->fieldDiffThreshold(0);

    fptype maxX = static_cast<fptype>(img->width - 1);
    fptype maxY = static_cast<fptype>(img->height - 1);

    for(unsigned int i = 0; i < this->sampleCount; i++)
    {
        sampleValue1 = static_cast<short>(*(img->get2D(
                static_cast<int>(clamp(samples1[i][0], static_cast<fptype>(0), maxX)),
                static_cast<int>(clamp(samples1[i][1], static_cast<fptype>(0), maxY))))
            );
        sampleValue2 = static_cast<short>(*(img->get2D(
                static_cast<int>(clamp(samples2[i][0], static_cast<fptype>(0), maxX)),
                static_cast<int>(clamp(samples2[i][1], static_cast<fptype>(0), maxY))))
            );
#ifdef UMF_DEBUG_COUNT_PIXELS
        dbg->addPixels(2);
#endif

        diff = (sampleValue1 - sampleValue2);
        iscore += static_cast<int>(diff > threshold);
        iscore -= static_cast<int>(diff < -threshold);


#ifdef UMF_DEBUG_DRAW
		if (show) {
			UMFDebug *dbg = UMFDSingleton::Instance();
			Renderer *rend = dbg->getRenderer();
			int lineWidth = 1;
			Eigen::Vector3i color(255, 255, 255 * i / this->sampleCount);
			Eigen::Vector3i equalsColor(0, 0, 255);
			if (diff > threshold) {
				drawArrow(rend, samples2[i].template cast<int>(), samples1[i].template cast<int>(), color, lineWidth);
			}
			else if (diff < -threshold) {
				drawArrow(rend, samples1[i].template cast<int>(), samples2[i].template cast<int>(), color, lineWidth);
			}
			else {
				drawLine(rend, samples2[i].template cast<int>(), samples1[i].template cast<int>(), equalsColor, lineWidth);
			}

			//drawCircle(rend, samples1[i].template cast<int>(), lineWidth, color, -1);
			//drawCircle(rend, samples2[i].template cast<int>(), lineWidth, color, -1);
		}
#endif

        if(iscore > this->scoreDecider || iscore < -this->scoreDecider)
        {
            break;
        }
    }

    score(0) = iscore;
    return score;
}

/**
 * @brief Extract edge directions for matching with database
 * @tparam NCHAN the number of channels used to extarct edge direction
 * @tparam T the image type
 * @param img The input image with NCHAN channels
 * @param pencil1 The first pencil of lines
 * @param pencil2 the second pencil of lines
 * @param show whether to show debug output
 *
 * This function does the following
 *  -# check if the pencils are correctly aligned (one of them is not reversed somehow) and optionally reverse one direction
 *  -# calculate intersection points for field centers (see red dots in the image)
 *  -# for each field center for each neighbour downwards or to the right:
 *      -# sample the area between fields and compare the points
 *      -# for each channel based on the comparison decide the edge direction
 *      -# the edge directions are shown for each channel in the image below
 *      .
 *  -# store these values and optionally show debug output
 *
 * \image html 6_edgedir.png
 */
template<int NCHAN, class fptype> template<class T>
void EdgeDirDetector<NCHAN, fptype>::extract(Image<T, NCHAN> *img,
                                             std::vector<Eigen::Vector3f> &pencil1,
                                             std::vector<Eigen::Vector3f> &pencil2,
                                             ImageGray *mask,
                                             bool show)
{
    //first check if the pencils are aligned in the good way
    //alignement should be:
    // 1 2..
    //1
    //2
    //... and not:
    // n-1 n-2
    //1
    //2
    //to get this calculate the three intersection firstxfirst; lastxfirst; firstxlast
    Eigen::Vector3f intff = pencil1.front().cross(pencil2.front());
    Eigen::Vector3f intlf = pencil1.back().cross(pencil2.front());
    Eigen::Vector3f intfl = pencil1.front().cross(pencil2.back());

    intff /= intff[2];
    intlf /= intlf[2];
    intfl /= intfl[2];

    //Get the the vectors connecting these points
    Eigen::Vector3f p1line = intfl - intff;
    Eigen::Vector3f p2line = intlf - intff;

    if(p1line.cross(p2line)(2) < 0) //bad alignment we want to loop through p1 as rows and p2 as columns
    {
        std::reverse(pencil2.begin(), pencil2.end());
    }

    this->rows = pencil1.size();
    this->cols = pencil2.size();

    this->extractionPoints.resize(this->rows*this->cols);
    std::vector<bool> pointValid(this->rows*this->cols, true);

	bool showDirection = show && false;

    //now we can calculate the field centers
    for(unsigned int row = 0; row < this->rows; row++)
    {
        for(unsigned int col = 0; col < this->cols; col++)
        {
            
            int pindex = row*this->cols + col;
            Eigen::Vector3f intersection = pencil1[row].cross(pencil2[col]);
            intersection /= intersection[2];
            this->extractionPoints[pindex][0] = static_cast<EdgeDirDetector::fpsampling>(intersection[0]);
            this->extractionPoints[pindex][1] = static_cast<EdgeDirDetector::fpsampling>(intersection[1]);
            pointValid[pindex] = checkBounds(this->extractionPoints[pindex], img->width, img->height);
        }
    }

    

    const int verticalOffset = this->rows*this->cols;
    typename Marker<NCHAN>::DirectionType tmp; tmp.setOnes(); tmp *= EDGE_DIRECTION_INVALID;
    this->edgeDirections.resize(2*verticalOffset, tmp);

    std::vector<Vector2fp> samples1(this->sampleCount);
    std::vector<Vector2fp> samples2(this->sampleCount);
#ifdef UMF_DEBUG_COUNT_PIXELS
    UMFDebug *dbg = UMFDSingleton::Instance();
#endif


    for(unsigned int rowI = 0; rowI < this->rows; rowI++)
    {
        for(unsigned int colI = 0; colI < this->cols; colI++)
        {
            int pindex = rowI*this->cols + colI;
            const Vector2fp &current = this->extractionPoints[pindex];
            if(!pointValid[pindex])
            {
                continue;
            }


            //HORIZONTAL EDGE
            if(colI != this->cols - 1) //ignore last column
            {

                const Vector2fp &right = this->extractionPoints[pindex+1];
                if(pointValid[pindex+1])
                {
                    Vector2fp verticalDirection;
                    if(rowI == this->rows - 1)
                    {
                        const Vector2fp &top = this->extractionPoints[pindex - this->cols];
                        verticalDirection = top - current;
                    } else {
                        const Vector2fp &bottom = this->extractionPoints[pindex + this->cols];
                        verticalDirection = bottom - current;
                    }


                    getSamplingPoints(current, right, verticalDirection, samples1, samples2);


                    Eigen::Matrix<int, NCHAN, 1> score = this->getScore(img, samples1, samples2, showDirection);

                    typename Marker<NCHAN>::DirectionType result; result.setZero();

                    result += ((score.array() > this->scoreDecider).template cast<EdgeType>().matrix())*EDGE_DIRECTION_LEFTUP;
                    result += ((score.array() < -this->scoreDecider).template cast<EdgeType>().matrix())*EDGE_DIRECTION_RIGHTDOWN;

                    this->edgeDirections[pindex ] = result;
                }

            }

            //VERTICAL EDGE
            if(rowI != this->rows - 1) //ignore last row
            {
                const Vector2fp &bottom = this->extractionPoints[pindex + this->cols];
                if(pointValid[pindex + this->cols])
                {
                    Vector2fp horizontalDirection;
                    if(colI == this->cols - 1)
                    {
                        const Vector2fp& left = this->extractionPoints[pindex-1];
                        horizontalDirection = left - current;
                    } else {
                        const Vector2fp& right = this->extractionPoints[pindex+1];
                        horizontalDirection = right - current;
                    }

                    getSamplingPoints(current, bottom, horizontalDirection, samples1, samples2);

                    Eigen::Matrix<int, NCHAN, 1> score = this->getScore(img, samples1, samples2, showDirection);

                    typename Marker<NCHAN>::DirectionType result; result.setZero();

                    result += ((score.array() > this->scoreDecider).template cast<EdgeType>().matrix())*EDGE_DIRECTION_LEFTUP;
                    result += ((score.array() < -this->scoreDecider).template cast<EdgeType>().matrix())*EDGE_DIRECTION_RIGHTDOWN;

                    this->edgeDirections[pindex + verticalOffset] = result;
                }
            }
        }
    }

#ifdef UMF_DEBUG_DRAW
    bool showFieldCenters = false;
    bool showEdgeDirections = true;
    if(show)
    {
        //generate pencils going through the corners
        if(showFieldCenters) this->showFieldCenters(mask);
        if(showEdgeDirections) this->showEdgeDirections(mask);
    }
#endif

}

/**
 * @brief Show the extraction field centers relative to which the sampling points are generated
 */
template<int NCHAN, class fptype>
void EdgeDirDetector<NCHAN, fptype>::showFieldCenters(ImageGray *mask)
{
    UMFDebug *dbg = UMFDSingleton::Instance();
    Renderer *rend = dbg->getRenderer();

    if(rend == NULL)
    {
        return;
    }

    Eigen::Vector3i color(168, 25, 25);
    int lineWidth = 10;

    //now we can calculate the field centers
    for(unsigned int row = 0; row < this->rows; row++)
    {
        for(unsigned int col = 0; col < this->cols; col++)
        {
            Eigen::Vector2f cpos = this->extractionPoints[row*this->cols + col].template cast<float>();

            if(mask != NULL)
            {
                if(checkBounds(cpos, mask->width, mask->height))
                {
                    if(*mask->get2D(static_cast<int>(cpos[0]), static_cast<int>(cpos[1])) != 255)
                    {
                        continue;
                    }
                }
            }

            //Eigen::Vector3i color(255 - (row*1.0/this->rows + col*1.0/this->cols)*127, row*255.0/this->rows, col*255.0/this->cols);
            drawCircle(rend, cpos.template cast<int>(), lineWidth, color, -1);
        }
    }
}

/**
 * @brief Show the edge directions for each channel
 */
template<int NCHAN, class fptype>
void EdgeDirDetector<NCHAN, fptype>::showEdgeDirections(ImageGray *mask)
{
    UMFDebug *dbg = UMFDSingleton::Instance();
    Renderer *rend = dbg->getRenderer();

    if(rend == NULL)
    {
        return;
    }

    std::vector<Eigen::Vector3i> colors(NCHAN);

    for(int i = 0; i < NCHAN; i++)
    {
        Eigen::Vector3f hsv(i*360.f/NCHAN, 100.f, 100.f);
        Eigen::Vector3f rgb = hsv2rgb(hsv);
        colors[i] = Eigen::Vector3i(static_cast<int>(rgb[0]*255/100), static_cast<int>(rgb[1]*255/100), static_cast<int>(rgb[2]*255/100));
    }

    int lineWidth = 1;

    //horizontal
    for(unsigned int row = 0; row < this->rows; row++)
    {
        for(unsigned int col = 0; col < this->cols - 1; col++) //ignore last column
        {
            int pindex = row*this->cols + col;
            Eigen::Vector2f current = this->extractionPoints[pindex].template cast<float>();
            Eigen::Vector2f right = this->extractionPoints[pindex+1].template cast<float>();

            if(mask != NULL)
            {
                if(checkBounds(current, mask->width, mask->height) && checkBounds(right, mask->width, mask->height))
                {
                    if(*mask->get2D(static_cast<int>(current[0]), static_cast<int>(current[1])) != 255 || *mask->get2D(static_cast<int>(right[0]), static_cast<int>(right[1])) != 255)
                    {
                        continue;
                    }
                } else {
                    continue;
                }
            }

            Eigen::Vector2f verticalDirection;
            if(row == this->rows - 1)
            {
                Eigen::Vector2f top = this->extractionPoints[pindex - this->cols].template cast<float>();
                verticalDirection = top - current;
            } else {
                Eigen::Vector2f bottom = this->extractionPoints[pindex + this->cols].template cast<float>();
                verticalDirection = bottom - current;
            }

            verticalDirection /= 2*NCHAN;
            typename Marker<NCHAN>::DirectionType &result = this->edgeDirections[pindex];


            Eigen::Vector2f offset = (right - current)*0.3f;
            current += offset;
            right -= offset;

            for(int i = 0; i < NCHAN; i++)
            {
                Eigen::Vector2f vertOffset = static_cast<float>(i - NCHAN/2)*verticalDirection;

                switch(result[i])
                {
                case EDGE_DIRECTION_LEFTUP:
                    drawArrow(rend, (right + vertOffset).template cast<int>(), (current + vertOffset).template cast<int>(), colors[i], lineWidth);
                    break;
                case EDGE_DIRECTION_RIGHTDOWN:
                    drawArrow(rend, (current + vertOffset).template cast<int>(), (right + vertOffset).template cast<int>(), colors[i], lineWidth);
                    break;
                default:
                    drawEquals(rend, (current + vertOffset).template cast<int>(), (right + vertOffset).template cast<int>(), colors[i], lineWidth);
                }
            }
        }
    }

    int verticalOffset = this->rows*this->cols; //size of the horizontal stuff

    //vertical
    for(unsigned int row = 0; row < this->rows - 1; row++) //ignore last row
    {
		for (unsigned int col = 0; col < this->cols; col++)
        {
            int pindex = row*this->cols + col;
            Eigen::Vector2f current = this->extractionPoints[pindex].template cast<float>();
            Eigen::Vector2f down = this->extractionPoints[pindex+this->cols].template cast<float>();;

            if(mask != NULL)
            {
                if(checkBounds(current, mask->width, mask->height) && checkBounds(down, mask->width, mask->height))
                {
                    if(*mask->get2D(static_cast<int>(current[0]), static_cast<int>(current[1])) != 255 || *mask->get2D(static_cast<int>(down[0]), static_cast<int>(down[1])) != 255)
                    {
                        continue;
                    }
                } else {
                    continue;
                }
            }

            Eigen::Vector2f horizontalDirection;
            if(col == this->cols - 1)
            {
                Eigen::Vector2f left = this->extractionPoints[pindex-1].template cast<float>();
                horizontalDirection = left - current;
            } else {
                Eigen::Vector2f right = this->extractionPoints[pindex+1].template cast<float>();
                horizontalDirection = right - current;
            }

            horizontalDirection /= 2*NCHAN;
            typename Marker<NCHAN>::DirectionType &result = this->edgeDirections[pindex+ verticalOffset];

            Eigen::Vector2f posOffset = (down - current)*0.3f;
            current += posOffset;
            down -= posOffset;

            for(int i = 0; i < NCHAN; i++)
            {
                Eigen::Vector2f offset = static_cast<float>(i - NCHAN/2)*horizontalDirection;

                switch(result[i])
                {
                case EDGE_DIRECTION_LEFTUP:
                    drawArrow(rend, (down + offset).template cast<int>(), (current + offset).template cast<int>(), colors[i], lineWidth);
                    break;
                case EDGE_DIRECTION_RIGHTDOWN:
                    drawArrow(rend, (current + offset).template cast<int>(), (down + offset).template cast<int>(), colors[i], lineWidth);
                    break;
                default:
                    drawEquals(rend, (current + offset).template cast<int>(), (down + offset).template cast<int>(), colors[i], lineWidth);
                }
            }
        }
    }

}


template EdgeDirDetector<1, EDGEDIR_FPTYPE>::EdgeDirDetector(); //for grayscale
template EdgeDirDetector<3, EDGEDIR_FPTYPE>::EdgeDirDetector(); //for RGB

template void EdgeDirDetector<1, EDGEDIR_FPTYPE>::extract(ImageGray *img, std::vector<Eigen::Vector3f> &pencil1, std::vector<Eigen::Vector3f> &pencil2, ImageGray* mask, bool show);
template void EdgeDirDetector<3, EDGEDIR_FPTYPE>::extract(ImageRGB *img, std::vector<Eigen::Vector3f> &pencil1, std::vector<Eigen::Vector3f> &pencil2, ImageGray* mask, bool show);

template void EdgeDirDetector<1, EDGEDIR_FPTYPE>::setSampleCount(unsigned int sampleCount);
template void EdgeDirDetector<3, EDGEDIR_FPTYPE>::setSampleCount(unsigned int sampleCount);

}
