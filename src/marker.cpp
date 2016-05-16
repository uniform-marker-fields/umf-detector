#include "marker.h"
#include <iostream>
#include <map>
#include <iterator>
#include <set>

namespace umf {

Location Location::invalid = {
    -1,//std::numeric_limits<unsigned>::max(),
    -1,//std::numeric_limits<unsigned>::max(),
    -1,//std::numeric_limits<unsigned>::max()
};

void MarkerType::decode(int code)
{
    this->color = (code & TYPE_COLOR_BIT) > 0;
    this->torus = (code & TYPE_TORUS_BIT) > 0;
    this->range = TYPE_RANGE_START + ((code & TYPE_RANGE_BITS) >> TYPE_RANGE_BIT_START);
}

int MarkerType::encode()
{
    int bit_torus = this->torus ? TYPE_TORUS_BIT: 0;
    int bit_color = this->color ? TYPE_COLOR_BIT: 0;
    int bit_range = (this->range < TYPE_RANGE_START ? 0 : this->range - TYPE_RANGE_START);
    bit_range <<= TYPE_RANGE_BIT_START;
    return bit_torus | bit_color | bit_range;
}

template <int NCHAN>
bool Marker<NCHAN>::setField(std::vector<unsigned short> &data)
{
    if((int) data.size() != this->w*this->h)
    {
        return false;
    }

	mMap.reserve(data.size());
	copy(data.begin(), data.end(), std::back_inserter(mMap));

    Marker<NCHAN>::DirectionType tmpZero; tmpZero.setZero();


    unsigned int verticalOffset = this->w*this->h;
    this->horizVert.assign(2*verticalOffset, tmpZero);

    for(unsigned int rowI = 0; rowI < this->h; rowI++)
    {
        for(unsigned int colI = 0; colI < this->w; colI++)
        {
            unsigned int pindex = rowI*this->w + colI;
            Eigen::Array<int, NCHAN, 1> currentC = this->colors[data[pindex]].template cast<int>().array();
            if((int)colI < this->w - 1) //ignore last column
            {
                Eigen::Array<int, NCHAN, 1> rightC = this->colors[data[pindex + 1]].template cast<int>().array();
                this->horizVert[pindex] =
                        ((currentC < rightC).template cast<EdgeType>() * EDGE_DIRECTION_RIGHTDOWN + ((currentC > rightC).template cast<EdgeType>() * EDGE_DIRECTION_LEFTUP)).matrix();
            }
            if((int)rowI < this->h - 1) //ignore last row
            {
                Eigen::Array<int, NCHAN, 1> bottomC = this->colors[data[pindex + this->w]].template cast<int>().array();
                this->horizVert[pindex + verticalOffset] =
                        ((currentC < bottomC).template cast<EdgeType>() * EDGE_DIRECTION_RIGHTDOWN + ((currentC > bottomC).template cast<EdgeType>() * EDGE_DIRECTION_LEFTUP)).matrix();
            }
        }
    }

    //fill in corner types
    this->cornerType.assign((this->w - 1)*(this->h - 1), CORNER_TYPE_NONE);
	this->scores.assign((this->w - 1)*(this->h - 1), 0.f);
	
	float invscale = 1.0f/(128.f);

    for(int rowI = 0; rowI < this->h-1; rowI++)
    {
        for(int colI = 0; colI < this->w-1; colI++)
        {
			e::Matrix<float, NCHAN, 1> ccol[4];
			ccol[0] = this->colors[data[this->w*rowI + colI]].template cast<float>()*invscale;
			ccol[1] = this->colors[data[this->w*rowI + colI + 1]].template cast<float>()*invscale;
			ccol[2] = this->colors[data[this->w*(rowI + 1) + colI + 1]].template cast<float>()*invscale;
			ccol[3] = this->colors[data[this->w*(rowI + 1) + colI]].template cast<float>()*invscale;
			float diffs[4];
			diffs[0] = std::min((ccol[1] - ccol[0]).squaredNorm(), (ccol[2]- ccol[1]).squaredNorm());
			diffs[1] = std::min((ccol[2] - ccol[1]).squaredNorm(), (ccol[3]- ccol[2]).squaredNorm());
			diffs[2] = std::min((ccol[3] - ccol[2]).squaredNorm(), (ccol[0]- ccol[3]).squaredNorm());
			diffs[3] = std::min((ccol[0] - ccol[3]).squaredNorm(), (ccol[1]- ccol[0]).squaredNorm());

			float score = std::max( std::max( diffs[0], diffs[1]), std::max( diffs[2] , diffs[3]));
			this->scores[(this->w - 1)*rowI + colI] = std::min(score, 1.0f);


            int pindex = rowI*this->w + colI;

            DirectionType values[4];
            values[0] = this->horizVert[pindex];
            values[1] = this->horizVert[pindex + 1 + verticalOffset];
            values[2] = this->horizVert[pindex + this->w];
            values[3] = this->horizVert[pindex + verticalOffset];
            std::vector<unsigned char> zeros;
            DirectionType tmpZero; tmpZero.setZero();

            for(int i = 0; i < 4; i++)
            {
                if((values[i].array() == 0).all())
                {
                    zeros.push_back(i);
                }
            }

            int cornerIndex = rowI*(this->w - 1) + colI;

            if(zeros.size() > 2)
            {
                this->cornerType[cornerIndex] = CORNER_TYPE_NONE;
                continue;
            }

            int localCornerIndex = -1;

            if(zeros.size() == 2)
            {
                int psum = zeros[0] + zeros[1];
                if((zeros[0] + zeros[1]) % 2 == 0) // this means the equals are not next to each other - straight line at the corner
                {
                    this->cornerType[cornerIndex] = CORNER_TYPE_NONE;
                    continue;
                }

                psum = 2*psum + zeros[1] - zeros[0];
                switch(psum)
                {
                case 3: localCornerIndex = 3; break;
                case 7: localCornerIndex = 0; break;
                case 11: localCornerIndex = 1; break;
                case 9: localCornerIndex = 2; break;
                }
            } else if(zeros.size() == 1)
            {
                const int mapping[] = {0, 1, this->w + 1, this->w};
                
                int localCornerIndex1 = (zeros[0] + 3) % 4;
                int localCornerIndex2 = (zeros[0] + 2) % 4;

                Eigen::Array<int, NCHAN, 1> colors[3];
                colors[0] = this->colors[data[pindex + mapping[zeros[0]]]].template cast<int>().array();
                colors[1] = this->colors[data[pindex + mapping[localCornerIndex1]]].template cast<int>().array();
                colors[2] = this->colors[data[pindex + mapping[localCornerIndex2]]].template cast<int>().array();

                if( (colors[2] - colors[0]).abs().sum() > (colors[1] - colors[0]).abs().sum() )
                {
                    localCornerIndex = localCornerIndex2;
                } else {
                    localCornerIndex = localCornerIndex1;
                }
            } else {
                this->cornerType[cornerIndex] = CORNER_TYPE_CROSS;
                continue;
            }

            this->cornerType[cornerIndex] = CORNER_TYPE_LEFT_TOP + localCornerIndex;
        }
    }


    return this->init();
}

template<int NCHAN>
bool Marker<NCHAN>::init()
{
    if(this->decisionTree != NULL)
    {
        delete this->decisionTree;
        this->decisionTree = NULL;
    }

    this->decisionTree = new DecisionTree<NCHAN>();

    //initialize decision path
    bool unique = true;

    for(unsigned short rotation = LOC_ROT_0; rotation < LOC_ROT_COUNT; ++rotation)
    {
        int width = this->w;
        int height = this->h;
        if(rotation%2 == 1)
        {
            std::swap(width, height);
        }

        std::vector< Marker<NCHAN>::DirectionType > hv = this->horizVert;

        this->rotatePart(rotation, this->w, this->h, hv);

        std::vector<unsigned int> indexes;
        this->getPathIndexes(width, height, indexes);

        Marker<NCHAN>::DirectionType p; p.setZero();
        std::vector< Marker<NCHAN>::DirectionType > path(indexes.size(), p);

        for(int r = 0; r < (height-this->nunique+1); ++r)
        {
            for(int c = 0; c < (width-this->nunique+1); ++c)
            {
                Location l = { r, c, rotation };

                this->getPath(indexes, hv, path, l.r*width + l.c);

                unique = unique && this->decisionTree->addPath(l, path);

                if(unique == false)
                {
                    Location second = this->decisionTree->getLocation(path);
                    std::cout << "second pos: " << second.r << ", " << second.c << ", " << second.rotation << std::endl;
                    std::cout << "first pos: " << l.r << ", " << l.c << ", " << l.rotation << std::endl;
                }

                assert(unique);
            }
        }
    }

    if(decisionTreeMinHeight > 0)
    {
        decisionTree->simplify(2*this->nunique*(this->nunique - 1) - this->decisionTreeMinHeight);
    }

    return unique;
}


template <int NCHAN>
bool Marker<NCHAN>::rotatePart(int rotation,
                               unsigned short subW,
                               unsigned short subH,
                               std::vector< typename Marker<NCHAN>::DirectionType > &horiz_vert) const
{


    if((int) horiz_vert.size() != 2*subW*subH)
    {
        assert(false);
        return false;
    }

    int directionOffset = subW*subH;

    Marker<NCHAN>::DirectionType tmp;
    tmp.setZero();

    std::vector< Marker<NCHAN>::DirectionType > subHorizontal(directionOffset, tmp);
    std::vector< Marker<NCHAN>::DirectionType > subVertical(directionOffset, tmp);

    if(rotation == LOC_ROT_0)
    {
        //pass everything stayes the same
        return true;
    } else if(rotation == LOC_ROT_180)
    {

        for(int row = 0; row < subH; row++)
        {
            for(int col = 0; col < subW - 1; col++) //ignore last column for horizontal
            {
                this->mapEdgeDirection(&(horiz_vert[(subH - row - 1)*subW + (subW - col - 2)]),&tmp);
                subHorizontal[row*subW + col] = tmp;
            }
        }

        for(int row = 0; row < subH - 1; row++) //ignore last row
        {
            for(int col = 0; col < subW; col++)
            {

                this->mapEdgeDirection(&(horiz_vert[(subH - row - 2)*subW + (subW - col - 1) + directionOffset]),&tmp);
                subVertical[row*subW + col] = tmp;
            }
        }

        //copy back the data
        std::copy(subHorizontal.begin(), subHorizontal.end(), horiz_vert.begin());
        std::copy(subVertical.begin(), subVertical.end(), horiz_vert.begin() + directionOffset);

    } else if(rotation == LOC_ROT_270)
    {
        std::swap(subW, subH);

        for(int row = 0; row < subH; row++)
        {
            for(int col = 0; col < subW - 1; col++) //ignore last column for horizontal
            {
                this->mapEdgeDirection(&horiz_vert[(subW - col - 2)*subH + row + directionOffset], &tmp);
                subHorizontal[row*subW + col] = tmp;
            }
        }

        for(int row = 0; row < subH - 1; row++) //ignore last row
        {
            for(int col = 0; col < subW; col++)
            {
                subVertical[row*subW + col] = horiz_vert[(subW - col - 1)*subH + row];
            }
        }

        //copy back the data
        std::copy(subHorizontal.begin(), subHorizontal.end(), horiz_vert.begin());
        std::copy(subVertical.begin(), subVertical.end(), horiz_vert.begin() + subHorizontal.size());

    } else if(rotation == LOC_ROT_90)
    {
        
        std::swap(subW, subH);

        for(int row = 0; row < subH; row++)
        {
            for(int col = 0; col < subW - 1; col++) //ignore last column
            {
                subHorizontal[row*subW + col] = horiz_vert[ col*subH + (subH - row  - 1) + directionOffset];
            }
        }

        for(int row = 0; row < subH - 1; row++) //ignore last row
        {
            for(int col = 0; col < subW; col++)
            {

                this->mapEdgeDirection(&(horiz_vert[col*subH + (subH - row - 2)]), &tmp);
                subVertical[row*subW + col] = tmp;
            }
        }

        //copy back the data
        std::copy(subHorizontal.begin(), subHorizontal.end(), horiz_vert.begin());
        std::copy(subVertical.begin(), subVertical.end(), horiz_vert.begin() + subHorizontal.size());
    }

    return true;
}


template <int NCHAN>
void Marker<NCHAN>::getPathIndexes(unsigned short subW, unsigned short subH,
                                   std::vector<unsigned int> &indexes) const
{
    indexes.clear();
    const int directionOffset = subH*subW;
    for(int width = 0; width < this->nunique - 1; width++)
    {
        for(int i = 0; i <= width; i++)
        {
            indexes.push_back(i*subW + width); //horizontal
            indexes.push_back(width*subW + i + directionOffset); //vertical

            //add matching passes
            indexes.push_back(i*(subW) + width + 1 + directionOffset); //continuation of the horizontal vertically
            indexes.push_back((width + 1)*subW + i); //horizontal
        }
    }
}


template <int NCHAN>
bool Marker<NCHAN>::getPath(std::vector<unsigned int> &indexes,
                            std::vector< typename Marker<NCHAN>::DirectionType > &horizvert,
                            std::vector< typename Marker<NCHAN>::DirectionType > &path,
                            unsigned int offset) const
{

    Marker<NCHAN>::DirectionType tmp; tmp.setZero();
    path.assign(indexes.size(), tmp);
    for(unsigned int i = 0; i < indexes.size(); i++)
    {
        path[i] = horizvert[indexes[i] + offset];
    }
    return true;
}


template <int NCHAN>
bool Marker<NCHAN>::getPath(unsigned short subW, unsigned short subH,
                            std::vector< typename Marker<NCHAN>::DirectionType > &horiz,
                            std::vector< typename Marker<NCHAN>::DirectionType > &vert,
                            std::vector< typename Marker<NCHAN>::DirectionType > &path)
{
    std::vector<unsigned int> indexes;
    this->getPathIndexes(subW, subH, indexes);

    std::vector< Marker<NCHAN>::DirectionType > horizvert(2*subW*subH, 0);

    std::copy(horiz.begin(), horiz.end(), horizvert.begin());
    std::copy(vert.begin(), vert.end(), horizvert.begin() + horiz.size());

    return this->getPath(indexes, horizvert, path);
}

//w and h are only shifts as if it was the origin of a subwindow and moves the origin
// don't use it with anything other than the starting position for the block that will
//be rotated
void changeBackLocation(Location &loc, int width, int height, int w, int h)
{
    int x = loc.c;
    int y = loc.r;
    switch(loc.rotation)
    {
    case LOC_ROT_0:
        //pass
        break;
    case LOC_ROT_180:
        loc.r = height - y -h;
        loc.c = width - x - w;
        break;
    case LOC_ROT_90:
        loc.r = x;
        loc.c = width - y - h;
        break;
    case LOC_ROT_270:
        loc.c = y;
        loc.r = height - x - w;
        break;
    }
}

void changeBackLocationf(Eigen::Vector2f &pos, int rotation, int width, int height, float w, float h)
{
    float x = pos[0];
    float y = pos[1];
    switch(rotation)
    {
    case LOC_ROT_0:
        //pass
        break;
    case LOC_ROT_180:
        pos[1] = height - y -h;
        pos[0] = width - x - w;
        break;
    case LOC_ROT_90:
        pos[1] = x;
        pos[0] = width - y - h;
        break;
    case LOC_ROT_270:
        pos[1] = height - x - w;
        pos[0] = y;
        break;
    }
}

template <int NCHAN>
float Marker<NCHAN>::getCorrectPercentage(std::vector< DirectionType > &edgeDir,
                                          unsigned short edgeWidth, unsigned short edgeHeight,
                                          int globalOffset, int localOffset,
                                          unsigned short width, unsigned short height) const
{
    assert(globalOffset >= 0 && localOffset >= 0);

    unsigned int inlierSum = 0;
    unsigned int allSum = 0;

    int globalDirectionOffset = this->w*this->h;
    int localDirectionOffset = (int)edgeWidth*(int)edgeHeight;

    //horizontal
    for(int row = 0; row < height; row++)
    {
        for(int col = 0; col < width - 1; col++) //ignore last column for horizontal
        {
            int lindex = row*edgeWidth + col + localOffset;

            if(edgeDir[lindex](0) == EDGE_DIRECTION_INVALID)
            {
                continue;
            }

            int gindex = row*this->w + col + globalOffset;

            //right - the edge is not invalid
            allSum += NCHAN;
            inlierSum += (edgeDir[lindex].array() == this->horizVert[gindex].array()).template cast<int>().sum();
        }
    }

    for(int row = 0; row < height - 1; row++) //ignore last row
    {
        for(int col = 0; col < width; col++)
        {
            int lindex = row*edgeWidth + col + localOffset + localDirectionOffset;

            if(edgeDir[lindex](0) == EDGE_DIRECTION_INVALID)
            {
                continue;
            }

            int gindex = row*this->w + col + globalOffset + globalDirectionOffset;

            //right - the edge is not invalid
            allSum += NCHAN;
            inlierSum += (edgeDir[lindex].array() == this->horizVert[gindex].array()).template cast<int>().sum();
        }
    }

    //we need at least 4 correct corners - that means a 3x3 part - 6 vertical and 6 horizontal
    //this is the bare minimum we need
    if(allSum < 12*NCHAN)
    {
        return -1.f;
    }

    return inlierSum*100.0f/allSum;
}

/**
 * @brief
 *
 * Could be more effective than this - 2x less comparisons if first the edges are masked out
 * and then the mask is used - possible improvement
 */
template <int NCHAN>
int Marker<NCHAN>::getCornerMask(std::vector< DirectionType > &edgeDir,
                                 unsigned short subW, unsigned short subH,
                                 Location loc, std::vector<unsigned char> &cornerMask) const
{
    Location rotatedLoc = loc;
    changeBackLocation(rotatedLoc, this->w, this->h, subW, subH);
    rotatedLoc.rotation = LOC_ROT_0;

    std::vector< DirectionType > rotatedDir = edgeDir;
    this->rotatePart((LOC_ROT_COUNT - loc.rotation)%LOC_ROT_COUNT, subW, subH, rotatedDir);
    if(loc.rotation % 2 == 1)
    {
        std::swap(subW, subH);
    }

    cornerMask.resize((subW - 1)*(subH - 1), CORNER_TYPE_NONE);
    int minCol = std::max(0, -rotatedLoc.c);
    int minRow = std::max(0, - rotatedLoc.r);
    int maxCol = std::min(subW - 1, this->w - 1 - rotatedLoc.c); //corners are only until this->w - 1
    int maxRow = std::min(subH - 1, this->h - 1 - rotatedLoc.r);

    int globalOffset = this->w*this->h;
    int localOffset = subW*subH;

    int nonNoneCounter = 0;

    //horizontal
    for(int row = minRow; row < maxRow; row++)
    {
        for(int col = minCol; col < maxCol; col++) //ignore last column for horizontal
        {
            DirectionType refValues[4];
            int pindex = (rotatedLoc.r + row)*this->w + (rotatedLoc.c + col);
            refValues[0] = this->horizVert[pindex];
            refValues[1] = this->horizVert[pindex + 1 + globalOffset];
            refValues[2] = this->horizVert[pindex + this->w];
            refValues[3] = this->horizVert[pindex + globalOffset];

            DirectionType currentValues[4];
            int lindex = row*subW + col;
            currentValues[0] = rotatedDir[lindex];
            currentValues[1] = rotatedDir[lindex + 1 + localOffset];
            currentValues[2] = rotatedDir[lindex + subW];
            currentValues[3] = rotatedDir[lindex + localOffset];

            bool correct = true;
            for(int i = 0; i < 4; i++)
            {
                if( (refValues[i].array() != currentValues[i].array()).any())
                {
                    correct = false;
                    break;
                }
            }

            if(correct)
            {
                unsigned char ctype = this->cornerType[(rotatedLoc.r + row)*(this->w - 1) + rotatedLoc.c + col];
                cornerMask[row*(subW-1) + col] = ctype;
                nonNoneCounter += (ctype != CORNER_TYPE_NONE);
            }
        }
    }

    this->rotateCorners(loc.rotation, subW - 1, subH - 1, cornerMask);

    return nonNoneCounter;
}


template <int NCHAN>
bool Marker<NCHAN>::rotateCorners(int rotation,
                                  unsigned short subW,
                                  unsigned short subH,
                                  std::vector< unsigned char > &corners) const
{
    std::vector< unsigned char > rotatedCorners;

    if((int) corners.size() != subW*subH)
    {
        return false;
    }

    if(rotation == LOC_ROT_0)
    {
        //pass everything stayes the same
        return true;
    } else if(rotation == LOC_ROT_180)
    {
        rotatedCorners.resize(corners.size(), CORNER_TYPE_NONE);

        const unsigned char mapping[CORNER_TYPE_NONE + 1] =
        {
            CORNER_TYPE_CROSS, //CORNER_TYPE_CROSS = 0,
            CORNER_TYPE_RIGHT_BOTTOM, //CORNER_TYPE_LEFT_TOP,
            CORNER_TYPE_LEFT_BOTTOM, //CORNER_TYPE_RIGHT_TOP,
            CORNER_TYPE_LEFT_TOP, //CORNER_TYPE_RIGHT_BOTTOM,
            CORNER_TYPE_RIGHT_TOP, //CORNER_TYPE_LEFT_BOTTOM,
            CORNER_TYPE_NONE //CORNER_TYPE_NONE
        };

        for(int row = 0; row < subH; row++)
        {
            for(int col = 0; col < subW; col++) //ignore last column for horizontal
            {
                int index = row*subW + col;
                rotatedCorners[index] = mapping[corners[(subH - row - 1)*subW + (subW - col - 1)]];
            }
        }

    } else if(rotation == LOC_ROT_270)
    {
        rotatedCorners.resize(corners.size(), CORNER_TYPE_NONE);

        const unsigned char mapping[CORNER_TYPE_NONE + 1] =
        {
            CORNER_TYPE_CROSS, //CORNER_TYPE_CROSS = 0,
            CORNER_TYPE_RIGHT_TOP, //CORNER_TYPE_LEFT_TOP,
            CORNER_TYPE_RIGHT_BOTTOM, //CORNER_TYPE_RIGHT_TOP,
            CORNER_TYPE_LEFT_BOTTOM, //CORNER_TYPE_RIGHT_BOTTOM,
            CORNER_TYPE_LEFT_TOP, //CORNER_TYPE_LEFT_BOTTOM,
            CORNER_TYPE_NONE //CORNER_TYPE_NONE
        };

        std::swap(subW, subH);

        for(int row = 0; row < subH; row++)
        {
            for(int col = 0; col < subW; col++) //ignore last column for horizontal
            {
                int index = row*subW + col;
                rotatedCorners[index] = mapping[corners[(subW - col - 1)*subH + row]];
            }
        }

    } else if(rotation == LOC_ROT_90)
    {
        rotatedCorners.resize(corners.size(), CORNER_TYPE_NONE);

        const unsigned char mapping[CORNER_TYPE_NONE + 1] =
        {
            CORNER_TYPE_CROSS, //CORNER_TYPE_CROSS = 0,
            CORNER_TYPE_LEFT_BOTTOM, //CORNER_TYPE_LEFT_TOP,
            CORNER_TYPE_LEFT_TOP, //CORNER_TYPE_RIGHT_TOP,
            CORNER_TYPE_RIGHT_TOP, //CORNER_TYPE_RIGHT_BOTTOM,
            CORNER_TYPE_RIGHT_BOTTOM, //CORNER_TYPE_LEFT_BOTTOM,
            CORNER_TYPE_NONE //CORNER_TYPE_NONE
        };

        std::swap(subW, subH);

        for(int row = 0; row < subH; row++)
        {
            for(int col = 0; col < subW; col++) //ignore last column for horizontal
            {
                int index = row*subW + col;
                rotatedCorners[index] = mapping[corners[col*subH + (subH - row - 1)]];
            }
        }
    }

    corners = rotatedCorners;

    return true;
}



struct LocationComp
{
    inline bool operator()(const Location& first, const Location& other) const
    {
        return first.r < other.r ||
                (first.r == other.r && (first.c < other.c ||
                                        (first.c == other.c && first.rotation < other.rotation)));
    }
};


template <int NCHAN>
Location Marker<NCHAN>::getLocation(unsigned short subW, unsigned short subH,
                                    std::vector< typename Marker<NCHAN>::DirectionType > &edgeDir,
                                    std::vector<unsigned char> &cornerMask) const
{
    typedef std::multimap<Location, int, LocationComp > LocationMap;
    typedef std::pair< LocationMap::iterator, LocationMap::iterator > LocationIterPair;
    std::set<Location, LocationComp> keys;

    LocationMap locMap;

    std::vector<unsigned int> pathIndexes;
    this->getPathIndexes(subW, subH, pathIndexes);

    Marker<NCHAN>::DirectionType p; p.setZero();
    std::vector< Marker<NCHAN>::DirectionType > path(pathIndexes.size(), p);

    const int MARKER_SIZE = this->nunique;

    //try decoding positions and store positions
    for(int row = 0; row < subH - MARKER_SIZE + 1; row++)
    {
        for(int col = 0; col < subW - MARKER_SIZE + 1; col++)
        {
            int idx = row*subW + col;

            this->getPath(pathIndexes, edgeDir, path, idx);

            Location currentLocation = this->decisionTree->getLocation(path);

            if(currentLocation == Location::invalid)
            {
                continue;
            }

            //encode location
            Location refLocation;
            refLocation.c = currentLocation.c - col;
            refLocation.r = currentLocation.r - row;
            refLocation.rotation = currentLocation.rotation;

            locMap.insert(std::pair<Location, int>(refLocation, idx));
            keys.insert(refLocation);
        }
    }

    if(locMap.empty())
    {
        //std::cout << "No keys found." << std::endl;
        return Location::invalid;
    }

    //now try matching the good positions

    Location bestLoc;
    float bestPercentage = 0.f;
    float bestScore = 0.f;
    for(std::set<Location>::iterator kit = keys.begin(); kit != keys.end(); kit++)
    {
        Location currLoc = *kit;

        //find the maximum and minimum from the maps
        int maxX = 0, maxY = 0, minX = subW, minY = subH;
        LocationIterPair miterpair = locMap.equal_range(currLoc);

        for (LocationMap::iterator miter=miterpair.first; miter!=miterpair.second; ++miter)
        {
            int cy = miter->second / subW;
            int cx = miter->second % subW;
            if(cy > maxY)
                maxY = cy;
            if(cy < minY)
                minY = cy;

            if(cx > maxX)
                maxX = cx;
            if(cx < minX)
                minX = cx;
        }

        const int border = 1;

        Location minLoc = {minY, minX, currLoc.rotation };
        Location maxLoc = { maxY + this->nunique, maxX + this->nunique, currLoc.rotation };

        int pwidth = maxLoc.c - minLoc.c;
        int pheight = maxLoc.r - minLoc.r;

        Location rotatedLoc = currLoc;
        changeBackLocation(minLoc, subW, subH, pwidth, pheight);
        changeBackLocation(rotatedLoc, this->w, this->h, subW, subH);
        //we can't use change back for anything other than starting points
        int rotatedW, rotatedH;
        if(currLoc.rotation % 2 == 1)
        {
            rotatedW = subH;
            rotatedH = subW;
            maxLoc.r = minLoc.r + pwidth;
            maxLoc.c = minLoc.c + pheight;
        } else {
            rotatedW = subW;
            rotatedH = subH;
            maxLoc.r = minLoc.r + pheight;
            maxLoc.c = minLoc.c + pwidth;
        }

        //we have to be inside both of the marker and the subpart
        minLoc.c = std::max(std::max(minLoc.c - border, 0 - rotatedLoc.c), 0);
        minLoc.r = std::max(std::max(minLoc.r - border, 0 - rotatedLoc.r), 0);

        maxLoc.c = std::min(std::min(maxLoc.c + border, this->w - rotatedLoc.c ), rotatedW);
        maxLoc.r = std::min(std::min(maxLoc.r + border, this->h - rotatedLoc.r ), rotatedH);

        int subWidth = maxLoc.c - minLoc.c;
        int subHeight = maxLoc.r - minLoc.r;

        int minSize = 2*(this->nunique + 2*border);
        if((subWidth + subHeight) < minSize)
        {
            //not enough size for detection anyways
            //std::cout << "Too small" << std::endl;
            //continue;
        }

        Location newLoc = rotatedLoc;
        newLoc.c += minLoc.c;
        newLoc.r += minLoc.r;

        //BUG IS HERE SOMEWHERE
        std::vector< Marker<NCHAN>::DirectionType > rotatedEdges = edgeDir;
        this->rotatePart((LOC_ROT_COUNT - currLoc.rotation) % LOC_ROT_COUNT, subW, subH, rotatedEdges);

        float currPercentage = this->getCorrectPercentage(rotatedEdges, rotatedW, rotatedH,
                                                          newLoc.r*this->w + newLoc.c, minLoc.r*rotatedW + minLoc.c,
                                                          subWidth, subHeight);

        float currScore = currPercentage/100.0f*subWidth*subHeight;


        if(/*currPercentage > this->locationCorrectPercentage &&*/ currScore > bestScore)
        {
            bestPercentage = currPercentage;
            bestLoc = currLoc;
            bestScore = currScore;
        }
    }


    if(bestPercentage < this->locationCorrectPercentage)
    {
        //std::cout << bestPercentage << " Not enough" << std::endl;
        return Location::invalid;
    } else {
        //std::cout << bestPercentage << " was enough" << std::endl;
    }


    //TODO store rather the whole marker in all rotations if this part would be too slow! mem vs speed
    int nonNone = this->getCornerMask(edgeDir, subW, subH, bestLoc, cornerMask);

    if(nonNone < 4) //we need at least 4 points worst case for successful detection
    {
        return Location::invalid;
    }

    return bestLoc;
}

template <int NCHAN>
float Marker<NCHAN>::getCornerScore(Location l) const
{
	return this->scores[(this->w-1)*l.r + l.c];
}

template <int NCHAN>
int Marker<NCHAN>::getCornerType(Location loc, e::Vector2f &corner, typename Marker<NCHAN>::DirectionType* edges, float *score) const
{

    changeBackLocation(loc, this->w, this->h, 2, 2);
    if(loc.c < 0 || loc.c >= this->w - 1 || loc.r < 0 || loc.r >= this->h - 1)
    {
        return CORNER_TYPE_NONE;
    }

    corner[0] = loc.c + 0.5f;
    corner[1] = loc.r + 0.5f;

    unsigned char type = this->cornerType[loc.r*(this->w - 1) + loc.c];

	if(score)
	{
		*score = this->getCornerScore(loc);
	}

    unsigned int verticalOffset = (this->w - 1)*this->h;
    if(edges)
    {
        DirectionType values[4];
        int pindex = loc.r*this->w + loc.c;
        values[0] = this->horizVert[pindex];
        values[1] = this->horizVert[pindex + 1 + verticalOffset];
        values[2] = this->horizVert[pindex + this->w];
        values[3] = this->horizVert[pindex + verticalOffset];

        if(loc.rotation == 0)
        {
            for(int i = 0; i < 4; i++)
            {
                edges[i] = values[i];
            }
        }
        else if (loc.rotation == 2)
        {
            this->mapEdgeDirection(&values[2], &edges[0]);
            this->mapEdgeDirection(&values[3], &edges[1]);
            this->mapEdgeDirection(&values[0], &edges[2]);
            this->mapEdgeDirection(&values[1], &edges[3]);
        } else if(loc.rotation == 1)
        {
            edges[0] = values[1];
            this->mapEdgeDirection(&values[2], &edges[1]);
            edges[2] = values[3];
            this->mapEdgeDirection(&values[0], &edges[3]);
        }
        else if(loc.rotation == 3)
        {

            this->mapEdgeDirection(&values[3], &edges[0]);
            edges[1] = values[0];
            this->mapEdgeDirection(&values[1], &edges[2]);
            edges[3] = values[2];
        }
    }


    switch(type)
    {
    case CORNER_TYPE_CROSS:
        return CORNER_TYPE_CROSS;
    case CORNER_TYPE_NONE:
        return CORNER_TYPE_NONE;
    default:
        return (type - CORNER_TYPE_LEFT_TOP - loc.rotation + 4)%4 + CORNER_TYPE_LEFT_TOP;
    }

    return CORNER_TYPE_NONE;
}

template Marker<1>::Marker(unsigned short rows, unsigned short cols, unsigned char _n, std::vector< e::Matrix<unsigned char, 1, 1> > colors);
template Marker<3>::Marker(unsigned short rows, unsigned short cols, unsigned char _n, std::vector< e::Matrix<unsigned char, 3, 1> > colors);

template bool Marker<1>::setField(std::vector<unsigned short> &data);
template bool Marker<3>::setField(std::vector<unsigned short> &data);

template Location Marker<1>::getLocation(unsigned short subW, unsigned short subH, std::vector< Marker<1>::DirectionType > &edgeDir, std::vector<unsigned char> &cornerMask) const;
template Location Marker<3>::getLocation(unsigned short subW, unsigned short subH, std::vector< Marker<3>::DirectionType > &edgeDir, std::vector<unsigned char> &cornerMask) const;


template int Marker<1>::getCornerType(Location l, e::Vector2f &corner, DirectionType* edges, float *score) const;
template int Marker<3>::getCornerType(Location l, e::Vector2f &corner, DirectionType* edges, float *score) const;

template float Marker<1>::getCornerScore(Location l) const;
template float Marker<3>::getCornerScore(Location l) const;
}
