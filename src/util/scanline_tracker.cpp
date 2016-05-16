#include "scanline_tracker.h"
#include "line_iterator.h"

namespace umf {

template<class T, int CHAN>
void ScanlineTracker::getScanlines(Model<CHAN> &model, Image<T, CHAN> *img, std::vector< LineIterator< Image<T, CHAN> > > &scanlines)
{
    Eigen::Vector2i imgSize(img->width, img->height);

    const Marker<CHAN> *m = model.getMarker();
    std::vector<Eigen::Vector3f> modelPoints;
    modelPoints.push_back(Eigen::Vector3f(0, 0, 0));
    modelPoints.push_back(Eigen::Vector3f(m->w, m->h, 0));
    std::vector<Eigen::Vector2f> projPoints;
    //first project the top left and bottom right corners to get an idea about the distance
    model.projectPoints(modelPoints, projPoints, imgSize, false, true);

    int minX = m->w;
    int minY = m->h;
    int maxX = -1;
    int maxY = -1;

    float diff = (projPoints[0] - projPoints[1]).squaredNorm();
    float imageDiagonalSq = (float)img->width*img->width + (float)img->height*img->height;

    if( diff > imageDiagonalSq)
    {
        const CorrespondenceSet &corr = model.getCorrespondences();
            
        for(unsigned int i = 0; i < corr.size(); i++)
        {
            if(corr[i].mx < minX)
            {
                minX = static_cast<int>(corr[i].mx);
            }
            if(corr[i].my < minY)
            {
                minY = static_cast<int>(corr[i].my);
            }
            if(corr[i].mx > maxX)
            {
                maxX = static_cast<int>(corr[i].mx);
            }
            
            if(corr[i].my > maxY)
            {
                maxY = static_cast<int>(corr[i].my);
            }
        }
        minX--; minY--;
        maxX++; maxY++;
    } else {
        minX = -1;
        minY = -1;
        maxX = m->w + 1;
        maxY = m->h + 1;
    }

    int height = maxY - minY;
    int width = maxX - minX;

    if(height < 1 || width < 1)
    {
        scanlines.clear();
        return;
    }

    Eigen::Vector2i markerSize(m->w + 1, m->h + 1);
    Eigen::Vector2i p1(0, 0);
    Eigen::Vector2i p2(0, 0);

    //first direction
    modelPoints.clear();
    modelPoints.reserve(2*this->maxCountHalf);

    if(width > height)
    {
        int poffset = width - 2;
        int totalwidth = poffset + width;
        for(int i = 1; i < this->maxCountHalf; i++)
        {
            int startOffset = totalwidth*i/this->maxCountHalf;
            p1[0] = minX + startOffset;
            p1[1] = minY;
            p2[0] = minX + startOffset - poffset;
            p2[1] = maxY;
            clipLine(markerSize, p1, p2, 0);
            //make it a bit slashed so we don't hit only edges by mistake
            modelPoints.push_back(Eigen::Vector3f(p1[0] + 0.5f, p1[1] + 0.f, 0.f));
            modelPoints.push_back(Eigen::Vector3f(p2[0] + 0.f, p2[1] + 0.5f, 0.f));
        }
        
        for(int i = 1; i < this->maxCountHalf; i++)
        {
            int startOffset = totalwidth*i/this->maxCountHalf - poffset;   
            p1[0] = minX + startOffset;
            p1[1] = minY;
            p2[0] = minX + startOffset + poffset;
            p2[1] = maxY;
            clipLine(markerSize, p1, p2, 0);
            //make it a bit slashed so we don't hit only edges by mistake
            modelPoints.push_back(Eigen::Vector3f(p1[0] + 0.5f, p1[1] + 0.f, 0.f));
            modelPoints.push_back(Eigen::Vector3f(p2[0] + 0.f, p2[1] + 0.5f, 0.f));
        }
    } else {
        int poffset = height - 2;
        int totalheight = poffset + height;
        for(int i = 1; i < this->maxCountHalf; i++)
        {
            int startOffset = totalheight*i/this->maxCountHalf - poffset;
            p1[0] = minX;
            p1[1] = minY + startOffset;
            p2[0] = maxX;
            p2[1] = minY + startOffset + poffset;
            clipLine(markerSize, p1, p2, 0);
            modelPoints.push_back(Eigen::Vector3f(p1[0] + 0.f, p1[1] + 0.5f, 0.f));
            modelPoints.push_back(Eigen::Vector3f(p2[0] + 0.5f, p2[1] + 0.f, 0.f));
        }

        for(int i = 1; i < this->maxCountHalf; i++)
        {
            int startOffset = totalheight*i/this->maxCountHalf;
            p1[0] = minX;
            p1[1] = minY + startOffset;
            p2[0] = maxX;
            p2[1] = minY + startOffset - poffset;
            clipLine(markerSize, p1, p2, 0);
            modelPoints.push_back(Eigen::Vector3f(p1[0] + 0.f, p1[1] + 0.5f, 0.f));
            modelPoints.push_back(Eigen::Vector3f(p2[0] + 0.5f, p2[1] + 0.f, 0.f));
        }
    }

        
    projPoints.clear();
    model.projectPoints(modelPoints, projPoints, imgSize, false, true);

    scanlines.clear();
    for(unsigned int i = 0; i < projPoints.size()/2; i++)
    {
		LineIterator< Image<T, CHAN> > currentLineIterator(img, projPoints[i * 2].template cast<int>(), projPoints[i * 2 + 1].template cast<int>());
        scanlines.push_back(currentLineIterator);
		scanlines.size();
    }

    //add a few extra lines just to be sure :)
    if(this->extraStep > 0)
    {
        for(int x = this->extraStep/2; x < img->width - this->extraStep/2; x += this->extraStep)
        {
            scanlines.push_back(LineIterator< Image<T, CHAN> >(img, Eigen::Vector2i(x, 3), Eigen::Vector2i(x, img->height - 3)));
        }

        for(int y = this->extraStep/2; y < img->height - this->extraStep/2; y += this->extraStep)
        {
            scanlines.push_back(LineIterator< Image<T, CHAN> >(img, Eigen::Vector2i(3, y), Eigen::Vector2i(img->width - 3, y)));
        }
    }
}


template void ScanlineTracker::getScanlines(Model<1> &model, ImageGray *img, std::vector< LineIterator< ImageGray > > &scanlines);
template void ScanlineTracker::getScanlines(Model<3> &model, ImageRGB *img, std::vector< LineIterator< ImageRGB > > &scanlines);

}
