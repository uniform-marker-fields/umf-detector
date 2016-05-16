#include "mask_tracker.h"
#include "umfdebug.h"
#include "draw.h"

namespace umf {

MaskTracker::MaskTracker()
{
	this->enabled = false;
}

void MaskTracker::update(std::vector<Eigen::Vector2f> &cornerPoints)
{
	if(cornerPoints.size() != 4)
	{
		this->enabled = false;
		return;
	}

	for(int i = 0; i < 4; i++)
	{
		this->prevPos[i] = cornerPoints[i];
	}
	this->enabled = true;
}

bool MaskTracker::filterPoints(std::vector<Eigen::Vector2i> &points)
{
	if(!this->enabled)
	{
		return false;
	}

	Eigen::Vector2f maskDirections[4];

	maskDirections[0] = this->prevPos[1] - this->prevPos[0];
	maskDirections[1] = this->prevPos[2] - this->prevPos[1];
	maskDirections[2] = this->prevPos[3] - this->prevPos[2];
	maskDirections[3] = this->prevPos[0] - this->prevPos[3];

	Eigen::Vector2f pointDirections[4];
    for(int pindex = (int) points.size() - 1; pindex >= 0; pindex--)
	{
		Eigen::Vector2f pointPos = points[pindex].cast<float>();
		bool remove = false;
		for(int i = 0; i < 4; i++)
		{
			pointDirections[i] = pointPos - this->prevPos[i];

			float crossVal = pointDirections[i][0]*maskDirections[i][1] - pointDirections[i][1]*maskDirections[i][0];

			if(crossVal > 0)
			{
				remove = true;
				break;
			}
		}

		if(remove)
		{
			points.erase( points.begin() + pindex);
		}
	}

	return true;
}

void MaskTracker::show()
{
	if(!this->enabled)
	{
		return;
	}
	UMFDebug *dbg = UMFDSingleton::Instance();
    Renderer *rend = dbg->getRenderer();
    if(rend != NULL)
    {
		Eigen::Vector3i color(200, 0, 200);
		int lineWidth = 3;

		for(int i = 0; i < 4; i++)
		{
			drawLine(rend, this->prevPos[(i+1)%4].cast<int>(), this->prevPos[i].cast<int>(), color, lineWidth);
		}
	}
}

}
