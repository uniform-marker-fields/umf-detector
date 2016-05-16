#include "homography2d.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>

namespace umf {

void normalise2dpts(const std::vector<Eigen::Vector2f> &src, std::vector<Eigen::Vector2f> &normvec, Eigen::Matrix3f &T, bool inverse = false)
{
	//compute the mean of the points
	Eigen::Vector2f mean(0, 0);
	for(unsigned int i = 0; i < src.size(); i++)
	{
		mean += src[i];
	}

	mean *= 1.0f/src.size();

	//compute variance
	Eigen::Vector2f dist(0.0f, 0.0f);
	for(unsigned int i = 0; i < src.size(); i++)
	{
		normvec[i] = src[i] - mean;
		dist += normvec[i].cwiseAbs();
	}
	Eigen::Vector2f scale;
	scale[0] = src.size()/dist[0];
	scale[1] = src.size()/dist[1];
	if(inverse)
	{
		T << 1.0f/scale[0], 0, mean[0],
			0, 1.0f/scale[1], mean[1],
			0, 0, 1;
	} else {
		T << scale[0], 0, -scale[0]*mean[0],
			0, scale[1], -scale[1]*mean[1],
			0, 0, 1;
	}

	for(unsigned int i = 0; i < normvec.size(); i++)
	{
		normvec[i] = normvec[i].cwiseProduct(scale);
	}
}

void computeHomography2d(const std::vector<Eigen::Vector2f> &src,const std::vector<Eigen::Vector2f> &dst, Eigen::Matrix3f &H)
{
	assert(src.size() == dst.size());
	std::vector<Eigen::Vector2f> srcNormalized(src.size());
	std::vector<Eigen::Vector2f> dstNormalized(dst.size());
	Eigen::Matrix3f T1, T2;

	normalise2dpts(src, srcNormalized, T1);
	normalise2dpts(dst, dstNormalized, T2, true);

	int Npts = src.size();
	Eigen::Matrix<float, 9, 9> LtL; LtL.setZero();
	Eigen::Matrix<float, 9, 1> Lx, Ly;
	for(int i = 0; i < Npts; i++)
	{
		float x = dstNormalized[i][0];
		float y = dstNormalized[i][1];
		float X = srcNormalized[i][0];
		float Y = srcNormalized[i][1];
		Lx <<    X,    Y,  1,   0,   0,  0, -x*X, -x*Y, -x;
		Ly <<    0,    0,  0,   X,   Y,  1, -y*X, -y*Y, -y;
		for(int j = 0; j < 9; j++)
		{
			for(int k = 0; k < 9; k++)
			{
				LtL(j, k) += Lx(j)*Lx(k) + Ly(j)*Ly(k);
			}
		}
	}
	
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(LtL);
	Eigen::MatrixXf eigV = eig.eigenvectors();

	//transpose first column for the results
	H.block<1, 3>(0, 0) = eigV.block<3, 1>(0, 0);
	H.block<1, 3>(1, 0) = eigV.block<3, 1>(3, 0);
	H.block<1, 3>(2, 0) = eigV.block<3, 1>(6, 0);

	//denormalize
	H = T2*H*T1;
	H *= 1.0f/H(2,2);
}

}