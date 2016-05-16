#include <Eigen/Core>
#include <vector>

namespace umf {

//extracted from opencv
void undistortPoints( std::vector<Eigen::Vector2f> &_src,
                      std::vector<Eigen::Vector2f> &_dst,
                      const Eigen::Matrix3f &_cameraMatrix,
                      const Eigen::VectorXf &_distCoeffs )
{
    double k[8]={0,0,0,0,0,0,0,0}, fx, fy, ifx, ify, cx, cy;

    _dst.resize(_src.size());

    int i, j, n, iters = 1;

    if( _distCoeffs.size() > 0 )
    {
        assert(_distCoeffs.size() == 4 || _distCoeffs.size() == 5 || _distCoeffs.size() == 8);
        for(i = 0; i < _distCoeffs.size(); i++)
        {
            k[i] = _distCoeffs(i);
        }
        iters = 5;
    }

    n = _src.size();

    _dst.resize(n);

    fx = _cameraMatrix(0,0);
    fy = _cameraMatrix(1,1);
    ifx = 1./fx;
    ify = 1./fy;
    cx = _cameraMatrix(0, 2);
    cy = _cameraMatrix(1, 2);

    for( i = 0; i < n; i++ )
    {
        double x, y, x0, y0;

        x = _src[i](0);
        y = _src[i](1);

        x0 = x = (x - cx)*ifx;
        y0 = y = (y - cy)*ify;

        // compensate distortion iteratively
        for( j = 0; j < iters; j++ )
        {
            double r2 = x*x + y*y;
            double icdist = (1 + ((k[7]*r2 + k[6])*r2 + k[5])*r2)/(1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2);
            double deltaX = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
            double deltaY = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;
            x = (x0 - deltaX)*icdist;
            y = (y0 - deltaY)*icdist;
        }

        _dst[i](0) = (float) x;
        _dst[i](1) = (float) y;

    }
}

void distortPoints(std::vector<Eigen::Vector2f> &src,
                   std::vector<Eigen::Vector2f> &dst,
                   const Eigen::Matrix3f &cameraMatrix,
                   const Eigen::VectorXf &distCoeffs )
{
    dst.resize(src.size());
    double fx = cameraMatrix(0,0);
    double fy = cameraMatrix(1,1);
    double cx = cameraMatrix(0,2);
    double cy = cameraMatrix(1,2);

    double k[8]={0,0,0,0,0,0,0,0};
    for(int i = 0; i < distCoeffs.size(); i++)
    {
        k[i] = distCoeffs(i);
    }

    double r2,r4,r6,a1,a2,a3,cdist,icdist2,xd,yd;

    for (unsigned int i = 0; i < src.size(); i++)
    {
        const Eigen::Vector2f &p = src[i];
        double x = p[0];
        double y = p[1];
        double xCorrected, yCorrected;

        r2 = x*x + y*y;
        r4 = r2*r2;
        r6 = r4*r2;
        a1 = 2*x*y;
        a2 = r2 + 2*x*x;
        a3 = r2 + 2*y*y;
        cdist = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6;
        icdist2 = 1./(1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
        xd = x*cdist*icdist2 + k[2]*a1 + k[3]*a2;
        yd = y*cdist*icdist2 + k[2]*a3 + k[3]*a1;

        xCorrected = xd*fx + cx;
        yCorrected = yd*fy + cy;


        dst[i] = Eigen::Vector2f(xCorrected, yCorrected);
    }
}

}
