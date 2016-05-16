#ifndef __UMF_PNP_SOLVER_OPENCV_H
#define __UMF_PNP_SOLVER_OPENCV_H

#include "../defines.h"
#include "pnp_solver.h"

#ifdef UMF_USE_OPENCV

namespace umf {
    
/**
 * PnP solver - use OpenCV implementation.
 */

class PnPSolverOpenCV: public PnPSolver
{

public:
    PnPSolverOpenCV();
    virtual ~PnPSolverOpenCV() {}

protected:

    virtual bool pnpSolve(CorrespondenceSet correspondences, bool useLast = false);
};


}

#endif // use opencv
#endif // PNP_SOLVER_OPENCV_H
