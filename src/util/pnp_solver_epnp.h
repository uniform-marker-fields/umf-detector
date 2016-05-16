#ifndef __UMF_PNP_SOLVER_EPNP_H
#define __UMF_PNP_SOLVER_EPNP_H

#include "pnp_solver.h"
#include "epnp.h"

namespace umf {

/**
 * PNP camera pose estimation using EPnP
 */

class PnPSolverEpnp : public PnPSolver
{
public:
    PnPSolverEpnp();
    virtual ~PnPSolverEpnp() {}

protected:

    virtual bool pnpSolve(CorrespondenceSet correspondences, bool useLast = false);

private:
    epnp solver;
};

}
#endif // PNP_SOLVER_EPNP_H
