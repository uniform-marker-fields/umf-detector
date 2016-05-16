#ifndef __UMF_PNP_SOLVER_LHM_H
#define __UMF_PNP_SOLVER_LHM_H

#include "pnp_solver.h"
#include "epnp.h"

namespace umf {

/**
 * PnP Solver based on simple LHM
 */

class PnPSolverLHM : public PnPSolver
{
public:
    PnPSolverLHM();
    virtual ~PnPSolverLHM() {}

protected:

    virtual bool pnpSolve(CorrespondenceSet correspondences, bool useLast = false);

};

/**
 * PnP Solver based on LHM initialized by EPnP
 */

class PnPSolverEPnPLHM : public PnPSolver
{
public:
    PnPSolverEPnPLHM();
    virtual ~PnPSolverEPnPLHM() {}

protected:

    virtual bool pnpSolve(CorrespondenceSet correspondences, bool useLast = false);

private:
    epnp solver;

};

}
#endif // PNP_SOLVER_EPNP_H
