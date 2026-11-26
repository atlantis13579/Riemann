#include "BackendAvailability.h"
#include "../Solver/LCPSolver.h"
#include "../Solver/PBDBackend.h"
#include "../Fluid/FluidBackend.h"
#include "../Continuum/SnowSimulation.h"
#include "../Continuum/SandSimulation.h"
#if defined(_WIN32) && defined(RIEMANN_ENABLE_COMPUTE_SHADER)
#include "DX11Compute.h"
#endif

namespace Riemann
{
    bool IsDirectComputeAvailable()
    {
#if defined(_WIN32) && defined(RIEMANN_ENABLE_COMPUTE_SHADER)
        return DX11Compute::QueryBackendInfo().Available;
#else
        return false;
#endif
    }
#if !defined(RIEMANN_ENABLE_METAL)
    bool IsMetalComputeAvailable() { return false; }
#endif

    const BackendCapabilities &GetBackendCapabilities()
    {
        static const BackendCapabilities Capabilities = []
        {
            BackendCapabilities Result;
            Result.Solvers[0][0] = true;
            for (int I = 0; I < 5; ++I)
            {
                Result.Solvers[1][I] = LCPSolver::IsBackendAvailable(static_cast<SolverBackend>(I));
                Result.Solvers[2][I] = Result.Solvers[3][I] = IsPBDBackendAvailable(static_cast<SolverBackend>(I));
                Result.FluidVelocity[I] = IsFluidBackendAvailable(static_cast<FluidBackend>(I));
                // Every compiled GPU fluid backend includes the incompressible solver.
                Result.Incompressible[I] = I != static_cast<int>(FluidBackend::SIMD) && Result.FluidVelocity[I];
                Result.Snow[I] = SnowSimulation::IsBackendAvailable(static_cast<SnowBackend>(I));
                Result.Sand[I] = SandSimulation::IsBackendAvailable(static_cast<SnowBackend>(I));
            }
            return Result;
        }();
        return Capabilities;
    }
} // namespace Riemann
