#pragma once

namespace Riemann
{
    struct BackendCapabilities
    {
        // Solver rows: SI, LCP, PBD, XPBD. Columns use SolverBackend values.
        bool Solvers[4][5]{};
        // FluidBackend and SnowBackend have their own enum order.
        bool FluidVelocity[5]{}, Incompressible[5]{}, Snow[5]{}, Sand[5]{};
    };

    // Build and device checks only. Kernel creation and execution are deferred until use.
    const BackendCapabilities &GetBackendCapabilities();
    bool IsDirectComputeAvailable();
    bool IsMetalComputeAvailable();

} // namespace Riemann
