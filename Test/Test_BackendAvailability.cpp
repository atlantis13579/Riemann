#include "Test.h"
#include "../Src/Compute/BackendAvailability.h"
#include "../Src/Solver/LCPSolver.h"
#include "../Src/Solver/PBDBackend.h"
#include "../Src/Fluid/FluidBackend.h"
#include <chrono>

void TestBackendAvailability()
{
    using namespace Riemann;
    const auto Start = std::chrono::steady_clock::now();
    const auto &C = GetBackendCapabilities();
    std::printf("Build/device checks completed: %.3f ms\n", std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - Start).count());
    EXPECT(&C == &GetBackendCapabilities());
    for (int S = 0; S < 4; ++S)
        EXPECT(C.Solvers[S][0]);
    EXPECT(C.FluidVelocity[0] && C.Incompressible[0] && C.Snow[0] && C.Sand[0]);
    for (int B = 0; B < 5; ++B)
    {
        std::printf("Backend %d: LCP=%d PBD=%d XPBD=%d velocity=%d air=%d snow=%d sand=%d\n", B, C.Solvers[1][B],
                    C.Solvers[2][B], C.Solvers[3][B], C.FluidVelocity[B], C.Incompressible[B], C.Snow[B], C.Sand[B]);
        EXPECT(!C.Solvers[1][B] || LCPSolver::IsBackendAvailable(static_cast<SolverBackend>(B)));
        EXPECT(!C.Solvers[2][B] || IsPBDBackendAvailable(static_cast<SolverBackend>(B)));
        EXPECT(!C.FluidVelocity[B] || IsFluidBackendAvailable(static_cast<FluidBackend>(B)));
        if (B != 0)
            EXPECT(!C.Solvers[0][B]);
    }
#if !defined(RIEMANN_ENABLE_CUDA)
    EXPECT(!C.Solvers[1][3] && !C.Solvers[2][3] && !C.Solvers[3][3]);
    EXPECT(!C.FluidVelocity[2] && !C.Incompressible[2] && !C.Snow[2] && !C.Sand[2]);
#endif
#if !defined(RIEMANN_ENABLE_METAL)
    EXPECT(!C.Solvers[1][4] && !C.Solvers[2][4] && !C.Solvers[3][4]);
    EXPECT(!C.FluidVelocity[4] && !C.Incompressible[4] && !C.Snow[4] && !C.Sand[4]);
#endif
}
