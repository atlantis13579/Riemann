#if defined(RIEMANN_ENABLE_METAL)
#include "SandGPUBackend.h"
#include "../Compute/MetalCompute.h"

#include <cmath>
#include <stdexcept>

namespace Riemann
{
    namespace
    {
        using namespace MetalCompute;
        const char *SandKernels =
#include "SandMetalKernels.h"
            ;
        class MetalSand final : public SandGPUBackend
        {
            std::unique_ptr<MetalDevice> m_Device;
            SharedBuffer m_Height, m_Next, m_Flux, m_Limits;
            SharedProgram m_Transport, m_Gather;
            std::vector<float> m_Readback;

          public:
            bool Relax(std::vector<float> &Heights, const std::vector<float> &Limits, int Resolution, float Slope,
                       float Rate, int Iterations, std::string &Error) override
            {
                try
                {
                    if (!m_Device)
                    {
                        m_Device = CreateMetalDevice();
                        if (!m_Device)
                            throw std::runtime_error("Sand compute support was not built");
                        m_Transport = m_Device->CreateProgram(SandKernels, "SandFlux");
                        m_Gather = m_Device->CreateProgram(SandKernels, "SandGather");
                    }
                    const int Count = Resolution * Resolution;
                    const size_t Bytes = Count * sizeof(float);
                    ReserveMetalBuffer(*m_Device, m_Height, Bytes);
                    ReserveMetalBuffer(*m_Device, m_Next, Bytes);
                    ReserveMetalBuffer(*m_Device, m_Limits, Bytes);
                    ReserveMetalBuffer(*m_Device, m_Flux, Bytes * 8);
                    m_Device->Upload(m_Height, Heights.data(), Bytes);
                    m_Device->Upload(m_Limits, Limits.data(), Bytes);
                    Constants P;
                    P.Integer(0, Resolution);
                    P.Float(1, Slope);
                    P.Float(2, Rate);
                    for (int I = 0; I < Iterations; ++I)
                    {
                        const Bindings B{m_Height, m_Flux, m_Limits, m_Next};
                        m_Device->Dispatch(m_Transport, B, P, Count);
                        m_Device->Dispatch(m_Gather, B, P, Count);
                        m_Height.swap(m_Next);
                    }
                    m_Readback.resize(Count);
                    m_Device->Download({{m_Height, m_Readback.data(), Bytes}});
                    for (float H : m_Readback)
                        if (!std::isfinite(H) || H < 0)
                            throw std::runtime_error("Compute sand produced invalid heights");
                    Heights.swap(m_Readback);
                    Error.clear();
                    return true;
                }
                catch (const std::exception &E)
                {
                    if (m_Device)
                        m_Device->Finish();
                    Error = E.what();
                    return false;
                }
            }
        };
    } // namespace
    std::unique_ptr<SandGPUBackend> CreateSandMetal()
    {
        return std::make_unique<MetalSand>();
    }
} // namespace Riemann
#endif
