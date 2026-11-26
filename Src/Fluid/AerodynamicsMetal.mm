#include "AerodynamicsGPUBackend.h"
#include "AerodynamicsTypes.h"
#include "../Compute/MetalCompute.h"
#include "../Maths/Vector4.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace Riemann
{
    namespace
    {
        using namespace MetalCompute;
        const char *TransportSource =
#include "AerodynamicsMetalKernels.h"
            ;
        const char *PressureSource =
#include "AerodynamicsMetalPressure.h"
            ;
        class MetalAerodynamics final : public AerodynamicsGPUBackend
        {
            enum
            {
                AdvectV,
                EmitSource,
                Curl,
                Force,
                Divergence,
                Pressure,
                Project,
                AdvectS,
                CorrectS,
                Impulse,
                MaxRate,
                Boundary,
                ClearPressure,
                KernelCount
            };
            enum
            {
                InitializeState,
                SurfaceResponse,
                RelaxPressure,
                ApplyCorrection,
                CouplingCount
            };
            std::unique_ptr<MetalDevice> m_Device;
            SharedProgram m_Kernels[KernelCount], m_Coupling[CouplingCount];
            SharedBuffer m_Velocity, m_Scalar, m_Curl, m_Pressure, m_Wall, m_Divergence, m_Emitters, m_Rates;
            SharedBuffer m_State, m_System, m_Response;
            AerodynamicsSimulationSettings m_Settings;
            Constants m_P;
            int m_Count = 0, m_V = 0, m_S[3] = {0, 1, 2};
            float m_MaxRate = 0;
            std::vector<Vector4> m_WallUpload;

            void Dispatch(int Kernel, int ScalarOutput = 2)
            {
                m_P.Integer(28, m_V * m_Count * 4);
                m_P.Integer(29, (1 - m_V) * m_Count * 4);
                m_P.Integer(30, m_S[0] * m_Count * 2);
                m_P.Integer(31, m_S[ScalarOutput] * m_Count * 2);
                m_P.Integer(32, Kernel);
                m_P.Integer(33, m_S[1] * m_Count * 2);
                m_Device->Dispatch(
                    m_Kernels[Kernel],
                    {m_Velocity, m_Scalar, m_Curl, m_Pressure, m_Wall, m_Divergence, m_Emitters, m_Rates}, m_P,
                    m_Count);
            }
            void Zero(const SharedBuffer &Buffer)
            {
                std::vector<uint32_t> Values(Buffer->Size / 4, 0);
                m_Device->Upload(Buffer, Values.data(), Buffer->Size);
            }
            static bool Finite(const std::vector<float> &Values)
            {
                for (float V : Values)
                    if (!std::isfinite(V))
                        return false;
                return true;
            }

          public:
            bool Initialize(const AerodynamicsSimulationSettings &Settings, const std::vector<unsigned char> &Solid)
            {
                m_Device = CreateMetalDevice();
                if (!m_Device || Settings.Emitters.size() > 8)
                    return false;
                const auto N = Settings.Resolution;
                const size_t Count = size_t(N.x) * N.y * N.z;
                if (N.x <= 0 || N.y <= 0 || N.z <= 0 || Count > 65535u * 256u || Solid.size() != Count)
                    return false;
                m_Count = static_cast<int>(Count);
                m_Settings = Settings;
                const char *Names[] = {"AdvectVelocity", "Emit",     "Curl",         "ApplyForces",   "Divergence",
                                       "Pressure",       "Project",  "AdvectScalar", "CorrectScalar", "Impulse",
                                       "MaxRate",        "Boundary", "ClearPressure"};
                for (int I = 0; I < KernelCount; ++I)
                    m_Kernels[I] = m_Device->CreateProgram(TransportSource, Names[I]);
                const char *CouplingNames[] = {"InitializeState", "SurfaceResponse", "RelaxPressure",
                                               "ApplyCorrection"};
                for (int I = 0; I < CouplingCount; ++I)
                    m_Coupling[I] = m_Device->CreateProgram(PressureSource, CouplingNames[I]);
                const Vector3 H = Settings.BoundsMax - Settings.BoundsMin;
                for (int A = 0; A < 3; ++A)
                {
                    m_P.Integer(A, N[A]);
                    m_P.Float(4 + A, H[A] / N[A]);
                    m_P.Float(8 + A, Settings.BoundsMin[A]);
                }
                m_P.Float(12, Settings.Buoyancy);
                m_P.Float(13, Settings.TracerWeight);
                m_P.Float(14, Settings.Vorticity);
                const int ShortAxis = std::min(N.x, std::min(N.y, N.z));
                m_P.Float(15, std::max(1.75f, std::min(1.93f, 2.0f / (1.0f + std::sin(3.14159265f / ShortAxis)))));
                m_P.Float(16, Settings.DensityDissipation);
                m_P.Float(17, Settings.Cooling);
                m_P.Float(18, static_cast<float>(Settings.Emitters.size()));
                std::vector<Vector4> Emitters(24, Vector4(0, 0, 0, 0));
                for (size_t I = 0; I < Settings.Emitters.size(); ++I)
                {
                    const auto &E = Settings.Emitters[I];
                    Emitters[I * 3] = Vector4(E.Position.x, E.Position.y, E.Position.z, E.Radius);
                    Emitters[I * 3 + 1] = Vector4(E.Velocity.x, E.Velocity.y, E.Velocity.z, 0);
                    Emitters[I * 3 + 2] = Vector4(E.Density, E.Temperature, 0, 0);
                }
                m_Velocity = m_Device->CreateBuffer(Count * 8 * sizeof(float));
                m_Scalar = m_Device->CreateBuffer(Count * 6 * sizeof(float));
                m_Curl = m_Device->CreateBuffer(Count * 4 * sizeof(float));
                m_Pressure = m_Device->CreateBuffer(Count * sizeof(float));
                m_Wall = m_Device->CreateBuffer(Count * 4 * sizeof(float));
                m_Divergence = m_Device->CreateBuffer(Count * sizeof(float));
                m_Emitters = m_Device->CreateBuffer(Emitters.size() * sizeof(Vector4));
                m_Rates = m_Device->CreateBuffer(((Count + 255) / 256) * sizeof(float));
                m_Device->Upload(m_Emitters, Emitters.data(), m_Emitters->Size);
                m_WallUpload.resize(Count);
                for (size_t I = 0; I < Count; ++I)
                    m_WallUpload[I] = Vector4(0, 0, 0, Solid[I] ? 1.0f : 0.0f);
                m_Device->Upload(m_Wall, m_WallUpload.data(), m_Wall->Size);
                Reset();
                return true;
            }
            void Reset() override
            {
                Zero(m_Velocity);
                Zero(m_Scalar);
                Zero(m_Pressure);
                m_MaxRate = 0;
            }
            bool UpdateObstacles(const std::vector<unsigned char> &Solid, const std::vector<Vector3> &Velocity,
                                 const AerodynamicsGridRegion *Region) override
            {
                if (Solid.size() != size_t(m_Count) || Velocity.size() != size_t(m_Count))
                    return false;
                try
                {
                    const auto N = m_Settings.Resolution;
                    const Vector3i Lo = Region ? Region->Min : Vector3i(0), Hi = Region ? Region->Max : N;
                    if (Lo.x < 0 || Lo.y < 0 || Lo.z < 0 || Hi.x > N.x || Hi.y > N.y || Hi.z > N.z)
                        return false;
                    const Vector3 H = m_Settings.BoundsMax - m_Settings.BoundsMin;
                    for (int Z = Lo.z; Z < Hi.z; ++Z)
                        for (int Y = Lo.y; Y < Hi.y; ++Y)
                            for (int X = Lo.x; X < Hi.x; ++X)
                            {
                                const int I = X + N.x * (Y + N.y * Z);
                                const auto V = Velocity[I];
                                m_WallUpload[I] = Vector4(V.x, V.y, V.z, Solid[I] ? 1.0f : 0.0f);
                                for (int A = 0; A < 3; ++A)
                                    m_MaxRate = std::max(m_MaxRate, std::fabs(V[A]) * N[A] / H[A]);
                            }
                    m_Device->Upload(m_Wall, m_WallUpload.data(), m_Wall->Size);
                    Dispatch(Boundary);
                    m_V = 1 - m_V;
                    std::swap(m_S[0], m_S[2]);
                    return true;
                }
                catch (const std::exception &)
                {
                    m_Device->Finish();
                    return false;
                }
            }
            void Step(float Dt, float Time, bool Transport, bool ProjectPressure) override
            {
                m_P.Float(7, Dt);
                m_P.Float(11, Time);
                Dispatch(AdvectV);
                m_V = 1 - m_V;
                Dispatch(EmitSource);
                m_V = 1 - m_V;
                std::swap(m_S[0], m_S[2]);
                Dispatch(Curl);
                Dispatch(Force);
                m_V = 1 - m_V;
                Dispatch(Divergence);
                Dispatch(ClearPressure);
                if (ProjectPressure)
                {
                    for (int I = 0; I < m_Settings.PressureIterations; ++I)
                        for (int Parity = 0; Parity < 2; ++Parity)
                        {
                            m_P.Integer(3, Parity);
                            Dispatch(Pressure);
                        }
                    Dispatch(Project);
                    m_V = 1 - m_V;
                }
                if (Transport)
                    TransportScalars();
            }
            void TransportScalars() override
            {
                Dispatch(AdvectS, 1);
                Dispatch(CorrectS);
                std::swap(m_S[0], m_S[2]);
            }
            void ApplyImpulse(const Vector3 &Position, float Radius, const Vector3 &Change) override
            {
                for (int A = 0; A < 3; ++A)
                {
                    m_P.Float(20 + A, Position[A]);
                    m_P.Float(24 + A, Change[A]);
                }
                m_P.Float(23, Radius);
                Dispatch(Impulse);
                m_V = 1 - m_V;
                const auto H = m_Settings.BoundsMax - m_Settings.BoundsMin;
                const auto N = m_Settings.Resolution;
                m_MaxRate += Change.Length() / std::min(H.x / N.x, std::min(H.y / N.y, H.z / N.z));
            }
            bool Readback(std::vector<float> &Density, std::vector<float> &Temperature) override
            {
                try
                {
                    Dispatch(MaxRate);
                    std::vector<float> Scalars(m_Count * 6), Rates((m_Count + 255) / 256);
                    m_Device->Download(
                        {{m_Scalar, Scalars.data(), m_Scalar->Size}, {m_Rates, Rates.data(), m_Rates->Size}});
                    if (!Finite(Scalars) || !Finite(Rates))
                        return false;
                    Density.resize(m_Count);
                    Temperature.resize(m_Count);
                    for (int I = 0; I < m_Count; ++I)
                    {
                        Density[I] = Scalars[m_S[0] * m_Count * 2 + I * 2];
                        Temperature[I] = Scalars[m_S[0] * m_Count * 2 + I * 2 + 1];
                    }
                    m_MaxRate = *std::max_element(Rates.begin(), Rates.end());
                    return true;
                }
                catch (const std::exception &)
                {
                    m_Device->Finish();
                    return false;
                }
            }
            bool ReadVelocity(std::vector<Vector3> &Velocity) const override
            {
                try
                {
                    std::vector<float> Values(m_Count * 8);
                    m_Device->Download({{m_Velocity, Values.data(), m_Velocity->Size}});
                    if (!Finite(Values))
                        return false;
                    Velocity.resize(m_Count);
                    for (int I = 0; I < m_Count; ++I)
                    {
                        const float *V = Values.data() + m_V * m_Count * 4 + I * 4;
                        Velocity[I] = Vector3(V[0], V[1], V[2]);
                    }
                    return true;
                }
                catch (const std::exception &)
                {
                    m_Device->Finish();
                    return false;
                }
            }
            bool ReadPressure(std::vector<float> &Pressure) const override
            {
                try
                {
                    std::vector<float> Values(m_Count);
                    m_Device->Download({{m_Pressure, Values.data(), m_Pressure->Size}});
                    if (!Finite(Values))
                        return false;
                    Pressure.swap(Values);
                    return true;
                }
                catch (const std::exception &)
                {
                    m_Device->Finish();
                    return false;
                }
            }
            float GetMaxRate() const override
            {
                return m_MaxRate;
            }
            bool ProjectSurfaceMass(const AerodynamicsPressureSystem &S, std::vector<float> &Response) override
            {
                try
                {
                    if (S.RowOffsets.empty() || S.CellOffsets.size() != S.CellIndices.size() + 1)
                        return false;
                    const int Rows = static_cast<int>(S.RowOffsets.size()) - 1;
                    Constants P;
                    for (int A = 0; A < 3; ++A)
                    {
                        P.Integer(A, m_Settings.Resolution[A]);
                        const float H = (m_Settings.BoundsMax[A] - m_Settings.BoundsMin[A]) / m_Settings.Resolution[A];
                        P.Float(4 + A, 1 / (H * H));
                    }
                    P.Integer(3, Rows);
                    P.Integer(20, m_Count * 3);
                    P.Integer(21, m_Count * 4);
                    P.Integer(22, m_Count * 5);
                    P.Integer(23, m_V * m_Count * 4);
                    std::vector<uint32_t> Packed;
                    auto Append = [&](int Slot, const void *Values, size_t Bytes)
                    {
                        P.Integer(Slot, static_cast<int>(Packed.size()));
                        const size_t Offset = Packed.size();
                        Packed.resize(Offset + Bytes / 4);
                        if (Bytes)
                            std::memcpy(Packed.data() + Offset, Values, Bytes);
                    };
                    Append(12, S.RowOffsets.data(), S.RowOffsets.size() * 4);
                    Append(13, S.RowEntries.data(), S.RowEntries.size() * sizeof(AerodynamicsPressureEntry));
                    std::vector<int> Ranges(m_Count * 2, 0);
                    for (size_t I = 0; I < S.CellIndices.size(); ++I)
                    {
                        const int Cell = S.CellIndices[I];
                        if (Cell < 0 || Cell >= m_Count)
                            return false;
                        Ranges[Cell * 2] = S.CellOffsets[I];
                        Ranges[Cell * 2 + 1] = S.CellOffsets[I + 1];
                    }
                    Append(14, Ranges.data(), Ranges.size() * 4);
                    Append(15, S.CellEntries.data(), S.CellEntries.size() * sizeof(AerodynamicsPressureEntry));
                    static_assert(sizeof(AerodynamicsPressureFace) == 32, "Pressure face layout mismatch");
                    Append(16, S.Faces.data(), S.Faces.size() * sizeof(AerodynamicsPressureFace));
                    std::vector<uint32_t> Lookup(m_Count * 3, UINT32_MAX);
                    for (size_t I = 0; I < S.Faces.size(); ++I)
                    {
                        const auto &F = S.Faces[I];
                        if (F.Cell < 0 || F.Cell >= m_Count || F.Axis < 0 || F.Axis >= 3)
                            return false;
                        Lookup[F.Cell * 3 + F.Axis] = static_cast<uint32_t>(I);
                    }
                    Append(17, Lookup.data(), Lookup.size() * 4);
                    ReserveMetalBuffer(*m_Device, m_System, Packed.size() * 4);
                    ReserveMetalBuffer(*m_Device, m_State, size_t(m_Count) * 6 * 4);
                    ReserveMetalBuffer(*m_Device, m_Response, size_t(std::max(1, Rows)) * 4);
                    m_Device->Upload(m_System, Packed.data(), Packed.size() * 4);
                    const Bindings B{m_State, m_Velocity, m_Wall, m_System, m_Response, m_Pressure};
                    auto DispatchPressure = [&](int Kernel)
                    {
                        m_Device->Dispatch(m_Coupling[Kernel], B, P,
                                           Kernel == SurfaceResponse ? std::max(1, Rows) : m_Count);
                        if (Kernel == RelaxPressure)
                        {
                            std::swap(P.Words[21], P.Words[20]);
                            std::swap(P.Words[20], P.Words[22]);
                        }
                    };
                    DispatchPressure(InitializeState);
                    float Alpha = 0;
                    constexpr float Theta = 1.001f, Delta = 0.999f;
                    for (int I = 0; I < std::max(40, m_Settings.PressureIterations); ++I)
                    {
                        const float Beta = I == 0 ? 0 : (I == 1 ? 0.5f : 0.25f) * Delta * Delta * Alpha * Alpha;
                        const float PreviousAlpha = Alpha;
                        Alpha = 1 / (Theta - (I == 0 ? 0 : Beta / Alpha));
                        P.Float(8, Alpha);
                        P.Float(9, I == 0 ? 0 : Beta * Alpha / PreviousAlpha);
                        DispatchPressure(SurfaceResponse);
                        DispatchPressure(RelaxPressure);
                    }
                    P.Float(8, 2.0f / 3);
                    P.Float(9, 0);
                    for (int I = 0; I < 4; ++I)
                    {
                        DispatchPressure(SurfaceResponse);
                        DispatchPressure(RelaxPressure);
                    }
                    DispatchPressure(SurfaceResponse);
                    DispatchPressure(ApplyCorrection);
                    std::vector<float> Values(Rows);
                    std::vector<MetalCompute::Readback> Requests{{m_Wall, m_WallUpload.data(), m_Wall->Size}};
                    if (Rows > 0) Requests.push_back({m_Response, Values.data(), Values.size() * sizeof(float)});
                    m_Device->Download(Requests);
                    if (!Finite(Values)) return false;
                    Response.swap(Values);
                    return true;
                }
                catch (const std::exception &)
                {
                    m_Device->Finish();
                    return false;
                }
            }
        };
    } // namespace
    std::unique_ptr<AerodynamicsGPUBackend> AerodynamicsGPUBackend::CreateMetal(
        const AerodynamicsSimulationSettings &Settings, const std::vector<unsigned char> &Solid)
    {
        try
        {
            std::unique_ptr<MetalAerodynamics> Backend(new MetalAerodynamics());
            if (!Backend->Initialize(Settings, Solid))
                return nullptr;
            return std::move(Backend);
        }
        catch (const std::exception &Error)
        {
            std::fprintf(stderr, "Aerodynamics Metal: %s\n", Error.what());
            return nullptr;
        }
    }
} // namespace Riemann
