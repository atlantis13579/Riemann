#include "../Compute/BackendAvailability.h"
#include "PBDBackend.h"
#include "../Compute/MetalCompute.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>
#include <stdexcept>

namespace Riemann
{
    namespace
    {
        using namespace MetalCompute;
        const char *ShaderSource =
#include "PBDMetalKernels.h"
            ;
        struct ComputeContext
        {
            std::unique_ptr<MetalDevice> Compute;
            SharedProgram Kernel;
            SharedBuffer Bodies, Contacts, Rows;
            std::vector<PBDBackendBody> HostBodies;
            std::vector<PBDBackendContact> HostContacts;
            std::mutex Mutex;
        };
    } // namespace
    void *CreatePBDMetalContext()
    {
        try
        {
            std::unique_ptr<ComputeContext> Context(new ComputeContext());
            Context->Compute = CreateMetalDevice();
            if (!Context->Compute)
                return nullptr;
            Context->Kernel = Context->Compute->CreateProgram(ShaderSource, "SolveContacts");
            return Context.release();
        }
        catch (const std::exception &)
        {
            return nullptr;
        }
    }
    void DestroyPBDMetalContext(void *Context)
    {
        delete static_cast<ComputeContext *>(Context);
    }
    bool IsPBDMetalBackendAvailable()
    {
        return IsMetalComputeAvailable();
    }
    bool ExecutePBDMetal(void *Opaque, PBDBackendBody *Bodies, uint32_t BodyCount, PBDBackendContact *Contacts,
                         uint32_t ContactCount, const uint32_t *RowIndices, const uint32_t *LayerOffsets,
                         uint32_t LayerCount, const PBDBackendParameters &Parameters, PBDBackendStage Stage)
    {
        if (ContactCount == 0)
            return true;
        if (!Opaque || !Bodies || !Contacts || !RowIndices || !LayerOffsets || BodyCount == 0 || LayerCount == 0 ||
            LayerCount > ContactCount || !std::isfinite(Parameters.DeltaTime) || Parameters.DeltaTime <= 0.0f ||
            !std::isfinite(Parameters.Alpha) || Parameters.Alpha < 0.0f || !std::isfinite(Parameters.PenetrationSlop) ||
            Parameters.PenetrationSlop < 0.0f || !std::isfinite(Parameters.MaxPositionCorrection) ||
            Parameters.MaxPositionCorrection <= 0.0f || !std::isfinite(Parameters.MaxAngularCorrection) ||
            Parameters.MaxAngularCorrection <= 0.0f || LayerOffsets[0] != 0 ||
            LayerOffsets[LayerCount] != ContactCount || (uint32_t)Stage > (uint32_t)PBDBackendStage::Velocity)
            return false;
        for (uint32_t Layer = 0; Layer < LayerCount; ++Layer)
            if (LayerOffsets[Layer] >= LayerOffsets[Layer + 1] || LayerOffsets[Layer + 1] > ContactCount)
                return false;
        for (uint32_t Row = 0; Row < ContactCount; ++Row)
            if (RowIndices[Row] >= ContactCount || Contacts[Row].Bodies.x >= BodyCount ||
                Contacts[Row].Bodies.y >= BodyCount)
                return false;

        ComputeContext &Context = *static_cast<ComputeContext *>(Opaque);
        std::lock_guard<std::mutex> Lock(Context.Mutex);
        try
        {
            const size_t BodyBytes = size_t(BodyCount) * sizeof(PBDBackendBody);
            const size_t ContactBytes = size_t(ContactCount) * sizeof(PBDBackendContact);
            ReserveMetalBuffer(*Context.Compute, Context.Bodies, BodyBytes);
            ReserveMetalBuffer(*Context.Compute, Context.Contacts, ContactBytes);
            ReserveMetalBuffer(*Context.Compute, Context.Rows, size_t(ContactCount) * sizeof(uint32_t));
            Context.HostBodies.resize(BodyCount);
            Context.HostContacts.resize(ContactCount);
            Context.Compute->Upload(Context.Bodies, Bodies, BodyBytes);
            Context.Compute->Upload(Context.Contacts, Contacts, ContactBytes);
            Context.Compute->Upload(Context.Rows, RowIndices, size_t(ContactCount) * sizeof(uint32_t));
            Constants P;
            P.Float(0, Parameters.DeltaTime);
            P.Float(1, Parameters.Alpha);
            P.Float(2, Parameters.PenetrationSlop);
            P.Float(3, Parameters.MaxPositionCorrection);
            P.Float(4, Parameters.MaxAngularCorrection);
            P.Integer(7, static_cast<int>(Stage));
            for (uint32_t Layer = 0; Layer < LayerCount; ++Layer)
            {
                const uint32_t End = LayerOffsets[Layer + 1];
                for (uint32_t Start = LayerOffsets[Layer]; Start < End;)
                {
                    const uint32_t Count = std::min(End - Start, 65535u * 256u);
                    P.Integer(5, Start);
                    P.Integer(6, Count);
                    Context.Compute->Dispatch(Context.Kernel, {Context.Bodies, Context.Contacts, Context.Rows}, P,
                                              Count);
                    Start += Count;
                }
            }
            Context.Compute->Download({{Context.Bodies, Context.HostBodies.data(), BodyBytes},
                                       {Context.Contacts, Context.HostContacts.data(), ContactBytes}});
            if (!ValidatePBDBackendOutput(Context.HostBodies.data(), BodyCount, Context.HostContacts.data(),
                                          ContactCount))
                return false;
            // Commit the complete result only after every device operation succeeds.
            std::memcpy(Bodies, Context.HostBodies.data(), BodyBytes);
            std::memcpy(Contacts, Context.HostContacts.data(), ContactBytes);
            return true;
        }
        catch (const std::exception &)
        {
            Context.Compute->Finish();
            return false;
        }
    }
} // namespace Riemann
