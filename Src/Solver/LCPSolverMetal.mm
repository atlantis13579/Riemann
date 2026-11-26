#include "../Compute/BackendAvailability.h"
#include "../Compute/MetalCompute.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace Riemann
{
    namespace
    {
        using namespace MetalCompute;
        const char *ShaderSource =
#include "LCPMetalKernels.h"
            ;
        struct SolverConstants
        {
            uint32_t MatrixOffsetBytes;
            uint32_t BiasOffsetBytes;
            uint32_t LowerOffsetBytes;
            uint32_t UpperOffsetBytes;
            uint32_t FrictionCoefficientOffsetBytes;
            uint32_t InitialLambdaOffsetBytes;
            uint32_t FrictionNormalOffsetBytes;
            uint32_t MatrixOffsetsOffsetBytes;
            uint32_t VectorOffsetsOffsetBytes;
            uint32_t DimensionsOffsetBytes;
            uint32_t LambdaOutputOffsetBytes;
            uint32_t ResultsOutputOffsetBytes;
            uint32_t IslandBase;
            uint32_t DispatchIslandCount;
            uint32_t MaxIterations;
            float Tolerance;
            float Relaxation;
            uint32_t Padding[3];
        };

        static_assert(sizeof(SolverConstants) == 80, "Metal constant layout must match the shader");

        struct InputLayout
        {
            size_t Matrix;
            size_t Bias;
            size_t Lower;
            size_t Upper;
            size_t FrictionCoefficient;
            size_t InitialLambda;
            size_t FrictionNormal;
            size_t MatrixOffsets;
            size_t VectorOffsets;
            size_t Dimensions;
            size_t TotalBytes;
        };

        struct OutputLayout
        {
            size_t Lambda;
            size_t Results;
            size_t TotalBytes;
        };

        bool CheckedAdd(size_t Left, size_t Right, size_t *Result)
        {
            if (Left > std::numeric_limits<size_t>::max() - Right)
            {
                return false;
            }
            *Result = Left + Right;
            return true;
        }

        bool CheckedMultiply(size_t Left, size_t Right, size_t *Result)
        {
            if (Left != 0 && Right > std::numeric_limits<size_t>::max() / Left)
            {
                return false;
            }
            *Result = Left * Right;
            return true;
        }

        bool AppendSection(size_t *Cursor, size_t ElementSize, size_t Count, size_t *Offset)
        {
            const size_t Aligned = (*Cursor + 15u) & ~size_t(15u);
            if (Aligned < *Cursor)
            {
                return false;
            }
            size_t Bytes = 0;
            if (!CheckedMultiply(ElementSize, Count, &Bytes) || !CheckedAdd(Aligned, Bytes, Cursor))
            {
                return false;
            }
            *Offset = Aligned;
            return true;
        }

        bool BuildLayouts(size_t MatrixElementCount, size_t VectorElementCount, size_t IslandCount, InputLayout *Input,
                          OutputLayout *Output)
        {
            size_t Cursor = 0;
            if (!AppendSection(&Cursor, sizeof(float), MatrixElementCount, &Input->Matrix) ||
                !AppendSection(&Cursor, sizeof(float), VectorElementCount, &Input->Bias) ||
                !AppendSection(&Cursor, sizeof(float), VectorElementCount, &Input->Lower) ||
                !AppendSection(&Cursor, sizeof(float), VectorElementCount, &Input->Upper) ||
                !AppendSection(&Cursor, sizeof(float), VectorElementCount, &Input->FrictionCoefficient) ||
                !AppendSection(&Cursor, sizeof(float), VectorElementCount, &Input->InitialLambda) ||
                !AppendSection(&Cursor, sizeof(int), VectorElementCount, &Input->FrictionNormal) ||
                !AppendSection(&Cursor, sizeof(int), IslandCount, &Input->MatrixOffsets) ||
                !AppendSection(&Cursor, sizeof(int), IslandCount, &Input->VectorOffsets) ||
                !AppendSection(&Cursor, sizeof(int), IslandCount, &Input->Dimensions))
            {
                return false;
            }
            Input->TotalBytes = (Cursor + 15u) & ~size_t(15u);
            if (Input->TotalBytes < Cursor)
            {
                return false;
            }

            Cursor = 0;
            if (!AppendSection(&Cursor, sizeof(float), VectorElementCount, &Output->Lambda) ||
                !AppendSection(&Cursor, 16u, IslandCount, &Output->Results))
            {
                return false;
            }
            Output->TotalBytes = (Cursor + 15u) & ~size_t(15u);
            return Output->TotalBytes >= Cursor && Input->TotalBytes <= std::numeric_limits<uint32_t>::max() &&
                   Output->TotalBytes <= std::numeric_limits<uint32_t>::max();
        }

        bool IsFiniteArray(const float *Values, size_t Count)
        {
            for (size_t Index = 0; Index < Count; ++Index)
            {
                if (!std::isfinite(Values[Index]))
                {
                    return false;
                }
            }
            return true;
        }
        bool ValidateBatchedInputs(const float *Matrices, const int *MatrixOffsets, const int *VectorOffsets,
                                   const int *Dimensions, int IslandCount, size_t MatrixElementCount,
                                   int VectorElementCount, const float *Bias, const float *Lower, const float *Upper,
                                   const int *FrictionNormal, const float *FrictionCoefficient, const float *Lambda,
                                   int MaxIterations, float Tolerance, float Relaxation, bool InputsPrevalidated)
        {
            if (IslandCount < 0 || VectorElementCount < 0 || MaxIterations <= 0 || !std::isfinite(Tolerance) ||
                Tolerance < 0.0f || !std::isfinite(Relaxation) || Relaxation <= 0.0f || Relaxation >= 2.0f ||
                ((FrictionNormal == nullptr) != (FrictionCoefficient == nullptr)))
            {
                return false;
            }
            if (IslandCount == 0)
            {
                return MatrixElementCount == 0 && VectorElementCount == 0;
            }
            if (!Matrices || !MatrixOffsets || !VectorOffsets || !Dimensions || !Bias || !Lambda)
            {
                return false;
            }

            size_t ExpectedMatrixOffset = 0;
            size_t ExpectedVectorOffset = 0;
            for (int Island = 0; Island < IslandCount; ++Island)
            {
                const int Dimension = Dimensions[Island];
                const int MatrixOffset = MatrixOffsets[Island];
                const int VectorOffset = VectorOffsets[Island];
                if (Dimension <= 0 || MatrixOffset < 0 || VectorOffset < 0 ||
                    static_cast<size_t>(MatrixOffset) != ExpectedMatrixOffset ||
                    static_cast<size_t>(VectorOffset) != ExpectedVectorOffset)
                {
                    return false;
                }
                const size_t DimensionSize = static_cast<size_t>(Dimension);
                size_t MatrixSize = 0;
                if (!CheckedMultiply(DimensionSize, DimensionSize, &MatrixSize) ||
                    static_cast<size_t>(MatrixOffset) > MatrixElementCount ||
                    MatrixSize > MatrixElementCount - static_cast<size_t>(MatrixOffset) ||
                    static_cast<size_t>(VectorOffset) > static_cast<size_t>(VectorElementCount) ||
                    DimensionSize > static_cast<size_t>(VectorElementCount) - static_cast<size_t>(VectorOffset))
                {
                    return false;
                }
                if (!CheckedAdd(ExpectedMatrixOffset, MatrixSize, &ExpectedMatrixOffset) ||
                    !CheckedAdd(ExpectedVectorOffset, DimensionSize, &ExpectedVectorOffset))
                {
                    return false;
                }

                if (InputsPrevalidated)
                {
                    continue;
                }
                for (int Row = 0; Row < Dimension; ++Row)
                {
                    const size_t GlobalRow = static_cast<size_t>(VectorOffset + Row);
                    const float *MatrixRow =
                        Matrices + static_cast<size_t>(MatrixOffset) + static_cast<size_t>(Row) * Dimension;
                    if (!std::isfinite(MatrixRow[Row]) || MatrixRow[Row] <= 1.0e-12f ||
                        !std::isfinite(Bias[GlobalRow]) || !std::isfinite(Lambda[GlobalRow]))
                    {
                        return false;
                    }
                    for (int Column = 0; Column < Dimension; ++Column)
                    {
                        if (!std::isfinite(MatrixRow[Column]))
                        {
                            return false;
                        }
                    }
                    const float LowerBound = Lower ? Lower[GlobalRow] : 0.0f;
                    const float UpperBound = Upper ? Upper[GlobalRow] : FLT_MAX;
                    if (std::isnan(LowerBound) || std::isnan(UpperBound) || LowerBound > UpperBound)
                    {
                        return false;
                    }
                    if (FrictionNormal)
                    {
                        const int NormalRow = FrictionNormal[GlobalRow];
                        if (NormalRow < -1 || NormalRow >= Dimension || NormalRow == Row ||
                            !std::isfinite(FrictionCoefficient[GlobalRow]) || FrictionCoefficient[GlobalRow] < 0.0f)
                        {
                            return false;
                        }
                        if (NormalRow >= 0)
                        {
                            const size_t Normal = static_cast<size_t>(VectorOffset + NormalRow);
                            const float NormalLower = Lower ? Lower[Normal] : 0.0f;
                            if (FrictionNormal[Normal] >= 0 || NormalLower < 0.0f || LowerBound > 0.0f ||
                                UpperBound < 0.0f)
                            {
                                return false;
                            }
                        }
                    }
                }
            }
            return ExpectedMatrixOffset == MatrixElementCount &&
                   ExpectedVectorOffset == static_cast<size_t>(VectorElementCount);
        }

    } // namespace
    struct LCPMetalBatchContext
    {
        std::unique_ptr<MetalDevice> Compute;
        SharedProgram Kernel;
        SharedBuffer InputBuffer, OutputBuffer;
        std::vector<unsigned char> HostInput, HostOutput;
    };
    LCPMetalBatchContext *CreateLCPMetalBatchContext()
    {
        try
        {
            std::unique_ptr<LCPMetalBatchContext> Context(new LCPMetalBatchContext());
            Context->Compute = CreateMetalDevice();
            if (!Context->Compute)
                return nullptr;
            Context->Kernel = Context->Compute->CreateProgram(ShaderSource, "SolveProjectedGaussSeidel");
            return Context.release();
        }
        catch (const std::exception &)
        {
            return nullptr;
        }
    }
    void DestroyLCPMetalBatchContext(LCPMetalBatchContext *Context)
    {
        delete Context;
    }
    bool IsLCPMetalBackendAvailable()
    {
        return IsMetalComputeAvailable();
    }
    bool SolveProjectedGaussSeidelMetalBatched(LCPMetalBatchContext *Context, const float *Matrices,
                                               const int *MatrixOffsets, const int *VectorOffsets,
                                               const int *Dimensions, int IslandCount, size_t MatrixElementCount,
                                               int VectorElementCount, const float *Bias, const float *Lower,
                                               const float *Upper, const int *FrictionNormal,
                                               const float *FrictionCoefficient, float *Lambda, int MaxIterations,
                                               float Tolerance, float Relaxation, bool InputsPrevalidated,
                                               int *Iterations, float *Residual, bool *Converged)
    {
        for (int Island = 0; Island < std::max(0, IslandCount); ++Island)
        {
            if (Iterations)
                Iterations[Island] = 0;
            if (Residual)
                Residual[Island] = 0.0f;
            if (Converged)
                Converged[Island] = false;
        }
        if (!ValidateBatchedInputs(Matrices, MatrixOffsets, VectorOffsets, Dimensions, IslandCount, MatrixElementCount,
                                   VectorElementCount, Bias, Lower, Upper, FrictionNormal, FrictionCoefficient, Lambda,
                                   MaxIterations, Tolerance, Relaxation, InputsPrevalidated))
        {
            return false;
        }
        if (IslandCount == 0)
        {
            return true;
        }
        if (!Context || !Context->Compute || !Context->Kernel)
            return false;

        InputLayout Input = {};
        OutputLayout Output = {};
        if (!BuildLayouts(MatrixElementCount, VectorElementCount, IslandCount, &Input, &Output))
            return false;
        try
        {
            ReserveMetalBuffer(*Context->Compute, Context->InputBuffer, Input.TotalBytes);
            ReserveMetalBuffer(*Context->Compute, Context->OutputBuffer, Output.TotalBytes);
            Context->HostInput.resize(Input.TotalBytes);
            Context->HostOutput.resize(Output.TotalBytes);
            std::fill(Context->HostInput.begin(), Context->HostInput.end(), 0);
            unsigned char *Packed = Context->HostInput.data();
            std::memcpy(Packed + Input.Matrix, Matrices, sizeof(float) * MatrixElementCount);
            std::memcpy(Packed + Input.Bias, Bias, sizeof(float) * static_cast<size_t>(VectorElementCount));
            std::memcpy(Packed + Input.InitialLambda, Lambda, sizeof(float) * static_cast<size_t>(VectorElementCount));
            std::memcpy(Packed + Input.MatrixOffsets, MatrixOffsets, sizeof(int) * static_cast<size_t>(IslandCount));
            std::memcpy(Packed + Input.VectorOffsets, VectorOffsets, sizeof(int) * static_cast<size_t>(IslandCount));
            std::memcpy(Packed + Input.Dimensions, Dimensions, sizeof(int) * static_cast<size_t>(IslandCount));

            float *PackedLower = reinterpret_cast<float *>(Packed + Input.Lower);
            float *PackedUpper = reinterpret_cast<float *>(Packed + Input.Upper);
            float *PackedFrictionCoefficient = reinterpret_cast<float *>(Packed + Input.FrictionCoefficient);
            int *PackedFrictionNormal = reinterpret_cast<int *>(Packed + Input.FrictionNormal);
            for (int Row = 0; Row < VectorElementCount; ++Row)
            {
                PackedLower[Row] = Lower ? Lower[Row] : 0.0f;
                PackedUpper[Row] = Upper ? Upper[Row] : FLT_MAX;
                PackedFrictionNormal[Row] = FrictionNormal ? FrictionNormal[Row] : -1;
                PackedFrictionCoefficient[Row] = FrictionCoefficient ? FrictionCoefficient[Row] : 0.0f;
            }

            SolverConstants Constants = {};
            Constants.MatrixOffsetBytes = static_cast<uint32_t>(Input.Matrix);
            Constants.BiasOffsetBytes = static_cast<uint32_t>(Input.Bias);
            Constants.LowerOffsetBytes = static_cast<uint32_t>(Input.Lower);
            Constants.UpperOffsetBytes = static_cast<uint32_t>(Input.Upper);
            Constants.FrictionCoefficientOffsetBytes = static_cast<uint32_t>(Input.FrictionCoefficient);
            Constants.InitialLambdaOffsetBytes = static_cast<uint32_t>(Input.InitialLambda);
            Constants.FrictionNormalOffsetBytes = static_cast<uint32_t>(Input.FrictionNormal);
            Constants.MatrixOffsetsOffsetBytes = static_cast<uint32_t>(Input.MatrixOffsets);
            Constants.VectorOffsetsOffsetBytes = static_cast<uint32_t>(Input.VectorOffsets);
            Constants.DimensionsOffsetBytes = static_cast<uint32_t>(Input.Dimensions);
            Constants.LambdaOutputOffsetBytes = static_cast<uint32_t>(Output.Lambda);
            Constants.ResultsOutputOffsetBytes = static_cast<uint32_t>(Output.Results);
            Constants.MaxIterations = static_cast<uint32_t>(MaxIterations);
            Constants.Tolerance = Tolerance;
            Constants.Relaxation = Relaxation;

            Context->Compute->Upload(Context->InputBuffer, Context->HostInput.data(), Input.TotalBytes);
            for (int Base = 0; Base < IslandCount; Base += 65535)
            {
                const int Count = std::min(65535, IslandCount - Base);
                Constants.IslandBase = Base;
                Constants.DispatchIslandCount = Count;
                MetalCompute::Constants P;
                std::memcpy(P.Words.data(), &Constants, sizeof(Constants));
                Context->Compute->Dispatch(Context->Kernel, {Context->InputBuffer, Context->OutputBuffer}, P,
                                           Count * 256);
            }
            Context->Compute->Download({{Context->OutputBuffer, Context->HostOutput.data(), Output.TotalBytes}});
            const unsigned char *Readback = Context->HostOutput.data();
            const float *SolvedLambda = reinterpret_cast<const float *>(Readback + Output.Lambda);
            if (!IsFiniteArray(SolvedLambda, static_cast<size_t>(VectorElementCount)))
            {
                return false;
            }
            for (int Island = 0; Island < IslandCount; ++Island)
            {
                const unsigned char *ResultBytes = Readback + Output.Results + static_cast<size_t>(Island) * 16u;
                uint32_t ResultIterations = 0;
                float ResultResidual = 0.0f;
                uint32_t ResultConverged = 0;
                std::memcpy(&ResultIterations, ResultBytes + 0, sizeof(ResultIterations));
                std::memcpy(&ResultResidual, ResultBytes + 4, sizeof(ResultResidual));
                std::memcpy(&ResultConverged, ResultBytes + 8, sizeof(ResultConverged));
                if (ResultIterations != static_cast<uint32_t>(MaxIterations) || !std::isfinite(ResultResidual) ||
                    ResultResidual < 0.0f || ResultConverged > 1u)
                {
                    return false;
                }
            }

            std::memcpy(Lambda, SolvedLambda, sizeof(float) * static_cast<size_t>(VectorElementCount));
            for (int Island = 0; Island < IslandCount; ++Island)
            {
                const unsigned char *ResultBytes = Readback + Output.Results + static_cast<size_t>(Island) * 16u;
                uint32_t ResultIterations = 0;
                float ResultResidual = 0.0f;
                uint32_t ResultConverged = 0;
                std::memcpy(&ResultIterations, ResultBytes + 0, sizeof(ResultIterations));
                std::memcpy(&ResultResidual, ResultBytes + 4, sizeof(ResultResidual));
                std::memcpy(&ResultConverged, ResultBytes + 8, sizeof(ResultConverged));
                if (Iterations)
                    Iterations[Island] = static_cast<int>(ResultIterations);
                if (Residual)
                    Residual[Island] = ResultResidual;
                if (Converged)
                    Converged[Island] = ResultConverged != 0;
            }
            return true;
        }
        catch (const std::exception &)
        {
            Context->Compute->Finish();
            return false;
        }
    }
    bool SolveProjectedGaussSeidelMetal(const float *Matrix, const float *Bias, const float *Lower, const float *Upper,
                                        const int *FrictionNormal, const float *FrictionCoefficient, int Dimension,
                                        float *Lambda, int MaxIterations, float Tolerance, float Relaxation,
                                        int *Iterations, float *Residual, bool *Converged)
    {
        if (Iterations)
            *Iterations = 0;
        if (Residual)
            *Residual = 0.0f;
        if (Converged)
            *Converged = false;
        if (Dimension < 0 || MaxIterations <= 0 || !std::isfinite(Tolerance) || Tolerance < 0.0f ||
            !std::isfinite(Relaxation) || Relaxation <= 0.0f || Relaxation >= 2.0f ||
            ((FrictionNormal == nullptr) != (FrictionCoefficient == nullptr)) ||
            (Dimension > 0 && (!Matrix || !Bias || !Lambda)) || !IsLCPMetalBackendAvailable())
        {
            return false;
        }
        if (Dimension == 0)
        {
            if (Converged)
                *Converged = true;
            return true;
        }

        struct ThreadContext
        {
            ThreadContext() : Context(CreateLCPMetalBatchContext()) {}
            ~ThreadContext()
            {
                DestroyLCPMetalBatchContext(Context);
            }
            LCPMetalBatchContext *Context;
        };
        static thread_local ThreadContext PerThread;
        if (!PerThread.Context)
        {
            return false;
        }

        const int MatrixOffset = 0;
        const int VectorOffset = 0;
        const size_t MatrixElementCount = static_cast<size_t>(Dimension) * Dimension;
        return SolveProjectedGaussSeidelMetalBatched(PerThread.Context, Matrix, &MatrixOffset, &VectorOffset,
                                                     &Dimension, 1, MatrixElementCount, Dimension, Bias, Lower, Upper,
                                                     FrictionNormal, FrictionCoefficient, Lambda, MaxIterations,
                                                     Tolerance, Relaxation, false, Iterations, Residual, Converged);
    }

} // namespace Riemann
