#include "PBDBackend.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <mutex>
#include <new>
#include <vector>

#if defined(RIEMANN_ENABLE_COMPUTE_SHADER) && defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <d3d11.h>
#include <d3dcompiler.h>
#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "d3dcompiler.lib")
#endif

namespace Riemann
{
#if defined(RIEMANN_ENABLE_COMPUTE_SHADER) && defined(_WIN32)
	namespace
	{
		template <class T> void SafeRelease(T*& Object)
		{
			if (Object) { Object->Release(); Object = nullptr; }
		}

		const char* ShaderSource = R"HLSL(
struct Body
{
	float4 Position;
	float4 Rotation;
	float4 LinearVelocity;
	float4 AngularVelocity;
	float4 InverseInertia0;
	float4 InverseInertia1;
	float4 InverseInertia2;
	uint4 Flags;
};

struct Contact
{
	uint4 Bodies;
	float4 AnchorA;
	float4 AnchorB;
	float4 Normal;
	float4 Tangent;
	float4 Binormal;
	float4 Lambda;
	float4 Material;
};

StructuredBuffer<uint> RowIndices : register(t0);
RWStructuredBuffer<Body> Bodies : register(u0);
RWStructuredBuffer<Contact> Contacts : register(u1);

cbuffer Parameters : register(b0)
{
	float DeltaTime;
	float Alpha;
	float PenetrationSlop;
	float MaxPositionCorrection;
	float MaxAngularCorrection;
	uint RowStart;
	uint RowCount;
	uint Stage;
};

precise float Dot3(float3 A, float3 B)
{
	precise float Result = (A.x * B.x + A.y * B.y) + A.z * B.z;
	return Result;
}

precise float3 Cross3(float3 A, float3 B)
{
	precise float3 Result = float3(A.y * B.z - A.z * B.y,
		A.z * B.x - A.x * B.z, A.x * B.y - A.y * B.x);
	return Result;
}

precise float3 RotateVector(float4 Q, float3 V)
{
	precise float3 T = Cross3(Q.xyz, V) * 2.0f;
	precise float3 Result = (V + T * Q.w) + Cross3(Q.xyz, T);
	return Result;
}

precise float4 MultiplyRotation(float4 A, float4 B)
{
	precise float3 Vector = (Cross3(A.xyz, B.xyz) + A.xyz * B.w) + B.xyz * A.w;
	precise float Scalar = ((A.w * B.w - A.x * B.x) - A.y * B.y) - A.z * B.z;
	return float4(Vector, Scalar);
}

precise float3 InverseInertia(Body B, float3 V)
{
	if (B.Flags.x == 0) return float3(0, 0, 0);
	precise float3 Local = RotateVector(float4(-B.Rotation.xyz, B.Rotation.w), V);
	precise float3 Result = float3(Dot3(B.InverseInertia0.xyz, Local),
		Dot3(B.InverseInertia1.xyz, Local), Dot3(B.InverseInertia2.xyz, Local));
	return RotateVector(B.Rotation, Result);
}

precise float EffectiveMass(Body A, Body B, float3 RA, float3 RB, float3 Axis)
{
	precise float3 AngularA = Cross3(RA, Axis), AngularB = Cross3(RB, Axis);
	precise float Result = A.Position.w + B.Position.w + Dot3(AngularA, InverseInertia(A, AngularA))
		+ Dot3(AngularB, InverseInertia(B, AngularB));
	return Result;
}

precise float3 PointVelocity(Body B, float3 Offset)
{
	precise float3 Result = B.Flags.y != 0 ? B.LinearVelocity.xyz + Cross3(B.AngularVelocity.xyz, Offset) : float3(0, 0, 0);
	return Result;
}

void PositionImpulse(inout Body B, float3 Impulse, float3 Offset)
{
	if (B.Flags.x == 0 || Dot3(Impulse, Impulse) <= 1.0e-20f) return;
	precise float3 Angular = InverseInertia(B, Cross3(Offset, Impulse));
	precise float3 Position = B.Position.xyz + Impulse * B.Position.w;
	B.Position.xyz = Position;
	precise float Angle = sqrt(Dot3(Angular, Angular));
	if (Angle > 1.0e-8f)
	{
		precise float3 Axis = float3(Angular.x / Angle, Angular.y / Angle, Angular.z / Angle);
		precise float AxisLength = sqrt(Dot3(Axis, Axis));
		Axis = float3(Axis.x / AxisLength, Axis.y / AxisLength, Axis.z / AxisLength);
		precise float Sine = sin(Angle * 0.5f);
		precise float4 Delta = float4(Axis * Sine, cos(Angle * 0.5f));
		precise float4 Result = MultiplyRotation(Delta, B.Rotation);
		precise float Length = sqrt(((Result.w * Result.w + Result.x * Result.x)
			+ Result.y * Result.y) + Result.z * Result.z);
		B.Rotation = float4(Result.x / Length, Result.y / Length, Result.z / Length, Result.w / Length);
	}
	B.Flags.z = 1;
}

void VelocityImpulse(inout Body B, float3 Impulse, float3 Offset)
{
	if (B.Flags.x == 0 || Dot3(Impulse, Impulse) <= 1.0e-20f) return;
	precise float3 Linear = B.LinearVelocity.xyz + Impulse * B.Position.w;
	precise float3 Angular = B.AngularVelocity.xyz + InverseInertia(B, Cross3(Offset, Impulse));
	B.LinearVelocity.xyz = Linear;
	B.AngularVelocity.xyz = Angular;
	B.Flags.z = 1;
}

void ProjectPosition(inout Body A, inout Body B, inout Contact C, float3 RA, float3 RB)
{
	precise float Weight = EffectiveMass(A, B, RA, RB, C.Normal.xyz);
	if (!isfinite(Weight) || Weight <= 1.0e-9f) return;
	precise float3 PA = A.Position.xyz + RA, PB = B.Position.xyz + RB;
	precise float Error = Dot3(PB - PA, C.Normal.xyz) + PenetrationSlop;
	precise float Delta;
	if (Alpha == 0.0f) Delta = -Error / Weight;
	else
	{
		precise float Scale = max(Weight, Alpha);
		precise float Denominator = Weight / Scale + Alpha / Scale;
		Delta = -(Error / Scale) / Denominator - C.Lambda.x * ((Alpha / Scale) / Denominator);
	}
	Delta = max(-C.Lambda.x, Delta);
	if (!isfinite(Delta)) return;
	precise float Translation = max(A.Position.w, B.Position.w) * abs(Delta);
	precise float3 AngularA = InverseInertia(A, Cross3(RA, C.Normal.xyz));
	precise float3 AngularB = InverseInertia(B, Cross3(RB, C.Normal.xyz));
	precise float Rotation = max(sqrt(Dot3(AngularA, AngularA)), sqrt(Dot3(AngularB, AngularB))) * abs(Delta);
	precise float Scale = 1.0f;
	if (Translation > MaxPositionCorrection) Scale = min(Scale, MaxPositionCorrection / Translation);
	if (Rotation > MaxAngularCorrection) Scale = min(Scale, MaxAngularCorrection / Rotation);
	Delta *= Scale;
	C.Lambda.x += Delta;
	PositionImpulse(A, -C.Normal.xyz * Delta, RA);
	PositionImpulse(B, C.Normal.xyz * Delta, RB);
}

void ProjectVelocity(inout Body A, inout Body B, inout Contact C, float3 RA, float3 RB)
{
	precise float Weight = EffectiveMass(A, B, RA, RB, C.Normal.xyz);
	if (!isfinite(Weight) || Weight <= 1.0e-9f) return;
	precise float Speed = Dot3(PointVelocity(B, RB) - PointVelocity(A, RA), C.Normal.xyz);
	precise float PreviousImpulse = C.Lambda.x / DeltaTime + C.Lambda.y;
	precise float ImpulseLimit = min(A.LinearVelocity.w, B.LinearVelocity.w);
	precise float TotalImpulse = max(0.0f, min(ImpulseLimit,
		PreviousImpulse + (C.Material.x - Speed) / Weight));
	precise float Delta = TotalImpulse - PreviousImpulse;
	C.Lambda.y += Delta;
	VelocityImpulse(A, -C.Normal.xyz * Delta, RA);
	VelocityImpulse(B, C.Normal.xyz * Delta, RB);

	precise float3 Relative = PointVelocity(B, RB) - PointVelocity(A, RA);
	precise float KT = EffectiveMass(A, B, RA, RB, C.Tangent.xyz);
	precise float KB = EffectiveMass(A, B, RA, RB, C.Binormal.xyz);
	precise float KTB = Dot3(Cross3(RA, C.Tangent.xyz), InverseInertia(A, Cross3(RA, C.Binormal.xyz)))
		+ Dot3(Cross3(RB, C.Tangent.xyz), InverseInertia(B, Cross3(RB, C.Binormal.xyz)));
	precise float Determinant = KT * KB - KTB * KTB;
	if (!isfinite(Determinant) || Determinant <= 1.0e-12f) return;
	precise float VT = Dot3(Relative, C.Tangent.xyz), VB = Dot3(Relative, C.Binormal.xyz);
	precise float Tangent = C.Lambda.z - (KB * VT - KTB * VB) / Determinant;
	precise float Binormal = C.Lambda.w - (KT * VB - KTB * VT) / Determinant;
	precise float Magnitude = sqrt(Tangent * Tangent + Binormal * Binormal);
	if (Magnitude > C.Material.z * TotalImpulse && Magnitude > 0.0f)
	{
		precise float Scale = min(1.0f, C.Material.y * TotalImpulse / Magnitude);
		Tangent *= Scale;
		Binormal *= Scale;
	}
	precise float3 FrictionImpulse = C.Tangent.xyz * (Tangent - C.Lambda.z)
		+ C.Binormal.xyz * (Binormal - C.Lambda.w);
	C.Lambda.z = Tangent;
	C.Lambda.w = Binormal;
	VelocityImpulse(A, -FrictionImpulse, RA);
	VelocityImpulse(B, FrictionImpulse, RB);
}

[numthreads(64, 1, 1)]
void SolveContacts(uint3 Id : SV_DispatchThreadID)
{
	if (Id.x >= RowCount) return;
	uint Row = RowIndices[RowStart + Id.x];
	Contact C = Contacts[Row];
	Body A = Bodies[C.Bodies.x], B = Bodies[C.Bodies.y];
	precise float3 RA = RotateVector(A.Rotation, C.AnchorA.xyz);
	precise float3 RB = RotateVector(B.Rotation, C.AnchorB.xyz);
	if (Stage == 0) ProjectPosition(A, B, C, RA, RB);
	else if (Stage == 1)
	{
		precise float3 Impulse = C.Normal.xyz * (C.Lambda.x / DeltaTime);
		VelocityImpulse(A, -Impulse, RA);
		VelocityImpulse(B, Impulse, RB);
	}
	else ProjectVelocity(A, B, C, RA, RB);
	// Static and kinematic bodies are shared read-only among independent rows.
	if (A.Flags.x != 0) Bodies[C.Bodies.x] = A;
	if (B.Flags.x != 0) Bodies[C.Bodies.y] = B;
	Contacts[Row] = C;
}
)HLSL";

		struct SolverConstants
		{
			float DeltaTime, Alpha, PenetrationSlop, MaxPositionCorrection;
			float MaxAngularCorrection;
			uint32_t RowStart, RowCount, Stage;
		};
		static_assert(sizeof(SolverConstants) == 32, "PBD constants must match the shader layout");

		struct StructuredBuffer
		{
			ID3D11Buffer* Buffer = nullptr;
			ID3D11ShaderResourceView* Input = nullptr;
			ID3D11UnorderedAccessView* Output = nullptr;
			ID3D11Buffer* Readback = nullptr;
			uint32_t Capacity = 0;

			void Release()
			{
				SafeRelease(Readback);
				SafeRelease(Output);
				SafeRelease(Input);
				SafeRelease(Buffer);
				Capacity = 0;
			}
		};

		struct ComputeContext
		{
			ID3D11Device* Device = nullptr;
			ID3D11DeviceContext* DeviceContext = nullptr;
			ID3D11ComputeShader* Shader = nullptr;
			ID3D11Buffer* Constants = nullptr;
			StructuredBuffer Bodies, Contacts, Rows;
			std::vector<PBDBackendBody> HostBodies;
			std::vector<PBDBackendContact> HostContacts;
			std::vector<uint32_t> HostRows;
			std::mutex Mutex;

			~ComputeContext()
			{
				Rows.Release(); Contacts.Release(); Bodies.Release();
				SafeRelease(Constants); SafeRelease(Shader);
				SafeRelease(DeviceContext); SafeRelease(Device);
			}
		};

		bool Initialize(ComputeContext& Context)
		{
			const D3D_FEATURE_LEVEL Levels[] = { D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0 };
			D3D_FEATURE_LEVEL Level = D3D_FEATURE_LEVEL_9_1;
			HRESULT Result = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0,
				Levels, 2, D3D11_SDK_VERSION, &Context.Device, &Level, &Context.DeviceContext);
			if (Result == E_INVALIDARG)
				Result = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0,
					Levels + 1, 1, D3D11_SDK_VERSION, &Context.Device, &Level, &Context.DeviceContext);
			if (FAILED(Result) || Level < D3D_FEATURE_LEVEL_11_0) return false;

			ID3DBlob* Code = nullptr;
			ID3DBlob* Errors = nullptr;
			Result = D3DCompile(ShaderSource, std::strlen(ShaderSource), "PBDBackendComputeShader",
				nullptr, nullptr, "SolveContacts", "cs_5_0", D3DCOMPILE_ENABLE_STRICTNESS
				| D3DCOMPILE_IEEE_STRICTNESS | D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &Code, &Errors);
			if (FAILED(Result) && Errors)
				std::fprintf(stderr, "PBD compute shader compilation failed: %s\n", static_cast<const char*>(Errors->GetBufferPointer()));
			SafeRelease(Errors);
			if (FAILED(Result) || !Code) { SafeRelease(Code); return false; }
			Result = Context.Device->CreateComputeShader(Code->GetBufferPointer(), Code->GetBufferSize(), nullptr, &Context.Shader);
			SafeRelease(Code);
			if (FAILED(Result)) return false;
			D3D11_BUFFER_DESC Description = {};
			Description.ByteWidth = sizeof(SolverConstants);
			Description.Usage = D3D11_USAGE_DYNAMIC;
			Description.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
			Description.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
			return SUCCEEDED(Context.Device->CreateBuffer(&Description, nullptr, &Context.Constants));
		}

		bool EnsureCapacity(ComputeContext& Context, StructuredBuffer& Resource,
			uint32_t Count, uint32_t Stride, bool Writable)
		{
			if (Resource.Buffer && Resource.Capacity >= Count) return true;
			const uint32_t Maximum = std::numeric_limits<uint32_t>::max() / Stride;
			if (Count == 0 || Count > Maximum) return false;
			uint32_t Capacity = std::max(64u, Resource.Capacity);
			while (Capacity < Count)
			{
				const uint32_t Growth = std::max(64u, Capacity / 2);
				if (Capacity > Maximum - Growth) { Capacity = Count; break; }
				Capacity += Growth;
			}
			if (Capacity > Maximum) return false;
			StructuredBuffer Candidate;
			D3D11_BUFFER_DESC Description = {};
			Description.ByteWidth = Capacity * Stride;
			Description.Usage = D3D11_USAGE_DEFAULT;
			Description.BindFlags = Writable ? D3D11_BIND_UNORDERED_ACCESS : D3D11_BIND_SHADER_RESOURCE;
			Description.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
			Description.StructureByteStride = Stride;
			HRESULT Result = Context.Device->CreateBuffer(&Description, nullptr, &Candidate.Buffer);
			if (SUCCEEDED(Result) && Writable)
			{
				D3D11_UNORDERED_ACCESS_VIEW_DESC View = {};
				View.Format = DXGI_FORMAT_UNKNOWN;
				View.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
				View.Buffer.NumElements = Capacity;
				Result = Context.Device->CreateUnorderedAccessView(Candidate.Buffer, &View, &Candidate.Output);
				if (SUCCEEDED(Result))
				{
					Description = {};
					Description.ByteWidth = Capacity * Stride;
					Description.Usage = D3D11_USAGE_STAGING;
					Description.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
					Result = Context.Device->CreateBuffer(&Description, nullptr, &Candidate.Readback);
				}
			}
			else if (SUCCEEDED(Result))
			{
				D3D11_SHADER_RESOURCE_VIEW_DESC View = {};
				View.Format = DXGI_FORMAT_UNKNOWN;
				View.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
				View.Buffer.NumElements = Capacity;
				Result = Context.Device->CreateShaderResourceView(Candidate.Buffer, &View, &Candidate.Input);
			}
			if (FAILED(Result)) { Candidate.Release(); return false; }
			Candidate.Capacity = Capacity;
			Resource.Release();
			Resource = Candidate;
			return true;
		}

		void Unbind(ComputeContext& Context)
		{
			ID3D11ShaderResourceView* Input = nullptr;
			ID3D11UnorderedAccessView* Outputs[2] = { nullptr, nullptr };
			ID3D11Buffer* Constants = nullptr;
			Context.DeviceContext->CSSetShaderResources(0, 1, &Input);
			Context.DeviceContext->CSSetUnorderedAccessViews(0, 2, Outputs, nullptr);
			Context.DeviceContext->CSSetConstantBuffers(0, 1, &Constants);
			Context.DeviceContext->CSSetShader(nullptr, nullptr, 0);
		}

		bool Readback(ComputeContext& Context, const StructuredBuffer& Resource, void* Output, size_t Bytes)
		{
			Context.DeviceContext->CopyResource(Resource.Readback, Resource.Buffer);
			D3D11_MAPPED_SUBRESOURCE Mapped = {};
			if (FAILED(Context.DeviceContext->Map(Resource.Readback, 0, D3D11_MAP_READ, 0, &Mapped))) return false;
			const bool Valid = Mapped.pData != nullptr;
			if (Valid) std::memcpy(Output, Mapped.pData, Bytes);
			Context.DeviceContext->Unmap(Resource.Readback, 0);
			return Valid;
		}
	}

	void* CreatePBDComputeShaderContext()
	{
		ComputeContext* Context = new (std::nothrow) ComputeContext();
		if (!Context) return nullptr;
		if (!Initialize(*Context)) { delete Context; return nullptr; }
		return Context;
	}

	void DestroyPBDComputeShaderContext(void* Context)
	{
		delete static_cast<ComputeContext*>(Context);
	}

	bool IsPBDComputeShaderBackendAvailable()
	{
		static std::once_flag Probe;
		static bool Available = false;
		std::call_once(Probe, []()
		{
			void* Context = CreatePBDComputeShaderContext();
			Available = Context != nullptr;
			DestroyPBDComputeShaderContext(Context);
		});
		return Available;
	}

	bool ExecutePBDComputeShader(void* Opaque, PBDBackendBody* Bodies, uint32_t BodyCount,
		PBDBackendContact* Contacts, uint32_t ContactCount, const uint32_t* RowIndices,
		const uint32_t* LayerOffsets, uint32_t LayerCount,
		const PBDBackendParameters& Parameters, PBDBackendStage Stage)
	{
		if (ContactCount == 0) return true;
		if (!Opaque || !Bodies || !Contacts || !RowIndices || !LayerOffsets || BodyCount == 0 || LayerCount == 0
			|| LayerCount > ContactCount || !std::isfinite(Parameters.DeltaTime) || Parameters.DeltaTime <= 0.0f
			|| !std::isfinite(Parameters.Alpha) || Parameters.Alpha < 0.0f
			|| !std::isfinite(Parameters.PenetrationSlop) || Parameters.PenetrationSlop < 0.0f
			|| !std::isfinite(Parameters.MaxPositionCorrection) || Parameters.MaxPositionCorrection <= 0.0f
			|| !std::isfinite(Parameters.MaxAngularCorrection) || Parameters.MaxAngularCorrection <= 0.0f
			|| LayerOffsets[0] != 0 || LayerOffsets[LayerCount] != ContactCount
			|| (uint32_t)Stage > (uint32_t)PBDBackendStage::Velocity) return false;
		for (uint32_t Layer = 0; Layer < LayerCount; ++Layer)
			if (LayerOffsets[Layer] >= LayerOffsets[Layer + 1] || LayerOffsets[Layer + 1] > ContactCount) return false;
		for (uint32_t Row = 0; Row < ContactCount; ++Row)
			if (RowIndices[Row] >= ContactCount || Contacts[Row].Bodies.x >= BodyCount || Contacts[Row].Bodies.y >= BodyCount) return false;
		ComputeContext& Context = *static_cast<ComputeContext*>(Opaque);
		std::lock_guard<std::mutex> Lock(Context.Mutex);
		if (FAILED(Context.Device->GetDeviceRemovedReason())
			|| !EnsureCapacity(Context, Context.Bodies, BodyCount, sizeof(PBDBackendBody), true)
			|| !EnsureCapacity(Context, Context.Contacts, ContactCount, sizeof(PBDBackendContact), true)
			|| !EnsureCapacity(Context, Context.Rows, ContactCount, sizeof(uint32_t), false)) return false;
		try
		{
			// Upload complete capacity spans; some drivers access beyond partial upload boxes.
			Context.HostBodies.resize(Context.Bodies.Capacity);
			Context.HostContacts.resize(Context.Contacts.Capacity);
			Context.HostRows.resize(Context.Rows.Capacity);
		}
		catch (const std::bad_alloc&) { return false; }
		std::memcpy(Context.HostBodies.data(), Bodies, size_t(BodyCount) * sizeof(PBDBackendBody));
		std::memcpy(Context.HostContacts.data(), Contacts, size_t(ContactCount) * sizeof(PBDBackendContact));
		std::memcpy(Context.HostRows.data(), RowIndices, size_t(ContactCount) * sizeof(uint32_t));
		Context.DeviceContext->UpdateSubresource(Context.Bodies.Buffer, 0, nullptr, Context.HostBodies.data(), 0, 0);
		Context.DeviceContext->UpdateSubresource(Context.Contacts.Buffer, 0, nullptr, Context.HostContacts.data(), 0, 0);
		Context.DeviceContext->UpdateSubresource(Context.Rows.Buffer, 0, nullptr, Context.HostRows.data(), 0, 0);
		ID3D11UnorderedAccessView* Outputs[] = { Context.Bodies.Output, Context.Contacts.Output };
		Context.DeviceContext->CSSetShader(Context.Shader, nullptr, 0);
		Context.DeviceContext->CSSetShaderResources(0, 1, &Context.Rows.Input);
		Context.DeviceContext->CSSetUnorderedAccessViews(0, 2, Outputs, nullptr);
		Context.DeviceContext->CSSetConstantBuffers(0, 1, &Context.Constants);
		SolverConstants Constants = { Parameters.DeltaTime, Parameters.Alpha, Parameters.PenetrationSlop,
			Parameters.MaxPositionCorrection, Parameters.MaxAngularCorrection, 0, 0, (uint32_t)Stage };
		bool Success = true;
		for (uint32_t Layer = 0; Success && Layer < LayerCount; ++Layer)
		{
			const uint32_t End = LayerOffsets[Layer + 1];
			for (uint32_t Start = LayerOffsets[Layer]; Start < End;)
			{
				Constants.RowStart = Start;
				Constants.RowCount = std::min(End - Start, 65535u * 64u);
				D3D11_MAPPED_SUBRESOURCE Mapped = {};
				if (FAILED(Context.DeviceContext->Map(Context.Constants, 0, D3D11_MAP_WRITE_DISCARD, 0, &Mapped)))
				{
					Success = false;
					break;
				}
				std::memcpy(Mapped.pData, &Constants, sizeof(Constants));
				Context.DeviceContext->Unmap(Context.Constants, 0);
				Context.DeviceContext->CSSetConstantBuffers(0, 1, &Context.Constants);
				// Dispatch boundaries order UAV accesses between dependent layers. Rows
				// within one layer never write the same dynamic body or contact record.
				Context.DeviceContext->Dispatch((Constants.RowCount + 63u) / 64u, 1, 1);
				Start += Constants.RowCount;
			}
		}
		Unbind(Context);
		if (!Success || !Readback(Context, Context.Bodies, Context.HostBodies.data(), size_t(BodyCount) * sizeof(PBDBackendBody))
			|| !Readback(Context, Context.Contacts, Context.HostContacts.data(), size_t(ContactCount) * sizeof(PBDBackendContact))
			|| FAILED(Context.Device->GetDeviceRemovedReason())
			|| !ValidatePBDBackendOutput(Context.HostBodies.data(), BodyCount, Context.HostContacts.data(), ContactCount)) return false;
		// Commit both record arrays only after every dispatch and readback has succeeded.
		std::memcpy(Bodies, Context.HostBodies.data(), size_t(BodyCount) * sizeof(PBDBackendBody));
		std::memcpy(Contacts, Context.HostContacts.data(), size_t(ContactCount) * sizeof(PBDBackendContact));
		return true;
	}
#else
	bool IsPBDComputeShaderBackendAvailable() { return false; }
	void* CreatePBDComputeShaderContext() { return nullptr; }
	void DestroyPBDComputeShaderContext(void*) {}
	bool ExecutePBDComputeShader(void*, PBDBackendBody*, uint32_t, PBDBackendContact*, uint32_t,
		const uint32_t*, const uint32_t*, uint32_t, const PBDBackendParameters&, PBDBackendStage)
	{
		return false;
	}
#endif
}
