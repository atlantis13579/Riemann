#pragma once

namespace Riemann
{
namespace SnowCompute
{
	// Algorithms use element indices; only this prelude knows the native buffer representation.
	static const char* const HlslPrelude = R"SHADER(
#define BUFFER RWByteAddressBuffer
#define DATA uint4 data[16]
#define SHARED_BARRIER GroupMemoryBarrierWithGroupSync()
RWByteAddressBuffer b0 : register(u0), b1 : register(u1), b2 : register(u2), b3 : register(u3);
RWByteAddressBuffer b4 : register(u4), b5 : register(u5), b6 : register(u6), b7 : register(u7);
cbuffer ParametersBuffer : register(b0) { uint4 constants[16]; };
groupshared float shared0[256];
#define KERNEL(name) [numthreads(256, 1, 1)] void name(uint3 tid : SV_DispatchThreadID, uint3 lid : SV_GroupThreadID, uint3 gid : SV_GroupID)
float ReadFloat(BUFFER b, uint i) { return asfloat(b.Load(i * 4)); }
uint ReadUInt(BUFFER b, uint i) { return b.Load(i * 4); }
void WriteFloat(BUFFER b, uint i, float v) { b.Store(i * 4, asuint(v)); }
void WriteUInt(BUFFER b, uint i, uint v) { b.Store(i * 4, v); }
uint Exchange(BUFFER b, uint i, uint v) { uint old; b.InterlockedExchange(i * 4, v, old); return old; }
float PF(DATA, uint i) { return asfloat(data[i / 4][i % 4]); }
int PI(DATA, uint i) { return (int)data[i / 4][i % 4]; }
)SHADER";

	static const char* const MetalPrelude = R"SHADER(
#include <metal_stdlib>
using namespace metal;
#define BUFFER device uint*
#define DATA constant uint4* data
#define SHARED_BARRIER threadgroup_barrier(mem_flags::mem_threadgroup)
#define KERNEL(name) kernel void name(device uint* b0 [[buffer(0)]], device uint* b1 [[buffer(1)]], \
	device uint* b2 [[buffer(2)]], device uint* b3 [[buffer(3)]], device uint* b4 [[buffer(4)]], \
	device uint* b5 [[buffer(5)]], device uint* b6 [[buffer(6)]], device uint* b7 [[buffer(7)]], \
	constant uint4* constants [[buffer(8)]], threadgroup float* shared0 [[threadgroup(0)]], \
	uint3 tid [[thread_position_in_grid]], \
	uint3 lid [[thread_position_in_threadgroup]], uint3 gid [[threadgroup_position_in_grid]])
float ReadFloat(BUFFER b, uint i) { return as_type<float>(b[i]); }
uint ReadUInt(BUFFER b, uint i) { return b[i]; }
void WriteFloat(BUFFER b, uint i, float v) { b[i] = as_type<uint>(v); }
void WriteUInt(BUFFER b, uint i, uint v) { b[i] = v; }
uint Exchange(BUFFER b, uint i, uint v) { return atomic_exchange_explicit((device atomic_uint*)(b + i), v, memory_order_relaxed); }
float PF(DATA, uint i) { return as_type<float>(data[i / 4][i % 4]); }
int PI(DATA, uint i) { return (int)data[i / 4][i % 4]; }
)SHADER";
}
}
