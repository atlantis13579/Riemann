#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "../Maths/SIMD.h"
#include "GemmCache.h"

namespace Maths
{
namespace LinearAlgebra
{
namespace GemmDetail
{

// Independent implementation of Eigen's packed-panel/register-block strategy.
// The packet width follows the translation unit ISA; no global ISA upgrade is needed.
template<typename T> struct Packet
{
	using Type = T;
	enum { Width = 1 };
	static Type Load(const T* P) { return *P; }
	static Type Broadcast(T V) { return V; }
	static Type AddProduct(Type A, Type B, Type C) { return A * B + C; }
	static void Store(T* P, Type V) { *P = V; }
};

template<> struct Packet<float>
{
#if defined(SIMD_INSTRUCTION_SSE) && defined(__AVX__)
	using Type = __m256;
	enum { Width = 8 };
	static Type Load(const float* P) { return _mm256_loadu_ps(P); }
	static Type Broadcast(float V) { return _mm256_set1_ps(V); }
	static Type AddProduct(Type A, Type B, Type C)
	{
#if defined(__FMA__) || (defined(_MSC_VER) && defined(__AVX2__))
		return _mm256_fmadd_ps(A, B, C);
#else
		return _mm256_add_ps(_mm256_mul_ps(A, B), C);
#endif
	}
	static void Store(float* P, Type V) { _mm256_storeu_ps(P, V); }
#else
	using Type = F128;
	enum { Width = 4 };
	static Type Load(const float* P) { return F128_LoadU(P); }
	static Type Broadcast(float V) { return F128_Load(V); }
	static Type AddProduct(Type A, Type B, Type C) { return F128_MAdd(A, B, C); }
	static void Store(float* P, Type V) { F128_StoreU(V, P); }
#endif
};

#if defined(SIMD_INSTRUCTION_SSE) || (defined(SIMD_INSTRUCTION_NEON) && (defined(__aarch64__) || defined(_M_ARM64)))
template<> struct Packet<double>
{
#if defined(SIMD_INSTRUCTION_SSE) && defined(__AVX__)
	using Type = __m256d;
	enum { Width = 4 };
	static Type Load(const double* P) { return _mm256_loadu_pd(P); }
	static Type Broadcast(double V) { return _mm256_set1_pd(V); }
	static Type AddProduct(Type A, Type B, Type C)
	{
#if defined(__FMA__) || (defined(_MSC_VER) && defined(__AVX2__))
		return _mm256_fmadd_pd(A, B, C);
#else
		return _mm256_add_pd(_mm256_mul_pd(A, B), C);
#endif
	}
	static void Store(double* P, Type V) { _mm256_storeu_pd(P, V); }
#elif defined(SIMD_INSTRUCTION_SSE)
	using Type = __m128d;
	enum { Width = 2 };
	static Type Load(const double* P) { return _mm_loadu_pd(P); }
	static Type Broadcast(double V) { return _mm_set1_pd(V); }
	static Type AddProduct(Type A, Type B, Type C) { return _mm_add_pd(_mm_mul_pd(A, B), C); }
	static void Store(double* P, Type V) { _mm_storeu_pd(P, V); }
#else
	using Type = float64x2_t;
	enum { Width = 2 };
	static Type Load(const double* P) { return vld1q_f64(P); }
	static Type Broadcast(double V) { return vdupq_n_f64(V); }
	static Type AddProduct(Type A, Type B, Type C) { return vfmaq_f64(C, A, B); }
	static void Store(double* P, Type V) { vst1q_f64(P, V); }
#endif
};
#endif

template<typename T> struct KernelShape
{
	// Fused multiply-add needs fewer temporaries, allowing a third packet per row.
#if defined(__FMA__) || (defined(_MSC_VER) && defined(__AVX2__)) || (defined(SIMD_INSTRUCTION_NEON) && (defined(__aarch64__) || defined(_M_ARM64)))
	enum { Rows = 4, Packets = 3, Columns = Packets * Packet<T>::Width };
#else
	enum { Rows = 4, Packets = 2, Columns = Packets * Packet<T>::Width };
#endif
};

inline void PrefetchRead(const void* Address)
{
#if defined(SIMD_INSTRUCTION_SSE)
	_mm_prefetch(static_cast<const char*>(Address), _MM_HINT_T0);
#elif defined(__GNUC__) || defined(__clang__)
	__builtin_prefetch(Address, 0, 3);
#else
	(void)Address;
#endif
}

template<typename T> class PackedBuffer
{
public:
	explicit PackedBuffer(size_t Count) : Storage(new T[Count + 64 / sizeof(T)])
	{
		const uintptr_t Address = reinterpret_cast<uintptr_t>(Storage.get());
		Data = reinterpret_cast<T*>((Address + 63) & ~uintptr_t(63));
	}
	T* Get() { return Data; }
private:
	std::unique_ptr<T[]> Storage;
	T* Data;
};

struct PackingStatistics
{
	uint64_t LeftElements = 0;
	uint64_t RightElements = 0;
	uint64_t LeftPacks = 0;
	uint64_t RightPacks = 0;
};

template<typename T>
void PackLeft(const T* Source, int Stride, int Rows, int Depth, T* Packed)
{
	const int MR = KernelShape<T>::Rows;
	for (int Row = 0; Row < Rows; Row += MR)
		for (int Inner = 0; Inner < Depth; ++Inner)
			for (int Lane = 0; Lane < MR; ++Lane)
				*Packed++ = Row + Lane < Rows ? Source[(size_t)(Row + Lane) * Stride + Inner] : T(0);
}

template<typename T>
void PackRight(const T* Source, int Stride, int Depth, int Columns, T* Packed)
{
	const int NR = KernelShape<T>::Columns;
	for (int Column = 0; Column < Columns; Column += NR)
	{
		const int Count = (std::min)(NR, Columns - Column);
		if (Count == NR)
		{
			for (int Inner = 0; Inner < Depth; ++Inner)
			{
				const T* Row = Source + (size_t)Inner * Stride + Column;
				for (int Lane = 0; Lane < NR; Lane += Packet<T>::Width)
					Packet<T>::Store(Packed + Lane, Packet<T>::Load(Row + Lane));
				Packed += NR;
			}
			continue;
		}
		for (int Inner = 0; Inner < Depth; ++Inner)
		{
			const T* Row = Source + (size_t)Inner * Stride + Column;
			std::copy(Row, Row + Count, Packed);
			std::fill(Packed + Count, Packed + NR, T(0));
			Packed += NR;
		}
	}
}

template<>
inline void PackLeft<float>(const float* Source, int Stride, int Rows, int Depth, float* Packed)
{
	for (int Row = 0; Row < Rows; Row += 4)
	{
		int Inner = 0;
		if (Rows - Row >= 4)
		{
			for (; Depth - Inner >= 4; Inner += 4)
			{
				const float* A = Source + (size_t)Row * Stride + Inner;
				F128 V0 = F128_LoadU(A), V1 = F128_LoadU(A + Stride);
				F128 V2 = F128_LoadU(A + (size_t)Stride * 2), V3 = F128_LoadU(A + (size_t)Stride * 3);
				F128_Transpose(V0, V1, V2, V3);
				F128_StoreU(V0, Packed); F128_StoreU(V1, Packed + 4);
				F128_StoreU(V2, Packed + 8); F128_StoreU(V3, Packed + 12);
				Packed += 16;
			}
		}
		for (; Inner < Depth; ++Inner)
			for (int Lane = 0; Lane < 4; ++Lane)
				*Packed++ = Row + Lane < Rows ? Source[(size_t)(Row + Lane) * Stride + Inner] : 0.0f;
	}
}

#if defined(_MSC_VER) && defined(__AVX2__) && !defined(__clang__)
template<>
inline void PackLeft<double>(const double* Source, int Stride, int Rows, int Depth, double* Packed)
{
	for (int Row = 0; Row < Rows; Row += 4)
	{
		int Inner = 0;
		if (Rows - Row >= 4)
		{
			// Interleave four source rows through a 4x4 register transpose.
			for (; Depth - Inner >= 4; Inner += 4)
			{
				const double* A = Source + (size_t)Row * Stride + Inner;
				const __m256d R0 = _mm256_loadu_pd(A), R1 = _mm256_loadu_pd(A + Stride);
				const __m256d R2 = _mm256_loadu_pd(A + (size_t)Stride * 2), R3 = _mm256_loadu_pd(A + (size_t)Stride * 3);
				const __m256d T0 = _mm256_unpacklo_pd(R0, R1), T1 = _mm256_unpackhi_pd(R0, R1);
				const __m256d T2 = _mm256_unpacklo_pd(R2, R3), T3 = _mm256_unpackhi_pd(R2, R3);
				_mm256_storeu_pd(Packed, _mm256_permute2f128_pd(T0, T2, 0x20));
				_mm256_storeu_pd(Packed + 4, _mm256_permute2f128_pd(T1, T3, 0x20));
				_mm256_storeu_pd(Packed + 8, _mm256_permute2f128_pd(T0, T2, 0x31));
				_mm256_storeu_pd(Packed + 12, _mm256_permute2f128_pd(T1, T3, 0x31));
				Packed += 16;
			}
		}
		for (; Inner < Depth; ++Inner)
			for (int Lane = 0; Lane < 4; ++Lane)
				*Packed++ = Row + Lane < Rows ? Source[(size_t)(Row + Lane) * Stride + Inner] : 0.0;
	}
}
#endif

// Direct kernel for small products; operands retain their original row strides.
template<typename T, int Packets>
void SmallKernel(const T* Left, const T* Right, int Depth, int LeftStride, int RightStride, T* Result)
{
	using P = Packet<T>;
	using V = typename P::Type;
	const int W = P::Width;
	V C00 = P::Broadcast(T(0)), C01 = C00;
	V C10 = C00, C11 = C00, C20 = C00, C21 = C00, C30 = C00, C31 = C00;
	for (int Inner = 0; Inner < Depth; ++Inner)
	{
		const T* B = Right + (size_t)Inner * RightStride;
		const V B0 = P::Load(B), B1 = Packets == 2 ? P::Load(B + W) : P::Broadcast(T(0));
		{ const V A = P::Broadcast(Left[Inner]);
		  C00 = P::AddProduct(A, B0, C00); if (Packets == 2) C01 = P::AddProduct(A, B1, C01); }
		{ const V A = P::Broadcast(Left[(size_t)LeftStride + Inner]);
		  C10 = P::AddProduct(A, B0, C10); if (Packets == 2) C11 = P::AddProduct(A, B1, C11); }
		{ const V A = P::Broadcast(Left[(size_t)LeftStride * 2 + Inner]);
		  C20 = P::AddProduct(A, B0, C20); if (Packets == 2) C21 = P::AddProduct(A, B1, C21); }
		{ const V A = P::Broadcast(Left[(size_t)LeftStride * 3 + Inner]);
		  C30 = P::AddProduct(A, B0, C30); if (Packets == 2) C31 = P::AddProduct(A, B1, C31); }
#if defined(_MSC_VER) && defined(__AVX2__) && !defined(__clang__)
		std::atomic_signal_fence(std::memory_order_acq_rel);
#endif
	}
	P::Store(Result, C00);
	P::Store(Result + (size_t)RightStride, C10);
	P::Store(Result + (size_t)RightStride * 2, C20);
	P::Store(Result + (size_t)RightStride * 3, C30);
	if (Packets == 2)
	{
		P::Store(Result + W, C01);
		P::Store(Result + (size_t)RightStride + W, C11);
		P::Store(Result + (size_t)RightStride * 2 + W, C21);
		P::Store(Result + (size_t)RightStride * 3 + W, C31);
	}
}

template<typename T>
bool TryMultiplySmall(const T* Left, const T* Right, int LeftStride, int RightStride,
	int Rows, int Columns, int Depth, T* Result)
{
	using P = Packet<T>;
	const int W = P::Width;
	if (W == 1 || Rows < 4 || Rows > 64 || Columns > 64 || Depth > 64 || Depth <= 0 || Columns % W != 0) return false;
#if !defined(SIMD_INSTRUCTION_SSE)
	// Keep unmeasured architectures on the established packed path.
	return false;
#elif !defined(__AVX2__)
	// Without AVX2/FMA, packing already wins beyond the tiny scalar fallback.
	if (Rows > 16 || Columns > 16 || Depth > 32) return false;
#endif
	int Row = 0;
	for (; Row + 4 <= Rows; Row += 4)
	{
		int Column = 0;
		for (; Column + 2 * W <= Columns; Column += 2 * W)
			SmallKernel<T, 2>(Left + (size_t)Row * LeftStride, Right + Column, Depth, LeftStride, RightStride,
				Result + (size_t)Row * RightStride + Column);
		if (Column < Columns)
			SmallKernel<T, 1>(Left + (size_t)Row * LeftStride, Right + Column, Depth, LeftStride, RightStride,
				Result + (size_t)Row * RightStride + Column);
	}
	for (; Row < Rows; ++Row)
		for (int Column = 0; Column < Columns; Column += W)
		{
			typename P::Type Sum = P::Broadcast(T(0));
			for (int Inner = 0; Inner < Depth; ++Inner)
				Sum = P::AddProduct(P::Broadcast(Left[(size_t)Row * LeftStride + Inner]),
					P::Load(Right + (size_t)Inner * RightStride + Column), Sum);
			P::Store(Result + (size_t)Row * RightStride + Column, Sum);
		}
	return true;
}


template<typename T>
void ScalarKernel(const T* Left, const T* Right, int Depth, int Rows, int Columns, T* Result, int Stride)
{
	for (int Row = 0; Row < Rows; ++Row)
		for (int Column = 0; Column < Columns; ++Column)
		{
			T Sum = Result[(size_t)Row * Stride + Column];
#if defined(_MSC_VER)
#pragma loop(no_vector)
#endif
			for (int Inner = 0; Inner < Depth; ++Inner)
				Sum += Left[(size_t)Inner * KernelShape<T>::Rows + Row]
					* Right[(size_t)Inner * KernelShape<T>::Columns + Column];
			Result[(size_t)Row * Stride + Column] = Sum;
		}
}

template<typename T>
void VectorKernel(const T* Left, const T* Right, int Depth, T* Result, int Stride)
{
	using P = Packet<T>;
	using V = typename P::Type;
	const int W = P::Width, MR = KernelShape<T>::Rows, NR = KernelShape<T>::Columns;
	const int Packets = KernelShape<T>::Packets;
	// C is read/written once per KC. Keep eight or twelve independent accumulators
	// live across the reduction rather than materializing intermediate output tiles.
	V C00 = P::Load(Result + (size_t)0 * Stride + 0 * W);
	V C01 = P::Load(Result + (size_t)0 * Stride + 1 * W);
	V C02 = Packets == 3 ? P::Load(Result + (size_t)0 * Stride + 2 * W) : P::Broadcast(T(0));
	V C10 = P::Load(Result + (size_t)1 * Stride + 0 * W);
	V C11 = P::Load(Result + (size_t)1 * Stride + 1 * W);
	V C12 = Packets == 3 ? P::Load(Result + (size_t)1 * Stride + 2 * W) : P::Broadcast(T(0));
	V C20 = P::Load(Result + (size_t)2 * Stride + 0 * W);
	V C21 = P::Load(Result + (size_t)2 * Stride + 1 * W);
	V C22 = Packets == 3 ? P::Load(Result + (size_t)2 * Stride + 2 * W) : P::Broadcast(T(0));
	V C30 = P::Load(Result + (size_t)3 * Stride + 0 * W);
	V C31 = P::Load(Result + (size_t)3 * Stride + 1 * W);
	V C32 = Packets == 3 ? P::Load(Result + (size_t)3 * Stride + 2 * W) : P::Broadcast(T(0));
#if defined(_MSC_VER) && defined(__AVX2__) && !defined(__clang__)
	const int Remainder = Depth % 8;
	// Bias integer offsets, not pointers. The initial switch skips every case
	// whose effective offset would be negative; all formed pointers stay in range.
	int64_t LeftOffset = (Remainder ? Remainder - 8 : 0) * MR;
	int64_t RightOffset = (Remainder ? Remainder - 8 : 0) * NR;
#define RIEMANN_GEMM_LEFT(Offset) (Left + (size_t)(LeftOffset + (Offset) * MR))
#define RIEMANN_GEMM_RIGHT(Offset) (Right + (size_t)(RightOffset + (Offset) * NR))
#else
	int Inner = 0;
#define RIEMANN_GEMM_LEFT(Offset) (Left + (size_t)(Inner + (Offset)) * MR)
#define RIEMANN_GEMM_RIGHT(Offset) (Right + (size_t)(Inner + (Offset)) * NR)
#endif
#define RIEMANN_GEMM_STEP(Offset) { \
		const T* A = RIEMANN_GEMM_LEFT(Offset); \
		const T* B = RIEMANN_GEMM_RIGHT(Offset); \
		const V B0 = P::Load(B), B1 = P::Load(B + W), B2 = Packets == 3 ? P::Load(B + 2 * W) : P::Broadcast(T(0)); \
		{ const V A0 = P::Broadcast(A[0]); \
		  C00 = P::AddProduct(A0, B0, C00); \
		  C01 = P::AddProduct(A0, B1, C01); \
		  if (Packets == 3) C02 = P::AddProduct(A0, B2, C02); \
		} \
		{ const V A1 = P::Broadcast(A[1]); \
		  C10 = P::AddProduct(A1, B0, C10); \
		  C11 = P::AddProduct(A1, B1, C11); \
		  if (Packets == 3) C12 = P::AddProduct(A1, B2, C12); \
		} \
		{ const V A2 = P::Broadcast(A[2]); \
		  C20 = P::AddProduct(A2, B0, C20); \
		  C21 = P::AddProduct(A2, B1, C21); \
		  if (Packets == 3) C22 = P::AddProduct(A2, B2, C22); \
		} \
		{ const V A3 = P::Broadcast(A[3]); \
		  C30 = P::AddProduct(A3, B0, C30); \
		  C31 = P::AddProduct(A3, B1, C31); \
		  if (Packets == 3) C32 = P::AddProduct(A3, B2, C32); \
		} \
	}
#if defined(_MSC_VER) && defined(__AVX2__) && !defined(__clang__)
	// Multiple loop entries keep MSVC from regrouping eight K steps by
	// accumulator, which otherwise spills the twelve-register output tile.
	// Each case consumes exactly one K element, including the first partial group.
	int Chunks = Depth / 8 + (Remainder != 0);
	if (Depth > 0) switch (Remainder)
	{
	case 0: do {
		RIEMANN_GEMM_STEP(0);
	case 7: RIEMANN_GEMM_STEP(1);
	case 6: RIEMANN_GEMM_STEP(2);
	case 5: RIEMANN_GEMM_STEP(3);
	case 4: RIEMANN_GEMM_STEP(4);
	case 3: RIEMANN_GEMM_STEP(5);
	case 2: RIEMANN_GEMM_STEP(6);
	case 1: RIEMANN_GEMM_STEP(7);
		LeftOffset += 8 * MR; RightOffset += 8 * NR;
	} while (--Chunks > 0);
	}
#else
	for (; Inner < Depth; ++Inner)
	{
		RIEMANN_GEMM_STEP(0);
#if defined(_MSC_VER) && defined(__AVX2__) && !defined(__clang__)
		// MSVC 19.44 otherwise auto-unrolls four steps and spills the AVX2
		// accumulator tile. This compiler-only barrier emits no CPU fence.
		std::atomic_signal_fence(std::memory_order_acq_rel);
#endif
	}
#endif
#undef RIEMANN_GEMM_STEP
#undef RIEMANN_GEMM_LEFT
#undef RIEMANN_GEMM_RIGHT
	P::Store(Result + (size_t)0 * Stride + 0 * W, C00);
	P::Store(Result + (size_t)0 * Stride + 1 * W, C01);
	if (Packets == 3) P::Store(Result + (size_t)0 * Stride + 2 * W, C02);
	P::Store(Result + (size_t)1 * Stride + 0 * W, C10);
	P::Store(Result + (size_t)1 * Stride + 1 * W, C11);
	if (Packets == 3) P::Store(Result + (size_t)1 * Stride + 2 * W, C12);
	P::Store(Result + (size_t)2 * Stride + 0 * W, C20);
	P::Store(Result + (size_t)2 * Stride + 1 * W, C21);
	if (Packets == 3) P::Store(Result + (size_t)2 * Stride + 2 * W, C22);
	P::Store(Result + (size_t)3 * Stride + 0 * W, C30);
	P::Store(Result + (size_t)3 * Stride + 1 * W, C31);
	if (Packets == 3) P::Store(Result + (size_t)3 * Stride + 2 * W, C32);
}

template<typename T, bool Vectorize, bool Prefetch>
void MultiplyPanel(const T* Left, const T* Right, int Rows, int Columns, int Depth, T* Result, int Stride)
{
	const int MR = KernelShape<T>::Rows, NR = KernelShape<T>::Columns;
	// Reuse a wide vector B micro-panel in L1 while streaming the narrower A
	// panels. Narrow SSE2 tiles retain row traversal for contiguous C writes.
	const bool ColumnFirst = NR > 2 * MR;
	for (int Outer = 0; Outer < (ColumnFirst ? Columns : Rows); Outer += ColumnFirst ? NR : MR)
		for (int Inner = 0; Inner < (ColumnFirst ? Rows : Columns); Inner += ColumnFirst ? MR : NR)
		{
			const int Row = ColumnFirst ? Inner : Outer, Column = ColumnFirst ? Outer : Inner;
			const T* A = Left + (size_t)Row * Depth;
			const T* B = Right + (size_t)Column * Depth;
			T* C = Result + (size_t)Row * Stride + Column;
			const int TileRows = (std::min)(MR, Rows - Row), TileColumns = (std::min)(NR, Columns - Column);
			if (Vectorize && TileRows == MR && TileColumns == NR)
			{
				if (Prefetch)
				{
					PrefetchRead(A);
					for (int Lane = 0; Lane < MR; ++Lane) PrefetchRead(C + (size_t)Lane * Stride);
					if (ColumnFirst)
					{
						if (Row + MR < Rows) PrefetchRead(A + (size_t)MR * Depth);
					}
					else if (Column + NR < Columns) PrefetchRead(B + (size_t)NR * Depth);
				}
				VectorKernel<T>(A, B, Depth, C, Stride);
			}
			else if (Vectorize && Packet<T>::Width > 1 && TileColumns >= Packet<T>::Width)
			{
				// Packed operands already contain zero padding. A small output tile
				// permits SIMD tails without reading or writing outside the caller's C.
				alignas(64) T Tail[MR * NR] = {};
				for (int Lane = 0; Lane < TileRows; ++Lane)
					std::copy(C + (size_t)Lane * Stride, C + (size_t)Lane * Stride + TileColumns, Tail + Lane * NR);
				VectorKernel<T>(A, B, Depth, Tail, NR);
				for (int Lane = 0; Lane < TileRows; ++Lane)
					std::copy(Tail + Lane * NR, Tail + Lane * NR + TileColumns, C + (size_t)Lane * Stride);
			}
			else ScalarKernel(A, B, Depth, TileRows, TileColumns, C, Stride);
		}
}

template<typename T, bool Vectorize, bool Prefetch>
void MultiplyBlocked(const T* Left, const T* Right, int LeftStride, int RightStride,
	int Rows, int Columns, int Depth, T* Result, Blocking Blocks, PackingStatistics* Statistics)
{
	const int MR = KernelShape<T>::Rows, NR = KernelShape<T>::Columns;
	Blocks.Rows = (std::max)(1, (std::min)(Rows, Blocks.Rows));
	Blocks.Columns = (std::max)(1, (std::min)(Columns, Blocks.Columns));
	Blocks.Depth = (std::max)(1, (std::min)(Depth, Blocks.Depth));
	PackedBuffer<T> PackedLeft(((size_t)Blocks.Rows + MR - 1) / MR * MR * Blocks.Depth);
	PackedBuffer<T> PackedRight(((size_t)Blocks.Columns + NR - 1) / NR * NR * Blocks.Depth);
	auto PackA = [&](int Row, int Inner, int Count, int K) {
		PackLeft(Left + (size_t)Row * LeftStride + Inner, LeftStride, Count, K, PackedLeft.Get());
		if (Statistics) { Statistics->LeftElements += (uint64_t)Count * K; ++Statistics->LeftPacks; }
	};
	auto PackB = [&](int Column, int Inner, int Count, int K) {
		PackRight(Right + (size_t)Inner * RightStride + Column, RightStride, K, Count, PackedRight.Get());
		if (Statistics) { Statistics->RightElements += (uint64_t)Count * K; ++Statistics->RightPacks; }
	};
	const uint64_t RowBlocks = ((uint64_t)Rows + Blocks.Rows - 1) / Blocks.Rows;
	const uint64_t ColumnBlocks = ((uint64_t)Columns + Blocks.Columns - 1) / Blocks.Columns;
	// Choose the panel reuse order with fewer packed input elements. This matters
	// for tall/skinny and short/wide products as well as square matrices.
	const bool LeftFirst = (uint64_t)Rows + (uint64_t)Columns * RowBlocks
		< (uint64_t)Rows * ColumnBlocks + (uint64_t)Columns;
	if (LeftFirst)
	{
		for (int Row = 0, M; Row < Rows; Row += M)
		{
			M = (std::min)(Blocks.Rows, Rows - Row);
			for (int Inner = 0, K; Inner < Depth; Inner += K)
			{
				K = (std::min)(Blocks.Depth, Depth - Inner);
				PackA(Row, Inner, M, K);
				for (int Column = 0, N; Column < Columns; Column += N)
				{
					N = (std::min)(Blocks.Columns, Columns - Column);
					PackB(Column, Inner, N, K);
					MultiplyPanel<T, Vectorize, Prefetch>(PackedLeft.Get(), PackedRight.Get(), M, N, K,
						Result + (size_t)Row * RightStride + Column, RightStride);
				}
			}
		}
	}
	else
	{
		for (int Column = 0, N; Column < Columns; Column += N)
		{
			N = (std::min)(Blocks.Columns, Columns - Column);
			for (int Inner = 0, K; Inner < Depth; Inner += K)
			{
				K = (std::min)(Blocks.Depth, Depth - Inner);
				PackB(Column, Inner, N, K);
				for (int Row = 0, M; Row < Rows; Row += M)
				{
					M = (std::min)(Blocks.Rows, Rows - Row);
					PackA(Row, Inner, M, K);
					MultiplyPanel<T, Vectorize, Prefetch>(PackedLeft.Get(), PackedRight.Get(), M, N, K,
						Result + (size_t)Row * RightStride + Column, RightStride);
				}
			}
		}
	}
}

template<typename T>
void MultiplyPacked(const T* Left, const T* Right, int LeftStride, int RightStride,
	int Rows, int Columns, int Depth, T* Result, const Blocking* Override = nullptr,
	bool Vectorize = true, bool Prefetch = true, PackingStatistics* Statistics = nullptr)
{
	if (Rows <= 0 || Columns <= 0 || Depth <= 0) return;
	const Blocking Blocks = Override ? *Override
		: SelectBlocking<T>(Rows, Depth, Columns, KernelShape<T>::Rows, KernelShape<T>::Columns);
	if (Vectorize)
	{
		if (Prefetch) MultiplyBlocked<T, true, true>(Left, Right, LeftStride, RightStride, Rows, Columns, Depth, Result, Blocks, Statistics);
		else MultiplyBlocked<T, true, false>(Left, Right, LeftStride, RightStride, Rows, Columns, Depth, Result, Blocks, Statistics);
	}
	else MultiplyBlocked<T, false, false>(Left, Right, LeftStride, RightStride, Rows, Columns, Depth, Result, Blocks, Statistics);
}

}
}
}
