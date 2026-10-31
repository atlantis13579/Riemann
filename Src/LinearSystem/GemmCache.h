#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

#if defined(_MSC_VER) && (defined(_M_IX86) || defined(_M_X64))
#include <intrin.h>
#elif defined(__i386__) || defined(__x86_64__)
#include <cpuid.h>
#elif defined(__APPLE__)
#include <sys/types.h>
#include <sys/sysctl.h>
#elif defined(__linux__)
#include <unistd.h>
#endif

namespace Maths
{
namespace LinearAlgebra
{
namespace GemmDetail
{

struct CacheSizes
{
	size_t L1 = 32 * 1024;
	size_t L2 = 256 * 1024;
	// A conservative share per logical processor, rather than the entire shared LLC.
	size_t LastLevelShare = 0;
	size_t Line = 64;
};

inline void CacheCPUID(unsigned int Leaf, unsigned int Subleaf, unsigned int* Values)
{
#if defined(_MSC_VER) && (defined(_M_IX86) || defined(_M_X64))
	int Registers[4];
	__cpuidex(Registers, (int)Leaf, (int)Subleaf);
	for (int Index = 0; Index < 4; ++Index) Values[Index] = (unsigned int)Registers[Index];
#elif defined(__i386__) || defined(__x86_64__)
	__cpuid_count(Leaf, Subleaf, Values[0], Values[1], Values[2], Values[3]);
#else
	(void)Leaf; (void)Subleaf;
	for (int Index = 0; Index < 4; ++Index) Values[Index] = 0;
#endif
}

inline CacheSizes QueryCacheSizes()
{
	CacheSizes Result;
#if defined(_M_IX86) || defined(_M_X64) || defined(__i386__) || defined(__x86_64__)
	unsigned int Registers[4];
	CacheCPUID(0, 0, Registers);
	unsigned int Leaf = Registers[0] >= 4 ? 4 : 0;
	CacheCPUID(0x80000000u, 0, Registers);
	if (Registers[0] >= 0x8000001du)
	{
		CacheCPUID(0x8000001du, 0, Registers);
		if ((Registers[0] & 31) != 0) Leaf = 0x8000001du;
	}
	for (unsigned int Index = 0; Leaf && Index < 32; ++Index)
	{
		CacheCPUID(Leaf, Index, Registers);
		const unsigned int Type = Registers[0] & 31;
		if (!Type) break;
		if (Type != 1 && Type != 3) continue;
		const unsigned int Level = (Registers[0] >> 5) & 7;
		const size_t Line = (Registers[1] & 4095) + 1;
		const uint64_t Bytes = uint64_t(Line) * (((Registers[1] >> 12) & 1023) + 1)
			* ((Registers[1] >> 22) + 1) * (uint64_t(Registers[2]) + 1);
		const size_t Capacity = (size_t)(std::min)(Bytes, uint64_t(64 * 1024 * 1024));
		if (Level == 1) { Result.L1 = Capacity; Result.Line = Line; }
		else if (Level == 2) Result.L2 = Capacity;
		else if (Level == 3) Result.LastLevelShare = Capacity / (((Registers[0] >> 14) & 4095) + 1);
	}
#elif defined(__APPLE__)
	size_t Bytes = sizeof(size_t), Value = 0;
	if (sysctlbyname("hw.l1dcachesize", &Value, &Bytes, nullptr, 0) == 0 && Value) Result.L1 = Value;
	Bytes = sizeof(size_t);
	if (sysctlbyname("hw.l2cachesize", &Value, &Bytes, nullptr, 0) == 0 && Value) Result.L2 = Value;
#elif defined(__linux__) && defined(_SC_LEVEL1_DCACHE_SIZE)
	const long L1 = sysconf(_SC_LEVEL1_DCACHE_SIZE);
	if (L1 > 0) Result.L1 = (size_t)L1;
#if defined(_SC_LEVEL2_CACHE_SIZE)
	const long L2 = sysconf(_SC_LEVEL2_CACHE_SIZE);
	if (L2 > 0) Result.L2 = (size_t)L2;
#endif
#endif
	return Result;
}

inline const CacheSizes& GetCacheSizes()
{
	static const CacheSizes Sizes = QueryCacheSizes();
	return Sizes;
}

struct Blocking
{
	int Rows, Depth, Columns;
};

// Balance the tail without increasing the number of blocks; retain kernel alignment.
inline int BalancedBlock(int Size, size_t Capacity, int Alignment)
{
	Capacity = (std::max)(Capacity / Alignment * Alignment, (size_t)Alignment);
	if ((size_t)Size <= Capacity) return Size;
	const size_t Blocks = ((size_t)Size + Capacity - 1) / Capacity;
	return (int)((((size_t)Size + Blocks - 1) / Blocks + Alignment - 1) / Alignment * Alignment);
}

template<typename T>
Blocking SelectBlocking(int Rows, int Depth, int Columns, int MicroRows, int MicroColumns,
	const CacheSizes& Cache = GetCacheSizes())
{
	// Like Eigen's product blocking, KC keeps both micro-panels in L1, while
	// MC/NC reserve space for packed panels and C in the higher-level caches.
	const size_t L1 = (std::max)(Cache.L1, size_t(4096));
	const size_t L2 = (std::max)(Cache.L2, L1);
	size_t OuterCache = (std::max)(L2, Cache.LastLevelShare);
	size_t L1Budget = L1 * 3 / 4;
#if defined(_MSC_VER) && defined(__AVX2__) && !defined(__clang__)
	// The measured single-thread AVX2 path benefits from a longer reduction
	// and fewer repacks. Keep conservative budgets when LLC information is absent.
	if (Cache.LastLevelShare > 0 && MicroColumns >= 12)
	{
		L1Budget = L1;
		OuterCache = (std::max)(OuterCache, size_t(1536 * 1024));
	}
#endif
	const size_t Registers = (size_t)MicroRows * MicroColumns * sizeof(T);
	const size_t MaxDepth = (L1Budget - (std::min)(Registers, L1 / 4))
		/ ((MicroRows + MicroColumns) * sizeof(T));
	Blocking Result;
	Result.Depth = BalancedBlock(Depth, MaxDepth, 8);
	const size_t PanelStride = (size_t)(std::max)(1, Result.Depth) * sizeof(T);
	Result.Columns = BalancedBlock(Columns, OuterCache / (2 * PanelStride), MicroColumns);
	Result.Rows = BalancedBlock(Rows, L2 / (2 * PanelStride), MicroRows);
	return Result;
}

}
}
}
