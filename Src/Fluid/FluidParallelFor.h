#pragma once

#include <algorithm>
#if defined(_MSC_VER)
#include <ppl.h>
#endif

namespace Riemann
{
	// PPL reuses the process scheduler instead of spawning threads per substep.
	// Small grids remain serial; each task owns a contiguous output range.
	// Conservative density transfers and per-particle reductions retain their
	// original order. Non-MSVC compilers use the serial path.
	// https://learn.microsoft.com/en-us/cpp/parallel/concrt/parallel-algorithms?view=msvc-170
	template<class Function>
	inline void FluidParallelFor(int Count, const Function& Execute)
	{
#if defined(_MSC_VER)
		if (Count >= 2048)
		{
			const int BatchSize = 256;
			concurrency::parallel_for(0, (Count + BatchSize - 1) / BatchSize, [&](int Batch)
			{
				const int End = std::min(Count, (Batch + 1) * BatchSize);
				for (int Index = Batch * BatchSize; Index < End; ++Index) Execute(Index);
			});
			return;
		}
#endif
		for (int Index = 0; Index < Count; ++Index) Execute(Index);
	}
}
