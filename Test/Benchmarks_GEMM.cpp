// Standalone benchmark entry points for the GEMM category.
// Each CMake target defines exactly one RIEMANN_BENCHMARK_* selector.
// The normal Test target compiles this file without a selector.

#if defined(RIEMANN_BENCHMARK_GEMM)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__linux__)
#include <sched.h>
#endif

#include <Eigen/Core>
#include "../Src/LinearSystem/gemm.inl"
#include "../Benchmarks/GemmLegacy.h"

#if defined(_MSC_VER)
#define GEMM_NOINLINE __declspec(noinline)
#else
#define GEMM_NOINLINE __attribute__((noinline))
#endif

namespace
{
	using namespace Maths::LinearAlgebra;
	using Clock = std::chrono::steady_clock;
	volatile double Checksum = 0.0;
	const char* Names[] = { "reference", "legacy_fixed", "packed_scalar", "simd_unblocked", "cache_no_prefetch", "cache_prefetch", "eigen" };

	template<typename T>
	void Fill(std::vector<T>& Data, unsigned int Seed)
	{
		for (T& Value : Data)
		{
			Seed = 1664525u * Seed + 1013904223u;
			Value = T(int((Seed >> 8) % 2001) - 1000) / T(1000);
		}
	}

	template<typename T>
	GEMM_NOINLINE void Run(int Mode, int M, int K, int N, const T* A, const T* B, T* C)
	{
		if (Mode == 0) gemm_block_ref(A, B, M, K, N, 0, M - 1, 0, K - 1, 0, N - 1, C);
		else if (Mode == 1) GemmLegacy::LinearAlgebra::gemm(A, B, M, K, N, C);
		else if (Mode == 5) gemm(A, B, M, K, N, C);
		else if (Mode == 6)
		{
			using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
			Eigen::Map<Matrix> Result(C, M, N);
			Result.noalias() = Eigen::Map<const Matrix>(A, M, K) * Eigen::Map<const Matrix>(B, K, N);
		}
		else
		{
			std::fill(C, C + (size_t)M * N, T(0));
			const GemmDetail::Blocking Full = { M, K, N };
			GemmDetail::MultiplyPacked(A, B, K, N, M, N, K, C, Mode == 3 ? &Full : nullptr, Mode != 2, false);
		}
	}

	template<typename T>
	bool Check(const std::vector<T>& Actual, const std::vector<T>& Expected, int K)
	{
		for (size_t I = 0; I < Actual.size(); ++I)
		{
			const double Tolerance = std::numeric_limits<T>::epsilon() * (K + 4.0) * 8.0 * (1.0 + std::abs(double(Expected[I])));
			if (!std::isfinite(double(Actual[I])) || std::abs(double(Actual[I]) - double(Expected[I])) > Tolerance)
			{
				std::fprintf(stderr, "Incorrect GEMM at %zu: %.12g vs %.12g (tolerance %.4g)\n", I, double(Actual[I]), double(Expected[I]), Tolerance);
				return false;
			}
		}
		return true;
	}

	template<typename T>
	bool Benchmark(int M, int K, int N, FILE* CSV)
	{
		const char* Type = sizeof(T) == 4 ? "float" : "double";
		std::vector<T> A((size_t)M * K), B((size_t)K * N), C((size_t)M * N), Expected(C.size());
		Fill(A, 17); Fill(B, 31);
		Run(6, M, K, N, A.data(), B.data(), Expected.data());
		// Check the reference independently against long-double dot products.
		for (int Sample = 0; Sample < 13; ++Sample)
		{
			const int Row = Sample * 97 % M, Column = Sample * 193 % N;
			long double Sum = 0;
			for (int Inner = 0; Inner < K; ++Inner) Sum += (long double)A[(size_t)Row * K + Inner] * B[(size_t)Inner * N + Column];
			const double Tolerance = std::numeric_limits<T>::epsilon() * (K + 4.0) * 8.0 * (1.0 + std::abs(double(Sum)));
			if (std::abs(double(Expected[(size_t)Row * N + Column]) - double(Sum)) > Tolerance) return false;
		}
		const GemmDetail::Blocking Blocks = GemmDetail::SelectBlocking<T>(M, K, N,
			GemmDetail::KernelShape<T>::Rows, GemmDetail::KernelShape<T>::Columns);
		std::printf("\n%s %dx%dx%d, MC/KC/NC=%d/%d/%d, micro=%dx%d\n", Type, M, K, N,
			Blocks.Rows, Blocks.Depth, Blocks.Columns, GemmDetail::KernelShape<T>::Rows, GemmDetail::KernelShape<T>::Columns);
		int Repeats[7] = {};
		std::vector<double> Times[7];
		for (int Mode = 0; Mode < 7; ++Mode)
		{
			// Bound the diagnostic scalar work; the production, legacy, unblocked,
			// prefetch-off and Eigen paths still run on every large case.
			if (Mode == 0 && (uint64_t)M * K * N > 134217728ull) continue;
			if (Mode == 2 && (uint64_t)M * K * N > 1073741824ull) continue;
			Run(Mode, M, K, N, A.data(), B.data(), C.data());
			if (!Check(C, Expected, K)) { std::fprintf(stderr, "Mode: %s\n", Names[Mode]); return false; }
			const auto Begin = Clock::now();
			Run(Mode, M, K, N, A.data(), B.data(), C.data());
			const double Elapsed = std::chrono::duration<double>(Clock::now() - Begin).count();
			Repeats[Mode] = (std::max)(1, (std::min)(1000, int(0.015 / (std::max)(Elapsed, 1.0e-9))));
		}
		// Rotate the order to reduce systematic thermal/order bias; all paths include
		// their normal packing, allocation and output writes. No prepacked inputs.
		for (int Sample = 0; Sample < 5; ++Sample)
			for (int Offset = 0; Offset < 7; ++Offset)
			{
				const int Mode = (Offset + Sample * 3) % 7;
				if (!Repeats[Mode]) continue;
				const auto Begin = Clock::now();
				for (int Repeat = 0; Repeat < Repeats[Mode]; ++Repeat)
				{
					Run(Mode, M, K, N, A.data(), B.data(), C.data());
					Checksum += double(C[(size_t)(Sample + Repeat) % C.size()]);
				}
				Times[Mode].push_back(std::chrono::duration<double, std::milli>(Clock::now() - Begin).count() / Repeats[Mode]);
			}
		for (int Mode = 0; Mode < 7; ++Mode)
		{
			if (Times[Mode].empty()) continue;
			std::sort(Times[Mode].begin(), Times[Mode].end());
			const double Median = Times[Mode][2];
			const double GFLOPS = 2.0 * M * K * N / (Median * 1.0e6);
			std::printf("  %-18s %9.4f ms  %8.2f GFLOP/s  [%.4f, %.4f]\n", Names[Mode], Median, GFLOPS, Times[Mode].front(), Times[Mode].back());
			if (CSV) std::fprintf(CSV, "%s,%d,%d,%d,%s,%.6f,%.6f,%.6f,%.4f,%d,%d,%d\n", Type, M, K, N, Names[Mode], Median, Times[Mode].front(), Times[Mode].back(), GFLOPS, Blocks.Rows, Blocks.Depth, Blocks.Columns);
		}
		std::fflush(stdout);
		if (CSV) std::fflush(CSV);
		return true;
	}
}

int main(int Argc, char** Argv)
{
	bool Full = false;
	int Processor = -1;
	int Size = 0;
	const char* Output = nullptr;
	for (int I = 1; I < Argc; ++I)
	{
		if (std::strcmp(Argv[I], "--full") == 0) Full = true;
		else if (std::strcmp(Argv[I], "--quick") == 0) Full = false;
		else if (std::strcmp(Argv[I], "--csv") == 0 && I + 1 < Argc) Output = Argv[++I];
		else if (std::strcmp(Argv[I], "--cpu") == 0 && I + 1 < Argc) Processor = std::atoi(Argv[++I]);
		else if (std::strcmp(Argv[I], "--size") == 0 && I + 1 < Argc) Size = std::atoi(Argv[++I]);
		else { std::fprintf(stderr, "Usage: GemmBenchmark [--quick|--full|--size n] [--csv path] [--cpu index]\n"); return 2; }
	}
	if (Processor >= 0)
	{
#if defined(_WIN32)
		if (Processor >= int(sizeof(DWORD_PTR) * 8) || !SetThreadAffinityMask(GetCurrentThread(), DWORD_PTR(1) << Processor)) return 2;
#elif defined(__linux__)
		if (Processor >= CPU_SETSIZE) return 2;
		cpu_set_t CPUs; CPU_ZERO(&CPUs); CPU_SET(Processor, &CPUs);
		if (sched_setaffinity(0, sizeof(CPUs), &CPUs) != 0) return 2;
#else
		std::fprintf(stderr, "CPU affinity is not supported on this platform\n"); return 2;
#endif
	}
	std::printf("Logical processor: %d (-1 means unpinned)\n", Processor);
	FILE* CSV = Output ? std::fopen(Output, "w") : nullptr;
	if (Output && !CSV) return 2;
	if (CSV) std::fprintf(CSV, "type,m,k,n,mode,median_ms,min_ms,max_ms,gflops,mc,kc,nc\n");
	Eigen::setNbThreads(1);
	const auto& Cache = Maths::LinearAlgebra::GemmDetail::GetCacheSizes();
	std::printf("Eigen %d.%d.%d, single thread, row-major A/B/C, unaligned maps, packing included\n",
		EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION);
	std::printf("Eigen ISA: %s; Riemann packet float=%d double=%d\n", Eigen::SimdInstructionSetsInUse(),
		Maths::LinearAlgebra::GemmDetail::Packet<float>::Width, Maths::LinearAlgebra::GemmDetail::Packet<double>::Width);
#if defined(EIGEN_VECTORIZE_FMA)
	std::printf("Eigen FMA: enabled\n");
#else
	std::printf("Eigen FMA: disabled\n");
#endif
	std::printf("Cache bytes: L1=%zu L2=%zu LLC/logical=%zu line=%zu; samples=5, warmed repeated calls\n",
		Cache.L1, Cache.L2, Cache.LastLevelShare, Cache.Line);
#if defined(_MSC_VER)
	std::printf("Compiler: MSVC %d\n", _MSC_FULL_VER);
#else
	std::printf("Compiler: %s\n", __VERSION__);
#endif
	const int Shapes[][3] = { {32,32,32}, {128,128,128}, {256,256,256}, {512,512,512},
		{67,257,131}, {1024,257,64}, {64,257,1024}, {1024,1024,1024}, {2048,2048,2048} };
	bool Success = true;
	if (Size > 0) Success = Benchmark<float>(Size, Size, Size, CSV) && Benchmark<double>(Size, Size, Size, CSV);
	for (int I = 0; Size <= 0 && I < (Full ? 9 : 7) && Success; ++I)
	{
		Success = Benchmark<float>(Shapes[I][0], Shapes[I][1], Shapes[I][2], CSV)
			&& Benchmark<double>(Shapes[I][0], Shapes[I][1], Shapes[I][2], CSV);
	}
	if (CSV) std::fclose(CSV);
	std::printf("Checksum: %.8g\n", Checksum);
	return Success ? 0 : 1;
}

#endif // RIEMANN_BENCHMARK_GEMM

#if defined(RIEMANN_BENCHMARK_GEMM_INVESTIGATION)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__linux__)
#include <sched.h>
#endif

#include <Eigen/Core>
#include "../Src/LinearSystem/gemm.inl"
#include "GemmPrior/gemm.inl"
#include "GemmFence/gemm.inl"
#include "GemmOrder/gemm.inl"

#if defined(_MSC_VER)
#define GEMM_NOINLINE __declspec(noinline)
#else
#define GEMM_NOINLINE __attribute__((noinline))
#endif

namespace
{
	namespace G = Maths::LinearAlgebra::GemmDetail;
	namespace Old = GemmPrior::LinearAlgebra::GemmDetail;
	using Clock = std::chrono::steady_clock;
	volatile double Checksum = 0;
	const char* Names[] = { "baseline_766fbcf", "compiler_fence_only", "panel_order_only",
		"current", "current_eigen_blocks", "current_no_prefetch", "eigen" };

	template<typename T>
	G::Blocking EigenBlocks(int M, int K, int N)
	{
		// Eigen evaluates row-major output as the transposed column-major product.
		Eigen::Index Depth = K, Rows = N, Columns = M;
		Eigen::internal::computeProductBlockingSizes<T, T>(Depth, Rows, Columns);
		return { (int)Columns, (int)Depth, (int)Rows };
	}

	template<typename T>
	GEMM_NOINLINE void Run(int Mode, int M, int K, int N, const T* A, const T* B, T* C)
	{
		if (Mode == 0) GemmPrior::LinearAlgebra::gemm(A, B, M, K, N, C);
		else if (Mode == 1) GemmFence::LinearAlgebra::gemm(A, B, M, K, N, C);
		else if (Mode == 2) GemmOrder::LinearAlgebra::gemm(A, B, M, K, N, C);
		else if (Mode == 3) Maths::LinearAlgebra::gemm(A, B, M, K, N, C);
		else if (Mode == 6)
		{
			using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
			Eigen::Map<Matrix> Result(C, M, N);
			Result.noalias() = Eigen::Map<const Matrix>(A, M, K) * Eigen::Map<const Matrix>(B, K, N);
		}
		else
		{
			std::fill(C, C + (size_t)M * N, T(0));
			const G::Blocking Blocks = EigenBlocks<T>(M, K, N);
			G::MultiplyPacked(A, B, K, N, M, N, K, C, Mode == 4 ? &Blocks : nullptr, true, Mode != 5);
		}
	}

	template<typename T>
	void Fill(std::vector<T>& Data, unsigned int Seed)
	{
		for (T& Value : Data)
		{
			Seed = 1664525u * Seed + 1013904223u;
			Value = T(int((Seed >> 8) % 2001) - 1000) / T(1000);
		}
	}

	template<typename T>
	bool Check(const std::vector<T>& Actual, const std::vector<T>& Expected, int K)
	{
		for (size_t I = 0; I < Actual.size(); ++I)
		{
			const double Tolerance = std::numeric_limits<T>::epsilon() * (K + 4.0) * 8.0 * (1.0 + std::abs(double(Expected[I])));
			if (!std::isfinite(double(Actual[I])) || std::abs(double(Actual[I]) - double(Expected[I])) > Tolerance)
			{
				std::fprintf(stderr, "Incorrect result at %zu: %.12g vs %.12g\n", I, double(Actual[I]), double(Expected[I]));
				return false;
			}
		}
		return true;
	}

	template<typename T, bool Previous>
	void Profile(int M, int K, int N, const T* A, const T* B, T* C, FILE* CSV, int Sample)
	{
		const int MR = G::KernelShape<T>::Rows, NR = G::KernelShape<T>::Columns;
		const auto BeforeBlocks = Old::SelectBlocking<T>(M, K, N, MR, NR);
		const auto Blocks = Previous ? G::Blocking{ BeforeBlocks.Rows, BeforeBlocks.Depth, BeforeBlocks.Columns }
			: G::SelectBlocking<T>(M, K, N, MR, NR);
		double Time[5] = {};
		auto Begin = Clock::now();
		G::PackedBuffer<T> PA(((size_t)Blocks.Rows + MR - 1) / MR * MR * Blocks.Depth);
		G::PackedBuffer<T> PB(((size_t)Blocks.Columns + NR - 1) / NR * NR * Blocks.Depth);
		auto Record = [&](int Phase) {
			const auto Now = Clock::now();
			Time[Phase] += std::chrono::duration<double, std::milli>(Now - Begin).count();
			Begin = Now;
		};
		Record(0);
		std::fill(C, C + (size_t)M * N, T(0));
		Record(1);
		uint64_t LeftElements = 0, RightElements = 0;
		auto PackA = [&](int Row, int Inner, int Rows, int Depth) {
			Begin = Clock::now();
			if (Previous) Old::PackLeft(A + (size_t)Row * K + Inner, K, Rows, Depth, PA.Get());
			else G::PackLeft(A + (size_t)Row * K + Inner, K, Rows, Depth, PA.Get());
			Record(2); LeftElements += (uint64_t)Rows * Depth;
		};
		auto PackB = [&](int Column, int Inner, int Columns, int Depth) {
			Begin = Clock::now();
			if (Previous) Old::PackRight(B + (size_t)Inner * N + Column, N, Depth, Columns, PB.Get());
			else G::PackRight(B + (size_t)Inner * N + Column, N, Depth, Columns, PB.Get());
			Record(3); RightElements += (uint64_t)Columns * Depth;
		};
		auto Compute = [&](int Row, int Column, int Rows, int Columns, int Depth) {
			Begin = Clock::now();
			if (Previous) Old::MultiplyPanel<T, true, true>(PA.Get(), PB.Get(), Rows, Columns, Depth, C + (size_t)Row * N + Column, N);
			else G::MultiplyPanel<T, true, true>(PA.Get(), PB.Get(), Rows, Columns, Depth, C + (size_t)Row * N + Column, N);
			Record(4);
		};
		const uint64_t RowBlocks = ((uint64_t)M + Blocks.Rows - 1) / Blocks.Rows;
		const uint64_t ColumnBlocks = ((uint64_t)N + Blocks.Columns - 1) / Blocks.Columns;
		if ((uint64_t)M + N * RowBlocks < (uint64_t)M * ColumnBlocks + N)
		{
			for (int Row = 0, Rows; Row < M; Row += Rows)
			{
				Rows = (std::min)(Blocks.Rows, M - Row);
				for (int Inner = 0, Depth; Inner < K; Inner += Depth)
				{
					Depth = (std::min)(Blocks.Depth, K - Inner);
					PackA(Row, Inner, Rows, Depth);
					for (int Column = 0, Columns; Column < N; Column += Columns)
					{
						Columns = (std::min)(Blocks.Columns, N - Column);
						PackB(Column, Inner, Columns, Depth); Compute(Row, Column, Rows, Columns, Depth);
					}
				}
			}
		}
		else
		{
			for (int Column = 0, Columns; Column < N; Column += Columns)
			{
				Columns = (std::min)(Blocks.Columns, N - Column);
				for (int Inner = 0, Depth; Inner < K; Inner += Depth)
				{
					Depth = (std::min)(Blocks.Depth, K - Inner);
					PackB(Column, Inner, Columns, Depth);
					for (int Row = 0, Rows; Row < M; Row += Rows)
					{
						Rows = (std::min)(Blocks.Rows, M - Row);
						PackA(Row, Inner, Rows, Depth); Compute(Row, Column, Rows, Columns, Depth);
					}
				}
			}
		}
		if (CSV) std::fprintf(CSV, "%s,%d,%d,%d,%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%llu,%llu\n",
			sizeof(T) == 4 ? "float" : "double", M, K, N, Previous ? Names[0] : Names[3], Sample,
			Time[0], Time[1], Time[2], Time[3], Time[4], (unsigned long long)LeftElements, (unsigned long long)RightElements);
	}

	template<typename T>
	bool Benchmark(int M, int K, int N, int Samples, FILE* CSV, FILE* Phases)
	{
		std::vector<T> A((size_t)M * K), B((size_t)K * N), C((size_t)M * N), Expected(C.size());
		Fill(A, 17); Fill(B, 31);
		Run(6, M, K, N, A.data(), B.data(), Expected.data());
		const G::Blocking Blocks = G::SelectBlocking<T>(M, K, N, G::KernelShape<T>::Rows, G::KernelShape<T>::Columns);
		const auto EB = EigenBlocks<T>(M, K, N);
		const char* Type = sizeof(T) == 4 ? "float" : "double";
		std::printf("\n%s %dx%dx%d Riemann MC/KC/NC=%d/%d/%d Eigen(row-major)=%d/%d/%d\n",
			Type, M, K, N, Blocks.Rows, Blocks.Depth, Blocks.Columns, EB.Rows, EB.Depth, EB.Columns);
		int Repeats[7];
		std::vector<double> Times[7];
		for (int Mode = 0; Mode < 7; ++Mode)
		{
			Run(Mode, M, K, N, A.data(), B.data(), C.data());
			if (!Check(C, Expected, K)) return false;
			const auto Begin = Clock::now();
			Run(Mode, M, K, N, A.data(), B.data(), C.data());
			const double Seconds = std::chrono::duration<double>(Clock::now() - Begin).count();
			Repeats[Mode] = (std::max)(1, (std::min)(1000, int(0.015 / (std::max)(Seconds, 1.0e-9))));
		}
		for (int Sample = 0; Sample < Samples; ++Sample)
			for (int Offset = 0; Offset < 7; ++Offset)
			{
				const int Mode = (Offset + Sample * 3) % 7;
				const auto Begin = Clock::now();
				for (int Repeat = 0; Repeat < Repeats[Mode]; ++Repeat)
				{
					Run(Mode, M, K, N, A.data(), B.data(), C.data());
					Checksum += double(C[(size_t)(Sample + Repeat) % C.size()]);
				}
				Times[Mode].push_back(std::chrono::duration<double, std::milli>(Clock::now() - Begin).count() / Repeats[Mode]);
			}
		for (int Mode = 0; Mode < 7; ++Mode)
		{
			std::sort(Times[Mode].begin(), Times[Mode].end());
			const double Median = Times[Mode][Samples / 2];
			const double GFLOPS = 2.0 * M * K * N / (Median * 1.0e6);
			const auto Used = Mode == 4 || Mode == 6 ? EB : Blocks;
			std::printf("  %-23s %9.4f ms [%9.4f, %9.4f] %7.2f GFLOP/s\n", Names[Mode], Median, Times[Mode].front(), Times[Mode].back(), GFLOPS);
			if (CSV) std::fprintf(CSV, "%s,%d,%d,%d,%s,%.6f,%.6f,%.6f,%.4f,%d,%d,%d\n", Type, M, K, N, Names[Mode], Median,
				Times[Mode].front(), Times[Mode].back(), GFLOPS, Used.Rows, Used.Depth, Used.Columns);
		}
		if (Phases)
			for (int Sample = 0; Sample < 3; ++Sample)
				for (int Offset = 0; Offset < 2; ++Offset)
				{
					if ((Sample + Offset) % 2) Profile<T, true>(M, K, N, A.data(), B.data(), C.data(), Phases, Sample);
					else Profile<T, false>(M, K, N, A.data(), B.data(), C.data(), Phases, Sample);
					if (!Check(C, Expected, K)) return false;
				}
		std::fflush(stdout);
		if (CSV) std::fflush(CSV);
		if (Phases) std::fflush(Phases);
		return true;
	}
}

int main(int Argc, char** Argv)
{
	bool Full = false, ProfilePhases = false;
	int Processor = -1, Size = 0, Samples = 7;
	const char* Output = nullptr;
	for (int I = 1; I < Argc; ++I)
	{
		if (std::strcmp(Argv[I], "--full") == 0) Full = true;
		else if (std::strcmp(Argv[I], "--quick") == 0) Full = false;
		else if (std::strcmp(Argv[I], "--profile") == 0) ProfilePhases = true;
		else if (std::strcmp(Argv[I], "--csv") == 0 && I + 1 < Argc) Output = Argv[++I];
		else if (std::strcmp(Argv[I], "--cpu") == 0 && I + 1 < Argc) Processor = std::atoi(Argv[++I]);
		else if (std::strcmp(Argv[I], "--size") == 0 && I + 1 < Argc) Size = std::atoi(Argv[++I]);
		else if (std::strcmp(Argv[I], "--samples") == 0 && I + 1 < Argc) Samples = std::atoi(Argv[++I]);
		else return 2;
	}
	if (Size < 0 || Samples < 3 || Samples % 2 == 0 || (ProfilePhases && !Output)) return 2;
	if (Processor >= 0)
	{
#if defined(_WIN32)
		if (Processor >= int(sizeof(DWORD_PTR) * 8) || !SetThreadAffinityMask(GetCurrentThread(), DWORD_PTR(1) << Processor)) return 2;
#elif defined(__linux__)
		if (Processor >= CPU_SETSIZE) return 2;
		cpu_set_t CPUs; CPU_ZERO(&CPUs); CPU_SET(Processor, &CPUs);
		if (sched_setaffinity(0, sizeof(CPUs), &CPUs) != 0) return 2;
#else
		return 2;
#endif
	}
	FILE* CSV = Output ? std::fopen(Output, "w") : nullptr;
	if (Output && !CSV) return 2;
	FILE* Phases = ProfilePhases ? std::fopen((std::string(Output) + ".phases.csv").c_str(), "w") : nullptr;
	if (ProfilePhases && !Phases) { if (CSV) std::fclose(CSV); return 2; }
	if (CSV) std::fprintf(CSV, "type,m,k,n,mode,median_ms,min_ms,max_ms,gflops,mc,kc,nc\n");
	if (Phases) std::fprintf(Phases, "type,m,k,n,mode,sample,allocation_ms,clear_ms,pack_a_ms,pack_b_ms,compute_ms,pack_a_elements,pack_b_elements\n");
	Eigen::setNbThreads(1);
	std::printf("Eigen %d.%d.%d; %s; Riemann packets=%d/%d; logical processor=%d; samples=%d\n",
		EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION, Eigen::SimdInstructionSetsInUse(),
		G::Packet<float>::Width, G::Packet<double>::Width, Processor, Samples);
	const int Shapes[][3] = { {32,32,32}, {128,128,128}, {256,256,256}, {512,512,512},
		{67,257,131}, {1024,257,64}, {64,257,1024}, {1024,1024,1024}, {2048,2048,2048} };
	bool Success = true;
	if (Size > 0) Success = Benchmark<float>(Size, Size, Size, Samples, CSV, Phases) && Benchmark<double>(Size, Size, Size, Samples, CSV, Phases);
	for (int I = 0; Size == 0 && I < (Full ? 9 : 7) && Success; ++I)
		Success = Benchmark<float>(Shapes[I][0], Shapes[I][1], Shapes[I][2], Samples, CSV, Phases)
			&& Benchmark<double>(Shapes[I][0], Shapes[I][1], Shapes[I][2], Samples, CSV, Phases);
	if (CSV) std::fclose(CSV);
	if (Phases) std::fclose(Phases);
	std::printf("Checksum: %.8g\n", Checksum);
	return Success ? 0 : 1;
}

#include "../Src/LinearSystem/GemmKernel.h"
#include "GemmPrior/GemmKernel.h"

// These external wrappers make the current and historical reduction loops easy
// to find in compiler assembly listings. They are not used by the timed calls.
extern "C" void GemmCurrentFloat(const float* A, const float* B, int K, float* C, int N)
{
	Maths::LinearAlgebra::GemmDetail::VectorKernel<float>(A, B, K, C, N);
}
extern "C" void GemmCurrentDouble(const double* A, const double* B, int K, double* C, int N)
{
	Maths::LinearAlgebra::GemmDetail::VectorKernel<double>(A, B, K, C, N);
}
extern "C" void GemmPriorFloat(const float* A, const float* B, int K, float* C, int N)
{
	GemmPrior::LinearAlgebra::GemmDetail::VectorKernel<float>(A, B, K, C, N);
}
extern "C" void GemmPriorDouble(const double* A, const double* B, int K, double* C, int N)
{
	GemmPrior::LinearAlgebra::GemmDetail::VectorKernel<double>(A, B, K, C, N);
}

#endif // RIEMANN_BENCHMARK_GEMM_INVESTIGATION

// Large GEMM benchmark variants.
#if defined(RIEMANN_BENCHMARK_LARGE_GEMM)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
#include <Eigen/Core>
#include "../Src/LinearSystem/gemm.inl"
namespace LargeStudyControl { int CacheMode = 0; }
#include "LargeBase/gemm.inl"
#include "LargeCache/gemm.inl"
#include "LargeUnroll2/gemm.inl"
#include "LargeUnroll4/gemm.inl"
#include "LargeUnroll8/gemm.inl"
#include "LargeDoublePack/gemm.inl"
#include "LargePrefetch/gemm.inl"
#include "LargeUnrollPrefetch/gemm.inl"
#include "LargeReuse/gemm.inl"
#include "LargeZero/gemm.inl"
#include "LargeCombined/gemm.inl"
#include "LargeShape6/gemm.inl"
#include "LargeShape6Unroll/gemm.inl"
#include "LargeRowGroup/gemm.inl"
#include "LargeInlineUnroll/gemm.inl"
#include "LargeInlineCombined/gemm.inl"
#include "LargeSwapUnroll/gemm.inl"
#include "LargeSwapCombined/gemm.inl"
#include "LargeForcePacket/gemm.inl"
#include "LargeRefAcc/gemm.inl"
#include "LargeSizeUnroll/gemm.inl"
#include "LargeTwoPackets2/gemm.inl"
#include "LargeTwoPackets4/gemm.inl"
#include "LargeDuff8/gemm.inl"
#include "LargeDuff4/gemm.inl"
#include "LargeDuffZero/gemm.inl"
#include "LargeTuned/gemm.inl"
#include "LargeTunedPrefetch/gemm.inl"
#include "LargeTunedIndex/gemm.inl"
#include "LargeTunedWholeA/gemm.inl"
#include "LargeSlabUnroll/gemm.inl"
#include "LargeSlabDuff/gemm.inl"
#include "LargeTunedPrepackA/gemm.inl"
#include "LargeTunedOffsets/gemm.inl"
#include "LargeTunedOffsetsPrefetch/gemm.inl"

#if defined(_MSC_VER)
#define GEMM_NOINLINE __declspec(noinline)
#else
#define GEMM_NOINLINE __attribute__((noinline))
#endif

namespace
{
	using Clock = std::chrono::steady_clock;
	volatile double Checksum = 0;
	int FirstVariant = 3, LastVariant = 40, TestPolicy = 0, Compare = -1, Reference = 0;
	bool Final = false, Chain = false;
	double BatchMilliseconds = 30.0;
	const char* Names[] = { "baseline_a079fd5", "eigen", "production", "full_L1", "outer_1p5MiB", "full_L1_outer", "larger_rows", "whole_columns", "unroll2", "unroll4", "unroll8", "double_pack", "prefetch", "unroll_prefetch", "reuse", "zero", "combined", "shape6", "shape6_unroll", "row_group", "inline_unroll", "inline_combined", "swap_unroll", "swap_combined", "force_packet", "ref_acc", "size_unroll", "two_packets2", "two_packets4", "duff8", "duff4", "duff_zero", "tuned", "tuned_prefetch", "tuned_index", "tuned_whole_a", "slab_unroll", "slab_duff", "tuned_prepack_a", "tuned_offsets", "tuned_offsets_prefetch" };
	const int StageCount = sizeof(Names) / sizeof(Names[0]);

	template<typename T>
	GEMM_NOINLINE void Run(int Stage, int M, int K, int N, const T* A, const T* B, T* C)
	{
		LargeStudyControl::CacheMode = Stage >= 3 && Stage <= 7 ? Stage - 2 : TestPolicy;
		if (Stage >= 3 && Stage <= 7) { LargeCache::LinearAlgebra::gemm(A,B,M,K,N,C); return; }
		switch (Stage)
		{
		case 0: LargeBase::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 2: Maths::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 8: LargeUnroll2::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 9: LargeUnroll4::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 10: LargeUnroll8::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 11: LargeDoublePack::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 12: LargePrefetch::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 13: LargeUnrollPrefetch::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 14: LargeReuse::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 15: LargeZero::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 16: LargeCombined::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 17: LargeShape6::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 18: LargeShape6Unroll::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 19: LargeRowGroup::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 20: LargeInlineUnroll::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 21: LargeInlineCombined::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 22: LargeSwapUnroll::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 23: LargeSwapCombined::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 24: LargeForcePacket::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 25: LargeRefAcc::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 26: LargeSizeUnroll::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 27: LargeTwoPackets2::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 28: LargeTwoPackets4::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 29: LargeDuff8::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 30: LargeDuff4::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 31: LargeDuffZero::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 32: LargeTuned::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 33: LargeTunedPrefetch::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 34: LargeTunedIndex::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 35: LargeTunedWholeA::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 36: LargeSlabUnroll::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 37: LargeSlabDuff::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 38: LargeTunedPrepackA::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 39: LargeTunedOffsets::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 40: LargeTunedOffsetsPrefetch::LinearAlgebra::gemm(A,B,M,K,N,C); break;
		case 1:
			using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
			Eigen::Map<Matrix>(C, M, N).noalias() = Eigen::Map<const Matrix>(A, M, K) * Eigen::Map<const Matrix>(B, K, N);
			break;
		}
	}

	template<typename T>
	void Fill(std::vector<T>& Data, unsigned int Seed)
	{
		for (T& Value : Data)
		{
			Seed = 1664525u * Seed + 1013904223u;
			Value = T(int((Seed >> 8) % 2001) - 1000) / T(1000);
		}
	}
	template<typename T>
	bool Check(const T* Actual, const std::vector<T>& Expected, int K, int Stage)
	{
		for (size_t I = 0; I < Expected.size(); ++I)
		{
			const double Tolerance = std::numeric_limits<T>::epsilon() * (K + 4.0) * 8.0 * (1.0 + std::abs(double(Expected[I])));
			if (!std::isfinite(double(Actual[I])) || std::abs(double(Actual[I]) - double(Expected[I])) > Tolerance)
			{
				std::fprintf(stderr, "Mismatch stage %d at %zu: %.12g vs %.12g\n", Stage, I, double(Actual[I]), double(Expected[I]));
				return false;
			}
		}
		return true;
	}

	double Median(std::vector<double> Values)
	{
		std::sort(Values.begin(), Values.end());
		return Values[Values.size() / 2];
	}

	template<typename T>
	bool Benchmark(int M, int K, int N, int Samples, bool Validate, FILE* CSV, FILE* Raw)
	{
		const char* Type = sizeof(T) == 4 ? "float" : "double";
		std::vector<T> A((size_t)M * K), B((size_t)K * N), Storage((size_t)M * N + 2, T(-999));
		std::vector<T> Expected((size_t)M * N);
		T* C = Storage.data() + (Validate ? 1 : 0);
		Fill(A, 17); Fill(B, 31);
		Run(1, M, K, N, A.data(), B.data(), Expected.data());
		for (int Sample = 0; Sample < 13; ++Sample)
		{
			const int I = Sample * 97 % M, J = Sample * 193 % N;
			long double Sum = 0;
			for (int P = 0; P < K; ++P) Sum += (long double)A[(size_t)I * K + P] * B[(size_t)P * N + J];
			const double Tolerance = std::numeric_limits<T>::epsilon() * (K + 4.0) * 8 * (1 + std::abs(double(Sum)));
			if (std::abs(double(Expected[(size_t)I * N + J]) - double(Sum)) > Tolerance) return false;
		}
		const int FirstStage = 0;
		std::vector<std::pair<int, int>> Edges = { {0,2}, {2,1} };
		if (!Final) for (int Stage = FirstVariant; Stage <= LastVariant; ++Stage) Edges.emplace_back(0, Stage);
		if (Compare >= 0) Edges = { {Reference,Compare}, {Compare,1} };
		if (Chain) Edges = { {0,3}, {3,5}, {5,11}, {11,32}, {32,39}, {39,40}, {39,1} };
		int Repeats[StageCount] = {};
		std::printf("\n%s %dx%dx%d, first stage=%d\n", Type, M, K, N, FirstStage);
		std::fflush(stdout);
		for (int Stage = FirstStage; Stage < StageCount; ++Stage)
		{
			bool Needed = false;
			for (const auto& Edge : Edges) Needed = Needed || Stage == Edge.first || Stage == Edge.second;
			if (!Validate && !Needed) continue;
			Run(Stage, M, K, N, A.data(), B.data(), C);
			if (!Check(C, Expected, K, Stage)) return false;
			if (Validate)
			{
				if (Storage.front() != T(-999) || Storage.back() != T(-999)) return false;
				continue;
			}
			const auto Begin = Clock::now();
			Run(Stage, M, K, N, A.data(), B.data(), C);
			const double Seconds = std::chrono::duration<double>(Clock::now() - Begin).count();
			Repeats[Stage] = (std::max)(1, (std::min)(1000000, int((BatchMilliseconds / 1000) / (std::max)(Seconds, 1.0e-9))));
		}
		if (Validate) return true;
		std::vector<std::vector<double>> Before(Edges.size()), After(Edges.size()), Ratios(Edges.size());
		std::vector<std::vector<double>> CycleRatios(Edges.size());
		struct Measurement { double Milliseconds, Cycles, HostLoad; };
		auto Timed = [&](int Stage, int Sample) {
#if defined(_WIN32)
			ULONG64 Cycles0 = 0, Cycles1 = 0;
			FILETIME Idle0{}, Kernel0{}, User0{}, Idle1{}, Kernel1{}, User1{};
			GetSystemTimes(&Idle0, &Kernel0, &User0);
			QueryThreadCycleTime(GetCurrentThread(), &Cycles0);
#endif
			const auto Begin = Clock::now();
			for (int Repeat = 0; Repeat < Repeats[Stage]; ++Repeat)
			{
				Run(Stage, M, K, N, A.data(), B.data(), C);
				Checksum += double(C[(size_t)(Sample + Repeat) % Expected.size()]);
			}
			const double Milliseconds = std::chrono::duration<double, std::milli>(Clock::now() - Begin).count() / Repeats[Stage];
			double Cycles = 0, HostLoad = -1;
#if defined(_WIN32)
			QueryThreadCycleTime(GetCurrentThread(), &Cycles1);
			GetSystemTimes(&Idle1, &Kernel1, &User1);
			auto Ticks = [](FILETIME T) { return (uint64_t(T.dwHighDateTime) << 32) | T.dwLowDateTime; };
			const uint64_t Total = Ticks(Kernel1) - Ticks(Kernel0) + Ticks(User1) - Ticks(User0);
			if (Total) HostLoad = 1.0 - double(Ticks(Idle1) - Ticks(Idle0)) / Total;
			Cycles = double(Cycles1 - Cycles0) / Repeats[Stage];
#endif
			return Measurement{Milliseconds, Cycles, HostLoad};
		};
		for (int Sample = 0; Sample < Samples; ++Sample)
		{
			for (size_t Offset = 0; Offset < Edges.size(); ++Offset)
			{
				const size_t E = (Offset + Sample * 5) % Edges.size();
				const int S0 = Edges[E].first, S1 = Edges[E].second;
				Measurement B, A;
				if ((Sample + E) % 2) { A = Timed(S1, Sample); B = Timed(S0, Sample); }
				else { B = Timed(S0, Sample); A = Timed(S1, Sample); }
				const double T0 = B.Milliseconds, T1 = A.Milliseconds;
				Before[E].push_back(T0); After[E].push_back(T1); Ratios[E].push_back(T0 / T1);
				CycleRatios[E].push_back(A.Cycles > 0 ? B.Cycles / A.Cycles : 0);
				if (Raw) std::fprintf(Raw, "%s,%d,%d,%d,%d,%d,%d,%s,%d,%d,%.9f,%.9f,%.9f,%.1f,%.1f,%.6f,%.6f\n", Type, M, K, N, S0, S1, Sample,
					(Sample + E) % 2 ? "after_first" : "before_first", Repeats[S0], Repeats[S1], T0, T1, T0 / T1, B.Cycles, A.Cycles, B.HostLoad, A.HostLoad);
			}
			if (Raw) std::fflush(Raw);
		}
		for (size_t E = 0; E < Edges.size(); ++E)
		{
			const int S0 = Edges[E].first, S1 = Edges[E].second;
			const double T0 = Median(Before[E]), T1 = Median(After[E]), Ratio = Median(Ratios[E]);
			std::printf("  S%02d -> S%02d %-24s %10.5f -> %10.5f ms  paired %.3fx  cycles %.3fx\n", S0, S1, Names[S1], T0, T1, Ratio, Median(CycleRatios[E]));
			if (CSV) std::fprintf(CSV, "%s,%d,%d,%d,%d,%d,%s,%s,%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n", Type, M, K, N, S0, S1, Names[S0], Names[S1], Samples,
				T0, T1, Ratio, *std::min_element(Ratios[E].begin(), Ratios[E].end()), *std::max_element(Ratios[E].begin(), Ratios[E].end()), Median(CycleRatios[E]));
		}
		std::fflush(stdout);
		if (CSV) std::fflush(CSV);
		return true;
	}
}

int main(int Argc, char** Argv)
{
	bool Full = false, Validate = false, Small = false, Large = false, HighPriority = false;
	int Samples = 5, Size = 0, CPU = 2;
	const char* Output = nullptr;
	for (int I = 1; I < Argc; ++I)
	{
		if (!std::strcmp(Argv[I], "--full")) Full = true;
		else if (!std::strcmp(Argv[I], "--large")) Large = true;
		else if (!std::strcmp(Argv[I], "--priority-high")) HighPriority = true;
		else if (!std::strcmp(Argv[I], "--chain")) Chain = true;
		else if (!std::strcmp(Argv[I], "--batch-ms") && I + 1 < Argc) BatchMilliseconds = std::atof(Argv[++I]);
		else if (!std::strcmp(Argv[I], "--small")) Small = true;
		else if (!std::strcmp(Argv[I], "--final")) Final = true;
		else if (!std::strcmp(Argv[I], "--first") && I + 1 < Argc) FirstVariant = std::atoi(Argv[++I]);
		else if (!std::strcmp(Argv[I], "--last") && I + 1 < Argc) LastVariant = std::atoi(Argv[++I]);
		else if (!std::strcmp(Argv[I], "--policy") && I + 1 < Argc) TestPolicy = std::atoi(Argv[++I]);
		else if (!std::strcmp(Argv[I], "--compare") && I + 1 < Argc) Compare = std::atoi(Argv[++I]);
		else if (!std::strcmp(Argv[I], "--reference") && I + 1 < Argc) Reference = std::atoi(Argv[++I]);
		else if (!std::strcmp(Argv[I], "--validate")) Validate = true;
		else if (!std::strcmp(Argv[I], "--csv") && I + 1 < Argc) Output = Argv[++I];
		else if (!std::strcmp(Argv[I], "--size") && I + 1 < Argc) Size = std::atoi(Argv[++I]);
		else if (!std::strcmp(Argv[I], "--samples") && I + 1 < Argc) Samples = std::atoi(Argv[++I]);
		else if (!std::strcmp(Argv[I], "--cpu") && I + 1 < Argc) CPU = std::atoi(Argv[++I]);
		else return 2;
	}
	if (Compare >= 3) FirstVariant = LastVariant = Compare;
	if (Chain) TestPolicy = 3;
	if (Samples < 3 || Samples % 2 == 0 || Size < 0 || FirstVariant < 3 || LastVariant >= StageCount || LastVariant < FirstVariant || TestPolicy < 0 || TestPolicy > 5 || Compare >= StageCount || Compare == 1 || Reference < 0 || Reference >= StageCount || BatchMilliseconds <= 0 || !std::isfinite(BatchMilliseconds)) return 2;
#if defined(_WIN32)
	if (CPU >= 0 && (CPU >= int(sizeof(DWORD_PTR) * 8) || !SetThreadAffinityMask(GetCurrentThread(), DWORD_PTR(1) << CPU))) return 2;
	if (HighPriority && (!SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS)
		|| !SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST))) return 2;
#endif
	FILE* CSV = Output ? std::fopen(Output, "w") : nullptr;
	FILE* Raw = Output ? std::fopen((std::string(Output) + ".samples.csv").c_str(), "w") : nullptr;
	if (Output && (!CSV || !Raw)) return 2;
	if (CSV) std::fprintf(CSV, "type,m,k,n,before_stage,after_stage,before_name,after_name,samples,before_median_ms,after_median_ms,paired_speedup,paired_min,paired_max,paired_cycle_speedup\n");
	if (Raw) std::fprintf(Raw, "type,m,k,n,before_stage,after_stage,sample,order,before_repeats,after_repeats,before_ms,after_ms,speedup,before_cycles,after_cycles,before_host_load,after_host_load\n");
	Eigen::setNbThreads(1);
	const auto Cache = Maths::LinearAlgebra::GemmDetail::GetCacheSizes();
	std::printf("Large study: public entry points, controlled variants from a079fd5, single thread, CPU=%d, paired samples=%d, candidate cache policy=%d\n", CPU, Samples, TestPolicy);
	std::printf("Target batch %.1f ms, chain=%d, high priority=%d\n", BatchMilliseconds, int(Chain), int(HighPriority));
	std::printf("Eigen %d.%d.%d, ISA %s; cache L1=%zu L2=%zu LLCshare=%zu\n", EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION,
		Eigen::SimdInstructionSetsInUse(), Cache.L1, Cache.L2, Cache.LastLevelShare);
#if defined(_MSC_VER)
	std::printf("MSVC=%d\n", _MSC_FULL_VER);
#endif
	const int Shapes[][3] = { {32,32,32}, {128,128,128}, {256,256,256}, {512,512,512},
		{67,257,131}, {1024,257,64}, {64,257,1024}, {1024,1024,1024}, {2048,2048,2048} };
	const int Checks[][3] = { {1,1,1}, {7,13,9}, {31,65,17}, {67,257,131}, {137,601,197}, {257,257,769} };
	const int LargeShapes[][3] = { {512,512,512},{1024,1024,1024},{1536,1536,1536},{2048,2048,2048},
		{3072,3072,3072},{4096,4096,4096},{256,2048,1024},{1024,2048,256},{2048,512,2048},{511,1023,769} };
	const int SmallShapes[][3] = { {4,4,4},{8,8,8},{16,16,16},{24,24,24},{32,32,32},{48,48,48},{64,64,64},{96,96,96},
		{3,31,32},{7,13,32},{31,33,32},{63,64,64},{64,65,64},{64,64,63} };
	bool Success = true;
	if (Validate)
	{
		for (const auto& Shape : Checks)
			Success = Success && Benchmark<float>(Shape[0], Shape[1], Shape[2], Samples, true, nullptr, nullptr)
				&& Benchmark<double>(Shape[0], Shape[1], Shape[2], Samples, true, nullptr, nullptr);
		for (const auto& Shape : SmallShapes)
			Success = Success && Benchmark<float>(Shape[0], Shape[1], Shape[2], Samples, true, nullptr, nullptr)
				&& Benchmark<double>(Shape[0], Shape[1], Shape[2], Samples, true, nullptr, nullptr);
	}
	else if (Small)
		for (const auto& Shape : SmallShapes)
			Success = Success && Benchmark<float>(Shape[0], Shape[1], Shape[2], Samples, false, CSV, Raw)
				&& Benchmark<double>(Shape[0], Shape[1], Shape[2], Samples, false, CSV, Raw);
	else if (Large)
		for (const auto& Shape : LargeShapes)
			Success = Success && Benchmark<float>(Shape[0], Shape[1], Shape[2], Samples, false, CSV, Raw)
				&& Benchmark<double>(Shape[0], Shape[1], Shape[2], Samples, false, CSV, Raw);
	else if (Size > 0) Success = Benchmark<float>(Size, Size, Size, Samples, false, CSV, Raw) && Benchmark<double>(Size, Size, Size, Samples, false, CSV, Raw);
	else for (int I = 0; I < (Full ? 9 : 7) && Success; ++I)
		Success = Benchmark<float>(Shapes[I][0], Shapes[I][1], Shapes[I][2], Samples, false, CSV, Raw)
			&& Benchmark<double>(Shapes[I][0], Shapes[I][1], Shapes[I][2], Samples, false, CSV, Raw);
	if (CSV) std::fclose(CSV);
	if (Raw) std::fclose(Raw);
	std::printf("Checksum %.8g, correctness=%s\n", Checksum, Success ? "passed" : "failed");
	return Success ? 0 : 1;
}

#endif // RIEMANN_BENCHMARK_LARGE_GEMM

#if defined(RIEMANN_BENCHMARK_LARGE_COMPONENTS)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>
#define NOMINMAX
#include <windows.h>
#include "../Src/LinearSystem/gemm.inl"
#include "LargeBase/gemm.inl"

namespace
{
	namespace Current = Maths::LinearAlgebra::GemmDetail;
	namespace Previous = LargeBase::LinearAlgebra::GemmDetail;
	using Clock = std::chrono::steady_clock;
	volatile double Checksum = 0;

	template<typename T>
	__declspec(noinline) void Pack(bool New, const T* A, int Stride, int Rows, int Depth, T* Packed)
	{
		if (New) Current::PackLeft(A, Stride, Rows, Depth, Packed);
		else Previous::PackLeft(A, Stride, Rows, Depth, Packed);
	}

	template<typename T>
	__declspec(noinline) void Compute(bool New, const T* A, const T* B, int Depth, T* C, int Stride)
	{
		if (New) Current::VectorKernel(A, B, Depth, C, Stride);
		else Previous::VectorKernel(A, B, Depth, C, Stride);
	}

	template<typename T>
	bool Measure(FILE* Out, bool Kernel, int Rows, int Depth, int Stride)
	{
		const int MR = Current::KernelShape<T>::Rows, NR = Current::KernelShape<T>::Columns;
		std::vector<T> A((size_t)Rows * Stride + 1), B((size_t)Depth * NR);
		for (size_t I = 0; I < A.size(); ++I) A[I] = T(int(I % 17) - 8) / 17;
		for (size_t I = 0; I < B.size(); ++I) B[I] = T(int(I % 13) - 6) / 13;
		const size_t Count = Kernel ? (size_t)MR * NR : ((size_t)Rows + MR - 1) / MR * MR * Depth;
		std::vector<T> Old(Count, T(0)), New(Count, T(0));
		auto Run = [&](bool Latest, T* C) {
			if (Kernel) Compute(Latest, A.data() + 1, B.data(), Depth, C, NR);
			else Pack(Latest, A.data() + 1, Stride, Rows, Depth, C);
		};
		Run(false, Old.data()); Run(true, New.data());
		for (size_t I = 0; I < Count; ++I)
			if (!std::isfinite(double(New[I])) || std::abs(double(New[I]) - Old[I])
				> std::numeric_limits<T>::epsilon() * (Depth + 4) * 8 * (1 + std::abs(double(Old[I])))) return false;
		const auto Begin = Clock::now();
		Run(false, Old.data());
		const double Seconds = std::chrono::duration<double>(Clock::now() - Begin).count();
		const int Repeats = (std::max)(1, (std::min)(1000000, int(0.060 / (std::max)(Seconds, 1e-9))));
		auto Timed = [&](bool Latest) {
			auto& C = Latest ? New : Old;
			std::fill(C.begin(), C.end(), T(0));
			const auto Start = Clock::now();
			for (int Repeat = 0; Repeat < Repeats; ++Repeat) Run(Latest, C.data());
			const double MS = std::chrono::duration<double, std::milli>(Clock::now() - Start).count() / Repeats;
			Checksum += C[0];
			return MS;
		};
		for (int Sample = 0; Sample < 15; ++Sample)
		{
			double Before, After;
			if (Sample % 2) { After = Timed(true); Before = Timed(false); }
			else { Before = Timed(false); After = Timed(true); }
			std::fprintf(Out, "%s,%s,%d,%d,%d,%d,%d,%.9f,%.9f,%.9f\n", sizeof(T) == 4 ? "float" : "double",
				Kernel ? "kernel" : "pack_left", Rows, Depth, Stride, Sample, Repeats, Before, After, Before / After);
		}
		std::fflush(Out);
		return true;
	}
}

int main(int Argc, char** Argv)
{
	if (Argc != 2 || !SetThreadAffinityMask(GetCurrentThread(), DWORD_PTR(1) << 18)) return 2;
	if (!SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS)) return 2;
	FILE* Out = std::fopen(Argv[1], "w");
	if (!Out) return 2;
	std::fprintf(Out, "type,component,rows,depth,stride,sample,repeats,before_ms,after_ms,speedup\n");
	bool Good = true;
	const int Cases[][3] = { {4,64,67},{68,232,2048},{128,256,2048},{129,257,1027},{512,232,2048},{2048,232,2048} };
	for (const auto& Shape : Cases)
		Good = Good && Measure<float>(Out, false, Shape[0], Shape[1], Shape[2])
			&& Measure<double>(Out, false, Shape[0], Shape[1], Shape[2]);
	for (int K : {31,176,208,232,256,512})
		Good = Good && Measure<float>(Out, true, 4, K, K) && Measure<double>(Out, true, 4, K, K);
	std::fclose(Out);
	std::printf("Component correctness=%s checksum=%.8g\n", Good ? "passed" : "failed", Checksum);
	return Good && std::isfinite(Checksum) ? 0 : 1;
}

#endif // RIEMANN_BENCHMARK_LARGE_COMPONENTS

// Production GEMM correctness regression used by the large benchmark suite.
#if defined(RIEMANN_BENCHMARK_GEMM_REGRESSION)
#include "Test_GEMM.cpp"

int main()
{
	TestResetFailures();
	TestGemmCorrectness<float>();
	TestGemmCorrectness<double>();
	return TestFailureCount() == 0 ? 0 : 1;
}

#endif // RIEMANN_BENCHMARK_GEMM_REGRESSION
