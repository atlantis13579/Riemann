#include "Test.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <vector>

#include "../Src/LinearSystem/DenseMatrix.h"
#include "../Src/LinearSystem/QRDecomposition.h"

using namespace Maths::LinearAlgebra;

namespace
{
	template<typename T>
	void FillGemmInput(std::vector<T>& values, int seed)
	{
		for (size_t i = 0; i < values.size(); ++i)
		{
			const int value = (int)((i * 37 + (size_t)seed * 19) % 127) - 63;
			values[i] = (T)value / (T)31;
		}
	}

	template<typename T>
	bool GemmResultsAgree(const T* actual, const T* expected, size_t count, int depth)
	{
		for (size_t i = 0; i < count; ++i)
		{
			const double tolerance = (double)std::numeric_limits<T>::epsilon()
				* (double)(depth + 4) * 4.0 * (1.0 + std::abs((double)expected[i]));
			if (!std::isfinite((double)actual[i]) || std::abs((double)actual[i] - (double)expected[i]) > tolerance)
			{
				printf("GEMM mismatch at %zu: actual %.12g, expected %.12g, tolerance %.3g\n",
					i, (double)actual[i], (double)expected[i], tolerance);
				return false;
			}
		}
		return true;
	}

	template<typename T>
	void TestGemmShape(int rows, int depth, int columns)
	{
		std::vector<T> left((size_t)rows * depth);
		std::vector<T> right((size_t)depth * columns);
		std::vector<T> expected((size_t)rows * columns);
		std::vector<T> result(expected.size() + 2, (T)-1234.25);
		FillGemmInput(left, 1);
		FillGemmInput(right, 7);
		gemm_block_ref(left.data(), right.data(), rows, depth, columns,
			0, rows - 1, 0, depth - 1, 0, columns - 1, expected.data());
		// The extra element deliberately misaligns the output on SIMD platforms.
		std::fill(result.begin() + 1, result.end() - 1, std::numeric_limits<T>::quiet_NaN());
		gemm(left.data(), right.data(), rows, depth, columns, result.data() + 1);
		EXPECT(GemmResultsAgree(result.data() + 1, expected.data(), expected.size(), depth));
		EXPECT(result.front() == (T)-1234.25 && result.back() == (T)-1234.25);
	}

	template<typename T>
	void TestGemmSubblocks()
	{
		const int rows = 75;
		const int depth = 291;
		const int columns = 143;
		const T sentinel = (T)-1234.25;
		std::vector<T> left((size_t)rows * depth);
		std::vector<T> right((size_t)depth * columns);
		std::vector<T> expected((size_t)rows * columns, sentinel);
		std::vector<T> result(expected);
		FillGemmInput(left, 3);
		FillGemmInput(right, 9);
		const int firstRow = 3, lastRow = 70;
		const int firstInner = 5, lastInner = 278;
		const int firstColumn = 3, lastColumn = 137;
		gemm_block_ref(left.data(), right.data(), rows, depth, columns,
			firstRow, lastRow, firstInner, lastInner, firstColumn, lastColumn, expected.data());
		for (int row = firstRow; row <= lastRow; ++row)
		{
			for (int column = firstColumn; column <= lastColumn; ++column)
			{
				result[(size_t)row * columns + column] = std::numeric_limits<T>::quiet_NaN();
			}
		}
		// Cross all three cache blocks with nonzero starts, then repeat to verify overwrite semantics.
		for (int pass = 0; pass < 2; ++pass)
		{
			gemm_block(left.data(), right.data(), rows, depth, columns,
				firstRow, lastRow, firstInner, lastInner, firstColumn, lastColumn, result.data());
			EXPECT(GemmResultsAgree(result.data(), expected.data(), expected.size(), lastInner - firstInner + 1));
			for (int row = 0; row < rows; ++row)
			{
				for (int column = 0; column < columns; ++column)
				{
					if (row < firstRow || row > lastRow || column < firstColumn || column > lastColumn)
					{
						EXPECT(result[(size_t)row * columns + column] == sentinel);
					}
				}
			}
		}
	}

	template<typename T>
	void TestGemmAliasing()
	{
		{
			const int rows = 67, depth = 131, columns = 131;
			std::vector<T> left((size_t)rows * depth), right((size_t)depth * columns);
			std::vector<T> expected((size_t)rows * columns);
			FillGemmInput(left, 4);
			FillGemmInput(right, 8);
			gemm_block_ref(left.data(), right.data(), rows, depth, columns,
				0, rows - 1, 0, depth - 1, 0, columns - 1, expected.data());
			gemm(left.data(), right.data(), rows, depth, columns, left.data());
			EXPECT(GemmResultsAgree(left.data(), expected.data(), expected.size(), depth));
		}
		{
			const int rows = 131, depth = 131, columns = 67;
			std::vector<T> left((size_t)rows * depth), right((size_t)depth * columns);
			std::vector<T> expected((size_t)rows * columns);
			FillGemmInput(left, 2);
			FillGemmInput(right, 5);
			gemm_block_ref(left.data(), right.data(), rows, depth, columns,
				0, rows - 1, 0, depth - 1, 0, columns - 1, expected.data());
			gemm(left.data(), right.data(), rows, depth, columns, right.data());
			EXPECT(GemmResultsAgree(right.data(), expected.data(), expected.size(), depth));
		}
		{
			const int size = 33;
			const size_t count = (size_t)size * size;
			std::vector<T> input(count), expected(count);
			FillGemmInput(input, 11);
			gemm_block_ref(input.data(), input.data(), size, size, size,
				0, size - 1, 0, size - 1, 0, size - 1, expected.data());
			std::vector<T> storage(count + 4, (T)-456);
			std::copy(input.begin(), input.end(), storage.begin() + 1);
			gemm(storage.data() + 1, storage.data() + 1, size, size, size, storage.data() + 2);
			EXPECT(GemmResultsAgree(storage.data() + 2, expected.data(), count, size));
			EXPECT(storage[0] == (T)-456 && storage[1] == input[0]);
			EXPECT(storage[count + 2] == (T)-456 && storage[count + 3] == (T)-456);
		}
		{
			const int size = 17;
			std::vector<T> left((size_t)size * size), right(left.size());
			FillGemmInput(left, 3);
			FillGemmInput(right, 6);
			std::vector<T> expected(left);
			gemm_block_ref(left.data(), right.data(), size, size, size, 3, 12, 2, 15, 1, 10, expected.data());
			gemm_block(left.data(), right.data(), size, size, size, 3, 12, 2, 15, 1, 10, left.data());
			EXPECT(GemmResultsAgree(left.data(), expected.data(), expected.size(), size));
		}
		{
			const int size = 19;
			const size_t count = (size_t)size * size;
			std::vector<T> storage(count * 2), right(count), expected(count);
			FillGemmInput(storage, 8);
			FillGemmInput(right, 2);
			gemm_block_ref(storage.data(), right.data(), size, size, size,
				0, size - 1, 0, size - 1, 0, size - 1, expected.data());
			// Adjacent ranges share an allocation but do not overlap.
			gemm(storage.data(), right.data(), size, size, size, storage.data() + count);
			EXPECT(GemmResultsAgree(storage.data() + count, expected.data(), count, size));
		}
	}

	template<typename T>
	void TestGemmEmptyDimensions()
	{
		T result[20];
		std::fill(result, result + 20, (T)7);
		gemm<T>(nullptr, nullptr, 0, 5, 4, result);
		gemm<T>(nullptr, nullptr, 4, 5, 0, result);
		gemm_block<T>(nullptr, nullptr, 4, 5, 5, 2, 1, 0, 4, 0, 4, result);
		gemm<T>(nullptr, nullptr, 0, 0, 0, nullptr);
		for (T value : result) EXPECT(value == (T)7);
		gemm_block<T>(nullptr, nullptr, 4, 9, 5, 1, 2, 6, 5, 2, 4, result);
		for (int row = 0; row < 4; ++row)
		{
			for (int column = 0; column < 5; ++column)
			{
				EXPECT(result[row * 5 + column] == (row >= 1 && row <= 2 && column >= 2 ? (T)0 : (T)7));
			}
		}
		gemm<T>(nullptr, nullptr, 4, 0, 5, result);
		for (T value : result) EXPECT(value == (T)0);
		TDenseMatrix<T> left(4, 0), right(0, 5);
		TDenseMatrix<T> product = left * right;
		EXPECT(product.GetRows() == 4 && product.GetCols() == 5 && product.IsZero());
		TDenseMatrix<T> empty(0, 7);
		TDenseMatrix<T> copied(empty);
		copied.LoadZero();
		copied.Assign(empty);
		EXPECT(copied.GetRows() == 0 && copied.GetCols() == 7 && copied.HoldsMemory());
		copied.SetSize(7, 0);
		copied.LoadZero();
		EXPECT(copied.GetSize() == 0 && copied.GetData() == nullptr && copied.HoldsMemory());
	}

	template<typename T>
	void TestGemmSquareQR()
	{
		const int size = 35;
		std::vector<T> input((size_t)size * size), q(input.size()), r(input.size()), reconstructed(input.size());
		FillGemmInput(input, 12);
		for (int i = 0; i < size; ++i) input[(size_t)i * size + i] += (T)40;
		EXPECT(QRDecomposition<T>()(input.data(), size, size, q.data(), r.data()));
		gemm(q.data(), r.data(), size, size, size, reconstructed.data());
		EXPECT(GemmResultsAgree(reconstructed.data(), input.data(), input.size(), size * 4));
		for (int row = 0; row < size; ++row)
		{
			for (int column = 0; column < size; ++column)
			{
				T dot = (T)0;
				for (int inner = 0; inner < size; ++inner) dot += q[(size_t)inner * size + row] * q[(size_t)inner * size + column];
				EXPECT_NEAR(dot, row == column ? (T)1 : (T)0, (T)(size * 20) * std::numeric_limits<T>::epsilon());
			}
		}
	}

	template<typename T>
	void TestGemmCachePolicy()
	{
		using namespace GemmDetail;
		CacheSizes cache;
		cache.L1 = 4096; cache.L2 = 16384; cache.LastLevelShare = 0;
		const int rows = 137, depth = 601, columns = 197;
		const int mr = KernelShape<T>::Rows, nr = KernelShape<T>::Columns;
		const Blocking blocks = SelectBlocking<T>(rows, depth, columns, mr, nr, cache);
		EXPECT(blocks.Rows > 0 && blocks.Rows < rows && blocks.Rows % mr == 0);
		EXPECT(blocks.Depth > 0 && blocks.Depth < depth && blocks.Depth % 8 == 0);
		EXPECT(blocks.Columns > 0 && blocks.Columns < columns && blocks.Columns % nr == 0);
		EXPECT((size_t)blocks.Depth * (mr + nr) * sizeof(T) + mr * nr * sizeof(T) <= cache.L1);
		EXPECT((size_t)blocks.Rows * blocks.Depth * sizeof(T) <= cache.L2 / 2);
		EXPECT((size_t)blocks.Columns * blocks.Depth * sizeof(T) <= cache.L2 / 2);
		std::vector<T> left((size_t)rows * depth), right((size_t)depth * columns), expected((size_t)rows * columns);
		FillGemmInput(left, 8); FillGemmInput(right, 13);
		gemm_block_ref(left.data(), right.data(), rows, depth, columns, 0, rows - 1, 0, depth - 1, 0, columns - 1, expected.data());
		for (int mode = 0; mode < 3; ++mode)
		{
			std::vector<T> result(expected.size() + 2, T(-999));
			std::fill(result.begin() + 1, result.end() - 1, T(0));
			PackingStatistics stats;
			MultiplyPacked(left.data(), right.data(), depth, columns, rows, columns, depth,
				result.data() + 1, &blocks, mode != 0, mode == 2, &stats);
			EXPECT(GemmResultsAgree(result.data() + 1, expected.data(), expected.size(), depth));
			EXPECT(result.front() == T(-999) && result.back() == T(-999));
			const uint64_t a = (uint64_t)rows * depth, b = (uint64_t)depth * columns;
			const uint64_t rowBlocks = (rows + blocks.Rows - 1) / blocks.Rows;
			const uint64_t columnBlocks = (columns + blocks.Columns - 1) / blocks.Columns;
			EXPECT(stats.LeftElements + stats.RightElements == (std::min)(a + b * rowBlocks, a * columnBlocks + b));
		}
		// Explicitly force both panel-reuse orders with non-multiple block boundaries.
		const Blocking orders[] = { { rows, 83, 23 }, { 17, 71, columns } };
		for (const Blocking& order : orders)
		{
			std::vector<T> result(expected.size(), T(0));
			PackingStatistics stats;
			MultiplyPacked(left.data(), right.data(), depth, columns, rows, columns, depth, result.data(), &order, true, true, &stats);
			EXPECT(GemmResultsAgree(result.data(), expected.data(), expected.size(), depth));
			EXPECT(stats.LeftElements == (uint64_t)rows * depth);
			EXPECT(stats.RightElements == (uint64_t)columns * depth);
		}
	}

	template<typename T>
	void TestGemmSmallSubblocksAndAliasing()
	{
		const int size = 37;
		std::vector<T> left(size * size), right(size * size);
		FillGemmInput(left, 4); FillGemmInput(right, 8);
		// A 7x13 by 13x8 subproduct exercises original strides and row tails.
		// Repeat with each operand as the destination to check snapshot ordering.
		for (int alias = 0; alias < 3; ++alias)
		{
			std::vector<T> a(left), b(right), output(size * size, T(-99));
			T* result = alias == 1 ? a.data() : alias == 2 ? b.data() : output.data();
			std::vector<T> expected(result, result + size * size);
			gemm_block_ref(left.data(), right.data(), size, size, size, 3, 9, 2, 14, 1, 8, expected.data());
			gemm_block(a.data(), b.data(), size, size, size, 3, 9, 2, 14, 1, 8, result);
			EXPECT(GemmResultsAgree(result, expected.data(), expected.size(), 13));
		}
		const int width = 32;
		std::vector<T> storage(width * width + 4, T(-456));
		FillGemmInput(storage, 11);
		std::vector<T> input(storage.begin() + 1, storage.begin() + 1 + width * width);
		std::vector<T> expected(width * width);
		gemm_block_ref(input.data(), input.data(), width, width, width, 0, width - 1, 0, width - 1, 0, width - 1, expected.data());
		const T first = storage.front(), last = storage.back();
		gemm(storage.data() + 1, storage.data() + 1, width, width, width, storage.data() + 2);
		EXPECT(GemmResultsAgree(storage.data() + 2, expected.data(), expected.size(), width));
		EXPECT(storage.front() == first && storage.back() == last && storage[1] == input[0]);
	}

	template<typename T>
	void TestGemmCorrectness()
	{
		const int shapes[][3] = {
			{ 1, 1, 1 }, { 4, 4, 4 }, { 8, 8, 8 }, { 16, 16, 16 }, { 24, 24, 24 },
			{ 32, 32, 32 }, { 48, 48, 48 }, { 64, 64, 64 }, { 63, 64, 64 },
			{ 64, 65, 64 }, { 65, 64, 64 }, { 64, 64, 63 }, { 31, 33, 32 },
			{ 10, 10, 10 }, { 31, 63, 65 }, { 63, 127, 127 },
			{ 64, 128, 128 }, { 65, 129, 129 }, { 67, 257, 131 }, { 5, 1031, 7 }, { 130, 17, 5 }
		};
		for (const auto& shape : shapes) TestGemmShape<T>(shape[0], shape[1], shape[2]);
		// Exercise every partial unrolled entry with full tiles and both output tails.
		for (int depth = 1; depth <= 23; ++depth) TestGemmShape<T>(9, depth, 77);
		for (int rows = 1; rows <= 7; ++rows)
		{
			for (int columns = 1; columns <= 7; ++columns) TestGemmShape<T>(rows, 35, columns);
		}
		TestGemmSubblocks<T>();
		TestGemmSmallSubblocksAndAliasing<T>();
		TestGemmAliasing<T>();
		TestGemmEmptyDimensions<T>();
		TestGemmSquareQR<T>();
		TestGemmCachePolicy<T>();
	}

	volatile double GemmBenchmarkChecksum = 0.0;

	template<typename T>
	void BenchmarkGemm(const char* typeName, int rows, int depth, int columns)
	{
		std::vector<T> left((size_t)rows * depth), right((size_t)depth * columns);
		std::vector<T> expected((size_t)rows * columns), result(expected.size());
		FillGemmInput(left, 3);
		FillGemmInput(right, 7);
		auto reference = [&]() { gemm_block_ref(left.data(), right.data(), rows, depth, columns,
			0, rows - 1, 0, depth - 1, 0, columns - 1, expected.data()); };
		auto packed = [&]() { gemm(left.data(), right.data(), rows, depth, columns, result.data()); };
		reference();
		packed();
		EXPECT(GemmResultsAgree(result.data(), expected.data(), result.size(), depth));
		double referenceTime[5];
		double packedTime[5];
		for (int sample = 0; sample < 5; ++sample)
		{
			auto begin = std::chrono::steady_clock::now();
			reference();
			referenceTime[sample] = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
			GemmBenchmarkChecksum += (double)expected[(size_t)sample % expected.size()];
			begin = std::chrono::steady_clock::now();
			packed();
			packedTime[sample] = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
			GemmBenchmarkChecksum += (double)result[(size_t)sample % result.size()];
		}
		std::sort(referenceTime, referenceTime + 5);
		std::sort(packedTime, packedTime + 5);
		printf("GEMM %s %dx%dx%d: reference %.3f ms, packed %.3f ms, speedup %.2fx (median, packing included)\n",
			typeName, rows, depth, columns, referenceTime[2], packedTime[2], referenceTime[2] / packedTime[2]);
	}
}

void TestPackedGEMM()
{
	printf("Running TestPackedGEMM\n");
	TestGemmCorrectness<float>();
	TestGemmCorrectness<double>();
	BenchmarkGemm<float>("float", 128, 128, 128);
	BenchmarkGemm<float>("float", 256, 256, 256);
	BenchmarkGemm<float>("float", 67, 257, 131);
	BenchmarkGemm<double>("double", 128, 128, 128);
}
