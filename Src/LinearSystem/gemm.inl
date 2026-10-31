#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "GemmKernel.h"

namespace Maths
{
namespace LinearAlgebra
{

template<typename T>
void gemm_block_ref(const T* mat1, const T* mat2, int r1, int c1, int c2, int i0, int i1, int j0, int j1, int k0, int k1, T* m)
{
	for (int i = i0; i <= i1; ++i)
	for (int k = k0; k <= k1; ++k)
	{
		T dp = (T)0;
		const T* p = mat1 + (size_t)i * c1;
		for (int j = j0; j <= j1; ++j)
			dp += p[j] * mat2[(size_t)j * c2 + k];
		m[(size_t)i * c2 + k] = dp;
	}
}

template<typename T>
void gemv_block_ref(const T* mat, const T* vec1, int r1, int c1, int i0, int i1, int j0, int j1, T* v)
{
	for (int i = i0; i <= i1; ++i)
	{
		T dp = (T)0;
		const T* p = mat + i * c1;
		for (int j = j0; j <= j1; ++j)
			dp += p[j] * vec1[j];
		v[i] = dp;
	}
}

template<typename T>
void gema_block_ref(const T* mat1, const T* mat2, int r, int c, int i0, int i1, int j0, int j1, T* m)
{
	for (int i = i0; i <= i1; ++i)
	for (int j = j0; j <= j1; ++j)
	{
		m[i * c + j] = mat1[i * c + j] + mat2[i * c + j];
	}
}

namespace GemmDetail
{
	template<typename T>
	bool ElementCount(int rows, int columns, size_t& count)
	{
		if (rows < 0 || columns < 0) return false;
		const size_t maximum = (size_t)(std::numeric_limits<std::ptrdiff_t>::max)() / sizeof(T);
		if (columns != 0 && (size_t)rows > maximum / (size_t)columns) return false;
		count = (size_t)rows * (size_t)columns;
		return true;
	}

	template<typename T>
	bool Overlaps(const T* first, size_t firstCount, const T* second, size_t secondCount)
	{
		if (firstCount == 0 || secondCount == 0) return false;
		const uintptr_t firstAddress = reinterpret_cast<uintptr_t>(first);
		const uintptr_t secondAddress = reinterpret_cast<uintptr_t>(second);
		return firstAddress <= secondAddress
			? secondAddress - firstAddress < firstCount * sizeof(T)
			: firstAddress - secondAddress < secondCount * sizeof(T);
	}
}

template<typename T>
void gemm_block(const T* m1, const T* m2, int r1, int c1, int c2, int i0, int i1, int j0, int j1, int k0, int k1, T* m)
{
	// Indices are inclusive: i selects rows, j the reduction range, k columns.
	// Empty row/column ranges are no-ops; an empty reduction overwrites with zero.
	if (r1 <= 0 || c1 < 0 || c2 <= 0 || i1 < i0 || k1 < k0) return;
	if (!m || i0 < 0 || i1 >= r1 || k0 < 0 || k1 >= c2 || j0 < 0 || j0 > c1 || j1 >= c1) return;
	const int rows = i1 - i0 + 1;
	const int columns = k1 - k0 + 1;
	const int depth = j1 >= j0 ? j1 - j0 + 1 : 0;
	size_t leftCount;
	size_t rightCount;
	size_t resultCount;
	if (!GemmDetail::ElementCount<T>(r1, c1, leftCount)
		|| !GemmDetail::ElementCount<T>(c1, c2, rightCount)
		|| !GemmDetail::ElementCount<T>(r1, c2, resultCount)) return;
	if (depth != 0 && (!m1 || !m2)) return;

	// Snapshot overlapping operands before clearing any output. Packing only the
	// current panel is insufficient when later panels overlap the result buffer.
	std::vector<T> leftCopy;
	std::vector<T> rightCopy;
	if (depth != 0 && GemmDetail::Overlaps(m1, leftCount, m, resultCount))
	{
		leftCopy.assign(m1, m1 + leftCount);
		m1 = leftCopy.data();
	}
	if (depth != 0 && GemmDetail::Overlaps(m2, rightCount, m, resultCount))
	{
		rightCopy.assign(m2, m2 + rightCount);
		m2 = rightCopy.data();
	}
	// Preserve overlap snapshots before the direct overwrite kernel.
	if (depth != 0 && GemmDetail::TryMultiplySmall(m1 + (size_t)i0 * c1 + j0, m2 + (size_t)j0 * c2 + k0,
		c1, c2, rows, columns, depth, m + (size_t)i0 * c2 + k0)) return;
	if (depth != 0 && rows <= 16 && columns <= 16 && depth <= 32)
	{
		gemm_block_ref<T>(m1, m2, r1, c1, c2, i0, i1, j0, j1, k0, k1, m);
		return;
	}
	for (int row = i0; row <= i1; ++row)
	{
		std::fill(m + (size_t)row * c2 + k0, m + (size_t)row * c2 + k1 + 1, (T)0);
	}
	if (depth == 0) return;
	GemmDetail::MultiplyPacked(m1 + (size_t)i0 * c1 + j0, m2 + (size_t)j0 * c2 + k0,
		c1, c2, rows, columns, depth, m + (size_t)i0 * c2 + k0);
}

template<typename T>
void gemv_block(const T* m1, const T* v1, int r1, int c1, int i0, int i1, int k0, int k1, T* v)
{
	gemv_block_ref<T>(m1, v1, r1, c1, i0, i1, k0, k1, v);
}

template<typename T>
void gema_block(const T* mat1, const T* mat2, int r, int c, int i0, int i1, int j0, int j1, T* m)
{
	gema_block_ref<T>(mat1, mat2, r, c, i0, i1, j0, j1, m);
}

template<typename T>
void gemm(const T* mat1, const T* mat2, int r1, int c1, int c2, T* mat)
{
	if (r1 <= 0 || c1 < 0 || c2 <= 0) return;
	gemm_block(mat1, mat2, r1, c1, c2, 0, r1 - 1, 0, c1 - 1, 0, c2 - 1, mat);
}

template<typename T>
void gemv(const T* mat1, const T* vec1, int r, int c, T* v)
{
	gemv_block(mat1, vec1, r, c, 0, r - 1, 0, c - 1, v);
}

template<typename T>
void gema(const T* mat1, const T* mat2, int r, int c, T* mat)
{
	gema_block(mat1, mat2, r, c, 0, r - 1, 0, c - 1, mat);
}

}	// namespace LinearAlgebra
}	// namespace Maths