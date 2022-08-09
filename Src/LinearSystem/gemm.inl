#pragma once

// #define GEMM_USE_SIMD

template<typename T>
void gemm_block_slow(const T* mat1, const T* mat2, int r1, int c1, int c2, int i0, int i1, int j0, int j1, int k0, int k1, T* m)
{
	for (int i = i0; i <= i1; ++i)
	for (int k = k0; k <= k1; ++k)
	{
		T dp = (T)0;
		const T* p = mat1 + i * c1;
		for (int j = j0; j <= j1; ++j)
			dp += p[j] * mat2[j * c2 + k];
		m[i * c2 + k] = dp;
	}
}

template<typename T>
void gemv_block_slow(const T* mat, const T* vec1, int r1, int c1, int i0, int i1, int j0, int j1, T* v)
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
void gema_block_slow(const T* mat1, const T* mat2, int r, int c, int i0, int i1, int j0, int j1, T* m)
{
	for (int i = i0; i <= i1; ++i)
	for (int j = j0; j <= j1; ++j)
	{
		m[i * c + j] = mat1[i * c + j] + mat2[i * c + j];
	}
}

#ifdef GEMM_USE_SIMD



#else

template<typename T>
void gemm_block(const T* m1, const T* m2, int r1, int c1, int c2, int i0, int i1, int j0, int j1, int k0, int k1, T* m)
{
	gemm_block_slow<T>(m1, m2, r1, c1, c2, i0, i1, j0, j1, k0, k1, m);
}

template<typename T>
void gemv_block(const T* m1, const T* v1, int r1, int c1, int i0, int i1, int k0, int k1, T* v)
{
	gemv_block_slow<T>(m1, v1, r1, c1, i0, i1, k0, k1, v);
}

template<typename T>
void gema_block(const T* mat1, const T* mat2, int r, int c, int i0, int i1, int j0, int j1, T* m)
{
	gema_block_slow<T>(mat1, mat2, r, c, i0, i1, j0, j1, m);
}

#endif 

template<typename T>
void gemm(const T* mat1, const T* mat2, int r1, int c1, int c2, T* mat)
{
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