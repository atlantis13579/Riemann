#pragma once

// Limited memory BFGS (L-BFGS)
// https://en.wikipedia.org/wiki/Limited-memory_BFGS

// The L-BFGS method solves the unconstrainted minimization problem :
// minimize F(x), x = (x1, x2, ..., xN),
// given a eval function F(x) and its gradient G(x)

// This is a C++ template port of the implementation of Limited-memory
// Broyden-Fletcher-Goldfarb-Shanno (L-BFGS) method written by Jorge Nocedal / Naoaki Okazaki
// The original C source code written by Naoaki Okazaki is available at :
// https://github.com/chokkan/liblbfgs
// The original FORTRAN source code written by Jorge Nocedal is available at :
// http://www.ece.northwestern.edu/~nocedal/lbfgs.html

#include <math.h>

#ifndef LBFGS_IEEE_FLOAT
#define LBFGS_IEEE_FLOAT    1
#endif

enum {
	LBFGS_SUCCESS = 0,
	LBFGS_CONVERGENCE = 0,
	LBFGS_STOP,
	LBFGS_ALREADY_MINIMIZED,
	LBFGSERR_UNKNOWNERROR = -1024,
	LBFGSERR_LOGICERROR,
	LBFGSERR_OUTOFMEMORY,
	LBFGSERR_CANCELED,
	LBFGSERR_INVALID_N,
	LBFGSERR_INVALID_N_SSE,
	LBFGSERR_INVALID_X_SSE,
	LBFGSERR_INVALID_EPSILON,
	LBFGSERR_INVALID_TESTPERIOD,
	LBFGSERR_INVALID_DELTA,
	LBFGSERR_INVALID_LINESEARCH,
	LBFGSERR_INVALID_MINSTEP,
	LBFGSERR_INVALID_MAXSTEP,
	LBFGSERR_INVALID_FTOL,
	LBFGSERR_INVALID_WOLFE,
	LBFGSERR_INVALID_GTOL,
	LBFGSERR_INVALID_XTOL,
	LBFGSERR_INVALID_MAXLINESEARCH,
	LBFGSERR_INVALID_ORTHANTWISE,
	LBFGSERR_INVALID_ORTHANTWISE_START,
	LBFGSERR_INVALID_ORTHANTWISE_END,
	LBFGSERR_OUTOFINTERVAL,
	LBFGSERR_INCORRECT_TMINMAX,
	LBFGSERR_ROUNDING_ERROR,
	LBFGSERR_MINIMUMSTEP,
	LBFGSERR_MAXIMUMSTEP,
	LBFGSERR_MAXIMUMLINESEARCH,
	LBFGSERR_MAXIMUMITERATION,
	LBFGSERR_WIDTHTOOSMALL,
	LBFGSERR_INVALIDPARAMETERS,
	LBFGSERR_INCREASEGRADIENT,
};

enum {
	LBFGS_LINESEARCH_DEFAULT = 0,
	LBFGS_LINESEARCH_MORETHUENTE = 0,
	LBFGS_LINESEARCH_BACKTRACKING_ARMIJO = 1,
	LBFGS_LINESEARCH_BACKTRACKING = 2,
	LBFGS_LINESEARCH_BACKTRACKING_WOLFE = 2,
	LBFGS_LINESEARCH_BACKTRACKING_STRONG_WOLFE = 3,
};

template<typename lbfgsfloatval_t>
struct lbfgs_parameter_t {
	lbfgs_parameter_t()
	{
		m = 6;
		epsilon = (lbfgsfloatval_t)1e-5;
		past = 0;
		delta = (lbfgsfloatval_t)1e-5;
		max_iterations = 0;
		linesearch = LBFGS_LINESEARCH_DEFAULT;
		max_linesearch = 40;
		min_step = (lbfgsfloatval_t)1e-20;
		max_step = (lbfgsfloatval_t)1e20;
		ftol = (lbfgsfloatval_t)1e-4;
		wolfe = (lbfgsfloatval_t)0.9;
		gtol = (lbfgsfloatval_t)0.9;
		xtol = (lbfgsfloatval_t)1.0e-16;
		orthantwise_c = (lbfgsfloatval_t)0.0;
		orthantwise_start = 0;
		orthantwise_end = -1;
	}
	int             m;
	lbfgsfloatval_t epsilon;
	int             past;
	lbfgsfloatval_t delta;
	int             max_iterations;
	int             linesearch;
	int             max_linesearch;
	lbfgsfloatval_t min_step;
	lbfgsfloatval_t max_step;
	lbfgsfloatval_t ftol;
	lbfgsfloatval_t wolfe;
	lbfgsfloatval_t gtol;
	lbfgsfloatval_t xtol;
	lbfgsfloatval_t orthantwise_c;
	int             orthantwise_start;
	int             orthantwise_end;
};

template<typename lbfgsfloatval_t>
using lbfgs_evaluate_t = lbfgsfloatval_t (*)(
	void *instance,
	const lbfgsfloatval_t *x,
	lbfgsfloatval_t *g,
	const int n,
	const lbfgsfloatval_t step
	);

template<typename lbfgsfloatval_t>
using lbfgs_progress_t = int (*)(
	void *instance,
	const lbfgsfloatval_t *x,
	const lbfgsfloatval_t *g,
	const lbfgsfloatval_t fx,
	const lbfgsfloatval_t xnorm,
	const lbfgsfloatval_t gnorm,
	const lbfgsfloatval_t step,
	int n,
	int k,
	int ls
	);

#if     defined(USE_SSE) && defined(__SSE2__)

	#include <stdlib.h>
	#ifndef __APPLE__
	#include <malloc.h>
	#endif
	#include <memory.h>

	#if     1400 <= _MSC_VER
	#include <intrin.h>
	#endif/*1400 <= _MSC_VER*/

	#if     HAVE_EMMINTRIN_H
	#include <emmintrin.h>
	#endif/*HAVE_EMMINTRIN_H*/

	inline static void* vecalloc(size_t size)
	{
	#if     defined(_WIN32)
		void *memblock = _aligned_malloc(size, 16);
	#elif   defined(__APPLE__)  /* OS X always aligns on 16-byte boundaries */
		void *memblock = malloc(size);
	#else
		void *memblock = NULL, *p = NULL;
		if (posix_memalign(&p, 16, size) == 0) {
			memblock = p;
		}
	#endif
		if (memblock != NULL) {
			memset(memblock, 0, size);
		}
		return memblock;
	}

	inline static void vecfree(void *memblock)
	{
	#ifdef	_MSC_VER
		_aligned_free(memblock);
	#else
		free(memblock);
	#endif
	}

	inline static int fsigndiff(double* x, double* y)
	{
		return ((_mm_movemask_pd(_mm_set_pd(*(x), *(y))) + 1) & 0x002);
	}

	inline static void vecset(double* x, const double c, const int n)
	{
		int i;
		__m128d XMM0 = _mm_set1_pd(c);
		for (i = 0;i < (n);i += 8) {
			_mm_store_pd((x)+i  , XMM0);
			_mm_store_pd((x)+i+2, XMM0);
			_mm_store_pd((x)+i+4, XMM0);
			_mm_store_pd((x)+i+6, XMM0);
		}
	}

	inline static void veccpy(double* y, const double* x, const int n)
	{
		int i;
		for (i = 0;i < (n);i += 8) {
			__m128d XMM0 = _mm_load_pd((x)+i  );
			__m128d XMM1 = _mm_load_pd((x)+i+2);
			__m128d XMM2 = _mm_load_pd((x)+i+4);
			__m128d XMM3 = _mm_load_pd((x)+i+6);
			_mm_store_pd((y)+i  , XMM0);
			_mm_store_pd((y)+i+2, XMM1);
			_mm_store_pd((y)+i+4, XMM2);
			_mm_store_pd((y)+i+6, XMM3);
		}
	}

	inline static void vecncpy(double* y, const double* x, const int n)
	{
		int i;
		for (i = 0;i < (n);i += 8) {
			__m128d XMM0 = _mm_setzero_pd();
			__m128d XMM1 = _mm_setzero_pd();
			__m128d XMM2 = _mm_setzero_pd();
			__m128d XMM3 = _mm_setzero_pd();
			__m128d XMM4 = _mm_load_pd((x)+i  );
			__m128d XMM5 = _mm_load_pd((x)+i+2);
			__m128d XMM6 = _mm_load_pd((x)+i+4);
			__m128d XMM7 = _mm_load_pd((x)+i+6);
			XMM0 = _mm_sub_pd(XMM0, XMM4);
			XMM1 = _mm_sub_pd(XMM1, XMM5);
			XMM2 = _mm_sub_pd(XMM2, XMM6);
			XMM3 = _mm_sub_pd(XMM3, XMM7);
			_mm_store_pd((y)+i, XMM0);
			_mm_store_pd((y)+i+2, XMM1);
			_mm_store_pd((y)+i+4, XMM2);
			_mm_store_pd((y)+i+6, XMM3);
		}
	}

	inline static void vecadd(double* y, const double* x, const double c, const int n)
	{
		int i;
		__m128d XMM7 = _mm_set1_pd(c);
		for (i = 0;i < (n);i += 4) {
			__m128d XMM0 = _mm_load_pd((x)+i  );
			__m128d XMM1 = _mm_load_pd((x)+i+2);
			__m128d XMM2 = _mm_load_pd((y)+i  );
			__m128d XMM3 = _mm_load_pd((y)+i+2);
			XMM0 = _mm_mul_pd(XMM0, XMM7);
			XMM1 = _mm_mul_pd(XMM1, XMM7);
			XMM2 = _mm_add_pd(XMM2, XMM0);
			XMM3 = _mm_add_pd(XMM3, XMM1);
			_mm_store_pd((y)+i  , XMM2);
			_mm_store_pd((y)+i+2, XMM3);
		}
	}

	inline static void vecdiff(double* z, const double* x, const double* y, const int n)
	{
		int i;
		for (i = 0;i < (n);i += 8) {
			__m128d XMM0 = _mm_load_pd((x)+i  );
			__m128d XMM1 = _mm_load_pd((x)+i+2);
			__m128d XMM2 = _mm_load_pd((x)+i+4);
			__m128d XMM3 = _mm_load_pd((x)+i+6);
			__m128d XMM4 = _mm_load_pd((y)+i  );
			__m128d XMM5 = _mm_load_pd((y)+i+2);
			__m128d XMM6 = _mm_load_pd((y)+i+4);
			__m128d XMM7 = _mm_load_pd((y)+i+6);
			XMM0 = _mm_sub_pd(XMM0, XMM4);
			XMM1 = _mm_sub_pd(XMM1, XMM5);
			XMM2 = _mm_sub_pd(XMM2, XMM6);
			XMM3 = _mm_sub_pd(XMM3, XMM7);
			_mm_store_pd((z)+i  , XMM0);
			_mm_store_pd((z)+i+2, XMM1);
			_mm_store_pd((z)+i+4, XMM2);
			_mm_store_pd((z)+i+6, XMM3);
		}
	}

	inline static void vecscale(double* y, const double c, const int n)
	{
		int i;
		__m128d XMM7 = _mm_set1_pd(c);
		for (i = 0;i < (n);i += 4) {
			__m128d XMM0 = _mm_load_pd((y)+i  );
			__m128d XMM1 = _mm_load_pd((y)+i+2);
			XMM0 = _mm_mul_pd(XMM0, XMM7);
			XMM1 = _mm_mul_pd(XMM1, XMM7);
			_mm_store_pd((y)+i  , XMM0);
			_mm_store_pd((y)+i+2, XMM1);
		}
	}

	inline static void vecmul(double* y, const double* x, const int n)
	{
		int i;
		for (i = 0;i < (n);i += 8) {
			__m128d XMM0 = _mm_load_pd((x)+i  );
			__m128d XMM1 = _mm_load_pd((x)+i+2);
			__m128d XMM2 = _mm_load_pd((x)+i+4);
			__m128d XMM3 = _mm_load_pd((x)+i+6);
			__m128d XMM4 = _mm_load_pd((y)+i  );
			__m128d XMM5 = _mm_load_pd((y)+i+2);
			__m128d XMM6 = _mm_load_pd((y)+i+4);
			__m128d XMM7 = _mm_load_pd((y)+i+6);
			XMM4 = _mm_mul_pd(XMM4, XMM0);
			XMM5 = _mm_mul_pd(XMM5, XMM1);
			XMM6 = _mm_mul_pd(XMM6, XMM2);
			XMM7 = _mm_mul_pd(XMM7, XMM3);
			_mm_store_pd((y)+i  , XMM4);
			_mm_store_pd((y)+i+2, XMM5);
			_mm_store_pd((y)+i+4, XMM6);
			_mm_store_pd((y)+i+6, XMM7);
		}
	}

	#if     3 <= __SSE__ || defined(__SSE3__)

	#define __horizontal_sum(r, rw) \
		r = _mm_hadd_ps(r, r); \
		r = _mm_hadd_ps(r, r);

	#else

	#define __horizontal_sum(r, rw) \
		rw = r; \
		r = _mm_shuffle_ps(r, rw, _MM_SHUFFLE(1, 0, 3, 2)); \
		r = _mm_add_ps(r, rw); \
		rw = r; \
		r = _mm_shuffle_ps(r, rw, _MM_SHUFFLE(2, 3, 0, 1)); \
		r = _mm_add_ps(r, rw);

	#endif

	inline static void vecdot(double* s, const double* x, const double* y, const int n)
	{
		int i;
		__m128d XMM0 = _mm_setzero_pd();
		__m128d XMM1 = _mm_setzero_pd();
		__m128d XMM2, XMM3, XMM4, XMM5;
		for (i = 0;i < (n);i += 4) {
			XMM2 = _mm_load_pd((x)+i  );
			XMM3 = _mm_load_pd((x)+i+2);
			XMM4 = _mm_load_pd((y)+i  );
			XMM5 = _mm_load_pd((y)+i+2);
			XMM2 = _mm_mul_pd(XMM2, XMM4);
			XMM3 = _mm_mul_pd(XMM3, XMM5);
			XMM0 = _mm_add_pd(XMM0, XMM2);
			XMM1 = _mm_add_pd(XMM1, XMM3);
		}
		XMM0 = _mm_add_pd(XMM0, XMM1);
		XMM1 = _mm_shuffle_pd(XMM0, XMM0, _MM_SHUFFLE2(1, 1));
		XMM0 = _mm_add_pd(XMM0, XMM1);
		_mm_store_sd((s), XMM0);
	}

	inline static void vec2norm(double* s, const double* x, const int n)
	{
		int i;
		__m128d XMM0 = _mm_setzero_pd();
		__m128d XMM1 = _mm_setzero_pd();
		__m128d XMM2, XMM3, XMM4, XMM5;
		for (i = 0;i < (n);i += 4) {
			XMM2 = _mm_load_pd((x)+i  );
			XMM3 = _mm_load_pd((x)+i+2);
			XMM4 = XMM2;
			XMM5 = XMM3;
			XMM2 = _mm_mul_pd(XMM2, XMM4);
			XMM3 = _mm_mul_pd(XMM3, XMM5);
			XMM0 = _mm_add_pd(XMM0, XMM2);
			XMM1 = _mm_add_pd(XMM1, XMM3);
		}
		XMM0 = _mm_add_pd(XMM0, XMM1);
		XMM1 = _mm_shuffle_pd(XMM0, XMM0, _MM_SHUFFLE2(1, 1));
		XMM0 = _mm_add_pd(XMM0, XMM1);
		XMM0 = _mm_sqrt_pd(XMM0);
		_mm_store_sd((s), XMM0);
	}

	inline static void vec2norminv(double* s, const double* x, const int n)
	{
		int i;
		__m128d XMM0 = _mm_setzero_pd();
		__m128d XMM1 = _mm_setzero_pd();
		__m128d XMM2, XMM3, XMM4, XMM5;
		for (i = 0;i < (n);i += 4) {
			XMM2 = _mm_load_pd((x)+i  );
			XMM3 = _mm_load_pd((x)+i+2);
			XMM4 = XMM2;
			XMM5 = XMM3;
			XMM2 = _mm_mul_pd(XMM2, XMM4);
			XMM3 = _mm_mul_pd(XMM3, XMM5);
			XMM0 = _mm_add_pd(XMM0, XMM2);
			XMM1 = _mm_add_pd(XMM1, XMM3);
		}
		XMM2 = _mm_set1_pd(1.0);
		XMM0 = _mm_add_pd(XMM0, XMM1);
		XMM1 = _mm_shuffle_pd(XMM0, XMM0, _MM_SHUFFLE2(1, 1));
		XMM0 = _mm_add_pd(XMM0, XMM1);
		XMM0 = _mm_sqrt_pd(XMM0);
		XMM2 = _mm_div_pd(XMM2, XMM0);
		_mm_store_sd((s), XMM2);
	}

	inline static int fsigndiff(float* x, float* y)
	{
		#if    LBFGS_IEEE_FLOAT
		return (((*(uint32_t*)(x)) ^ (*(uint32_t*)(y))) & 0x80000000U);
		#else
		return (*(x) * (*(y) / fabs(*(y))) < 0.0f);
		#endif
	}

	inline static void vecset(float* x, const float c, const int n)
	{
		int i;
		__m128 XMM0 = _mm_set_ps1(c);
		for (i = 0;i < (n);i += 16) {
			_mm_store_ps((x)+i   , XMM0);
			_mm_store_ps((x)+i+ 4, XMM0);
			_mm_store_ps((x)+i+ 8, XMM0);
			_mm_store_ps((x)+i+12, XMM0);
		}
	}

	inline static void veccpy(float* y, const float* x, const int n)
	{
		int i;
		for (i = 0;i < (n);i += 16) {
			__m128 XMM0 = _mm_load_ps((x)+i   );
			__m128 XMM1 = _mm_load_ps((x)+i+ 4);
			__m128 XMM2 = _mm_load_ps((x)+i+ 8);
			__m128 XMM3 = _mm_load_ps((x)+i+12);
			_mm_store_ps((y)+i   , XMM0);
			_mm_store_ps((y)+i+ 4, XMM1);
			_mm_store_ps((y)+i+ 8, XMM2);
			_mm_store_ps((y)+i+12, XMM3);
		}
	}

	inline static void vecncpy(float* y, const float* x, const int n)
	{
		int i;
		const uint32_t mask = 0x80000000;
		__m128 XMM4 = _mm_load_ps1((float*)&mask);
		for (i = 0;i < (n);i += 16) {
			__m128 XMM0 = _mm_load_ps((x)+i   );
			__m128 XMM1 = _mm_load_ps((x)+i+ 4);
			__m128 XMM2 = _mm_load_ps((x)+i+ 8);
			__m128 XMM3 = _mm_load_ps((x)+i+12);
			XMM0 = _mm_xor_ps(XMM0, XMM4);
			XMM1 = _mm_xor_ps(XMM1, XMM4);
			XMM2 = _mm_xor_ps(XMM2, XMM4);
			XMM3 = _mm_xor_ps(XMM3, XMM4);
			_mm_store_ps((y)+i   , XMM0);
			_mm_store_ps((y)+i+ 4, XMM1);
			_mm_store_ps((y)+i+ 8, XMM2);
			_mm_store_ps((y)+i+12, XMM3);
		}
	}

	inline static void vecadd(float* y, const float* x, const float c, const int n)
	{
		int i;
		__m128 XMM7 = _mm_set_ps1(c);
		for (i = 0;i < (n);i += 8) {
			__m128 XMM0 = _mm_load_ps((x)+i  );
			__m128 XMM1 = _mm_load_ps((x)+i+4);
			__m128 XMM2 = _mm_load_ps((y)+i  );
			__m128 XMM3 = _mm_load_ps((y)+i+4);
			XMM0 = _mm_mul_ps(XMM0, XMM7);
			XMM1 = _mm_mul_ps(XMM1, XMM7);
			XMM2 = _mm_add_ps(XMM2, XMM0);
			XMM3 = _mm_add_ps(XMM3, XMM1);
			_mm_store_ps((y)+i  , XMM2);
			_mm_store_ps((y)+i+4, XMM3);
		}
	}

	inline static void vecdiff(float* z, const float* x, const float* y, const int n)
	{
		int i;
		for (i = 0;i < (n);i += 16) {
			__m128 XMM0 = _mm_load_ps((x)+i   );
			__m128 XMM1 = _mm_load_ps((x)+i+ 4);
			__m128 XMM2 = _mm_load_ps((x)+i+ 8);
			__m128 XMM3 = _mm_load_ps((x)+i+12);
			__m128 XMM4 = _mm_load_ps((y)+i   );
			__m128 XMM5 = _mm_load_ps((y)+i+ 4);
			__m128 XMM6 = _mm_load_ps((y)+i+ 8);
			__m128 XMM7 = _mm_load_ps((y)+i+12);
			XMM0 = _mm_sub_ps(XMM0, XMM4);
			XMM1 = _mm_sub_ps(XMM1, XMM5);
			XMM2 = _mm_sub_ps(XMM2, XMM6);
			XMM3 = _mm_sub_ps(XMM3, XMM7);
			_mm_store_ps((z)+i   , XMM0);
			_mm_store_ps((z)+i+ 4, XMM1);
			_mm_store_ps((z)+i+ 8, XMM2);
			_mm_store_ps((z)+i+12, XMM3);
		}
	}

	inline static void vecscale(float* y, const float c, const int n)
	{
		int i;
		__m128 XMM7 = _mm_set_ps1(c);
		for (i = 0;i < (n);i += 8) {
			__m128 XMM0 = _mm_load_ps((y)+i  );
			__m128 XMM1 = _mm_load_ps((y)+i+4);
			XMM0 = _mm_mul_ps(XMM0, XMM7);
			XMM1 = _mm_mul_ps(XMM1, XMM7);
			_mm_store_ps((y)+i  , XMM0);
			_mm_store_ps((y)+i+4, XMM1);
		}
	}

	inline static void vecmul(float* y, const float* x, const int n)
	{
		int i;
		for (i = 0;i < (n);i += 16) {
			__m128 XMM0 = _mm_load_ps((x)+i   );
			__m128 XMM1 = _mm_load_ps((x)+i+ 4);
			__m128 XMM2 = _mm_load_ps((x)+i+ 8);
			__m128 XMM3 = _mm_load_ps((x)+i+12);
			__m128 XMM4 = _mm_load_ps((y)+i   );
			__m128 XMM5 = _mm_load_ps((y)+i+ 4);
			__m128 XMM6 = _mm_load_ps((y)+i+ 8);
			__m128 XMM7 = _mm_load_ps((y)+i+12);
			XMM4 = _mm_mul_ps(XMM4, XMM0);
			XMM5 = _mm_mul_ps(XMM5, XMM1);
			XMM6 = _mm_mul_ps(XMM6, XMM2);
			XMM7 = _mm_mul_ps(XMM7, XMM3);
			_mm_store_ps((y)+i   , XMM4);
			_mm_store_ps((y)+i+ 4, XMM5);
			_mm_store_ps((y)+i+ 8, XMM6);
			_mm_store_ps((y)+i+12, XMM7);
		}
	}

	inline static void vecdot(float* s, const float* x, const float* y, const int n)
	{
		int i;
		__m128 XMM0 = _mm_setzero_ps();
		__m128 XMM1 = _mm_setzero_ps();
		__m128 XMM2, XMM3, XMM4, XMM5;
		for (i = 0;i < (n);i += 8) {
			XMM2 = _mm_load_ps((x)+i  );
			XMM3 = _mm_load_ps((x)+i+4);
			XMM4 = _mm_load_ps((y)+i  );
			XMM5 = _mm_load_ps((y)+i+4);
			XMM2 = _mm_mul_ps(XMM2, XMM4);
			XMM3 = _mm_mul_ps(XMM3, XMM5);
			XMM0 = _mm_add_ps(XMM0, XMM2);
			XMM1 = _mm_add_ps(XMM1, XMM3);
		}
		XMM0 = _mm_add_ps(XMM0, XMM1);
		__horizontal_sum(XMM0, XMM1);
		_mm_store_ss((s), XMM0);
	}

	inline static void vec2norm(float* s, const float* x, const int n)
	{
		int i;
		__m128 XMM0 = _mm_setzero_ps();
		__m128 XMM1 = _mm_setzero_ps();
		__m128 XMM2, XMM3;
		for (i = 0;i < (n);i += 8) {
			XMM2 = _mm_load_ps((x)+i  );
			XMM3 = _mm_load_ps((x)+i+4);
			XMM2 = _mm_mul_ps(XMM2, XMM2);
			XMM3 = _mm_mul_ps(XMM3, XMM3);
			XMM0 = _mm_add_ps(XMM0, XMM2);
			XMM1 = _mm_add_ps(XMM1, XMM3);
		}
		XMM0 = _mm_add_ps(XMM0, XMM1);
		__horizontal_sum(XMM0, XMM1);
		XMM2 = XMM0;
		XMM1 = _mm_rsqrt_ss(XMM0);
		XMM3 = XMM1;
		XMM1 = _mm_mul_ss(XMM1, XMM1);
		XMM1 = _mm_mul_ss(XMM1, XMM3);
		XMM1 = _mm_mul_ss(XMM1, XMM0);
		XMM1 = _mm_mul_ss(XMM1, _mm_set_ss(-0.5f));
		XMM3 = _mm_mul_ss(XMM3, _mm_set_ss(1.5f));
		XMM3 = _mm_add_ss(XMM3, XMM1);
		XMM3 = _mm_mul_ss(XMM3, XMM2);
		_mm_store_ss((s), XMM3);
	}

	inline static void vec2norminv(float* s, const float* x, const int n)
	{
		int i;
		__m128 XMM0 = _mm_setzero_ps();
		__m128 XMM1 = _mm_setzero_ps();
		__m128 XMM2, XMM3;
		for (i = 0;i < (n);i += 16) {
			XMM2 = _mm_load_ps((x)+i  );
			XMM3 = _mm_load_ps((x)+i+4);
			XMM2 = _mm_mul_ps(XMM2, XMM2);
			XMM3 = _mm_mul_ps(XMM3, XMM3);
			XMM0 = _mm_add_ps(XMM0, XMM2);
			XMM1 = _mm_add_ps(XMM1, XMM3);
		}
		XMM0 = _mm_add_ps(XMM0, XMM1);
		__horizontal_sum(XMM0, XMM1);
		XMM2 = XMM0;
		XMM1 = _mm_rsqrt_ss(XMM0);
		XMM3 = XMM1;
		XMM1 = _mm_mul_ss(XMM1, XMM1);
		XMM1 = _mm_mul_ss(XMM1, XMM3);
		XMM1 = _mm_mul_ss(XMM1, XMM0);
		XMM1 = _mm_mul_ss(XMM1, _mm_set_ss(-0.5f));
		XMM3 = _mm_mul_ss(XMM3, _mm_set_ss(1.5f));
		XMM3 = _mm_add_ss(XMM3, XMM1);
		_mm_store_ss((s), XMM3);
	}

#else

	#include <stdlib.h>
	#include <memory.h>

	inline static int fsigndiff(float* x, float* y)
	{
	#if    LBFGS_IEEE_FLOAT
		return (((*(uint32_t*)(x)) ^ (*(uint32_t*)(y))) & 0x80000000U);
	#else
		return (*(x) * (*(y) / fabs(*(y))) < 0.0f);
	#endif
	}

	inline static int fsigndiff(double* x, double* y)
	{
		return (*(x) * (*(y) / fabs(*(y))) < 0.0f);
	}

	inline static void* vecalloc(size_t size)
	{
		void *memblock = malloc(size);
		if (memblock) {
			memset(memblock, 0, size);
		}
		return memblock;
	}

	inline static void vecfree(void *memblock)
	{
		free(memblock);
	}

	template<typename lbfgsfloatval_t>
	inline static void vecset(lbfgsfloatval_t *x, const lbfgsfloatval_t c, const int n)
	{
		int i;
		
		for (i = 0;i < n;++i) {
			x[i] = c;
		}
	}

	template<typename lbfgsfloatval_t>
	inline static void veccpy(lbfgsfloatval_t *y, const lbfgsfloatval_t *x, const int n)
	{
		int i;

		for (i = 0;i < n;++i) {
			y[i] = x[i];
		}
	}

	template<typename lbfgsfloatval_t>
	inline static void vecncpy(lbfgsfloatval_t *y, const lbfgsfloatval_t *x, const int n)
	{
		int i;

		for (i = 0;i < n;++i) {
			y[i] = -x[i];
		}
	}

	template<typename lbfgsfloatval_t>
	inline static void vecadd(lbfgsfloatval_t *y, const lbfgsfloatval_t *x, const lbfgsfloatval_t c, const int n)
	{
		int i;

		for (i = 0;i < n;++i) {
			y[i] += c * x[i];
		}
	}
	
	template<typename lbfgsfloatval_t>
	inline static void vecdiff(lbfgsfloatval_t *z, const lbfgsfloatval_t *x, const lbfgsfloatval_t *y, const int n)
	{
		int i;

		for (i = 0;i < n;++i) {
			z[i] = x[i] - y[i];
		}
	}

	template<typename lbfgsfloatval_t>
	inline static void vecscale(lbfgsfloatval_t *y, const lbfgsfloatval_t c, const int n)
	{
		int i;

		for (i = 0;i < n;++i) {
			y[i] *= c;
		}
	}

	template<typename lbfgsfloatval_t>
	inline static void vecmul(lbfgsfloatval_t *y, const lbfgsfloatval_t *x, const int n)
	{
		int i;

		for (i = 0;i < n;++i) {
			y[i] *= x[i];
		}
	}

	template<typename lbfgsfloatval_t>
	inline static void vecdot(lbfgsfloatval_t* s, const lbfgsfloatval_t *x, const lbfgsfloatval_t *y, const int n)
	{
		int i;
		*s = 0.;
		for (i = 0;i < n;++i) {
			*s += x[i] * y[i];
		}
	}

	template<typename lbfgsfloatval_t>
	inline static void vec2norm(lbfgsfloatval_t* s, const lbfgsfloatval_t *x, const int n)
	{
		vecdot(s, x, x, n);
		*s = (lbfgsfloatval_t)sqrt(*s);
	}
	
	template<typename lbfgsfloatval_t>
	inline static void vec2norminv(lbfgsfloatval_t* s, const lbfgsfloatval_t *x, const int n)
	{
		vec2norm(s, x, n);
		*s = (lbfgsfloatval_t)(1.0 / *s);
	}

#endif

#define min2(a, b)      ((a) <= (b) ? (a) : (b))
#define max2(a, b)      ((a) >= (b) ? (a) : (b))
#define max3(a, b, c)   max2(max2((a), (b)), (c));

template<typename lbfgsfloatval_t>
struct callback_data_t {
	int n;
	void *instance;
	lbfgs_evaluate_t<lbfgsfloatval_t> proc_evaluate;
	lbfgs_progress_t<lbfgsfloatval_t> proc_progress;
};

template<typename lbfgsfloatval_t>
struct iteration_data_t {
	lbfgsfloatval_t alpha;
	lbfgsfloatval_t *s;
	lbfgsfloatval_t *y;
	lbfgsfloatval_t ys;
};

template<typename lbfgsfloatval_t>
using line_search_proc = int (*)(
	int n,
	lbfgsfloatval_t *x,
	lbfgsfloatval_t *f,
	lbfgsfloatval_t *g,
	lbfgsfloatval_t *s,
	lbfgsfloatval_t *stp,
	const lbfgsfloatval_t* xp,
	const lbfgsfloatval_t* gp,
	lbfgsfloatval_t *wa,
	callback_data_t<lbfgsfloatval_t> *cd,
	const lbfgs_parameter_t<lbfgsfloatval_t> *param
	);

#define USES_MINIMIZER \
	lbfgsfloatval_t a, d, gamma, theta, p, q, r, s;

#define CUBIC_MINIMIZER(cm, u, fu, du, v, fv, dv) \
	d = (v) - (u); \
	theta = ((fu) - (fv)) * 3 / d + (du) + (dv); \
	p = abs(theta); \
	q = abs(du); \
	r = abs(dv); \
	s = max3(p, q, r); \
	a = theta / s; \
	gamma = s * (lbfgsfloatval_t)sqrt(a * a - ((du) / s) * ((dv) / s)); \
	if ((v) < (u)) gamma = -gamma; \
	p = gamma - (du) + theta; \
	q = gamma - (du) + gamma + (dv); \
	r = p / q; \
	(cm) = (u) + r * d;

#define CUBIC_MINIMIZER2(cm, u, fu, du, v, fv, dv, xmin, xmax) \
	d = (v) - (u); \
	theta = ((fu) - (fv)) * 3 / d + (du) + (dv); \
	p = abs(theta); \
	q = abs(du); \
	r = abs(dv); \
	s = max3(p, q, r); \
	a = theta / s; \
	gamma = s * (lbfgsfloatval_t)sqrt(max2(0, a * a - ((du) / s) * ((dv) / s))); \
	if ((u) < (v)) gamma = -gamma; \
	p = gamma - (dv) + theta; \
	q = gamma - (dv) + gamma + (du); \
	r = p / q; \
	if (r < 0. && gamma != 0.) { \
		(cm) = (v) - r * d; \
	} else if (d > 0) { \
		(cm) = (xmax); \
	} else { \
		(cm) = (xmin); \
	}

#define QUAD_MINIMIZER(qm, u, fu, du, v, fv) \
	a = (v) - (u); \
	(qm) = (u) + (du) / (((fu) - (fv)) / a + (du)) / 2 * a;

#define QUAD_MINIMIZER2(qm, u, du, v, dv) \
	a = (u) - (v); \
	(qm) = (v) + (dv) / ((dv) - (du)) * a;

template<typename lbfgsfloatval_t>
int update_trial_interval(
	lbfgsfloatval_t* x,
	lbfgsfloatval_t* fx,
	lbfgsfloatval_t* dx,
	lbfgsfloatval_t* y,
	lbfgsfloatval_t* fy,
	lbfgsfloatval_t* dy,
	lbfgsfloatval_t* t,
	lbfgsfloatval_t* ft,
	lbfgsfloatval_t* dt,
	const lbfgsfloatval_t tmin,
	const lbfgsfloatval_t tmax,
	int* brackt
)
{
	int bound;
	int dsign = fsigndiff(dt, dx);
	lbfgsfloatval_t mc;
	lbfgsfloatval_t mq;
	lbfgsfloatval_t newt;
	USES_MINIMIZER;

	if (*brackt) {
		if (*t <= min2(*x, *y) || max2(*x, *y) <= *t) {
			return LBFGSERR_OUTOFINTERVAL;
		}
		if (0. <= *dx * (*t - *x)) {
			return LBFGSERR_INCREASEGRADIENT;
		}
		if (tmax < tmin) {
			return LBFGSERR_INCORRECT_TMINMAX;
		}
	}

	if (*fx < *ft) {
		*brackt = 1;
		bound = 1;
		CUBIC_MINIMIZER(mc, *x, *fx, *dx, *t, *ft, *dt);
		QUAD_MINIMIZER(mq, *x, *fx, *dx, *t, *ft);
		if (fabs(mc - *x) < fabs(mq - *x)) {
			newt = mc;
		}
		else {
			newt = mc + (lbfgsfloatval_t)0.5 * (mq - mc);
		}
	}
	else if (dsign) {
		*brackt = 1;
		bound = 0;
		CUBIC_MINIMIZER(mc, *x, *fx, *dx, *t, *ft, *dt);
		QUAD_MINIMIZER2(mq, *x, *dx, *t, *dt);
		if (fabs(mc - *t) > fabs(mq - *t)) {
			newt = mc;
		}
		else {
			newt = mq;
		}
	}
	else if (fabs(*dt) < fabs(*dx)) {
		bound = 1;
		CUBIC_MINIMIZER2(mc, *x, *fx, *dx, *t, *ft, *dt, tmin, tmax);
		QUAD_MINIMIZER2(mq, *x, *dx, *t, *dt);
		if (*brackt) {
			if (fabs(*t - mc) < fabs(*t - mq)) {
				newt = mc;
			}
			else {
				newt = mq;
			}
		}
		else {
			if (fabs(*t - mc) > fabs(*t - mq)) {
				newt = mc;
			}
			else {
				newt = mq;
			}
		}
	}
	else {
		bound = 0;
		if (*brackt) {
			CUBIC_MINIMIZER(newt, *t, *ft, *dt, *y, *fy, *dy);
		}
		else if (*x < *t) {
			newt = tmax;
		}
		else {
			newt = tmin;
		}
	}

	if (*fx < *ft) {
		*y = *t;
		*fy = *ft;
		*dy = *dt;
	}
	else {
		if (dsign) {
			*y = *x;
			*fy = *fx;
			*dy = *dx;
		}
		*x = *t;
		*fx = *ft;
		*dx = *dt;
	}

	if (tmax < newt) newt = tmax;
	if (newt < tmin) newt = tmin;

	if (*brackt && bound) {
		mq = *x + (lbfgsfloatval_t)0.66 * (*y - *x);
		if (*x < *y) {
			if (mq < newt) newt = mq;
		}
		else {
			if (newt < mq) newt = mq;
		}
	}

	*t = newt;
	return 0;
}

template<typename lbfgsfloatval_t>
int line_search_morethuente(
	int n,
	lbfgsfloatval_t* x,
	lbfgsfloatval_t* f,
	lbfgsfloatval_t* g,
	lbfgsfloatval_t* s,
	lbfgsfloatval_t* stp,
	const lbfgsfloatval_t* xp,
	const lbfgsfloatval_t* gp,
	lbfgsfloatval_t* wa,
	callback_data_t<lbfgsfloatval_t>* cd,
	const lbfgs_parameter_t<lbfgsfloatval_t>* param
)
{
	int count = 0;
	int brackt, stage1, uinfo = 0;
	lbfgsfloatval_t dg;
	lbfgsfloatval_t stx, fx, dgx;
	lbfgsfloatval_t sty, fy, dgy;
	lbfgsfloatval_t fxm, dgxm, fym, dgym, fm, dgm;
	lbfgsfloatval_t finit, ftest1, dginit, dgtest;
	lbfgsfloatval_t width, prev_width;
	lbfgsfloatval_t stmin, stmax;

	if (*stp <= 0.) {
		return LBFGSERR_INVALIDPARAMETERS;
	}

	vecdot(&dginit, g, s, n);

	if (0 < dginit) {
		return LBFGSERR_INCREASEGRADIENT;
	}

	brackt = 0;
	stage1 = 1;
	finit = *f;
	dgtest = param->ftol * dginit;
	width = param->max_step - param->min_step;
	prev_width = (lbfgsfloatval_t)2.0 * width;

	stx = sty = 0.;
	fx = fy = finit;
	dgx = dgy = dginit;

	for (;;) {
		if (brackt) {
			stmin = min2(stx, sty);
			stmax = max2(stx, sty);
		}
		else {
			stmin = stx;
			stmax = *stp + (lbfgsfloatval_t)4.0 * (*stp - stx);
		}

		if (*stp < param->min_step) *stp = param->min_step;
		if (param->max_step < *stp) *stp = param->max_step;

		if ((brackt && ((*stp <= stmin || stmax <= *stp) || param->max_linesearch <= count + 1 || uinfo != 0)) || (brackt && (stmax - stmin <= param->xtol * stmax))) {
			*stp = stx;
		}

		veccpy(x, xp, n);
		vecadd(x, s, *stp, n);

		*f = cd->proc_evaluate(cd->instance, x, g, cd->n, *stp);
		vecdot(&dg, g, s, n);

		ftest1 = finit + *stp * dgtest;
		++count;

		if (brackt && ((*stp <= stmin || stmax <= *stp) || uinfo != 0)) {
			return LBFGSERR_ROUNDING_ERROR;
		}
		if (*stp == param->max_step && *f <= ftest1 && dg <= dgtest) {
			return LBFGSERR_MAXIMUMSTEP;
		}
		if (*stp == param->min_step && (ftest1 < *f || dgtest <= dg)) {
			return LBFGSERR_MINIMUMSTEP;
		}
		if (brackt && (stmax - stmin) <= param->xtol * stmax) {
			return LBFGSERR_WIDTHTOOSMALL;
		}
		if (param->max_linesearch <= count) {
			return LBFGSERR_MAXIMUMLINESEARCH;
		}
		if (*f <= ftest1 && fabs(dg) <= param->gtol * (-dginit)) {
			return count;
		}

		if (stage1 && *f <= ftest1 && min2(param->ftol, param->gtol) * dginit <= dg) {
			stage1 = 0;
		}

		if (stage1 && ftest1 < *f && *f <= fx) {
			fm = *f - *stp * dgtest;
			fxm = fx - stx * dgtest;
			fym = fy - sty * dgtest;
			dgm = dg - dgtest;
			dgxm = dgx - dgtest;
			dgym = dgy - dgtest;

			uinfo = update_trial_interval(
				&stx, &fxm, &dgxm,
				&sty, &fym, &dgym,
				stp, &fm, &dgm,
				stmin, stmax, &brackt
			);

			fx = fxm + stx * dgtest;
			fy = fym + sty * dgtest;
			dgx = dgxm + dgtest;
			dgy = dgym + dgtest;
		}
		else {
			uinfo = update_trial_interval(
				&stx, &fx, &dgx,
				&sty, &fy, &dgy,
				stp, f, &dg,
				stmin, stmax, &brackt
			);
		}

		if (brackt) {
			if ((lbfgsfloatval_t)0.66 * prev_width <= abs(sty - stx)) {
				*stp = stx + (lbfgsfloatval_t)0.5 * (sty - stx);
			}
			prev_width = width;
			width = abs(sty - stx);
		}
	}

	return LBFGSERR_LOGICERROR;
}

template<typename lbfgsfloatval_t>
lbfgsfloatval_t owlqn_x1norm(
	const lbfgsfloatval_t* x,
	const int start,
	const int n
)
{
	int i;
	lbfgsfloatval_t norm = 0.;

	for (i = start; i < n; ++i) {
		norm += abs(x[i]);
	}

	return norm;
}

template<typename lbfgsfloatval_t>
void owlqn_pseudo_gradient(
	lbfgsfloatval_t* pg,
	const lbfgsfloatval_t* x,
	const lbfgsfloatval_t* g,
	const int n,
	const lbfgsfloatval_t c,
	const int start,
	const int end
)
{
	int i;

	for (i = 0; i < start; ++i) {
		pg[i] = g[i];
	}

	for (i = start; i < end; ++i) {
		if (x[i] < 0.) {
			pg[i] = g[i] - c;
		}
		else if (0. < x[i]) {
			pg[i] = g[i] + c;
		}
		else {
			if (g[i] < -c) {
				pg[i] = g[i] + c;
			}
			else if (c < g[i]) {
				pg[i] = g[i] - c;
			}
			else {
				pg[i] = 0.;
			}
}
	}

	for (i = end; i < n; ++i) {
		pg[i] = g[i];
	}
}

template<typename lbfgsfloatval_t>
void owlqn_project(
	lbfgsfloatval_t* d,
	const lbfgsfloatval_t* sign,
	const int start,
	const int end
)
{
	int i;

	for (i = start; i < end; ++i) {
		if (d[i] * sign[i] <= 0) {
			d[i] = 0;
		}
	}
}

template<typename lbfgsfloatval_t>
int line_search_backtracking(
	int n,
	lbfgsfloatval_t* x,
	lbfgsfloatval_t* f,
	lbfgsfloatval_t* g,
	lbfgsfloatval_t* s,
	lbfgsfloatval_t* stp,
	const lbfgsfloatval_t* xp,
	const lbfgsfloatval_t* gp,
	lbfgsfloatval_t* wp,
	callback_data_t<lbfgsfloatval_t>* cd,
	const lbfgs_parameter_t<lbfgsfloatval_t>* param
)
{
	int count = 0;
	lbfgsfloatval_t width, dg;
	lbfgsfloatval_t finit, dginit = 0., dgtest;
	const lbfgsfloatval_t dec = (lbfgsfloatval_t)0.5, inc = (lbfgsfloatval_t)2.1;

	if (*stp <= 0.) {
		return LBFGSERR_INVALIDPARAMETERS;
	}

	vecdot(&dginit, g, s, n);
	if (0 < dginit) {
		return LBFGSERR_INCREASEGRADIENT;
	}

	finit = *f;
	dgtest = param->ftol * dginit;

	for (;;) {
		veccpy(x, xp, n);
		vecadd(x, s, *stp, n);

		*f = cd->proc_evaluate(cd->instance, x, g, cd->n, *stp);

		++count;

		if (*f > finit + *stp * dgtest) {
			width = dec;
		}
		else {
			if (param->linesearch == LBFGS_LINESEARCH_BACKTRACKING_ARMIJO) {
				return count;
			}

			vecdot(&dg, g, s, n);
			if (dg < param->wolfe * dginit) {
				width = inc;
			}
			else {
				if (param->linesearch == LBFGS_LINESEARCH_BACKTRACKING_WOLFE) {
					return count;
				}

				if (dg > -param->wolfe * dginit) {
					width = dec;
				}
				else {
					return count;
				}
			}
		}

		if (*stp < param->min_step) {
			return LBFGSERR_MINIMUMSTEP;
		}
		if (*stp > param->max_step) {
			return LBFGSERR_MAXIMUMSTEP;
		}
		if (param->max_linesearch <= count) {
			return LBFGSERR_MAXIMUMLINESEARCH;
		}

		(*stp) *= width;
	}
}

template<typename lbfgsfloatval_t>
int line_search_backtracking_owlqn(
	int n,
	lbfgsfloatval_t* x,
	lbfgsfloatval_t* f,
	lbfgsfloatval_t* g,
	lbfgsfloatval_t* s,
	lbfgsfloatval_t* stp,
	const lbfgsfloatval_t* xp,
	const lbfgsfloatval_t* gp,
	lbfgsfloatval_t* wp,
	callback_data_t<lbfgsfloatval_t>* cd,
	const lbfgs_parameter_t<lbfgsfloatval_t>* param
)
{
	int i, count = 0;
	lbfgsfloatval_t width = 0.5, norm = 0.;
	lbfgsfloatval_t finit = *f, dgtest;

	if (*stp <= 0.) {
		return LBFGSERR_INVALIDPARAMETERS;
	}

	for (i = 0; i < n; ++i) {
		wp[i] = (xp[i] == 0.) ? -gp[i] : xp[i];
	}

	for (;;) {
		veccpy(x, xp, n);
		vecadd(x, s, *stp, n);

		owlqn_project(x, wp, param->orthantwise_start, param->orthantwise_end);

		*f = cd->proc_evaluate(cd->instance, x, g, cd->n, *stp);

		norm = owlqn_x1norm(x, param->orthantwise_start, param->orthantwise_end);
		*f += norm * param->orthantwise_c;

		++count;

		dgtest = 0.;
		for (i = 0; i < n; ++i) {
			dgtest += (x[i] - xp[i]) * gp[i];
		}

		if (*f <= finit + param->ftol * dgtest) {
			return count;
		}

		if (*stp < param->min_step) {
			return LBFGSERR_MINIMUMSTEP;
		}
		if (*stp > param->max_step) {
			return LBFGSERR_MAXIMUMSTEP;
		}
		if (param->max_linesearch <= count) {
			return LBFGSERR_MAXIMUMLINESEARCH;
		}

		(*stp) *= width;
	}
}


#if     defined(USE_SSE) && (defined(__SSE__) || defined(__SSE2__))
static int round_out_variables(int n)
{
	n += 7;
	n /= 8;
	n *= 8;
	return n;
}
#endif

template<typename lbfgsfloatval_t>
inline lbfgsfloatval_t* lbfgs_malloc(int n)
{
#if     defined(USE_SSE) && (defined(__SSE__) || defined(__SSE2__))
	n = round_out_variables(n);
#endif
	return (lbfgsfloatval_t*)vecalloc(sizeof(lbfgsfloatval_t) * n);
}

template<typename lbfgsfloatval_t>
inline void lbfgs_free(lbfgsfloatval_t *x)
{
	vecfree(x);
}

template<typename lbfgsfloatval_t>
inline int lbfgs(
	int n,
	lbfgsfloatval_t *x,
	lbfgsfloatval_t *ptr_fx,
	lbfgs_evaluate_t<lbfgsfloatval_t> proc_evaluate,
	lbfgs_progress_t<lbfgsfloatval_t> proc_progress,
	void *instance,
	lbfgs_parameter_t<lbfgsfloatval_t>*_param
	)
{
	int ret;
	int i, j, k, ls, end, bound;
	lbfgsfloatval_t step;

	/* Constant parameters and their default values. */
	lbfgs_parameter_t<lbfgsfloatval_t> param = *_param;
	const int m = param.m;

	lbfgsfloatval_t *xp = NULL;
	lbfgsfloatval_t *g = NULL, *gp = NULL, *pg = NULL;
	lbfgsfloatval_t *d = NULL, *w = NULL, *pf = NULL;
	iteration_data_t<lbfgsfloatval_t> *lm = NULL, *it = NULL;
	lbfgsfloatval_t ys, yy;
	lbfgsfloatval_t xnorm, gnorm, beta;
	lbfgsfloatval_t fx = 0.;
	lbfgsfloatval_t rate = 0.;
	line_search_proc<lbfgsfloatval_t> linesearch = line_search_morethuente<lbfgsfloatval_t>;

	callback_data_t<lbfgsfloatval_t> cd;
	cd.n = n;
	cd.instance = instance;
	cd.proc_evaluate = proc_evaluate;
	cd.proc_progress = proc_progress;

#if     defined(USE_SSE) && (defined(__SSE__) || defined(__SSE2__))
	n = round_out_variables(n);
#endif

	if (n <= 0) {
		return LBFGSERR_INVALID_N;
	}
#if     defined(USE_SSE) && (defined(__SSE__) || defined(__SSE2__))
	if (n % 8 != 0) {
		return LBFGSERR_INVALID_N_SSE;
	}
	if ((uintptr_t)(const void*)x % 16 != 0) {
		return LBFGSERR_INVALID_X_SSE;
	}
#endif/*defined(USE_SSE)*/
	if (param.epsilon < 0.) {
		return LBFGSERR_INVALID_EPSILON;
	}
	if (param.past < 0) {
		return LBFGSERR_INVALID_TESTPERIOD;
	}
	if (param.delta < 0.) {
		return LBFGSERR_INVALID_DELTA;
	}
	if (param.min_step < 0.) {
		return LBFGSERR_INVALID_MINSTEP;
	}
	if (param.max_step < param.min_step) {
		return LBFGSERR_INVALID_MAXSTEP;
	}
	if (param.ftol < 0.) {
		return LBFGSERR_INVALID_FTOL;
	}
	if (param.linesearch == LBFGS_LINESEARCH_BACKTRACKING_WOLFE ||
		param.linesearch == LBFGS_LINESEARCH_BACKTRACKING_STRONG_WOLFE) {
		if (param.wolfe <= param.ftol || 1. <= param.wolfe) {
			return LBFGSERR_INVALID_WOLFE;
		}
	}
	if (param.gtol < 0.) {
		return LBFGSERR_INVALID_GTOL;
	}
	if (param.xtol < 0.) {
		return LBFGSERR_INVALID_XTOL;
	}
	if (param.max_linesearch <= 0) {
		return LBFGSERR_INVALID_MAXLINESEARCH;
	}
	if (param.orthantwise_c < 0.) {
		return LBFGSERR_INVALID_ORTHANTWISE;
	}
	if (param.orthantwise_start < 0 || n < param.orthantwise_start) {
		return LBFGSERR_INVALID_ORTHANTWISE_START;
	}
	if (param.orthantwise_end < 0) {
		param.orthantwise_end = n;
	}
	if (n < param.orthantwise_end) {
		return LBFGSERR_INVALID_ORTHANTWISE_END;
	}
	if (param.orthantwise_c != 0.) {
		switch (param.linesearch) {
		case LBFGS_LINESEARCH_BACKTRACKING:
			linesearch = line_search_backtracking_owlqn;
			break;
		default:
			/* Only the backtracking method is available. */
			return LBFGSERR_INVALID_LINESEARCH;
		}
	} else {
		switch (param.linesearch) {
		case LBFGS_LINESEARCH_MORETHUENTE:
			linesearch = line_search_morethuente;
			break;
		case LBFGS_LINESEARCH_BACKTRACKING_ARMIJO:
		case LBFGS_LINESEARCH_BACKTRACKING_WOLFE:
		case LBFGS_LINESEARCH_BACKTRACKING_STRONG_WOLFE:
			linesearch = line_search_backtracking;
			break;
		default:
			return LBFGSERR_INVALID_LINESEARCH;
		}
	}

	xp = (lbfgsfloatval_t*)vecalloc(n * sizeof(lbfgsfloatval_t));
	g = (lbfgsfloatval_t*)vecalloc(n * sizeof(lbfgsfloatval_t));
	gp = (lbfgsfloatval_t*)vecalloc(n * sizeof(lbfgsfloatval_t));
	d = (lbfgsfloatval_t*)vecalloc(n * sizeof(lbfgsfloatval_t));
	w = (lbfgsfloatval_t*)vecalloc(n * sizeof(lbfgsfloatval_t));
	if (xp == NULL || g == NULL || gp == NULL || d == NULL || w == NULL) {
		ret = LBFGSERR_OUTOFMEMORY;
		goto lbfgs_exit;
	}

	if (param.orthantwise_c != 0) {
		pg = (lbfgsfloatval_t*)vecalloc(n * sizeof(lbfgsfloatval_t));
		if (pg == NULL) {
			ret = LBFGSERR_OUTOFMEMORY;
			goto lbfgs_exit;
		}
	}

	lm = (iteration_data_t<lbfgsfloatval_t>*)vecalloc(m * sizeof(iteration_data_t<lbfgsfloatval_t>));
	if (lm == NULL) {
		ret = LBFGSERR_OUTOFMEMORY;
		goto lbfgs_exit;
	}

	for (i = 0;i < m;++i) {
		it = &lm[i];
		it->alpha = 0;
		it->ys = 0;
		it->s = (lbfgsfloatval_t*)vecalloc(n * sizeof(lbfgsfloatval_t));
		it->y = (lbfgsfloatval_t*)vecalloc(n * sizeof(lbfgsfloatval_t));
		if (it->s == NULL || it->y == NULL) {
			ret = LBFGSERR_OUTOFMEMORY;
			goto lbfgs_exit;
		}
	}

	if (0 < param.past) {
		pf = (lbfgsfloatval_t*)vecalloc(param.past * sizeof(lbfgsfloatval_t));
	}

	fx = cd.proc_evaluate(cd.instance, x, g, cd.n, 0);
	if (0. != param.orthantwise_c) {
		xnorm = owlqn_x1norm(x, param.orthantwise_start, param.orthantwise_end);
		fx += xnorm * param.orthantwise_c;
		owlqn_pseudo_gradient(
			pg, x, g, n,
			param.orthantwise_c, param.orthantwise_start, param.orthantwise_end
			);
	}

	if (pf != NULL) {
		pf[0] = fx;
	}

	if (param.orthantwise_c == 0.) {
		vecncpy(d, g, n);
	} else {
		vecncpy(d, pg, n);
	}

	vec2norm(&xnorm, x, n);
	if (param.orthantwise_c == 0.) {
		vec2norm(&gnorm, g, n);
	} else {
		vec2norm(&gnorm, pg, n);
	}
	if (xnorm < 1.0) xnorm = 1.0;
	if (gnorm / xnorm <= param.epsilon) {
		ret = LBFGS_ALREADY_MINIMIZED;
		goto lbfgs_exit;
	}

	vec2norminv(&step, d, n);

	k = 1;
	end = 0;
	for (;;) {
		veccpy(xp, x, n);
		veccpy(gp, g, n);

		if (param.orthantwise_c == 0.) {
			ls = linesearch(n, x, &fx, g, d, &step, xp, gp, w, &cd, &param);
		} else {
			ls = linesearch(n, x, &fx, g, d, &step, xp, pg, w, &cd, &param);
			owlqn_pseudo_gradient(
				pg, x, g, n,
				param.orthantwise_c, param.orthantwise_start, param.orthantwise_end
				);
		}
		if (ls < 0) {
			veccpy(x, xp, n);
			veccpy(g, gp, n);
			ret = ls;
			goto lbfgs_exit;
		}

		vec2norm(&xnorm, x, n);
		if (param.orthantwise_c == 0.) {
			vec2norm(&gnorm, g, n);
		} else {
			vec2norm(&gnorm, pg, n);
		}

		if (cd.proc_progress) {
			if ((ret = cd.proc_progress(cd.instance, x, g, fx, xnorm, gnorm, step, cd.n, k, ls))) {
				goto lbfgs_exit;
			}
		}

		// Convergence test.
		if (xnorm < 1.0) xnorm = 1.0;
		if (gnorm / xnorm <= param.epsilon) {
			/* Convergence. */
			ret = LBFGS_SUCCESS;
			break;
		}

		if (pf != NULL) {
			if (param.past <= k) {
				rate = (pf[k % param.past] - fx) / fx;
				if (fabs(rate) < param.delta) {
					ret = LBFGS_STOP;
					break;
				}
			}

			pf[k % param.past] = fx;
		}

		if (param.max_iterations != 0 && param.max_iterations < k+1) {
			ret = LBFGSERR_MAXIMUMITERATION;
			break;
		}

		it = &lm[end];
		vecdiff(it->s, x, xp, n);
		vecdiff(it->y, g, gp, n);
		vecdot(&ys, it->y, it->s, n);
		vecdot(&yy, it->y, it->y, n);
		it->ys = ys;

		bound = (m <= k) ? m : k;
		++k;
		end = (end + 1) % m;

		if (param.orthantwise_c == 0.) {
			vecncpy(d, g, n);
		} else {
			vecncpy(d, pg, n);
		}

		j = end;
		for (i = 0;i < bound;++i) {
			j = (j + m - 1) % m;
			it = &lm[j];
			vecdot(&it->alpha, it->s, d, n);
			it->alpha /= it->ys;
			vecadd(d, it->y, -it->alpha, n);
		}

		vecscale(d, ys / yy, n);

		for (i = 0;i < bound;++i) {
			it = &lm[j];
			vecdot(&beta, it->y, d, n);
			beta /= it->ys;
			vecadd(d, it->s, it->alpha - beta, n);
			j = (j + 1) % m;
		}

		if (param.orthantwise_c != 0.) {
			for (i = param.orthantwise_start;i < param.orthantwise_end;++i) {
				if (d[i] * pg[i] >= 0) {
					d[i] = 0;
				}
			}
		}

		step = 1.0;
	}

lbfgs_exit:
	if (ptr_fx != NULL) {
		*ptr_fx = fx;
	}

	vecfree(pf);
	if (lm != NULL) {
		for (i = 0;i < m;++i) {
			vecfree(lm[i].s);
			vecfree(lm[i].y);
		}
		vecfree(lm);
	}
	vecfree(pg);
	vecfree(w);
	vecfree(d);
	vecfree(gp);
	vecfree(g);
	vecfree(xp);

	return ret;
}

inline const char* lbfgs_strerror(int err)
{
	switch(err) {
		case LBFGS_SUCCESS:
			return "Success: reached convergence (gtol).";
		case LBFGS_STOP:
			return "Success: met stopping criteria (ftol).";
		case LBFGS_ALREADY_MINIMIZED:
			return "The initial variables already minimize the objective function.";
		case LBFGSERR_UNKNOWNERROR:
			return "Unknown error.";
		case LBFGSERR_LOGICERROR:
			return "Logic error.";
		case LBFGSERR_OUTOFMEMORY:
			return "Insufficient memory.";
		case LBFGSERR_CANCELED:
			return "The minimization process has been canceled.";
		case LBFGSERR_INVALID_N:
			return "Invalid number of variables specified.";
		case LBFGSERR_INVALID_N_SSE:
			return "Invalid number of variables (for SSE) specified.";
		case LBFGSERR_INVALID_X_SSE:
			return "The array x must be aligned to 16 (for SSE).";
		case LBFGSERR_INVALID_EPSILON:
			return "Invalid parameter lbfgs_parameter_t::epsilon specified.";
		case LBFGSERR_INVALID_TESTPERIOD:
			return "Invalid parameter lbfgs_parameter_t::past specified.";
		case LBFGSERR_INVALID_DELTA:
			return "Invalid parameter lbfgs_parameter_t::delta specified.";
		case LBFGSERR_INVALID_LINESEARCH:
			return "Invalid parameter lbfgs_parameter_t::linesearch specified.";
		case LBFGSERR_INVALID_MINSTEP:
			return "Invalid parameter lbfgs_parameter_t::max_step specified.";
		case LBFGSERR_INVALID_MAXSTEP:
			return "Invalid parameter lbfgs_parameter_t::max_step specified.";
		case LBFGSERR_INVALID_FTOL:
			return "Invalid parameter lbfgs_parameter_t::ftol specified.";
		case LBFGSERR_INVALID_WOLFE:
			return "Invalid parameter lbfgs_parameter_t::wolfe specified.";
		case LBFGSERR_INVALID_GTOL:
			return "Invalid parameter lbfgs_parameter_t::gtol specified.";
		case LBFGSERR_INVALID_XTOL:
			return "Invalid parameter lbfgs_parameter_t::xtol specified.";
		case LBFGSERR_INVALID_MAXLINESEARCH:
			return "Invalid parameter lbfgs_parameter_t::max_linesearch specified.";
		case LBFGSERR_INVALID_ORTHANTWISE:
			return "Invalid parameter lbfgs_parameter_t::orthantwise_c specified.";
		case LBFGSERR_INVALID_ORTHANTWISE_START:
			return "Invalid parameter lbfgs_parameter_t::orthantwise_start specified.";
		case LBFGSERR_INVALID_ORTHANTWISE_END:
			return "Invalid parameter lbfgs_parameter_t::orthantwise_end specified.";
		case LBFGSERR_OUTOFINTERVAL:
			return "The line-search step went out of the interval of uncertainty.";
		case LBFGSERR_INCORRECT_TMINMAX:
			return "A logic error occurred; alternatively, the interval of uncertainty"
				   " became too small.";
		case LBFGSERR_ROUNDING_ERROR:
			return "A rounding error occurred; alternatively, no line-search step"
				   " satisfies the sufficient decrease and curvature conditions.";
		case LBFGSERR_MINIMUMSTEP:
			return "The line-search step became smaller than lbfgs_parameter_t::min_step.";
		case LBFGSERR_MAXIMUMSTEP:
			return "The line-search step became larger than lbfgs_parameter_t::max_step.";
		case LBFGSERR_MAXIMUMLINESEARCH:
			return "The line-search routine reaches the maximum number of evaluations.";
		case LBFGSERR_MAXIMUMITERATION:
			return "The algorithm routine reaches the maximum number of iterations.";
		case LBFGSERR_WIDTHTOOSMALL:
			return "Relative width of the interval of uncertainty is at most"
				   " lbfgs_parameter_t::xtol.";
		case LBFGSERR_INVALIDPARAMETERS:
			return "A logic error (negative line-search step) occurred.";
		case LBFGSERR_INCREASEGRADIENT:
			return "The current search direction increases the objective function value.";
		default:
			return "(unknown)";
	}
}

template<typename lbfgsfloatval_t>
class LBFGSEvalFunction
{
public:
	virtual ~LBFGSEvalFunction() {}
	virtual lbfgsfloatval_t Evaluate(const lbfgsfloatval_t* X, int Dim, lbfgsfloatval_t* Gradient) const = 0;
};

template<typename lbfgsfloatval_t>
struct LBFGSInstance
{
	LBFGSEvalFunction<lbfgsfloatval_t>* eval;
	void* minimizer;
};

template<typename lbfgsfloatval_t>
class LBFGSMinimizer
{
public:
	LBFGSMinimizer()
	{
		status = 0;
		iterations = 0;
	}

	lbfgsfloatval_t Minimize(LBFGSEvalFunction<lbfgsfloatval_t>* func, int Dim)
	{
		lbfgsfloatval_t min_f;
		lbfgsfloatval_t* x = lbfgs_malloc<lbfgsfloatval_t>(Dim);
		for (int i = 0; i < Dim; i += 2) {
			x[i] = (lbfgsfloatval_t)0;
		}
		bool success = _solve(func, x, Dim, &min_f);
		lbfgs_free<lbfgsfloatval_t>(x);
		return success ? min_f : std::numeric_limits<lbfgsfloatval_t>::max();
	}

	lbfgsfloatval_t Minimize(LBFGSEvalFunction<lbfgsfloatval_t>* func, lbfgsfloatval_t* init_x, int Dim)
	{
		lbfgsfloatval_t ymin;
		bool success = _solve(func, init_x, Dim, &ymin);
		return success ? ymin : std::numeric_limits<lbfgsfloatval_t>::max();
	}

	bool Minimize(LBFGSEvalFunction<lbfgsfloatval_t>* func, lbfgsfloatval_t* init_x, int Dim, lbfgsfloatval_t* min_f)
	{
		return _solve(func, init_x, Dim, min_f);
	}

private:
	bool _solve(LBFGSEvalFunction<lbfgsfloatval_t>* func, lbfgsfloatval_t* init_x, int Dim, lbfgsfloatval_t * min_f)
	{
		iterations = 0;
		LBFGSInstance<lbfgsfloatval_t> instance;
		instance.eval = func;
		instance.minimizer = this;
		lbfgs_parameter_t<lbfgsfloatval_t> param;
		status = lbfgs<lbfgsfloatval_t>(Dim, init_x, min_f, lbfgs_evaluate, lbfgs_progress, (void*)&instance, &param);
		return status == LBFGS_SUCCESS ? true : false;
	}

	static lbfgsfloatval_t lbfgs_evaluate(void* instance, const lbfgsfloatval_t* x, lbfgsfloatval_t* g, const int n, const lbfgsfloatval_t step)
	{
		LBFGSInstance<lbfgsfloatval_t>* p = static_cast<LBFGSInstance<lbfgsfloatval_t>*>(instance);
		lbfgsfloatval_t f = p->eval->Evaluate(x, n, g);
		return f;
	}

	static int lbfgs_progress(
		void* instance,
		const lbfgsfloatval_t* x,
		const lbfgsfloatval_t* g,
		const lbfgsfloatval_t fx,
		const lbfgsfloatval_t xnorm,
		const lbfgsfloatval_t gnorm,
		const lbfgsfloatval_t step,
		int n,
		int k,
		int ls
	)
	{
		LBFGSInstance<lbfgsfloatval_t>* p = static_cast<LBFGSInstance<lbfgsfloatval_t>*>(instance);
		((LBFGSMinimizer<lbfgsfloatval_t>*)p->minimizer)->iterations = k;
		return 0;
	}

public:
	int status;
	int iterations;
};