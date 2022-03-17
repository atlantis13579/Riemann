
#include <memory>
#include "MatrixN.h"

void gemm_slow(const float* m1, const float* m2, int r1, int c1, int c2, float* m)
{
	for (int i = 0; i < r1; ++i)
	for (int j = 0; j < c2; ++j)
	{
		float dp = 0.0f;
		const float* p = m1 + i * c1;
		for (int k = 0; k < c1; ++k)
			dp += p[k] * m2[k * c2 + j];
		m[i * r1 + j] = dp;
	}
}

TMatrix<float> TMatrix<float>::operator*(const TMatrix<float>& v) const
{
	if (GetCols() != v.GetRows())
	{
		return TMatrix<float>();
	}

	TMatrix<float> Ret(GetRows(), v.GetCols());
	gemm_slow(GetData(), v.GetData(), mRows, mCols, v.GetCols(), Ret.GetData());
	return std::move(Ret);
}
