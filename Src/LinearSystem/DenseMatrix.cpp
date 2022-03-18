#include "DenseMatrix.h"
#include "GaussianElimination.h"

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

void gemv_slow(const float* m1, const float* v1, int r1, int c1, float* v)
{
	for (int i = 0; i < r1; ++i)
	{
		float dp = 0.0f;
		const float* p = m1 + i * c1;
		for (int k = 0; k < c1; ++k)
			dp += p[k] * v1[k];
		v[i] = dp;
	}
}

template<>
TDenseSquare<float> TDenseSquare<float>::operator*(const TDenseSquare<float>& v) const
{
	if (GetSize() != v.GetSize())
	{
		return TDenseSquare<float>();
	}

	TDenseSquare<float> Ret(GetSize());
	gemm_slow(GetData(), v.GetData(), mSize, mSize, mSize, Ret.GetData());
	return std::move(Ret);
}

template<>
TDenseVector<float> TDenseSquare<float>::operator*(const TDenseVector<float>& v) const
{
	if (GetSize() != v.GetSize())
	{
		return TDenseVector<float>();
	}

	TDenseVector<float> Ret(GetSize());
	gemv_slow(GetData(), v.GetData(), GetSize(), GetSize(), Ret.GetData());
	return std::move(Ret);
}

template<>
bool TDenseSquare<float>::GetInverse(TDenseSquare<float>& InvM) const
{
	InvM.SetSize(mSize);
	return GaussianElimination<float>()(GetData(), mSize, InvM.GetData(), nullptr);
}

template<>
float TDenseSquare<float>::Determinant() const
{
	float Det;
	if (GaussianElimination<float>()(GetData(), mSize, nullptr, &Det))
	{
		return Det;
	}
	return 0.0f;
}