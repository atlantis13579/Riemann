#pragma once

#include <assert.h>
#include <math.h>
#include <vector>
#include "DenseVector.h"
#include "gemm.inl"

template<typename T>
bool FloatEqual(const T a, const T b, const T Eps = (T)1e-6)
{
	return fabs(a - b) < Eps;
}

template<typename T>
class TDenseMatrix
{
public:
	explicit TDenseMatrix() : mRows(0), mCols(0), pData(nullptr)
	{

	}

	explicit TDenseMatrix(int nSize) : TDenseMatrix(nSize, nSize)
	{

	}

	explicit TDenseMatrix(int nRows, int nCols) : mRows(nRows), mCols(nCols), mData(nCols* nRows)
	{
		pData = &mData[0];
	}
	
	explicit TDenseMatrix(const TDenseVector<T>& diag) : mRows(diag.GetSize()), mCols(diag.GetSize()), mData(diag.GetSize() * diag.GetSize())
	{
		pData = &mData[0];
		LoadDiagonal(diag);
	}

	explicit TDenseMatrix(const TDenseMatrix& m) : mRows(m.mRows), mCols(m.mCols), mData(mCols* mRows)
	{
		pData = &mData[0];
		memcpy(pData, m.pData, sizeof(T) * mRows * mCols);
	}

	TDenseMatrix(TDenseMatrix&& m) : mRows(m.mRows), mCols(m.mCols)
	{
		if (!m.mData.empty())
			mData = std::move(m.mData);
		pData = mData.size() > 0 ? &mData[0] : m.pData;
	}

	explicit TDenseMatrix(T* p, int nRows, int nCols) : mRows(nRows), mCols(nCols)
	{
		pData = p;
	}

	void		SetSize(int nRows, int nCols)
	{
		mRows = nRows;
		mCols = nCols;
		mData.resize(nCols * nRows);
		pData = &mData[0];
	}

	inline int	GetSize() const
	{
		return mRows * mCols;
	}

	inline int	GetRows() const
	{
		return mRows;
	}

	inline	int	GetCols() const
	{
		return mCols;
	}

	inline T*	GetData()
	{
		return pData;
	}

	inline const T*	GetData() const
	{
		return pData;
	}

	TDenseVector<T>	GetRow(int i) const
	{
		TDenseVector<T> Ret(pData + i * mCols, mCols);
		return std::move(Ret);
	}

	TDenseVector<T>	GetCol(int i) const
	{
		TDenseVector<T> Ret(mRows);
		for (int j = 0; j < mRows; ++j)
		{
			Ret[j] = pData[j * mCols + i];
		}
		return std::move(Ret);
	}
	
	void		SetRowZero(int i)
	{
		memset(pData + i * mCols, 0, sizeof(T)*mCols);
	}
	
	void		SetColZero(int i)
	{
		for (int j = 0; j < mRows; ++j)
		{
			pData[j*mCols + i] = (T)0;
		}
	}

	bool			HoldsMemory() const
	{
		return mData.size() > 0 || pData == nullptr;
	}

	void			Assign(const TDenseMatrix<T> &rhs)
	{
		if (!HoldsMemory())
		{
			memcpy(pData, rhs.GetData(), mRows * mCols * sizeof(T));
		}
		else
		{
			SetSize(rhs.GetRows(), rhs.GetCols());
			memcpy(pData, rhs.GetData(), mRows * mCols * sizeof(T));
		}
	}

	inline bool		IsSquare() const
	{
		return mCols == mRows;
	}

	inline bool		IsColumnVector() const
	{
		return mCols == 1;
	}

	inline bool		IsRowVector() const
	{
		return mRows == 1;
	}

	inline const T* operator[](int i) const
	{
		return pData + i * mCols;
	}

	inline T* operator[](int i)
	{
		return pData + i * mCols;
	}

	inline T operator()(int i, int j) const
	{
		return pData[i * mCols + j];
	}

	inline T& operator()(int i, int j)
	{
		return pData[i * mCols + j];
	}

	TDenseMatrix<T>& operator=(const TDenseMatrix<T>& m)
	{
		mRows = m.mRows;
		mCols = m.mCols;
		mData = m.mData;
		pData = mData.size() > 0 ? &mData[0] : m.pData;
		return *this;
	}

	TDenseMatrix<T>& operator=(TDenseMatrix<T>&& m)
	{
		mRows = m.mRows;
		mCols = m.mCols;
		mData = std::move(m.mData);
		pData = mData.size() > 0 ? &mData[0] : m.pData;
		return *this;
	}

	TDenseMatrix<T> operator+(const TDenseMatrix<T>& m) const
	{
		assert(mCols == m.mCols && mRows == m.mRows);
		TDenseMatrix<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret.pData[i] += m.pData[i];
		}
		return std::move(Ret);
	}

	TDenseMatrix<T> operator-(const TDenseMatrix<T>& m) const
	{
		assert(mCols == m.mCols && mRows == m.mRows);
		TDenseMatrix<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret.pData[i] -= m.pData[i];
		}
		return std::move(Ret);
	}

	TDenseMatrix<T> operator*(T k) const
	{
		TDenseMatrix<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret.pData[i] *= k;
		}
		return std::move(Ret);
	}

	TDenseMatrix<T>	operator*(const TDenseMatrix<T>& m) const
	{
		assert(mCols == m.mRows);
		TDenseMatrix<T> Ret(mRows, m.mCols);
		gemm<T>(pData, m.pData, mRows, mCols, m.mCols, Ret.pData);
		return Ret;
	}

	TDenseVector<T>	operator*(const TDenseVector<T>& v) const
	{
		assert(mCols == v.GetSize());
		TDenseVector<T> Ret(mRows);
		gemv<T>(pData, v.GetData(), mRows, mCols, Ret.GetData());
		return Ret;
	}

	TDenseMatrix<T>	operator-() const
	{
		TDenseMatrix<T> Ret(*this);
		for (int i = 0; i < GetSize(); ++i)
		{
			Ret.pData[i] = -Ret.pData[i];
		}
		return std::move(Ret);
	}

	void		operator+= (const TDenseMatrix<T>& m)
	{
		assert(mCols == m.mCols && mRows == m.mRows);
		for (int i = 0; i < GetSize(); ++i)
		{
			pData[i] += m.pData[i];
		}
	}

	void		operator-= (const TDenseMatrix<T>& m)
	{
		assert(mCols == m.mCols && mRows == m.mRows);
		for (int i = 0; i < GetSize(); ++i)
		{
			pData[i] -= m.pData[i];
		}
	}

	void		operator*= (T k)
	{
		for (int i = 0; i < GetSize(); ++i)
		{
			pData[i] *= k;
		}
	}

	void		SubAdd(const TDenseMatrix<T>& m, int i0, int i1, int j0, int j1)
	{
		assert(mCols == m.mCols && mRows == m.mRows);
		gema_block<T>(pData, m.pData, mRows, mCols, i0, i1, j0, j1, pData);
	}

	TDenseMatrix<T>	SubMultiply(const TDenseMatrix<T>& m, int i0, int i1, int j0, int j1, int k0, int k1)
	{
		assert(mCols == m.mRows);
		TDenseMatrix<T> Ret(mRows, m.mCols);
		Ret.LoadZero();
		gemm_block<T>(pData, m.pData, mRows, mCols, m.mCols, i0, i1, j0, j1, k0, k1, Ret.pData);
		return Ret;
	}

	bool		FuzzyEqual(const TDenseMatrix<T> &rhs, const T Eplison = (T)1e-6) const
	{
		if (mRows != rhs.GetRows() || mCols != rhs.GetCols())
			return false;

		const T* p1 = GetData();
		const T* p2 = rhs.GetData();
		for (int i = 0; i < mRows * mCols; ++i)
		{
			if (!::FloatEqual(p1[i], p2[i], Eplison))
				return false;
		}

		return true;
	}

	// https://en.wikipedia.org/wiki/In-place_matrix_transposition
	void		TransposeInPlace()
	{
		for (int i = 0; i < mRows * mCols; ++i)
		{
			int next = i;
			int j = 0;
			do
			{
				++j;
				next = (next % mRows) * mCols + next / mRows;
			} while (next > i);

			if (next >= i && j != 1)
			{
				const T t = pData[i];
				next = i;
				do
				{
					j = (next % mRows) * mCols + next / mRows;
					pData[next] = (j == i) ? t : pData[j];
					next = j;
				} while (next > i);
			}
		}
		
		std::swap(mRows, mCols);
	}

	T			DotProductRow(int i, int j) const
	{
		T sum = (T)0;
		T* pi = pData + i * mCols;
		T* pj = pData + j * mCols;
		for (int k = 0; k < mCols; ++k)
		{
			sum += pi[k] * pj[k];
		}
		return sum;
	}

	T			DotProductCol(int i, int j) const
	{
		T sum = (T)0;
		for (int k = 0; k < mRows; ++k)
		{
			sum += pData[k * mCols + i] * pData[k * mCols + j];
		}
		return sum;
	}

	TDenseMatrix<T>		CoProduct() const		// X^T * X
	{
		TDenseMatrix<T> Ret(mCols, mCols);
		for (int i = 0; i < mCols; ++i)
		for (int j = i; j < mCols; ++j)
		{
			Ret(i, j) = DotProductCol(i, j);
		}
		for (int i = 0; i < mCols; ++i)
		for (int j = 0; j < i; ++j)
		{
			Ret(i, j) = Ret(j, i);
		}
		return std::move(Ret);
	}

	bool		IsOrthogonal(const T Eplison = (T)1e-6) const
	{
		if (!IsSquare())
			return false;

		for (int i = 0; i < mCols; ++i)
		for (int j = i; j < mCols; ++j)
		{
			T dp = DotProductCol(i, j);
			if (i == j)
			{
				if (!::FloatEqual(dp, (T)1, Eplison))
					return false;
			}
			else
			{
				if (!::FloatEqual(dp, (T)0, Eplison))
					return false;
			}
		}

		return true;
	}

	bool		IsSymmetric(const T Eplison = (T)1e-3) const
	{
		if (!IsSquare())
			return false;

		for (int i = 0; i < mCols; ++i)
		for (int j = i + 1; j < mCols; ++j)
		{
			const T& a = pData[i * mCols + j];
			const T& b = pData[j * mCols + i];
			if (!::FloatEqual(a, b, Eplison))
				return false;
		}

		return true;
	}

	bool		IsUpperTriangle(const T Eplison = (T)1e-6) const
	{
		if (!IsSquare())
			return false;

		for (int i = 1; i < mRows; ++i)
		for (int j = 0; j < i; ++j)
		{
			T a = mData[i * mRows + j];
			if (!::FloatEqual(a, (T)0, Eplison))
				return false;
		}
		return true;
	}

	bool		IsLowerTriangle(const T Eplison = (T)1e-6) const
	{
		if (!IsSquare())
			return false;

		for (int i = 0; i < mRows - 1; ++i)
		for (int j = i + 1; j < mRows; ++j)
		{
			T a = mData[i * mRows + j];
			if (!::FloatEqual(a, (T)0, Eplison))
				return false;
		}
		return true;
	}

	void		LoadZero()
	{
		memset(pData, 0, sizeof(T) * mRows * mCols);
	}

	void		LoadIdentity()
	{
		if (!IsSquare())
		{
			return;
		}

		LoadZero();
		for (int i = 0; i < mRows; ++i)
		{
			pData[i * mRows + i] = (T)1;
		}
	}
	
	void 		LoadDiagonal(const TDenseVector<T> &diag)
	{
		int Size = mRows < mCols ? mRows : mCols;
		if (Size != diag.GetSize())
		{
			return;
		}
		
		LoadZero();
		for (int i = 0; i < Size; ++i)
		{
			pData[i * mCols + i] = diag[i];
		}
	}

	bool		IsZero() const
	{
		for (int i = 0; i < GetSize(); ++i)
		{
			if (!FloatEqual(pData[i], (T)0))
				return false;
		}
		return true;
	}

	bool		IsIdentity(const T Eplison = (T)1e-6) const
	{
		if (!IsSquare())
		{
			return false;
		}

		int n = mRows;
		for (int i = 0; i < n; ++i)
		for (int j = 0; j < n; ++j)
		{
			T v = pData[i * n + j];
			if (i == j)
			{
				if (!::FloatEqual(v, (T)1, Eplison))
					return false;
			}
			else
			{
				if (!::FloatEqual(v, (T)0, Eplison))
					return false;
			}
		}
		return true;
	}

	T			Trace() const
	{
		assert(IsSquare());
		T tr = (T)0;
		for (int i = 0; i < mRows; ++i)
		{
			tr += pData[i * mRows + i];
		}
		return tr;
	}

	T			Determinant() const;

	TDenseMatrix<T>	Inverse() const
	{
		TDenseMatrix<T> InvM(GetRows());
		bool success = GetInverse(InvM);
		if (!success)
		{
			InvM.LoadIdentity();
		}
		return std::move(InvM);
	}
	
	bool		GetInverse(TDenseMatrix<T>& InvM) const;

	TDenseMatrix<T> PseudoInverse() const
	{
		TDenseMatrix<T> pinv;
		bool succ = GetPseudoInverse(pinv);
		assert(succ);
		return pinv;
	}
	
	bool		GetPseudoInverse(TDenseMatrix<T>& pinv) const;
	
	TDenseMatrix<T>	Transpose() const
	{
		TDenseMatrix<T> Ret(*this);
		Ret.TransposeInPlace();
		return std::move(Ret);
	}

	T			L1Norm() const
	{
		T sum = (T)0;
		for (int i = 1; i < mRows * mCols; ++i)
		{
			sum += std::abs(pData[i]);
		}
		return sum;
	}

	T			L2Norm() const
	{
		T sum = (T)0;
		for (int i = 1; i < mRows * mCols; ++i)
		{
			sum += pData[i] * pData[i];
		}
		return std::sqrt(sum);
	}

	T			LpNorm(int p) const
	{
		T sum = (T)0;
		for (int i = 1; i < mRows * mCols; ++i)
		{
			sum += std::pow(pData[i], p);
		}
		return std::pow(sum, (T)1 / p);
	}

	T			LInfinityNorm() const
	{
		T maxVal = pData[0];
		for (int i = 1; i < mRows * mCols; ++i)
		{
			maxVal = std::max(maxVal, pData[i]);
		}
		return maxVal;
	}

	// M = U * S * V^T
	bool 	SingularValueDecompose(TDenseMatrix<T> &U, TDenseVector<T> &S, TDenseMatrix<T> &V) const;
	
	// M = U * P
	bool 	PolarDecompose(TDenseMatrix<T> &U, TDenseMatrix<T> &P) const;

	// M = Q * R
	bool 	QRDecompose(TDenseMatrix<T>& Q, TDenseMatrix<T>& R) const;

	// Get Eigen Values and EigenVectors as column vector
	bool	EigenDecompose(TDenseVector<T>& EigenValues, TDenseMatrix<T>& EigenVectors) const;

	// M = L * L^T
	bool	LUDecompose(TDenseMatrix<T>& L, TDenseMatrix<T>& U) const;
	
	// M = L * L^T
	bool	CholeskyDecompose(TDenseMatrix<T>& L) const;
	
	// Solve M * x = b uisng Cholesky Decompose
	bool	SolveCholesky(const TDenseVector<T>& B, TDenseVector<T>& X);
	
protected:
	int				mRows;
	int				mCols;
	std::vector<T>	mData;
	T* 				pData;
};

template <typename T>
inline TDenseMatrix<T> operator* (T s, const TDenseMatrix<T>& m)
{
	return m * s;
}

using DenseMatrix = TDenseMatrix<float>;
