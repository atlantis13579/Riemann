#pragma once

#include <algorithm>
#include "../LinearSystem/DenseMatrix.h"

namespace Maths
{
namespace Optimization
{

template<typename T>
class PrincipalComponentAnalysis
{
public:
	void Fit(const Maths::LinearAlgebra::TDenseMatrix<T>& Data)
	{
		int Dim = Data.GetRows();
		means.SetSize(Dim);

		int n = Data.GetCols();
		for (int i = 0; i < Dim; ++i)
		{
			const T *p = Data[i];
					
			T sum = 0;
			for (int j = 0; j < n; ++j)
			{
				sum += p[j];
			}
			means[i] = sum / n;
		}

		covariance.SetSize(Dim, Dim);
		for (int i = 0; i < Dim; ++i)
		for (int j = i; j < Dim; ++j)
		{
			const T *pi = Data[i];
			const T *pj = Data[j];
				
			T sum = (T)0;
			for (int k = 0; k < n; ++k)
			{
				sum += (pi[k] - means[i]) * (pj[k] - means[j]);
			}
			covariance(i, j) = sum / n;
		}

		for (int i = 0; i < Dim; ++i)
		for (int j = 0; j < i; ++j)
		{
			covariance(i, j) = covariance(j, i);
		}
		
		covariance.EigenDecompose(eigens, componentsMatrix);
	}
	
	void TopKComponents(int k)
	{
		std::sort(eigens.GetData(), eigens.GetData() + eigens.GetSize(), [](const float a, const float b) {return a > b; });
		
		for (int i = k; i < eigens.GetSize(); ++i)
		{
			eigens[i] = 0;
			componentsMatrix.SetColZero(i);
		}
	}
	
	void Transform(Maths::LinearAlgebra::TDenseMatrix<T>& src)
	{
		src = componentsMatrix * src;
	}
	
	void Transform(Maths::LinearAlgebra::TDenseVector<T>& src)
	{
		src = componentsMatrix * src;
	}
	
public:
	Maths::LinearAlgebra::TDenseVector<T> means;
	Maths::LinearAlgebra::TDenseMatrix<T> covariance;
	Maths::LinearAlgebra::TDenseVector<T> eigens;
	Maths::LinearAlgebra::TDenseMatrix<T> componentsMatrix;
};


template<typename T>
class PrincipalComponentAnalysisRowWise
{
public:
	void Fit(const Maths::LinearAlgebra::TDenseMatrix<T>& Data)
	{
		int Dim = Data.GetCols();
		means.SetSize(Dim);

		means.LoadZero();
		int n = Data.GetRows();
		for (int i = 0; i < n; ++i)
		{
			means += Data.GetRow(i);
		}
		means /= n;
			
		covariance.SetSize(Dim, Dim);
		for (int i = 0; i < Dim; ++i)
		for (int j = i; j < Dim; ++j)
		{	
			T sum = (T)0;
			for (int k = 0; k < n; ++k)
			{
				sum += (Data[k][i] - means[i]) * (Data[k][j] - means[j]);
			}
			covariance(i, j) = sum / n;
		}

		for (int i = 0; i < Dim; ++i)
		for (int j = 0; j < i; ++j)
		{
			covariance(i, j) = covariance(j, i);
		}
		
		covariance.EigenDecompose(eigens, componentsMatrix);
		componentsMatrix.TransposeInPlace();
	}
	
	void TopKComponents(int k)
	{
		std::sort(eigens.begin(), eigens.end(), [](const float a, const float b) {return a > b; });

		for (int i = k; i < eigens.GetSize(); ++i)
		{
			eigens[i] = 0;
			componentsMatrix.SetRowZero(i);
		}
	}
	
	void Transform(Maths::LinearAlgebra::TDenseMatrix<T>& src)
	{
		src = src * componentsMatrix;
	}
	
	void Transform(Maths::LinearAlgebra::TDenseVector<T>& src)
	{
		src = componentsMatrix * src;
	}
	
public:
	Maths::LinearAlgebra::TDenseVector<T> means;
	Maths::LinearAlgebra::TDenseMatrix<T> covariance;
	Maths::LinearAlgebra::TDenseVector<T> eigens;
	Maths::LinearAlgebra::TDenseMatrix<T> componentsMatrix;
};

}	// namespace Optimization
}	// namespace Maths