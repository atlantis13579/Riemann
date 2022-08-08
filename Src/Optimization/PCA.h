#pragma once

#include "../LinearSystem/DenseMatrix.h"

template<typename T>
class PrincipalComponentAnalysis
{
public:
	void Fit(const TDenseMatrix<T>& Data)
	{
		int Dim = Data.GetRows();
		Means.SetSize(Dim);

		int n = Data.GetCols();
		for (int i = 0; i < Dim; ++i)
		{
			const T *p = Data[i];
			
			Means[i] = 0;
			for (int j = 0; j < n; ++j)
			{
				Means[i] += p[j];
			}
			Means[i] /= n;
		}
		
		Covariance.SetSize(Dim, Dim);
		for (int i = 0; i < Dim; ++i)
		for (int j = i; j < Dim; ++j)
		{
			const T *p = Data[i];
			
			T sum = (T)0;
			for (int k = 0; k < n; ++k)
			{
				sum += p[k] * Data[k][j];
			}
			Covariance(i, j) = sum / n;
		}
		for (int i = 0; i < Dim; ++i)
		for (int j = 0; j < i; ++j)
		{
			Covariance(i, j) = Covariance(j, i);
		}
		
		Covariance.EigenDecompose(Eigens, ComponentsMatrix);
	}
	
	void TopKComponents(int k)
	{
		// Sort ?
		
		for (int i = k; i < Eigens.GetSize(); ++i)
		{
			Eigens[i] = 0;
			ComponentsMatrix.SetColZero(i);
		}
	}
	
	void Transform(DenseMatrix& src)
	{
		src = ComponentsMatrix * src;
	}
	
public:
	TDenseVector<T> Means;
	TDenseMatrix<T> Covariance;
	TDenseVector<T> Eigens;
	TDenseMatrix<T> ComponentsMatrix;
};
