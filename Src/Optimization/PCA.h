#pragma once

#include "../LinearSystem/DenseMatrix.h"

template<typename T, bool ColumnWise = true>
class PrincipalComponentAnalysis
{
public:
	void Fit(const TDenseMatrix<T>& Data)
	{
		int Dim, n;
		
		if (ColumnWise)
		{
			Dim = Data.GetRows();
			means.SetSize(Dim);

			n = Data.GetCols();
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
		}
		else
		{
			Dim = Data.GetCols();
			means.SetSize(Dim);

			means.LoadZero();
			n = Data.GetRows();
			for (int i = 0; i < n; ++i)
			{
				means += Data.GetRow(i);
			}
			means /= n;
		}
			
		covariance.SetSize(Dim, Dim);
		for (int i = 0; i < Dim; ++i)
		for (int j = i; j < Dim; ++j)
		{
			const T *pi = Data[i];
				
			T sum = (T)0;
			for (int k = 0; k < n; ++k)
			{
				sum += (pi[k] - means[i]) * (Data[k][j] - means[j]);
			}
			covariance(i, j) = sum / n;
		}

		for (int i = 0; i < Dim; ++i)
		for (int j = 0; j < i; ++j)
		{
			covariance(i, j) = covariance(j, i);
		}
		
		covariance.EigenDecompose(eigens, componentsMatrix);
		
		if (!ColumnWise)
			componentsMatrix.TransposeInPlace();
	}
	
	void TopKComponents(int k)
	{
		// Sort ?
		
		for (int i = k; i < eigens.GetSize(); ++i)
		{
			eigens[i] = 0;
			componentsMatrix.SetColZero(i);
		}
	}
	
	void Transform(TDenseMatrix<T>& src)
	{
		if (ColumnWise)
		{
			src = componentsMatrix * src;
		}
		else
		{
			src = src * componentsMatrix;
		}
	}
	
	void Transform(TDenseVector<T>& src)
	{
		if (ColumnWise)
		{
			src = componentsMatrix * src;
		}
		else
		{
			src = src * componentsMatrix;
		}
	}
	
public:
	TDenseVector<T> means;
	TDenseMatrix<T> covariance;
	TDenseVector<T> eigens;
	TDenseMatrix<T> componentsMatrix;
};
