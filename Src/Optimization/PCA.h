#pragma once

#include "../LinearSystem/DenseMatrix.h"

template<typename T>
class PrincipalComponentAnalysis
{
public:
	void Fit(const TDenseMatrix<T>& Data)
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
		// Sort ?
		
		for (int i = k; i < eigens.GetSize(); ++i)
		{
			eigens[i] = 0;
			componentsMatrix.SetColZero(i);
		}
	}
	
	void Transform(TDenseMatrix<T>& src)
	{
		src = componentsMatrix * src;
	}
	
	void Transform(TDenseVector<T>& src)
	{
		src = componentsMatrix * src;
	}
	
public:
	TDenseVector<T> means;
	TDenseMatrix<T> covariance;
	TDenseVector<T> eigens;
	TDenseMatrix<T> componentsMatrix;
};


template<typename T>
class PrincipalComponentAnalysisRowWise
{
public:
	void Fit(const TDenseMatrix<T>& Data)
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
		// Sort ?
		
		for (int i = k; i < eigens.GetSize(); ++i)
		{
			eigens[i] = 0;
			componentsMatrix.SetColZero(i);
		}
	}
	
	void Transform(TDenseMatrix<T>& src)
	{
		src = src * componentsMatrix;
	}
	
	void Transform(TDenseVector<T>& src)
	{
		src = src * componentsMatrix;
	}
	
public:
	TDenseVector<T> means;
	TDenseMatrix<T> covariance;
	TDenseVector<T> eigens;
	TDenseMatrix<T> componentsMatrix;
};
