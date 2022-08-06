#pragma once

// perform Singular Value Decomposition of a matrix

#include <string.h>

template<typename T>
class SingularValueDecomposition
{
public:
	bool operator()(const T* M, int nRows, int nCols, T* Singular, T* U, T* V) const
	{
		memcpy(U, M, sizeof(T)*nRows*nCols);
		return compute_svd(U, nRows, nCols, Singular, V);
	}
	
private:
	// The code is adapted from svdecomp.c in XLISP-STAT 2.1 which is
	// code from Numerical Recipes adapted by Luke Tierney and David Betz.
	bool compute_svd(T *a, int m, int n, T *w, T *v) const
	{
		int flag, i, its, j, jj, k, l = 0, nm = 0;
		T c, f, h, s, x, y, z;
		T anorm = 0, g = 0, scale = 0;
		T *rv1;
	  
		if (m < n)
		{
			return false;
		}
	  
		rv1 = new T[n];
		const int max_iterations = 30;

		// Householder reduction to bidiagonal form
		for (i = 0; i < n; i++)
		{
			// left-hand reduction
			l = i + 1;
			rv1[i] = scale * g;
			g = s = scale = 0;
			if (i < m)
			{
				for (k = i; k < m; k++)
					scale += fabs(a[k*n+i]);
				if (scale)
				{
					for (k = i; k < m; k++)
					{
						a[k*n+i] = a[k*n+i]/scale;
						s += a[k*n+i] * a[k*n+i];
					}
					f = a[i*n+i];
					g = -SIGN(sqrt(s), f);
					h = f * g - s;
					a[i*n+i] = f - g;
					if (i != n - 1)
					{
						for (j = l; j < n; j++)
						{
							for (s = 0.0, k = i; k < m; k++)
								s += a[k*n+i] * a[k*n+j];
							f = s / h;
							for (k = i; k < m; k++)
								a[k*n+j] += f * a[k*n+i];
						}
					}
					for (k = i; k < m; k++)
						a[k*n+i] = a[k*n+i] * scale;
				}
			}
			w[i] = (scale * g);
		
			// right-hand reduction
			g = s = scale = 0.0;
			if (i < m && i != n - 1)
			{
				for (k = l; k < n; k++)
					scale += fabs(a[i*n+k]);
				if (scale)
				{
					for (k = l; k < n; k++)
					{
						a[i*n+k] = a[i*n+k] / scale;
						s += a[i*n+k] * a[i*n+k];
					}
					f = a[i*n+l];
					g = -SIGN(sqrt(s), f);
					h = f * g - s;
					a[i*n+l] = (f - g);
					for (k = l; k < n; k++)
						rv1[k] = a[i*n+k] / h;
					if (i != m - 1)
					{
						for (j = l; j < m; j++)
						{
							for (s = 0.0, k = l; k < n; k++)
								s += a[j*n+k] * a[i*n+k];
							for (k = l; k < n; k++)
								a[j*n+k] += s * rv1[k];
						}
					}
					for (k = l; k < n; k++)
						a[i*n+k] = (a[i*n+k]*scale);
				}
			}
			anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
		}
	  
		// accumulate the right-hand transformation
		for (i = n - 1; i >= 0; i--)
		{
			if (i < n - 1)
			{
				if (g)
				{
					for (j = l; j < n; j++)
						v[j*n+i] = (a[i*n+j] / a[i*n+l]) / g;
						// T division to avoid underflow
					for (j = l; j < n; j++)
					{
						for (s = 0.0, k = l; k < n; k++)
							s += a[i*n+k] * v[k*n+j];
						for (k = l; k < n; k++)
							v[k*n+j] += s * v[k*n+i];
					}
				}
				for (j = l; j < n; j++)
					v[i*n+j] = v[j*n+i] = 0.0;
			}
			v[i*n+i] = 1.0;
			g = rv1[i];
			l = i;
		}
	  
		// accumulate the left-hand transformation
		for (i = n - 1; i >= 0; i--)
		{
			l = i + 1;
			g = w[i];
			if (i < n - 1)
				for (j = l; j < n; j++)
					a[i*n+j] = 0.0;
			if (g)
			{
				g = 1.0 / g;
				if (i != n - 1)
				{
					for (j = l; j < n; j++)
					{
						for (s = 0.0, k = l; k < m; k++)
							s += a[k*n+i] * a[k*n+j];
						f = (s / a[i*n+i]) * g;
						for (k = i; k < m; k++)
							a[k*n+j] += f * a[k*n+i];
					}
				}
				for (j = i; j < m; j++)
					a[j*n+i] = a[j*n+i]*g;
			}
			else
			{
				for (j = i; j < m; j++)
					a[j*n+i] = 0.0;
			}
			++a[i*n+i];
		}

		// diagonalize the bidiagonal form
		for (k = n - 1; k >= 0; k--)
		{ 				// loop over singular values
			for (its = 0; its < max_iterations; its++)
			{			// loop over allowed iterations
				flag = 1;
				for (l = k; l >= 0; l--)
				{		// test for splitting
					nm = l - 1;
					if (fabs(rv1[l]) + anorm == anorm)
					{
						flag = 0;
						break;
					}
					if (fabs(w[nm]) + anorm == anorm)
						break;
				}
				if (flag)
				{
					c = 0.0;
					s = 1.0;
					for (i = l; i <= k; i++)
					{
						f = s * rv1[i];
						if (fabs(f) + anorm != anorm)
						{
							g = w[i];
							h = PYTHAG(f, g);
							w[i] = h;
							h = 1.0 / h;
							c = g * h;
							s = - f * h;
							for (j = 0; j < m; j++)
							{
								y = a[j*n+nm];
								z = a[j*n+i];
								a[j*n+nm] = y * c + z * s;
								a[j*n+i] = z * c - y * s;
							}
						}
					}
				}
				z = w[k];
				if (l == k)
				{					// convergence
					if (z < 0.0)
					{ 				// make singular value nonnegative
						w[k] = (-z);
						for (j = 0; j < n; j++)
							v[j*n+k] = -v[j*n+k];
					}
					break;
				}
				if (its >= max_iterations)
				{
					delete []rv1;
					return false;
				}
		
				// shift from bottom 2 x 2 minor
				x = w[l];
				nm = k - 1;
				y = w[nm];
				g = rv1[nm];
				h = rv1[k];
				f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
				g = PYTHAG(f, 1.0);
				f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
			  
				// next QR transformation
				c = s = 1.0;
				for (j = l; j <= nm; j++)
				{
					i = j + 1;
					g = rv1[i];
					y = w[i];
					h = s * g;
					g = c * g;
					z = PYTHAG(f, h);
					rv1[j] = z;
					c = f / z;
					s = h / z;
					f = x * c + g * s;
					g = g * c - x * s;
					h = y * s;
					y = y * c;
					for (jj = 0; jj < n; jj++)
					{
						x = v[jj*n+j];
						z = v[jj*n+i];
						v[jj*n+j] = x * c + z * s;
						v[jj*n+i] = z * c - x * s;
					}
					z = PYTHAG(f, h);
					w[j] = z;
					if (z)
					{
						z = 1.0 / z;
						c = f * z;
						s = h * z;
					}
					f = c * g + s * y;
					x = c * y - s * g;
					for (jj = 0; jj < m; jj++)
					{
						y = a[jj*n+j];
						z = a[jj*n+i];
						a[jj*n+j] = y * c + z * s;
						a[jj*n+i] = z * c - y * s;
					}
				}
				rv1[l] = 0.0;
				rv1[k] = f;
				w[k] = x;
			}
		}
		delete []rv1;
		return true;
	}
	
	static inline T Abs(const T x)
	{
		return x >= 0 ? x : -x;
	}
	
	static inline T SIGN(const T u, const T v)
	{
		return v >= 0 ? Abs(u) : -Abs(u);
	}
	
	static inline T MAX(const T u, const T v)
	{
		return u >= v ? u : v;
	}
	
	static T PYTHAG(T a, T b)
	{
		T at = fabs(a), bt = fabs(b), ct, result;
		if (at > bt) { ct = bt / at; result = at * sqrt(1.0 + ct * ct); }
		else if (bt > 0) { ct = at / bt; result = bt * sqrt(1.0 + ct * ct); }
		else result = 0;
		return(result);
	}
};
