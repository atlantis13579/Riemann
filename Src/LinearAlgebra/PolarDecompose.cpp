
#include "PolarDecompose.h"

#define TOLERANCE 0.01f

float PolarDecompose::OneNorm(const Matrix3d& F)
{
	float norm = 0.0f;
	for (int i = 0; i < 3; i++)
	{
		float columnAbsSum = fabsf(F(0, i)) + fabsf(F(1, i)) + fabsf(F(2, i));
		if (columnAbsSum > norm)
			norm = columnAbsSum;
	}
	return norm;
}

float PolarDecompose::InfNorm(const Matrix3d& F)
{
	float norm = 0.0;
	for (int i = 0; i < 3; i++)
	{
		float rowSum = fabsf(F(i, 0)) + fabsf(F(i, 1)) + fabsf(F(i, 2));
		if (rowSum > norm)
			norm = rowSum;
	}
	return norm;
}

void PolarDecompose::Compute(const Matrix3d& F, Matrix3d& R, Matrix3d& S)
{
	Matrix3d Mk;
	Matrix3d Ek;
	float det, M_oneNorm, M_infNorm, E_oneNorm;

	Mk = F.Transpose();

	M_oneNorm = OneNorm(Mk);
	M_infNorm = InfNorm(Mk);

	do
	{
		Vector3d v1 = Vector3d(Mk(2, 0), Mk(2, 1), Mk(2, 2));
		Vector3d v2 = Vector3d(Mk(0, 0), Mk(0, 1), Mk(0, 2));
		Vector3d v3 = Vector3d(Mk(1, 0), Mk(1, 1), Mk(1, 2));

		Vector3d r1 = Vector3d(Mk(1, 0), Mk(1, 1), Mk(1, 2)).Cross(v1);
		Vector3d r2 = Vector3d(Mk(2, 0), Mk(2, 1), Mk(2, 2)).Cross(v2);
		Vector3d r3 = Vector3d(Mk(0, 0), Mk(0, 1), Mk(0, 2)).Cross(v3);
		Matrix3d MadjTk(r1.x, r1.y, r1.z,
			r2.x, r2.y, r2.z,
			r3.x, r3.y, r3.z);


		det = Mk(0, 0) * MadjTk(0, 0) + Mk(0, 1) * MadjTk(0, 1) + Mk(0, 2) * MadjTk(0, 2);
		if (det == 0.0)
		{
			printf("Warning (polarDecomposition) : zero determinant encountered.\n");
			break;
		}

		float MadjT_one = OneNorm(MadjTk);
		float MadjT_inf = InfNorm(MadjTk);

		float gamma = sqrtf(sqrtf((MadjT_one * MadjT_inf) / (M_oneNorm * M_infNorm)) / fabsf(det));
		float g1 = gamma * 0.5f;
		float g2 = 0.5f / (gamma * det);

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
			{
				Ek(i, j) = Mk(i, j);
				Mk(i, j) = g1 * Mk(i, j) + g2 * MadjTk(i, j);
				Ek(i, j) -= Mk(i, j);
			}

		E_oneNorm = OneNorm(Ek);
		M_oneNorm = OneNorm(Mk);
		M_infNorm = InfNorm(Mk);
	} while (E_oneNorm > M_oneNorm * TOLERANCE);

	// Q = Mk^T 

	R = Mk.Transpose();
}

void PolarDecompose::ComputeFull(const Matrix3d& F, Matrix3d& R, Matrix3d& S)
{
	Matrix3d Mk;
	Matrix3d Ek;
	float det, M_oneNorm, M_infNorm, E_oneNorm;

	Mk = F.Transpose();

	M_oneNorm = OneNorm(Mk);
	M_infNorm = InfNorm(Mk);

	do
	{
		Vector3d v1 = Vector3d(Mk(2, 0), Mk(2, 1), Mk(2, 2));
		Vector3d v2 = Vector3d(Mk(0, 0), Mk(0, 1), Mk(0, 2));
		Vector3d v3 = Vector3d(Mk(1, 0), Mk(1, 1), Mk(1, 2));

		Vector3d r1 = Vector3d(Mk(1, 0), Mk(1, 1), Mk(1, 2)).Cross(v1);
		Vector3d r2 = Vector3d(Mk(2, 0), Mk(2, 1), Mk(2, 2)).Cross(v2);
		Vector3d r3 = Vector3d(Mk(0, 0), Mk(0, 1), Mk(0, 2)).Cross(v3);
		Matrix3d MadjTk(r1.x, r1.y, r1.z,
						r2.x, r2.y, r2.z,
						r3.x, r3.y, r3.z);


		det = Mk(0, 0) * MadjTk(0, 0) + Mk(0, 1) * MadjTk(0, 1) + Mk(0, 2) * MadjTk(0, 2);
		if (det == 0.0)
		{
			printf("Warning (polarDecomposition) : zero determinant encountered.\n");
			break;
		}

		float MadjT_one = OneNorm(MadjTk);
		float MadjT_inf = InfNorm(MadjTk);

		float gamma = sqrtf(sqrtf((MadjT_one * MadjT_inf) / (M_oneNorm * M_infNorm)) / fabsf(det));
		float g1 = gamma * 0.5f;
		float g2 = 0.5f / (gamma * det);

		for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
		{
			Ek(i, j) = Mk(i, j);
			Mk(i, j) = g1 * Mk(i, j) + g2 * MadjTk(i, j);
			Ek(i, j) -= Mk(i, j);
		}

		E_oneNorm = OneNorm(Ek);
		M_oneNorm = OneNorm(Mk);
		M_infNorm = InfNorm(Mk);
	} while (E_oneNorm > M_oneNorm * TOLERANCE);

	// Q = Mk^T 

	R = Mk.Transpose();

	for (int i = 0; i < 3; ++i)
	for (int j = 0; j < 3; ++j)
	{
		S(i, j) = 0.0;
		for (int k = 0; k < 3; ++k)
			S(i, j) += Mk(i, k) * F(k, j);
	}

	// S must be symmetric; enforce the symmetry
	for (int i = 0; i < 3; i++)
	for (int j = i; j < 3; j++)
		S(i, j) = S(j, i) = 0.5f * (S(i, j) + S(j, i));
}
