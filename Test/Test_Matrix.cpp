
#include "Test.h"

#include "../Src/Maths/MatrixMxN.h"
#include "../Src/LinearSystem/DenseMatrix.h"
#include "../Src/LinearSystem/JacobiIteration_CPU.h"
#include "../Src/LinearSystem/GaussSeidelIteration_CPU.h"
#include "../Src/LinearSystem/LUFactorization.h"

void TestMatrix1()
{
	printf("Running TestMatrix\n");

	TDenseMatrix<float> M(10);
	M[0][0] = 2.0f;
	M[0][1] = 3.0f;
	M(1, 0) = -1.5f;
	M(1, 1) = 9.0f;
	for (int i = 0; i < M.GetRows(); ++i)
	for (int j = 0; j < M.GetRows(); ++j)
		M(i, j) = 1.0f * rand() / RAND_MAX;

	TDenseMatrix<float> invM = (M+M).Inverse();
	TDenseMatrix<float> Id = (M+M) * invM;
	bool IsId = Id.IsIdentity(1e-5f);
	EXPECT(IsId);

	float det = (2.0f * M).Determinant();
	float detI = invM.Determinant();

	EXPECT(fabsf(det * detI - 0.9999f) < 0.001f);

	TDenseVector<float>	Vec = M * M.GetCol(0);
	float dp = M.GetRow(0).Dot(M.GetCol(0));
	TDenseMatrix<float>	Mat = M * M.Transpose().Transpose();

	EXPECT(dp == Vec[0]);
	EXPECT(dp == Mat[0][0]);
	return;
}

void TestMatrix2()
{
	printf("Running TestMatrix2\n");

	SquareMatrix<10> M;
	M[0][0] = 2.0f;
	M[0][1] = 3.0f;
	M(1, 0) = -1.5f;
	M(1, 1) = 9.0f;
	for (int i = 0; i < 10; ++i)
	for (int j = 0; j < 10; ++j)
		M(i, j) = 1.0f * rand() / RAND_MAX;

	SquareMatrix<10> invM = (M + M).Inverse();
	SquareMatrix<10> Id = (M + M) * invM;
	bool IsId = Id.IsIdentity();
	EXPECT(IsId);

	float det = (2.0f * M).Determinant();
	float detI = invM.Determinant();

	EXPECT(fabsf(det * detI - 0.9999f) < 0.001f);

	VectorNd<10>	Vec = M * M.GetCol(0);
	float dp = M.GetRow(0).Dot(M.GetCol(0));
	SquareMatrix<10>	Mat = M * M.Transpose().Transpose();

	EXPECT(dp == Vec[0]);
	EXPECT(dp == Mat[0][0]);
	return;
}

void TestMatrix()
{
	TestMatrix1();
	TestMatrix2();
}
