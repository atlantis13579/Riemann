
#include "Test.h"

#include "../Src/Maths/MatrixMxN.h"
#include "../Src/LinearSystem/DenseMatrix.h"
#include "../Src/LinearSystem/JacobiIteration.h"
#include "../Src/LinearSystem/GaussSeidelIteration.h"
#include "../Src/LinearSystem/ProjectedGaussSeidel.h"
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

	MatrixN<10> M;
	M[0][0] = 2.0f;
	M[0][1] = 3.0f;
	M(1, 0) = -1.5f;
	M(1, 1) = 9.0f;
	for (int i = 0; i < 10; ++i)
	for (int j = 0; j < 10; ++j)
		M(i, j) = 1.0f * rand() / RAND_MAX;

	MatrixN<10> invM = (M + M).Inverse();
	MatrixN<10> Id = (M + M) * invM;
	bool IsId = Id.IsIdentity(1e-4f);
	EXPECT(IsId);

	float det = (2.0f * M).Determinant();
	float detI = invM.Determinant();

	EXPECT(fabsf(det * detI - 0.9999f) < 0.001f);

	VectorNd<10>	Vec = M * M.GetCol(0);
	float dp = M.GetRow(0).Dot(M.GetCol(0));
	MatrixN<10>	Mat = M * M.Transpose().Transpose();

	EXPECT(dp == Vec[0]);
	EXPECT(dp == Mat[0][0]);
	return;
}

void TestSolve()
{
	printf("Running TestSolve\n");

	DenseMatrix A(3);
	A[0][0] = 12.0f;
	A[0][1] = 3.0f;
	A[0][2] = 5.0f;
	A(1, 0) = -1.5f;
	A(1, 1) = 6.5f;
	A(1, 2) = 11.5f;
	A[2][0] = 10.0f;
	A[2][1] = -10.0f;
	A[2][2] = 160.1f;

	DenseVector B(3);
	B[0] = -10.0f;
	B[1] = -7.0f;
	B[2] = 122.0f;

	DenseVector X(3);

	X.LoadZero();
	JacobiIteration_CPU<float> J;
	J.Solve(A.GetData(), B.GetData(), X.GetSize(), X.GetData(), 100);
	DenseVector Y = A * X;
	EXPECT(Y.FuzzyEqual(B, 1e-2f));

	GaussSeidelIteration_CPU<float>::Solve(A.GetData(), B.GetData(), X.GetSize(), X.GetData(), 100);
	Y = A * X;
	EXPECT(Y.FuzzyEqual(B, 1e-2f));

	return;
}

void TestPGS()
{
	printf("Running TestPGS\n");

	DenseMatrix A(3);
	A[0][0] = 12.0f;
	A[0][1] = 3.0f;
	A[0][2] = 5.0f;
	A(1, 0) = -1.5f;
	A(1, 1) = 6.5f;
	A(1, 2) = 11.5f;
	A[2][0] = 10.0f;
	A[2][1] = -10.0f;
	A[2][2] = 160.1f;

	DenseVector B(3);
	B[0] = -10.0f;
	B[1] = -7.0f;
	B[2] = 122.0f;

	DenseVector X(3);

	X.LoadZero();
	ProjectedGaussSeidel_CPU::Solve(A.GetData(), B.GetData(), X.GetSize(), X.GetData(), 50, 1e-10f);
	DenseVector Y = A * X + B;
	float dp = X.Dot(Y);
	EXPECT(X >= -0.00001f);
	EXPECT(Y >= -0.00001f);
	EXPECT(dp <= 0.00001f);

	DenseVector X1(3), X2(3);
	X1[0] = 0.0f; X1[1] = -15.0f; X1[2] = 1.0f;
	X2[0] = 1.0f; X2[1] = 10.0f; X2[2] = 3.0f;

	X.LoadZero();
	B[0] = 10.5f;
	B[1] = 17.0f;
	B[2] = 242.0f;
	ProjectedGaussSeidel_CPU::Solve(A.GetData(), B.GetData(), X.GetSize(), X.GetData(), X1.GetData(), X2.GetData(), 50);
	Y = A * X + B;
	dp = Y.Dot(X);
	EXPECT(X >= X1);
	EXPECT(X <= X2);
	for (int i = 0; i < X.GetSize(); ++i)
	{
		EXPECT((X[i] == X1[i] && Y[i] > 0) || (X[i] == X2[i] && Y[i] < 0) || (X1[i] < X[i] && X[i] < X2[i] && Y[i] == 0));
	}

	return;
}

void TestMatrix()
{
	TestMatrix1();
	TestMatrix2();
	TestSolve();
	TestPGS();
}
