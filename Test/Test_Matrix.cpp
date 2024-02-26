
#include "Test.h"

#include "../Src/Maths/Matrix3.h"
#include "../Src/Maths/MatrixMxN.h"
#include "../Src/LinearSystem/DenseMatrix.h"
#include "../Src/LinearSystem/JacobiIteration.h"
#include "../Src/LinearSystem/GaussSeidelIteration.h"
#include "../Src/LinearSystem/ProjectedGaussSeidel.h"
#include "../Src/LinearSystem/LUFactorization.h"
#include "../Src/LinearSystem/SingularValueDecomposition.h"
#include "../Src/LinearSystem/QRDecomposition.h"
#include "../Src/LinearSystem/MoorePenrosePseudoInverse.h"
#include "../Src/LinearSystem/SparseMatrix.h"
#include "../Src/LinearSystem/SparseVector.h"
#include "../Src/LinearSystem/ConjugateGradientSolver.h"

// using namespace Riemann;

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

	VectorN<10>	Vec = M * M.GetCol(0);
	float dp = M.GetRow(0).Dot(M.GetCol(0));
	MatrixN<10>	Mat = M * M.Transpose().Transpose();

	EXPECT(dp == Vec[0]);
	EXPECT(dp == Mat[0][0]);
	return;
}

void TestSparse()
{
	printf("Running TestSparse\n");
	float aaa[] = {0, 1, 0, 1, 0, 0, 0};
	
	TSparseVector<float> x(aaa, 7);
	
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
	JacobiIteration_CPU<float>::Solve(A.GetData(), B.GetData(), X.GetSize(), X.GetData(), 100);
	DenseVector Y = A * X;
	EXPECT(Y.FuzzyEqual(B, 1e-2f));

	X.LoadZero();
	GaussSeidelIteration<float>::Solve(A.GetData(), B.GetData(), X.GetSize(), X.GetData(), 100);
	Y = A * X;
	EXPECT(Y.FuzzyEqual(B, 1e-2f));

	X.LoadZero();
	ConjugateGradientSolver<float>::Solve(A.GetData(), B.GetData(), X.GetSize(), X.GetData(), 100);
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
	ProjectedGaussSeidel<float>::Solve(A.GetData(), B.GetData(), X.GetSize(), X.GetData(), 50, 1e-10f);
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
	ProjectedGaussSeidel<float>::Solve(A.GetData(), B.GetData(), X.GetSize(), X.GetData(), X1.GetData(), X2.GetData(), 50);
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

void TestSVD()
{
	printf("Running TestSVD\n");
	
	Matrix3 A;
	A[0][0] = 12.0f;
	A[0][1] = 3.0f;
	A[0][2] = 5.0f;
	A(1, 0) = -1.5f;
	A(1, 1) = 6.5f;
	A(1, 2) = 11.5f;
	A[2][0] = 10.0f;
	A[2][1] = -10.0f;
	A[2][2] = 160.1f;
	
	Vector3 s1;
	Matrix3 u1, v1;
	A.SingularValueDecompose(u1, s1, v1);
	Matrix3 AA = u1 * Matrix3(s1) * v1.Transpose();
	
	Vector3 s2;
	Matrix3 u2, v2;
	SingularValueDecomposition<float>()(A.Data(), 3, 3, u2.Data(), s2.Data(), v2.Data());
	Matrix3 AA2 = u2 * Matrix3(s2) * v2.Transpose();
	
	float me1 = sqrtf((AA - A).L2Norm() / 9);
	float me2 = sqrtf((AA2 - A).L2Norm() / 9);

	EXPECT(DenseMatrix(u1.Data(), 3, 3).IsOrthogonal(0.2f));	// the algorithm is not good
	EXPECT(DenseMatrix(v1.Data(), 3, 3).IsOrthogonal(0.2f));	// the algorithm is not good
	EXPECT(DenseMatrix(u2.Data(), 3, 3).IsOrthogonal());
	EXPECT(DenseMatrix(v2.Data(), 3, 3).IsOrthogonal());

	EXPECT(me1 < 3.0f);		// the algorithm is not good
	EXPECT(me2 < 0.1f);
	
	return;
}

void TestPseudoInverse()
{
	printf("Running TestPseudoInverse\n");
	
	TDenseMatrix<float> A(2);
	A[0][0] = 12.0f;
	A[0][1] = 3.0f;
	A[1][0] = 5.0f;
	A[1][1] = 1.0f;
	
	TDenseMatrix<float> InvA = A.Inverse();
	TDenseMatrix<float> pInvA = A.PseudoInverse();
	TDenseMatrix<float> AA = A * pInvA * A;
	
	float *p1 = A.GetData(), *p2 = AA.GetData();
	for (int i = 0; i < 4; ++i)
	{
		EXPECT(fabsf(p1[i] - p2[i]) < 0.01f);
	}
	
	A.LoadIdentity();
	A[1][1] = 0.0f;
	
	pInvA = A.PseudoInverse();
	AA = A * pInvA * A;
	
	p1 = A.GetData();
	p2 = AA.GetData();
	for (int i = 0; i < 4; ++i)
	{
		EXPECT(fabsf(p1[i] - p2[i]) < 0.01f);
	}
	
	return;
}

void TestPolarDecomp()
{
	printf("Running TestPolarDecomp\n");
	
	Matrix3 A;
	A[0][0] = 12.0f;
	A[0][1] = 3.0f;
	A[0][2] = 5.0f;
	A(1, 0) = -1.5f;
	A(1, 1) = 6.5f;
	A(1, 2) = 11.5f;
	A[2][0] = 10.0f;
	A[2][1] = -10.0f;
	A[2][2] = 160.1f;
	
	Matrix3 u1, p1;
	A.PolarDecomposeUP(u1, p1);
	
	Matrix3 AA = u1 * p1;
	float norm = (AA - A).L1Norm() / 9;
	EXPECT(norm < 0.001f);
	
	return;
}

void TestQR()
{
	printf("Running TestQR\n");

	DenseMatrix A(3, 3);
	A[0][0] = 12.0f;
	A[0][1] = -51.0f;
	A[0][2] = 4.0f;
	A[1][0] = 6.0f;
	A[1][1] = 167.0f;
	A[1][2] = -68.0f;
	A[2][0] = -4.0f;
	A[2][1] = 24.0f;
	A[2][2] = -41.1f;

	DenseMatrix q, r;
	A.QRDecompose(q, r);
	EXPECT(q.IsOrthogonal(0.00001f));
	EXPECT(r.IsUpperTriangle(0.00001f));

	DenseMatrix A2 = q * r;
	EXPECT(A.FuzzyEqual(A2, 0.01f));
	return;
}

void TestEigen()
{
	printf("Running TestEigen\n");

	TDenseMatrix<float> A(4, 4);
	A[0][0] = 54.0f;
	A[0][1] = 30.0f;
	A[0][2] = 49.0f;
	A[0][3] = 28.0f;

	A[1][0] = 30.0f;
	A[1][1] = 50.0f;
	A[1][2] = 8.0f;
	A[1][3] = 44.0f;

	A[2][0] = 49.0f;
	A[2][1] = 8.0f;
	A[2][2] = 46.0f;
	A[2][3] = 16.0f;

	A[3][0] = 28.0f;
	A[3][1] = 44.0f;
	A[3][2] = 16.0f;
	A[3][3] = 12.0f;

	EXPECT(A.IsSymmetric());

	TDenseVector<float> EigenValues;
	TDenseMatrix<float> EigenVectors;
	A.EigenDecompose(EigenValues, EigenVectors);

	EXPECT(fabsf(EigenValues[0] - 132.0f) < 1.0f);

	for (int i = 0; i < EigenVectors.GetCols(); ++i)
	{
		TDenseVector<float> v = EigenVectors.GetCol(i);
		TDenseVector<float> Av = A * v;
		TDenseVector<float> Ev = EigenValues[i] * v;
		EXPECT(Av.FuzzyEqual(Ev, 0.001f));
	}
	
	EXPECT((A * EigenVectors).FuzzyEqual(EigenVectors * TDenseMatrix<float>(EigenValues), 0.001f));

	return;
}

void TestCholesky()
{
	printf("Running TestCholesky\n");
	TDenseMatrix<float> A(3, 3), L;
	A[0][0] = 1.0f;
	A[0][1] = 0.0f;
	A[0][2] = 1.0f;
	A[1][0] = 0.0f;
	A[1][1] = 2.0f;
	A[1][2] = 0.0f;
	A[2][0] = 1.0f;
	A[2][1] = 0.0f;
	A[2][2] = 3.0f;
	
	EXPECT(A.IsSymmetric());
	EXPECT(A.CholeskyDecompose(L));
	EXPECT(L.IsLowerTriangle());
	EXPECT(A.FuzzyEqual(L * L.Transpose()));
	
	TDenseVector<float>	B(3), X;
	B[0] = 2.0f;
	B[1] = 2.0f;
	B[2] = 4.0f;
	
	EXPECT(A.SolveCholesky(B, X));
	EXPECT(FuzzyEqual(X.Dot(X), 3.0f));
	
	return;
}

void TestMatrix()
{
	TestMatrix1();
	TestMatrix2();
	TestSparse();
	TestSolve();
	TestPGS();
	TestSVD();
	TestPseudoInverse();
	TestPolarDecomp();
	TestQR();
	TestEigen();
	TestCholesky();
	return;
}
