
#include "Test.h"

#include "../Src/Maths/Tensor.h"
#include "../Src/Maths/Box3.h"
#include "../Src/Maths/Maths.h"
#include "../Src/Maths/Matrix2.h"
#include "../Src/Maths/Transform.h"
#include "../Src/Maths/Float16.h"
#include "../Src/Maths/Frame3.h"
#include "../Src/Geometry/Spline.h"

void TestMat3()
{
	Matrix3 mat, m1, m2;
	m1.LoadRotateY(1.0f);
	m2.LoadRotateZ(2.0f);
	mat = m1 * m2;
	Matrix3 invMat = mat.Inverse();
	Matrix3 transpose = mat.Transpose();
	EXPECT(fabsf((invMat - transpose).L2Norm()) < 1e-6f);
	Vector3 v(1.0f, 20.0f, -5.0f);
	Vector3 v1 = invMat * v, v2 = v * mat, v3 = transpose * v;
	EXPECT((v1 - v2).SquareLength() < 1e-6f);
	EXPECT((v1 - v3).SquareLength() < 1e-6f);
	return;
}

void TestBasicMath()
{
	printf("Running TestBasicMath\n");
	Matrix3 mat1, mat2;
	Quaternion q1, q2;
	Vector3 v1, v2, s1, s2;

	mat1.FromAxisAngle(Vector3::UnitX(), Maths::ToRadian(45.0f));
	q1.FromRotationAxis(Vector3::UnitX(), Maths::ToRadian(45.0f));
	mat2 = q1.ToRotationMatrix3();

	float dist = (mat1 - mat2).L1Norm();
	EXPECT1(dist < 0.0001f, dist);

	v1 = Vector3(1.0f, 2.0f, 3.0f);

	mat1.FromAxisAngle(v1.Unit(), Maths::ToRadian(30.0f));
	q1.FromRotationMatrix3(mat1);
	mat2 = q1.ToRotationMatrix3();

	dist = (mat1 - mat2).L1Norm();
	EXPECT1(dist < 0.0001f, dist);

	Transform3 trans;
	trans.SetRotation(q1);
	trans.SetTranslation(v1);
	Transform3::WorldMatrixToTR(trans.GetWorldMatrix(), v2, q2);

	EXPECT1((v1 - v2).SquareLength() < 0.000001f, (v1 - v2).SquareLength());
	EXPECT1((q1 - q2).SquareLength() < 0.000001f, (q1 - q2).SquareLength());

	s1 = Vector3(2.0f, 1.0f, 3.0f);
	trans.SetScale(s1);
	
	Transform3::WorldMatrixToTRS(trans.GetWorldMatrix(), v2, q2, s2);

	EXPECT((v1 - v2).SquareLength() < 0.0001f);
	EXPECT((q1 - q2).SquareLength() < 0.0001f);
	EXPECT((s1 - s2).SquareLength() < 0.0001f);

	trans.SetScale(Vector3::One());
	Transform3::WorldMatrixToTRS(trans.GetWorldMatrix() * trans.GetWorldMatrix(), v2, q2, s2);

	EXPECT((v1 + v1 - v2).SquareLength() < 0.00001f);
	EXPECT((q1 * q1 - q2).SquareLength() < 0.001f);

	return;
}

void TestTensor()
{
	printf("Running TestTensor\n");
	Maths::Tensor<float, 4> t(10, 20, 30, 40);
	t(0, 0, 0, 0) = 1.0f;
	t(9, 19, 29, 39) = 2.0f;

	auto t2 = t[9][19][29];

	EXPECT(t2[39] == 2.0f);
	EXPECT(t.GetRank() == 4);
}

void TestFloat16()
{
	// float	x1 = Float16::FromFloat32(1.0f).ToFloat();
	// float	x2 = Float16::FromFloat32(1.111111f).ToFloat();
	printf("Running TestFloat16\n");
	std::vector<Vector3> cc;
	std::vector<float> dd, d2;
	for (int i = 0; i <= 20; ++i)
	{
		float t = 1.0f * i / 20;
		Vector3 p0 = Vector3(0, 0, 0);
		Vector3 p1 = Vector3(1, 0, 1);
		Vector3 t1 = Geometry::CubicHermite::Calculate(p0, p1, Vector3(1, 0, 1).Unit(), Vector3(1, 0, 1).Unit(), t);
		Vector3 t2 = LinearInterp(p0, p1, t);
		cc.push_back(t1);
		if (i != 0)
		{
			dd.push_back((t1 - cc[i - 1]).Length());
			d2.push_back((t1 - t2).Length());
		}
	}
	return;
}

void TestFrame3()
{
	Vector3 Origin(10.0f, 10.0f, 10.0f);
	Vector3 Normal(0.0f, 1.0f, 0.0f);
	Maths::Frame3 PlaneFrame(Origin, Normal, true);

	Vector3 X, Y, Z;
	PlaneFrame.GetAxes(X, Y, Z);

	bool right_handed = PlaneFrame.IsRightHanded();
	return;
}


void TestMaths()
{
	TestMat3();
	TestBasicMath();
	TestTensor();
	TestFloat16();
	TestFrame3();
}
