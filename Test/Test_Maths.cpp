
#include "Test.h"

#include "../Src/Maths/Tensor.h"
#include "../Src/Maths/Box3d.h"
#include "../Src/Maths/Maths.h"
#include "../Src/Maths/Matrix2.h"
#include "../Src/Maths/Transform.h"
#include "../Src/Maths/Float16.h"
#include "../Src/Geometry/Spline.h"

void TestBasicMath()
{
	printf("Running TestBasicMath\n");
	Matrix3d mat1, mat2;
	Quaternion q1, q2;
	Vector3d v1, v2, s1, s2;

	mat1.FromAxisAngle(Vector3d::UnitX(), ToRadian(45.0f));
	q1.FromRotationAxis(Vector3d::UnitX(), ToRadian(45.0f));
	mat2 = q1.ToRotationMatrix();

	float dist = (mat1 - mat2).L1Norm();
	EXPECT1(dist < 0.0001f, dist);

	v1 = Vector3d(1.0f, 2.0f, 3.0f);

	mat1.FromAxisAngle(v1.Unit(), ToRadian(30.0f));
	q1.FromRotationMatrix(mat1);
	mat2 = q1.ToRotationMatrix();

	dist = (mat1 - mat2).L1Norm();
	EXPECT1(dist < 0.0001f, dist);

	Transform trans;
	trans.SetRotation(q1);
	trans.SetTranslation(v1);
	Transform::WorldMatrixToTR(trans.GetWorldMatrix(), v2, q2);

	EXPECT1((v1 - v2).SquareLength() < 0.000001f, (v1 - v2).SquareLength());
	EXPECT1((q1 - q2).SquareLength() < 0.000001f, (q1 - q2).SquareLength());

	s1 = Vector3d(2.0f, 1.0f, 3.0f);
	trans.SetScale(s1);
	
	Transform::WorldMatrixToTRS(trans.GetWorldMatrix(), v2, q2, s2);

	EXPECT((v1 - v2).SquareLength() < 0.0001f);
	EXPECT((q1 - q2).SquareLength() < 0.0001f);
	EXPECT((s1 - s2).SquareLength() < 0.0001f);

	trans.SetScale(Vector3d::One());
	Transform::WorldMatrixToTRS(trans.GetWorldMatrix() * trans.GetWorldMatrix(), v2, q2, s2);

	EXPECT((v1 + v1 - v2).SquareLength() < 0.00001f);
	EXPECT((q1 * q1 - q2).SquareLength() < 0.001f);

	return;
}

void TestTensor()
{
	printf("Running TestTensor\n");
	Tensor<float, 4> t(10, 20, 30, 40);
	t(0, 0, 0, 0) = 1.0f;
	t(9, 19, 29, 39) = 2.0f;

	auto t2 = t[9][19][29];

	EXPECT(t2[39] == 2.0f);
}

void TestFloat16()
{
	// float	x1 = Float16::FromFloat32(1.0f).ToFloat();
	// float	x2 = Float16::FromFloat32(1.111111f).ToFloat();
	printf("Running TestFloat16\n");
	std::vector<Vector3d> cc;
	std::vector<float> dd, d2;
	for (int i = 0; i <= 20; ++i)
	{
		float t = 1.0f * i / 20;
		Vector3d p0 = Vector3d(0, 0, 0);
		Vector3d p1 = Vector3d(1, 0, 1);
		Vector3d t1 = CubicHermite::Calculate(p0, p1, Vector3d(1, 0, 1).Unit(), Vector3d(1, 0, 1).Unit(), t);
		Vector3d t2 = LinearInterp(p0, p1, t);
		cc.push_back(t1);
		if (i != 0)
		{
			dd.push_back((t1 - cc[i - 1]).Length());
			d2.push_back((t1 - t2).Length());
		}
	}
	return;
}


void TestMaths()
{
	TestBasicMath();
	TestTensor();
	TestFloat16();
}
