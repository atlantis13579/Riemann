
#include "TetrahedralMesh.h"
#include "../LinearSystem/PolarDecomposition.h"

TetrahedralMesh::TetrahedralMesh()
{
}

TetrahedralMesh::~TetrahedralMesh()
{
}

void TetrahedralMesh::GetTransforms(int tetindex, Matrix3& Ds, Matrix3& F, Matrix3& R, Matrix3& S) const
{
	const TetrahedralNode& t = m_tets[tetindex];

	// int i0 = t.node[0];
	// int i1 = t.node[1];
	// int i2 = t.node[2];
	// int i3 = t.node[3];

	const Matrix3 DmInv = t.Dm.Inverse();

	F = Ds * DmInv;

	PolarDecompose::ComputeFull(F, R, S);
}

Matrix3 TetrahedralMesh::GetRotationMatrix(int tetindex, Matrix3& Ds) const
{
	const TetrahedralNode& t = m_tets[tetindex];
	Matrix3 F = Ds * t.Bm;
	Matrix3 R, S;

	PolarDecompose::Compute(F, R, S);

	if(R.Determinant() < 0)
		R = Matrix3(-1, 0, 0,
					 0, -1, 0,
					 0, 0, -1) * R;

	return R;
}

Vector3 TetrahedralMesh::GetCentroid(int index1, int index2) const
{
	int count = 0;
	Vector3 curr = Vector3(0.0f, 0.0f, 0.0f);

	for (int i = index1; i < index2; ++i)
	{
		const TetrahedralNode* t = GetTetrahedral(i);
		if (t == nullptr)
			continue;

		int i0 = t->node[0];
		int i1 = t->node[1];
		int i2 = t->node[2];
		int i3 = t->node[3];

		const Node& n0 = GetNode(i0);
		const Node& n1 = GetNode(i1);
		const Node& n2 = GetNode(i2);
		const Node& n3 = GetNode(i3);

		curr.x += (n0.position.x + n1.position.x + n2.position.x + n3.position.x) * 0.25f;
		curr.y += (n0.position.y + n1.position.y + n2.position.y + n3.position.y) * 0.25f;
		curr.z += (n0.position.z + n1.position.z + n2.position.z + n3.position.z) * 0.25f;

		++count;
	}

	if (count > 0)
	{
		curr.x /= count;
		curr.y /= count;
		curr.z /= count;
	}

	return curr;
}
