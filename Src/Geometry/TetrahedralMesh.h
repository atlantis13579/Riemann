#pragma once

#include "../Core/StaticArray.h"
#include "../Maths/Vector3d.h"
#include "../Maths/Matrix3d.h"

#define MAX_NODES 100000
#define MAX_TETS 100000

struct Node
{
	Vector3d position;
	float mass;
	Node() {}
	Node(const Vector3d& _position, float _mass) : position(_position), mass(_mass) {}
};

struct TetrahedralNode
{
	int node[4];
	float weight;
	float volume;
	Matrix3d Dm;
	Matrix3d Bm;
	TetrahedralNode() = default;
	TetrahedralNode(int _n1, int _n2, int _n3, int _n4, float _weight, float _volume, Matrix3d& _Dm) : Dm(_Dm), Bm(_Dm.Inverse())
	{
		weight = _weight;
		volume = _volume;
		node[0] = _n1;
		node[1] = _n2;
		node[2] = _n3;
		node[3] = _n4;
	}
};

class TetrahedralMesh
{
public:
	TetrahedralMesh();
	~TetrahedralMesh();

	void AddNode(const Node& node)
	{
		m_nodes.Push(node);
		m_restpos.Push(node.position);
	}

	Matrix3d GenerateD(Vector3d& v0, Vector3d& v1, Vector3d& v2, Vector3d& v3)
	{
		return Matrix3d(v0.x - v3.x, v1.x - v3.x, v2.x - v3.x,
						v0.y - v3.y, v1.y - v3.y, v2.y - v3.y,
						v0.z - v3.z, v1.z - v3.z, v2.z - v3.z);
	}


	void AddTetrahedral(int t1, int t2, int t3, int t4, float weight)
	{
		Matrix3d Dm = Matrix3d(m_restpos[t1].x - m_restpos[t4].x, m_restpos[t2].x - m_restpos[t4].x, m_restpos[t3].x - m_restpos[t4].x,
							   m_restpos[t1].y - m_restpos[t4].y, m_restpos[t2].y - m_restpos[t4].y, m_restpos[t3].y - m_restpos[t4].y,
							   m_restpos[t1].z - m_restpos[t4].z, m_restpos[t2].z - m_restpos[t4].z, m_restpos[t3].z - m_restpos[t4].z);

		float vol = 1.0f / 6.0f * fabsf(Dm.Determinant());

		m_tets.Push(TetrahedralNode(t1, t2, t3, t4, weight,vol,Dm));
	}

	void AddTetrahedral(int t[4], float weight)
	{
		Matrix3d Dm = Matrix3d(m_restpos[t[0]].x - m_restpos[t[3]].x, m_restpos[t[1]].x - m_restpos[t[3]].x, m_restpos[t[2]].x - m_restpos[t[3]].x,
							   m_restpos[t[0]].y - m_restpos[t[3]].y, m_restpos[t[1]].y - m_restpos[t[3]].y, m_restpos[t[2]].y - m_restpos[t[3]].y,
							   m_restpos[t[0]].z - m_restpos[t[3]].z, m_restpos[t[1]].z - m_restpos[t[3]].z, m_restpos[t[2]].z - m_restpos[t[3]].z);

		float vol = 1.0f / 6.0f * fabsf(Dm.Determinant());

		m_tets.Push(TetrahedralNode(t[0], t[1], t[2], t[3], weight, vol, Dm));
	}

	const Node& GetNode(int index) const
	{
		return m_nodes[index]; 
	}

	Node& GetNode(int index)
	{
		return m_nodes[index];
	}

	void SetPosition(int index, const Vector3d& pos)
	{
		m_nodes[index].position = pos;
		m_restpos[index] = pos;
	}

	const Vector3d& GetPosition(int index) const
	{
		return m_nodes[index].position;
	}

	const TetrahedralNode* GetTetrahedral(int index) const
	{
		if (index >= m_tets.Size())
			return nullptr;
		return &m_tets[index];
	}

	const Vector3d& GetRestPosition(int index) const
	{
		return m_restpos[index];
	}

	int GetNumNodes() const
	{
		return m_nodes.Size();
	}

	int GetNumTetrahedrals() const
	{
		return m_tets.Size();
	}

	void GetTransforms(int tetindex, Matrix3d& Ds, Matrix3d& F, Matrix3d& R, Matrix3d& S) const;

	Matrix3d GetRotationMatrix(int tetindex, Matrix3d& Ds) const;

	Vector3d GetCentroid(int i, int j) const;

private:
	StaticArray<Node, MAX_NODES> m_nodes;
	StaticArray<Vector3d, MAX_NODES> m_restpos;
	StaticArray<TetrahedralNode, MAX_TETS> m_tets;
};

