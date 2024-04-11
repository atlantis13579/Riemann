#pragma once

#include <vector>
#include "SparseConjugateGradientSolver.h"
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"
#include "../Core/StaticArray.h"

namespace Riemann
{
#define MAX_NODES 100000
#define MAX_TETS 100000

	struct Node
	{
		Vector3 position;
		float mass;
		Node() {}
		Node(const Vector3& _position, float _mass) : position(_position), mass(_mass) {}
	};

	struct TetrahedralNode
	{
		int node[4];
		float weight;
		float volume;
		Matrix3 Dm;
		Matrix3 Bm;
		TetrahedralNode() = default;
		TetrahedralNode(int _n1, int _n2, int _n3, int _n4, float _weight, float _volume, Matrix3& _Dm) : Dm(_Dm), Bm(_Dm.Inverse())
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
			m_nodes.Add(node);
			m_restpos.Add(node.position);
		}

		Matrix3 GenerateD(Vector3& v0, Vector3& v1, Vector3& v2, Vector3& v3)
		{
			return Matrix3(v0.x - v3.x, v1.x - v3.x, v2.x - v3.x,
				v0.y - v3.y, v1.y - v3.y, v2.y - v3.y,
				v0.z - v3.z, v1.z - v3.z, v2.z - v3.z);
		}


		void AddTetrahedral(int t1, int t2, int t3, int t4, float weight)
		{
			Matrix3 Dm = Matrix3(m_restpos[t1].x - m_restpos[t4].x, m_restpos[t2].x - m_restpos[t4].x, m_restpos[t3].x - m_restpos[t4].x,
				m_restpos[t1].y - m_restpos[t4].y, m_restpos[t2].y - m_restpos[t4].y, m_restpos[t3].y - m_restpos[t4].y,
				m_restpos[t1].z - m_restpos[t4].z, m_restpos[t2].z - m_restpos[t4].z, m_restpos[t3].z - m_restpos[t4].z);

			float vol = 1.0f / 6.0f * fabsf(Dm.Determinant());

			m_tets.Add(TetrahedralNode(t1, t2, t3, t4, weight, vol, Dm));
		}

		void AddTetrahedral(int t[4], float weight)
		{
			Matrix3 Dm = Matrix3(m_restpos[t[0]].x - m_restpos[t[3]].x, m_restpos[t[1]].x - m_restpos[t[3]].x, m_restpos[t[2]].x - m_restpos[t[3]].x,
				m_restpos[t[0]].y - m_restpos[t[3]].y, m_restpos[t[1]].y - m_restpos[t[3]].y, m_restpos[t[2]].y - m_restpos[t[3]].y,
				m_restpos[t[0]].z - m_restpos[t[3]].z, m_restpos[t[1]].z - m_restpos[t[3]].z, m_restpos[t[2]].z - m_restpos[t[3]].z);

			float vol = 1.0f / 6.0f * fabsf(Dm.Determinant());

			m_tets.Add(TetrahedralNode(t[0], t[1], t[2], t[3], weight, vol, Dm));
		}

		const Node& GetNode(int index) const
		{
			return m_nodes[index];
		}

		Node& GetNode(int index)
		{
			return m_nodes[index];
		}

		void SetPosition(int index, const Vector3& pos)
		{
			m_nodes[index].position = pos;
			m_restpos[index] = pos;
		}

		const Vector3& GetPosition(int index) const
		{
			return m_nodes[index].position;
		}

		const TetrahedralNode* GetTetrahedral(int index) const
		{
			if (index >= m_tets.GetSize())
				return nullptr;
			return &m_tets[index];
		}

		const Vector3& GetRestPosition(int index) const
		{
			return m_restpos[index];
		}

		int GetNumNodes() const
		{
			return m_nodes.GetSize();
		}

		int GetNumTetrahedrals() const
		{
			return m_tets.GetSize();
		}

		void GetTransforms(int tetindex, Matrix3& Ds, Matrix3& F, Matrix3& R, Matrix3& S) const;

		Matrix3 GetRotationMatrix(int tetindex, Matrix3& Ds) const;

		Vector3 GetCentroid(int i, int j) const;

	private:
		StaticArray<Node, MAX_NODES> m_nodes;
		StaticArray<Vector3, MAX_NODES> m_restpos;
		StaticArray<TetrahedralNode, MAX_TETS> m_tets;
	};


#define MAX_ITERATIONS 5
#define DAMPING 0.95f

	class ProjectiveDynamicsSolver
	{
	public:
		ProjectiveDynamicsSolver(TetrahedralMesh* _tetmesh);
		~ProjectiveDynamicsSolver();

		void Init();
		void Simulate();

		void SetPosition(int nodeindex, Vector3 pos);
		void SetVelocity(int nodeindex, Vector3 vel);
		void SetExtForce(int nodeindex, Vector3 force);
		void AddExtForce(int nodeindex, Vector3 force);
		void SetPinnedNode(int node, bool isConstrained) { m_pinned[node] = isConstrained; }

	private:
		void InitLaplacianMatrix();
		void InitMassMatrix();
		void InitSystemMatrix();

	private:
		TetrahedralMesh* m_tetmesh;
		float m_DT;
		int m_nNodes;
		int m_nTets;
		int m_nDof;

		float* m_Mass;
		float* m_Laplacian;
		float* m_Sys;
		float* m_invSys;
		float* m_qn, * m_q, * m_sn, * m_v, * m_fext, * m_b;
		bool* m_pinned;
		SparseConjugateGradientSolver m_solver;
	};

}