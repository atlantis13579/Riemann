#pragma once

#include <vector>
#include "../Geometry/TetrahedralMesh.h"
#include "SparseConjugateGradientSolver.h"

namespace Riemann
{
#define MAX_ITERATIONS 5
#define DAMPING 0.95f

	class ProjectiveDynamicsSolver
	{
	public:
		ProjectiveDynamicsSolver(Geometry::TetrahedralMesh* _tetmesh);
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
		Geometry::TetrahedralMesh* m_tetmesh;
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