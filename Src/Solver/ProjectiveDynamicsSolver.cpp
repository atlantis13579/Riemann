#include "ProjectiveDynamicsSolver.h"

ProjectiveDynamicsSolver::ProjectiveDynamicsSolver(TetrahedralMesh* _tetmesh) : m_tetmesh(_tetmesh)
{
	m_DT = (1.0f / 60.0f);
	m_nNodes = m_tetmesh->GetNumNodes();
	m_nTets = m_tetmesh->GetNumTetrahedrals();
	m_nDof = m_nNodes * 3;

	m_Mass = new float[m_nNodes*m_nNodes];
	m_Laplacian = new float[m_nNodes*m_nNodes];
	m_Sys = new float[m_nNodes*m_nNodes];
	m_invSys = new float[m_nNodes*m_nNodes];

	memset(m_Mass, 0, sizeof(float)*m_nNodes*m_nNodes);
	memset(m_Laplacian, 0, sizeof(float)*m_nNodes*m_nNodes);
	memset(m_Sys, 0, sizeof(float)*m_nNodes*m_nNodes);
	memset(m_invSys, 0, sizeof(float)*m_nNodes*m_nNodes);

	m_qn = new float[m_nDof];
	m_q = new float[m_nDof];
	m_v = new float[m_nDof];
	m_sn = new float[m_nDof];
	m_fext = new float[m_nDof];
	m_b = new float[m_nDof];

	memset(m_qn, 0, sizeof(float)*m_nDof);
	memset(m_q, 0, sizeof(float)*m_nDof);
	memset(m_v, 0, sizeof(float)*m_nDof);
	memset(m_sn, 0, sizeof(float)*m_nDof);
	memset(m_fext, 0, sizeof(float)*m_nDof);
	memset(m_b, 0, sizeof(float)*m_nDof);

	m_pinned = new bool[m_nNodes];
	memset(m_pinned, 0, sizeof(bool)*m_nNodes);
}

ProjectiveDynamicsSolver::~ProjectiveDynamicsSolver()
{
	delete []m_Mass;
	delete []m_Laplacian;
	delete []m_Sys;
	delete []m_invSys;

	delete []m_qn;
	delete []m_q;
	delete []m_v;
	delete []m_sn;
	delete []m_fext;
	delete []m_b;

	delete []m_pinned;
}

void ProjectiveDynamicsSolver::InitLaplacianMatrix()
{
	for (int i = 0; i < m_nTets; ++i)
	{
		const Tetrahedral* t = m_tetmesh->GetTetrahedral(i);

		int i0 = t->node[0];
		int i1 = t->node[1];
		int i2 = t->node[2];
		int i3 = t->node[3];

		float tetweight = t->weight * t->volume;

		m_Laplacian[i0*m_nNodes + i0] += 1 * tetweight;
		m_Laplacian[i1*m_nNodes + i1] += 1 * tetweight;
		m_Laplacian[i2*m_nNodes + i2] += 1 * tetweight;
		m_Laplacian[i3*m_nNodes + i3] += 3 * tetweight;

		m_Laplacian[i0*m_nNodes + i3] += -1 * tetweight;
		m_Laplacian[i1*m_nNodes + i3] += -1 * tetweight;
		m_Laplacian[i2*m_nNodes + i3] += -1 * tetweight;

		m_Laplacian[i3*m_nNodes + i0] += -1 * tetweight;
		m_Laplacian[i3*m_nNodes + i1] += -1 * tetweight;
		m_Laplacian[i3*m_nNodes + i2] += -1 * tetweight;
	}
}

void ProjectiveDynamicsSolver::InitMassMatrix()
{
	for (int i = 0; i < m_nNodes; i++)
	{
		float mass = m_tetmesh->GetNode(i).mass;
		m_Mass[i*m_nNodes + i] = mass;
	}
}

void ProjectiveDynamicsSolver::InitSystemMatrix()
{
	for (int i = 0; i < m_nNodes * m_nNodes; ++i)
	{
		m_Sys[i] = m_Mass[i] / (m_DT * m_DT) + m_Laplacian[i];
	}
}

void ProjectiveDynamicsSolver::Init()
{
	InitLaplacianMatrix();
	InitMassMatrix();
	InitSystemMatrix();
	m_solver.InitSparseSolverCompressed3x3(m_Sys, m_nNodes);

	int count = 0;
	for (int i = 0; i < m_nNodes; i++)
	{
		int rowcount = 0;
		for (int j = 0; j < m_nNodes; j++)
		{
			if (fabsf(m_Sys[i*m_nNodes+j]) < 1e-9)
			{
				count++;
			}
			else
			{
				rowcount++;
			}
		}
	}

	for (int i = 0; i < m_nDof; i++)
		m_v[i] = 0;

	for (int i = 0; i < m_nNodes; i++)
	{
		m_qn[i * 3] = m_q[i * 3] = m_tetmesh->GetNode(i).position.x;
		m_qn[i * 3 + 1] = m_q[i * 3 + 1] = m_tetmesh->GetNode(i).position.y;
		m_qn[i * 3 + 2] = m_q[i * 3 + 2] = m_tetmesh->GetNode(i).position.z;
	}


}

void ProjectiveDynamicsSolver::SetPosition(int nodeindex, Vector3d pos)
{
	m_q[nodeindex * 3] = pos.x;
	m_q[nodeindex * 3 + 1] = pos.y;
	m_q[nodeindex * 3 + 2] = pos.z;

	m_qn[nodeindex * 3] = pos.x;
	m_qn[nodeindex * 3 + 1] = pos.y;
	m_qn[nodeindex * 3 + 2] = pos.z;

	m_tetmesh->GetNode(nodeindex).position = pos;
}

void ProjectiveDynamicsSolver::SetVelocity(int nodeindex, Vector3d vel)
{
	m_v[nodeindex * 3] = vel.x;
	m_v[nodeindex * 3 + 1] = vel.y;
	m_v[nodeindex * 3 + 2] = vel.z;

}
void ProjectiveDynamicsSolver::SetExtForce(int nodeindex, Vector3d force)
{
	m_fext[nodeindex * 3] = force.x;
	m_fext[nodeindex * 3 + 1] = force.y;
	m_fext[nodeindex * 3 + 2] = force.z;
}

void ProjectiveDynamicsSolver::AddExtForce(int nodeindex, Vector3d force)
{
	m_fext[nodeindex * 3] += force.x;
	m_fext[nodeindex * 3 + 1] += force.y;
	m_fext[nodeindex * 3 + 2] += force.z;
}


void ProjectiveDynamicsSolver::Simulate()
{
	for (int i = 0; i < m_nDof; i++)
	{
		m_qn[i] = m_q[i];
		m_sn[i] = m_qn[i] + m_DT * m_v[i] * DAMPING + m_DT * m_DT * m_fext[i] / m_Mass[(i / 3)*m_nNodes + (i / 3)];
		//clear external forces
		m_fext[i] = 0;
	}

	for (int n = 0; n < MAX_ITERATIONS; ++n)
	{
		for (int i = 0; i < m_nDof; ++i)
		{
			m_b[i] = m_Mass[(i / 3) * m_nNodes + (i / 3)] / (m_DT * m_DT) * m_sn[i];
		}

		for (int i = 0; i < m_nTets; ++i)
		{
			const Tetrahedral* t = m_tetmesh->GetTetrahedral(i);

			int i0 = t->node[0];
			int i1 = t->node[1];
			int i2 = t->node[2];
			int i3 = t->node[3];

			Vector3d v0(m_q[i0 * 3], m_q[i0 * 3 + 1], m_q[i0 * 3 + 2]);
			Vector3d v1(m_q[i1 * 3], m_q[i1 * 3 + 1], m_q[i1 * 3 + 2]);
			Vector3d v2(m_q[i2 * 3], m_q[i2 * 3 + 1], m_q[i2 * 3 + 2]);
			Vector3d v3(m_q[i3 * 3], m_q[i3 * 3 + 1], m_q[i3 * 3 + 2]);

			Matrix3d D = m_tetmesh->GenerateD(v0, v1, v2, v3);
			Matrix3d R = m_tetmesh->GetRotationMatrix(i, D);
			Matrix3d edges = R * t->Dm * t->weight * t->volume;

			for (int j = 0; j < 3; j++)
			{
				m_b[i0 * 3 + j] += edges(j, 0);
				m_b[i1 * 3 + j] += edges(j, 1);
				m_b[i2 * 3 + j] += edges(j, 2);
				m_b[i3 * 3 + j] += -edges(j, 0) - edges(j, 1) - edges(j, 2);
			}
		}

		m_solver.SolveSparseCompressed3x3(m_q, m_b);

		for (int i = 0; i < m_nNodes; ++i)
		{
			if (m_pinned[i])
			{
				SetPosition(i, m_tetmesh->GetRestPosition(i));
			}
		}

		//to be continued
	}

	for (int i = 0; i < m_nDof; ++i)
	{
		m_v[i] = (m_q[i] - m_qn[i]) / m_DT;
	}


	for (int i = 0; i < m_nNodes; i++)
	{
		m_tetmesh->GetNode(i).position = Vector3d(m_q[i * 3], m_q[i * 3 + 1], m_q[i * 3 + 2]);
	}
}

