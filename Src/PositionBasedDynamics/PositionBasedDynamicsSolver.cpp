
#include "PositionBasedDynamicsSolver.h"

#include <math.h>
#include <string.h>
#include <cstddef>

class PBDProxy
{
public:
	void SetPosition(int Id, float x, float y, float z);
	void GetPosition(int Id, float *x, float *y, float *z);
};

class PBDPositionConstraint : public PBDConstraint
{
public:
	PBDPositionConstraint(PBDProxy* Proxy, int Id, float x, float y, float z) : PBDConstraint(Proxy)
	{
		m_Id = Id;
		m_x = x;
		m_y = y;
		m_z = z;
	}
	virtual ~PBDPositionConstraint() {}
	virtual void Solve() override
	{
		m_Proxy->SetPosition(m_Id, m_x, m_y, m_z);
	}
private:
	PBDProxy* m_Proxy;
	int m_Id;
	float m_x, m_y, m_z;
};

class PBDFixedLengthConstraint : public PBDConstraint
{
public:
	PBDFixedLengthConstraint(PBDProxy* Proxy, int Id1, int Id2) : PBDConstraint(Proxy)
	{
		m_Id1 = Id1;
		m_Id2 = Id2;

		float v[6];
		m_Proxy->GetPosition(m_Id1, v + 0, v + 1, v + 2);
		m_Proxy->GetPosition(m_Id1, v + 3, v + 4, v + 5);
		m_Length = sqrtf((v[0] - v[3])*(v[0] - v[3]) + (v[1] - v[4]) * (v[1] - v[4]) + (v[2] - v[5]) * (v[2] - v[5]));
	}
	virtual ~PBDFixedLengthConstraint() {}
	virtual void Solve() override
	{
	}
private:
	int m_Id1, m_Id2;
	float m_Length;
};

PositionBasedDynamicsSolver::PositionBasedDynamicsSolver()
{
	m_AirFriction = 0.1f;
	m_Damping = 0.98f;
	m_SimulationStep = 0.033f;
}

PositionBasedDynamicsSolver::~PositionBasedDynamicsSolver()
{
}

void PositionBasedDynamicsSolver::Solve(float dt)
{
	VerletIntegration(dt);

	int nIter = (int)(dt / m_SimulationStep) + 1;
	SolveConstraints(nIter);

	SolveCollisions();
}

void PositionBasedDynamicsSolver::VerletIntegration(float dt)
{
	size_t nDof = m_p.size();

	// TODO, SIMD
	for (size_t i = 0; i < nDof; ++i)
	{
		float force = m_f[i] + m_f_ext[i];
		float v = (m_p[i] - m_p0[i]) / dt;
		force += -v * m_AirFriction;
		float t = m_p[i];
		m_p[i] += (m_p[i] - m_p0[i]) * m_Damping + (force / m_mass[i]) * dt * dt;
		m_p0[i] = t;
	}

	memset(&m_f_ext[0], 0, sizeof(m_f_ext[0]) * m_f_ext.size());
}

void PositionBasedDynamicsSolver::SolveConstraints(int nIter)
{
	for (int j = 0; j < nIter; ++j)
	{
		for (size_t i = 0; i < m_Constraints.size(); ++i)
		{
			m_Constraints[i]->Solve();
		}
	}
}

void PositionBasedDynamicsSolver::SolveCollisions()
{
	for (size_t i = 0; i < m_Collisions.size(); ++i)
	{
		m_Collisions[i]->Solve();
	}
}
