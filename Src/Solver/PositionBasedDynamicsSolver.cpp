
#include "PositionBasedDynamicsSolver.h"
#include "NumericalODESolver.h"

class PBDFixPosConstraint : public PBDConstraint
{
public:
	virtual ~PBDFixPosConstraint() {}
	virtual void Solve() override
	{
	}
private:
};

class PBDFixedLengthConstraint : public PBDConstraint
{
public:
	virtual ~PBDFixedLengthConstraint() {}
	virtual void Solve() override
	{
	}
private:
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
