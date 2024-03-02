#include <math.h>
#include <string.h>
#include <cstddef>

#include "MassSpring.h"
#include "../CollisionPrimitive/Sphere3d.h"

namespace Riemann
{
class MSProxy
{
public:
	virtual void SetPosition(int Id, const Vector3& pos) = 0;
	virtual void GetPosition(int Id, Vector3 *pos) = 0;
};

class MSFixedPositionConstraint : public MSConstraint
{
public:
	MSFixedPositionConstraint(MSProxy* Proxy, int Id, const Vector3 &pos) : MSConstraint(Proxy)
	{
		m_Id = Id;
		m_pos = pos;
	}
	virtual ~MSFixedPositionConstraint() {}
	virtual void Solve() override final
	{
		m_Proxy->SetPosition(m_Id, m_pos);
	}
private:
	MSProxy* m_Proxy;
	int m_Id;
	Vector3 m_pos;
};

class MSFixedLengthConstraint : public MSConstraint
{
public:
	MSFixedLengthConstraint(MSProxy* Proxy, int Id1, int Id2) : MSConstraint(Proxy)
	{
		m_Id1 = Id1;
		m_Id2 = Id2;

		Vector3 v[2];
		m_Proxy->GetPosition(m_Id1, v);
		m_Proxy->GetPosition(m_Id2, v + 1);
		m_HalfLength = 0.5f * (v[0] - v[1]).Length();
	}
	virtual ~MSFixedLengthConstraint() {}
	virtual void Solve() override final
	{
		Vector3 v[2];
		m_Proxy->GetPosition(m_Id1, v);
		m_Proxy->GetPosition(m_Id2, v + 1);
		if ((v[1] - v[0]).SquareLength() < 1e-6)
		{
			return;
		}
		Vector3 c = v[0] + v[1];
		Vector3 d = (v[0] - c).Unit();
		m_Proxy->SetPosition(m_Id1, c + d * m_HalfLength);
		m_Proxy->SetPosition(m_Id2, c - d * m_HalfLength);
	}
private:
	int m_Id1, m_Id2;
	float m_HalfLength;
};

class MSJointConstraint : public MSConstraint
{
public:
	MSJointConstraint(MSProxy* Proxy, int Id1, int Id2) : MSConstraint(Proxy)
	{
		m_Id1 = Id1;
		m_Id2 = Id2;

		Vector3 v;
		m_Proxy->GetPosition(m_Id1, &m_pos1);
		m_Proxy->GetPosition(m_Id2, &v);
		m_Length = (v - m_pos1).Length();
	}
	virtual ~MSJointConstraint() {}
	virtual void Solve() override final
	{
		Vector3 v;
		m_Proxy->GetPosition(m_Id2, &v);
		if ((v - m_pos1).SquareLength() < 1e-6)
		{
			v = m_pos1 + Vector3::UnitZ();
		}
		Vector3 d = (v - m_pos1).Unit();
		m_Proxy->SetPosition(m_Id1, m_pos1);
		m_Proxy->SetPosition(m_Id2, m_pos1 + d * m_Length);
	}
private:
	int m_Id1, m_Id2;
	Vector3 m_pos1;
	float m_Length;
};

MassSpringSolver::MassSpringSolver()
{
	m_AirFriction = 0.1f;
	m_Damping = 0.98f;
	m_SimulationStep = 0.033f;
}

MassSpringSolver::~MassSpringSolver()
{
}

void MassSpringSolver::Solve(float dt)
{
	VerletIntegration(dt);

	int nIter = (int)(dt / m_SimulationStep) + 1;
	SolveConstraints(nIter);
	SolveCollisions(1);
}

void MassSpringSolver::VerletIntegration(float dt)
{
	for (size_t i = 0; i < m_p.size(); ++i)
	{
		Vector3 force = m_f[i] + m_f_ext[i];
		Vector3 v = (m_p[i] - m_p0[i]) / dt;
		force += -v * m_AirFriction;
		Vector3 t = m_p[i];
		m_p[i] += (m_p[i] - m_p0[i]) * m_Damping + (force / m_mass[i]) * dt * dt;
		m_p0[i] = t;
	}

	memset(&m_f_ext[0], 0, sizeof(m_f_ext[0]) * m_f_ext.size());
}

void MassSpringSolver::SolveConstraints(int nIter)
{
	for (int j = 0; j < nIter; ++j)
	{
		for (size_t i = 0; i < m_Constraints.size(); ++i)
		{
			m_Constraints[i]->Solve();
		}
	}
}

void MassSpringSolver::SolveCollisions(int nIter)
{
	for (size_t i = 0; i < m_p.size(); ++i)
	{
		for (size_t i = 0; i < m_Collisions.size(); ++i)
		{
			m_Collisions[i]->Solve((int)i);
		}
	}
}
}