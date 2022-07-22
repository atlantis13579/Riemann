
#include "SequentialImpulseSolver.h"
#include "Jacobian.h"
#include "WarmStart.h"
#include "../Collision/GeometryObject.h"

// Velocity Constraint consists of three Jacobian constraint
// JN * V + b == 0, JT * V + b == 0, JB * V + b == 0
// Where V is 12x1 generalized velocity [va, wa, vb, wb]^T
// b is the bias term
struct ContactVelocityConstraintSolver
{
	ContactVelocityConstraintSolver() :
		m_jN(JacobianType::Normal),
		m_jT(JacobianType::Tangent),
		m_jB(JacobianType::Binormal)
	{
	}

	void Setup(ContactManifold& manifold, int j, float dt)
	{
		m_Contact = &manifold.ContactPoints[j];
		m_rigidA = static_cast<RigidBody*>(manifold.GeomA->GetEntity());
		m_rigidB = static_cast<RigidBody*>(manifold.GeomB->GetEntity());
		m_jN.Setup(m_Contact, m_rigidA, m_rigidB, manifold.ContactPoints[j].Normal, dt);
		m_jT.Setup(m_Contact, m_rigidA, m_rigidB, manifold.ContactPoints[j].Tangent, dt);
		m_jB.Setup(m_Contact, m_rigidA, m_rigidB, manifold.ContactPoints[j].Binormal, dt);
	}

	void Solve()
	{
		float totalLambda = m_jN.m_totalLambda;
		m_jN.Solve(m_rigidA, m_rigidB, totalLambda);
		m_jT.Solve(m_rigidA, m_rigidB, totalLambda);
		m_jB.Solve(m_rigidA, m_rigidB, totalLambda);
	}

	ContactResult*	m_Contact;
	RigidBody*		m_rigidA;
	RigidBody*		m_rigidB;

	Jacobian		m_jN;
	Jacobian		m_jT;
	Jacobian		m_jB;
};

class SequentialImpulseSolver : public ResolutionPhase
{
public:
	SequentialImpulseSolver()
	{
		m_nMaxVelocityIterations = 8;
	}

	virtual ~SequentialImpulseSolver()
	{
	}

	virtual void	ResolveContact(std::vector<ContactManifold>& manifolds, float dt) override final
	{
		if (manifolds.empty())
			return;

		WarmStart::Manifolds(manifolds, dt);

		std::vector<ContactVelocityConstraintSolver> velocityConstraints;
		for (size_t i = 0; i < manifolds.size(); ++i)
		{
			for (int j = 0; j < manifolds[i].NumContactPointCount; ++j)
			{
				ContactManifold& manifold = manifolds[i];
				velocityConstraints.push_back(ContactVelocityConstraintSolver());
				velocityConstraints.back().Setup(manifold, j, dt);
			}
		}

		int it = 0;
		while (it++ <= m_nMaxVelocityIterations)
		{
			for (size_t k = 0; k < velocityConstraints.size(); ++k)
			{
				velocityConstraints[k].Solve();
			}
		}
	}

private:
	int m_nMaxVelocityIterations;
};

ResolutionPhase* ResolutionPhase::CreateSequentialImpulseSolver()
{
	return new SequentialImpulseSolver();
}
