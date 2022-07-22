
#include "SequentialImpulseSolver.h"
#include "Jacobian.h"
#include "WarmStart.h"

// Velocity Constraint consists of three Jacobian constraint
// JN * V + b == 0, JT * V + b == 0, JB * V + b == 0
// Where V is 12x1 generalized velocity [va, wa, vb, wb]^T
// b is the bias term
struct VelocityConstraint
{
	VelocityConstraint() :
		jN(JacobianType::Normal),
		jT(JacobianType::Tangent),
		jB(JacobianType::Binormal)
	{
	}

	void Setup(ContactManifold& manifold, int j, float dt)
	{
		jN.Setup(&manifold.ContactPoints[j], manifold.GeomA, manifold.GeomB, manifold.ContactPoints[j].Normal, dt);
		jT.Setup(&manifold.ContactPoints[j], manifold.GeomA, manifold.GeomB, manifold.ContactPoints[j].Tangent, dt);
		jB.Setup(&manifold.ContactPoints[j], manifold.GeomA, manifold.GeomB, manifold.ContactPoints[j].Binormal, dt);
	}

	void Solve()
	{
		float totalLambda = jN.m_totalLambda;
		jN.Solve(totalLambda);
		jT.Solve(totalLambda);
		jB.Solve(totalLambda);
	}

	Jacobian jN;
	Jacobian jT;
	Jacobian jB;
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

		std::vector<VelocityConstraint> velocityConstraints;
		for (size_t i = 0; i < manifolds.size(); ++i)
		{
			for (int j = 0; j < manifolds[i].NumContactPointCount; ++j)
			{
				ContactManifold& manifold = manifolds[i];
				velocityConstraints.push_back(VelocityConstraint());
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
