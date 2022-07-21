
#include "SequentialImpulseSolver.h"
#include "Jacobian.h"
#include "WarmStart.h"

struct ContactJacobians
{
	Jacobian jN;
	Jacobian jT;
	Jacobian jB;
};

class SequentialImpulseSolver : public ResolutionPhase
{
public:
	SequentialImpulseSolver()
	{

	}

	virtual ~SequentialImpulseSolver()
	{

	}

	virtual void	ResolveContact(std::vector<ContactManifold>& manifolds, float dt) override final
	{
		if (manifolds.empty())
			return;

		WarmStart::Manifolds(manifolds, dt);

		int nJacobians = 0;
		for (size_t i = 0; i < manifolds.size(); ++i)
		{
			for (int j = 0; j < manifolds[i].NumContactPointCount; ++j)
			{
				nJacobians++;
			}
		}

		std::vector<ContactJacobians> jacobians;
		jacobians.resize(nJacobians);

		int k = 0;
		for (size_t i = 0; i < manifolds.size(); ++i)
		{
			for (int j = 0; j < manifolds[i].NumContactPointCount; ++j)
			{
				ContactManifold* manifold = &manifolds[i];
				jacobians[k].jN.Setup(manifold, j, JacobianType::Normal, manifold->ContactPoints[j].Normal, dt);
				jacobians[k].jT.Setup(manifold, j, JacobianType::Tangent, manifold->ContactPoints[j].Tangent1, dt);
				jacobians[k].jB.Setup(manifold, j, JacobianType::Tangent, manifold->ContactPoints[j].Tangent2, dt);
				k++;
			}
		}

		k = 0;
		for (size_t i = 0; i < manifolds.size(); ++i)
		{
			for (int j = 0; j < manifolds[i].NumContactPointCount; ++j)
			{
				ContactManifold* manifold = &manifolds[i];
				jacobians[k].jN.Solve(manifold, jacobians[k].jN, dt);
				jacobians[k].jT.Solve(manifold, jacobians[k].jN, dt);
				jacobians[k].jB.Solve(manifold, jacobians[k].jN, dt);
				k++;
			}
		}
	}
};

ResolutionPhase* ResolutionPhase::CreateSequentialImpulseSolver()
{
	return new SequentialImpulseSolver();
}
