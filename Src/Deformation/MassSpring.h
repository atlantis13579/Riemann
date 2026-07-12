
#include <vector>
#include "../Maths/Vector3.h"

namespace Riemann
{
class MSProxy;

class MSConstraint
{
public:
	MSConstraint(MSProxy* Proxy)
	{
		m_Proxy = Proxy;
	}
	virtual ~MSConstraint() = 0;
	virtual void Solve() = 0;
protected:
	MSProxy* m_Proxy;
};

class MSCollision
{
public:
	MSCollision(MSProxy* Proxy)
	{
		m_Proxy = Proxy;
	}
	virtual ~MSCollision() = 0;
	virtual void Solve(int Id) = 0;
protected:
	MSProxy* m_Proxy;
};

class MassSpringSolver
{
public:
	MassSpringSolver();
	~MassSpringSolver();

	void Solve(float dt);
	void VerletIntegration(float dt);
	void SolveConstraints(int nIter);
	void SolveCollisions(int nIter);

private:
	std::vector<Vector3> m_p, m_p0;		// position
	std::vector<Vector3> m_f, m_f_ext;	// force
	std::vector<float> 	 m_mass;		// mass
	float m_AirFriction;
	float m_Damping;
	float m_SimulationStep;

	std::vector<MSConstraint*> m_Constraints;
	std::vector<MSCollision*>  m_Collisions;
};
}