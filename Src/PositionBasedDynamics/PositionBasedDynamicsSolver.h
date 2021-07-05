
#include <vector>

class PBDProxy;

class PBDConstraint
{
public:
	PBDConstraint(PBDProxy* Proxy)
	{
		m_Proxy = Proxy;
	}
	virtual ~PBDConstraint() = 0;
	virtual void Solve() = 0;
protected:
	PBDProxy* m_Proxy;
};

class PBDCollision
{
public:
	virtual ~PBDCollision() = 0;
	virtual void Solve() = 0;
};

class PositionBasedDynamicsSolver
{
public:
	PositionBasedDynamicsSolver();
	~PositionBasedDynamicsSolver();

	void Solve(float dt);
	void VerletIntegration(float dt);
	void SolveConstraints(int nIter);
	void SolveCollisions();

private:
	std::vector<float> m_p, m_p0;		// position
	std::vector<float> m_f, m_f_ext;	// force
	std::vector<float> m_mass;			// mass
	float m_AirFriction;
	float m_Damping;
	float m_SimulationStep;

	std::vector<PBDConstraint*> m_Constraints;
	std::vector<PBDCollision*>  m_Collisions;
};