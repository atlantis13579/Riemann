#pragma once

#include "../Maths/Vector3.h"

// http://allenchou.net/2013/12/game-physics-resolution-contact-constraints/

struct Contact;
class RigidBody;
class ContactManifold;
struct JacobianSolver;

struct GeneralizedVelocity
{
	Vector3	v;
	Vector3 w;
};

// Jacobian is a 1x12 Matrix which satisfy JV + b = 0
// Where V is 12x1 generalized velocity [va, wa, vb, wb]^T
// b is the bias term
struct  Jacobian
{
	Jacobian()
	{
	}

	void	Setup(Contact* contact, RigidBody* bodyA, RigidBody* bodyB, GeneralizedVelocity* ga, GeneralizedVelocity* gb, const Vector3& dir, float bias);
	void	Solve(float lambdamin, float lambdamax);

	Vector3 m_jva;			// J
	Vector3 m_jwa;
	Vector3 m_jvb;
	Vector3 m_jwb;
	Vector3 m_invmjva;		// Inv(Mass Matrix) * J^T
	Vector3 m_invmjwa;
	Vector3 m_invmjvb;
	Vector3 m_invmjwb;
	float	m_error;
	float	m_bias;
	float	m_effectiveMass;
	float	m_totalLambda;

	GeneralizedVelocity* m_ga;
	GeneralizedVelocity* m_gb;
};

// Velocity Constraint consists of three Jacobian constraint
// JN * V + b == 0, JT * V + b == 0, JB * V + b == 0
// Where V is 12x1 generalized velocity [va, wa, vb, wb]^T
// b is the bias term
// to model the equation : (vb + CrossProduct(wb, rb) - va - CrossProduct(wa, ra)) * n == 0
struct JacobianSolver
{
	JacobianSolver(GeneralizedVelocity *_phase, int _ia, int _ib, RigidBody* _bodyA, RigidBody* _bodyB)
	{
		phase = _phase;
		bodyA = _bodyA;
		bodyB = _bodyB;
		indexA = _ia;
		indexB = _ib;
	}

	void	SetupPositionPass(Contact* contact, float dt);
	void	SetupVelocityPass(int n);
	void	Solve();
	float	GetSquaredError() const;
	void	Finalize();

	Contact*	m_contact;
	Jacobian	m_jN;
	Jacobian	m_jT;
	Jacobian	m_jB;
	
	GeneralizedVelocity* phase;
	RigidBody *bodyA, *bodyB;
	int indexA, indexB;
};
