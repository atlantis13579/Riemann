#include "PositionConstraint.h"

class FixedConstraint : public PositionConstraint
{
public:
	FixedConstraint() {}
	virtual ~FixedConstraint() {}
	virtual void Setup();
	virtual void Solve();
};

class PointConstraint : public PositionConstraint
{
public:
	PointConstraint() {}
	virtual ~PointConstraint() {}
	virtual void Setup();
	virtual void Solve();
};

class DistanceConstraint : public PositionConstraint
{
public:
	DistanceConstraint() {}
	virtual ~DistanceConstraint() {}
	virtual void Setup();
	virtual void Solve();
};

class HingeConstraint : public PositionConstraint
{
public:
	HingeConstraint() {}
	virtual ~HingeConstraint() {}
	virtual void Setup();
	virtual void Solve();
};

class JointConstraint : public PositionConstraint
{
public:
	JointConstraint() {}
	virtual ~JointConstraint() {}
	virtual void Setup();
	virtual void Solve();
};







