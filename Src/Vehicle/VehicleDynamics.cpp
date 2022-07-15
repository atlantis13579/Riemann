
#include <math.h>
#include "VehicleDynamics.h"
#include "Vehicle.h"

// https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357
// dx / dt = v * cos(theta)
// 
// 
class KinematicBicycleModel
{
public:
	KinematicBicycleModel(float _AxleLength)
	{
		AxleLength = _AxleLength;
	}

	void IntegrateExplicitEuler(float dt)
	{
		// dx / dt = v * cos(theta)
		// dy / dt = v * sin(theta)
		// dw / dt = v * tan(steering angle) / L
		X = X + Speed * cosf(W) * dt;
		Y = Y + Speed * sinf(W) * dt;
		W = W + (Speed * tanf(SteeringAngle) / AxleLength) * dt;
	}

	void SetControlInput(float _SteeringAngle, float _Speed)
	{
		Speed = _Speed;
		SteeringAngle = _SteeringAngle;
	}

private:
	// State variable
	float		X;				// pos x for rear wheel
	float		Y;				// pos y for rear wheel
	float		W;				// rotation angle
	
	// Input
	float		Speed;			// current speed
	float		SteeringAngle;	// steering angle for front wheel

	// Vehicle constant
	float		AxleLength;
};
