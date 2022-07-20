
#include <functional>
#include "ForceField.h"
#include "RigidBody.h"
#include "../Geometry/DenseTensorField3d.h"

class GravityField : public ForceField
{
public:
	GravityField(const Vector3d& Gravity) : m_Gravity(Gravity)
	{

	}

	virtual bool ApplyForce(RigidBodyDynamic* Rigid) override final
	{
		if (Rigid->DisableGravity)
		{
			return true;
		}
		Rigid->ApplyForce(m_Gravity);
		return true;
	}

private:
	Vector3d m_Gravity;
};

class DenseField : public ForceField, DenseTensorField3d<Vector3d>
{
public:
	DenseField()
	{

	}

	virtual bool ApplyForce(RigidBodyDynamic* Rigid) override final
	{
		Vector3d Force = DenseTensorField3d::GetTensorByPosition(Rigid->X);
		Rigid->ApplyForce(Force);
		return true;
	}
};

class ExplosionField : public ForceField
{
public:
	ExplosionField(const ExplosionFieldParam& param) : m_Param(param)
	{
		if ((int)m_Param.Attenuation < 0 || (int)m_Param.Attenuation >= 4)
		{
			m_Param.Attenuation = ExplosionFieldParam::AttenuationType::LINEAR;
		}
	}

	virtual bool ApplyForce(RigidBodyDynamic* Rigid) override final
	{
		Vector3d RelativePos = Rigid->X - m_Param.Center;
		float Dist = RelativePos.Length();
		float att_coef = m_AttenuationTable[(int)m_Param.Attenuation](Dist / m_Param.Radius);
		Vector3d Force = LinearInterp(m_Param.ExplosionForce0, m_Param.ExplosionForce1, att_coef);
		Rigid->ApplyForce(Force);
		return true;
	}

private:
	static AttenuationFunc  m_AttenuationTable[(int)ExplosionFieldParam::AttenuationType::ATTENUATION_COUNT];
	ExplosionFieldParam		m_Param;
};

// static
AttenuationFunc ExplosionField::m_AttenuationTable[] = { Attenuation::Linear, Attenuation::Square, Attenuation::Cubic, Attenuation::Sqrt };

ForceField* ForceField::CreateGrivityField(const Vector3d& Gravity)
{
	return new GravityField(Gravity);
}

ForceField* ForceField::CreateExplosionField(const ExplosionFieldParam& param)
{
	return new ExplosionField(param);
}
