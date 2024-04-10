
#include <assert.h>
#include "ForceField.h"
#include "RigidBody.h"
#include "../Geometry/DenseTensorField3.h"

namespace Riemann
{
	class GravityField : public ForceField
	{
	public:
		GravityField(const Vector3& Gravity) : m_GravityAcc(Gravity)
		{

		}

		virtual bool ApplyForce(RigidBodyDynamic* Rigid) override final
		{
			if (Rigid->Sleeping || Rigid->DisableGravity)
			{
				return true;
			}
			Rigid->ApplyLinearAcceleration(m_GravityAcc);
			return true;
		}

	private:
		Vector3 m_GravityAcc;
	};

	class DenseField : public ForceField, DenseTensorField3<Vector3>
	{
	public:
		DenseField()
		{
		}

		virtual bool ApplyForce(RigidBodyDynamic* Rigid) override final
		{
			Vector3 Force = DenseTensorField3<Vector3>::GetTensorByPosition(Rigid->X);
			Rigid->ApplyForce(Force);
			return true;
		}
	};

	class TimeEvoluteDenseField3 : public ForceField
	{
	public:
		TimeEvoluteDenseField3()
		{
			m_curr_idx = -1;
		}

		virtual void Update(float dt) override final
		{
			if (m_Frames.size() > 0)
			{
				m_curr = std::fmod(m_curr + dt, m_Frames.back());
			}

			m_curr_idx = -1;
			for (size_t i = 0; i < m_Frames.size(); ++i)
			{
				if (m_curr < m_Frames.size())
				{
					m_curr_idx = (int)i;
					break;
				}
			}

			assert(m_curr_idx != -1);
			assert(m_Frames.size() + 1 == m_Fields.size());
		}

		virtual bool ApplyForce(RigidBodyDynamic* Rigid) override final
		{
			if (m_curr_idx != -1)
			{
				Vector3 Force1 = m_Fields[m_curr_idx].GetTensorByPosition(Rigid->X);
				Vector3 Force2 = m_Fields[m_curr_idx + 1].GetTensorByPosition(Rigid->X);
				float dt = m_curr_idx > 1 ? (m_curr - m_Frames[m_curr_idx - 1] / m_Frames[m_curr_idx] - m_Frames[m_curr_idx - 1]) : (m_curr / m_Frames[m_curr_idx]);
				Vector3 Force = LinearInterp(Force1, Force2, dt);
				Rigid->ApplyForce(Force);
				return true;
			}
			return false;
		}

	private:
		std::vector<float> m_Frames;
		std::vector<DenseTensorField3<Vector3>> m_Fields;
		float m_curr;
		int m_curr_idx;
	};

	typedef float (*AttenuationFunc)(float);

	class Attenuation
	{
	public:
		static float	Linear(float x) { x = Maths::Clamp(1.0f - x, 0.0f, 1.0f); return x; }
		static float	Square(float x) { x = Maths::Clamp(1.0f - x, 0.0f, 1.0f); return x * x; }
		static float	Cubic(float x) { x = Maths::Clamp(1.0f - x, 0.0f, 1.0f); return x * x * x; }
		static float	Sqrt(float x) { x = Maths::Clamp(1.0f - x, 0.0f, 1.0f); return sqrtf(x); }
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

		virtual void Update(float dt) override final
		{

		}

		virtual bool ApplyForce(RigidBodyDynamic* Rigid) override final
		{
			Vector3 RelativePos = Rigid->X - m_Param.Center;
			float Dist = RelativePos.Length();
			float att_coef = m_AttenuationTable[(int)m_Param.Attenuation](Dist / m_Param.Radius);
			Vector3 Force = LinearInterp(m_Param.ExplosionForce0, m_Param.ExplosionForce1, att_coef);
			Rigid->ApplyForce(Force);
			return true;
		}

	private:
		static AttenuationFunc  m_AttenuationTable[(int)ExplosionFieldParam::AttenuationType::ATTENUATION_COUNT];
		ExplosionFieldParam		m_Param;
	};

	// static
	AttenuationFunc ExplosionField::m_AttenuationTable[] = { Attenuation::Linear, Attenuation::Square, Attenuation::Cubic, Attenuation::Sqrt };

	ForceField* ForceField::CreateGrivityField(const Vector3& Gravity)
	{
		return new GravityField(Gravity);
	}

	ForceField* ForceField::CreateExplosionField(const ExplosionFieldParam& param)
	{
		return new ExplosionField(param);
	}
}