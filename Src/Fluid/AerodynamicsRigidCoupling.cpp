#include "AerodynamicsRigidCoupling.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "AerodynamicsSimulation.h"
#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/AxisAlignedBox3.h"
#include "../CollisionPrimitive/Sphere3.h"
#include "../RigidBodyDynamics/RigidBody.h"

namespace Riemann
{
	namespace
	{
		constexpr float Pi = 3.14159265358979323846f;
		constexpr float ClampEpsilon = 1.0e-6f;
		constexpr int MaximumSurfaceSamplesPerAxis = 16;

		bool IsFiniteVector(const Vector3& Value)
		{
			return std::isfinite(Value.x) && std::isfinite(Value.y) && std::isfinite(Value.z);
		}

		bool AreSettingsValid(const AerodynamicsRigidCouplingSettings& Settings)
		{
			return Settings.SurfaceSamplesPerAxis >= 1
				&& Settings.SurfaceSamplesPerAxis <= MaximumSurfaceSamplesPerAxis
				&& std::isfinite(Settings.DragCoefficient) && Settings.DragCoefficient >= 0.0f
				&& std::isfinite(Settings.MaxLinearAcceleration) && Settings.MaxLinearAcceleration > 0.0f
				&& std::isfinite(Settings.MaxAngularAcceleration) && Settings.MaxAngularAcceleration > 0.0f;
		}

		bool IsInsideSimulation(const Vector3& Position,
			const AerodynamicsSimulationSettings& Settings)
		{
			return Position.x >= Settings.BoundsMin.x && Position.x <= Settings.BoundsMax.x
				&& Position.y >= Settings.BoundsMin.y && Position.y <= Settings.BoundsMax.y
				&& Position.z >= Settings.BoundsMin.z && Position.z <= Settings.BoundsMax.z;
		}

		void AppendSurfaceSample(const AerodynamicsSimulationSettings& SimulationSettings,
			const RigidBodyDynamic* Rigid, const Transform& ShapeTransform,
			const Vector3& LocalPosition, const Vector3& LocalNormal, float Area,
			std::vector<AerodynamicSurfaceSample>* Samples, float* SampledArea)
		{
			if (!std::isfinite(Area) || Area <= 0.0f)
			{
				return;
			}
			const Vector3 Position = ShapeTransform * LocalPosition;
			if (!IsFiniteVector(Position) || !IsInsideSimulation(Position, SimulationSettings))
			{
				return;
			}
			const Vector3 Normal = (ShapeTransform.quat * LocalNormal).SafeUnit();
			if (!IsFiniteVector(Normal) || Normal.IsZero())
			{
				return;
			}

			AerodynamicSurfaceSample Sample;
			Sample.Position = Position;
			Sample.Normal = Normal;
			Sample.Velocity = Rigid->V + Rigid->W.Cross(Position - Rigid->X);
			Sample.Area = Area;
			if (!IsFiniteVector(Sample.Velocity))
			{
				return;
			}
			Samples->push_back(Sample);
			*SampledArea += Area;
		}

		void AppendBoxFace(const AerodynamicsSimulationSettings& SimulationSettings,
			const RigidBodyDynamic* Rigid, const Transform& ShapeTransform,
			const AxisAlignedBox3& Box, int Axis, float Sign, int SamplesPerAxis,
			std::vector<AerodynamicSurfaceSample>* Samples, float* SampledArea)
		{
			const Vector3 Size = Box.Max - Box.Min;
			const int AxisU = (Axis + 1) % 3;
			const int AxisV = (Axis + 2) % 3;
			const float Area = Size[AxisU] * Size[AxisV]
				/ ((float)SamplesPerAxis * (float)SamplesPerAxis);
			Vector3 Normal = Vector3::Zero();
			Normal[Axis] = Sign;
			for (int V = 0; V < SamplesPerAxis; ++V)
			for (int U = 0; U < SamplesPerAxis; ++U)
			{
				Vector3 Position = (Box.Min + Box.Max) * 0.5f;
				Position[Axis] = Sign < 0.0f ? Box.Min[Axis] : Box.Max[Axis];
				Position[AxisU] = Box.Min[AxisU] + Size[AxisU]
					* ((float)U + 0.5f) / (float)SamplesPerAxis;
				Position[AxisV] = Box.Min[AxisV] + Size[AxisV]
					* ((float)V + 0.5f) / (float)SamplesPerAxis;
				AppendSurfaceSample(SimulationSettings, Rigid, ShapeTransform,
					Position, Normal, Area, Samples, SampledArea);
			}
		}

		bool BuildSurfaceSamples(const AerodynamicsSimulation* Simulation,
			const AerodynamicsRigidCouplingSettings& CouplingSettings,
			const RigidBodyDynamic* Rigid, std::vector<AerodynamicSurfaceSample>* Samples,
			float* SampledArea)
		{
			if (Simulation == nullptr || !Simulation->IsInitialized() || Rigid == nullptr
				|| Samples == nullptr || SampledArea == nullptr)
			{
				return false;
			}
			Samples->clear();
			*SampledArea = 0.0f;
			const AerodynamicsSimulationSettings& SimulationSettings = Simulation->GetSettings();
			const int SamplesPerAxis = CouplingSettings.SurfaceSamplesPerAxis;
			for (const Geometry* GeometryObject : Rigid->Geometries())
			{
				if (GeometryObject == nullptr)
				{
					continue;
				}
				const Transform ShapeTransform = Rigid->GetGeometryTransform(GeometryObject);
				if (const AxisAlignedBox3* Box = GeometryObject->GetShapeObj<AxisAlignedBox3>())
				{
					const Vector3 Size = Box->Max - Box->Min;
					if (!(Size.x > 0.0f && Size.y > 0.0f && Size.z > 0.0f)
						|| !IsFiniteVector(Size))
					{
						continue;
					}
					for (int Axis = 0; Axis < 3; ++Axis)
					{
						AppendBoxFace(SimulationSettings, Rigid, ShapeTransform, *Box,
							Axis, -1.0f, SamplesPerAxis, Samples, SampledArea);
						AppendBoxFace(SimulationSettings, Rigid, ShapeTransform, *Box,
							Axis, 1.0f, SamplesPerAxis, Samples, SampledArea);
					}
				}
				else if (const Sphere3* Sphere = GeometryObject->GetShapeObj<Sphere3>())
				{
					if (!std::isfinite(Sphere->Radius) || Sphere->Radius <= 0.0f
						|| !IsFiniteVector(Sphere->Center))
					{
						continue;
					}
					const int LongitudeCount = SamplesPerAxis * 2;
					const float Area = 4.0f * Pi * Sphere->Radius * Sphere->Radius
						/ ((float)SamplesPerAxis * (float)LongitudeCount);
					for (int Latitude = 0; Latitude < SamplesPerAxis; ++Latitude)
					for (int Longitude = 0; Longitude < LongitudeCount; ++Longitude)
					{
						const float Y = 1.0f - 2.0f * ((float)Latitude + 0.5f)
							/ (float)SamplesPerAxis;
						const float Radial = std::sqrt(std::max(0.0f, 1.0f - Y * Y));
						const float Phi = 2.0f * Pi * ((float)Longitude + 0.5f)
							/ (float)LongitudeCount;
						const Vector3 Normal(Radial * std::cos(Phi), Y, Radial * std::sin(Phi));
						AppendSurfaceSample(SimulationSettings, Rigid, ShapeTransform,
							Sphere->Center + Normal * Sphere->Radius, Normal, Area,
							Samples, SampledArea);
					}
				}
			}
			return std::isfinite(*SampledArea) && *SampledArea >= 0.0f;
		}
	}

	AerodynamicsRigidCouplingSettings::AerodynamicsRigidCouplingSettings()
		: SurfaceSamplesPerAxis(4)
		, DragCoefficient(1.0f)
		, MaxLinearAcceleration(100.0f)
		, MaxAngularAcceleration(200.0f)
	{
	}

	AerodynamicsRigidCouplingResult::AerodynamicsRigidCouplingResult()
		: Force(Vector3::Zero())
		, Torque(Vector3::Zero())
		, ClampScale(1.0f)
		, SampledSurfaceArea(0.0f)
		, SampleCount(0)
		, Clamped(false)
	{
	}

	AerodynamicsRigidCoupling::AerodynamicsRigidCoupling(AerodynamicsSimulation* Simulation,
		const AerodynamicsRigidCouplingSettings& Settings)
		: m_Simulation(Simulation)
		, m_Settings(Settings)
		, m_SettingsValid(AreSettingsValid(Settings))
	{
	}

	bool AerodynamicsRigidCoupling::ApplyForce(RigidBodyDynamic* Rigid)
	{
		AerodynamicsRigidCouplingResult Result;
		if (!Evaluate(Rigid, &Result))
		{
			return false;
		}
		Rigid->ApplyForce(Result.Force);
		Rigid->ApplyTorgue(Result.Torque);
		return true;
	}

	bool AerodynamicsRigidCoupling::Evaluate(const RigidBodyDynamic* Rigid,
		AerodynamicsRigidCouplingResult* Result) const
	{
		if (Result == nullptr)
		{
			return false;
		}
		*Result = AerodynamicsRigidCouplingResult();
		if (!IsValid() || Rigid == nullptr || !IsFiniteVector(Rigid->X)
			|| !IsFiniteVector(Rigid->V) || !IsFiniteVector(Rigid->W))
		{
			return false;
		}

		std::vector<AerodynamicSurfaceSample> Samples;
		float SampledArea = 0.0f;
		if (!BuildSurfaceSamples(m_Simulation, m_Settings, Rigid, &Samples, &SampledArea))
		{
			return false;
		}
		const AerodynamicForceResult ForceResult = m_Simulation->ComputeSurfaceForce(
			Rigid->X, Samples, m_Settings.DragCoefficient);
		if (!IsFiniteVector(ForceResult.Force) || !IsFiniteVector(ForceResult.Torque))
		{
			return false;
		}

		Result->Force = ForceResult.Force;
		Result->Torque = ForceResult.Torque;
		Result->SampledSurfaceArea = SampledArea;
		Result->SampleCount = (int)Samples.size();

		const float InverseMass = Rigid->GetInverseMass();
		if (!std::isfinite(InverseMass) || InverseMass < 0.0f)
		{
			return false;
		}
		const float LinearAcceleration = Result->Force.Length() * InverseMass;
		const Vector3 AngularAcceleration = Rigid->GetInverseInertia_WorldSpace() * Result->Torque;
		if (!std::isfinite(LinearAcceleration) || !IsFiniteVector(AngularAcceleration))
		{
			return false;
		}

		float ClampScale = 1.0f;
		if (LinearAcceleration > m_Settings.MaxLinearAcceleration)
		{
			ClampScale = std::min(ClampScale,
				m_Settings.MaxLinearAcceleration / LinearAcceleration);
		}
		const float AngularAccelerationMagnitude = AngularAcceleration.Length();
		if (!std::isfinite(AngularAccelerationMagnitude))
		{
			return false;
		}
		if (AngularAccelerationMagnitude > m_Settings.MaxAngularAcceleration)
		{
			ClampScale = std::min(ClampScale,
				m_Settings.MaxAngularAcceleration / AngularAccelerationMagnitude);
		}
		if (!std::isfinite(ClampScale) || ClampScale < 0.0f)
		{
			return false;
		}

		Result->ClampScale = ClampScale;
		Result->Clamped = ClampScale < 1.0f - ClampEpsilon;
		if (Result->Clamped)
		{
			Result->Force *= ClampScale;
			Result->Torque *= ClampScale;
		}
		return true;
	}

	bool AerodynamicsRigidCoupling::IsValid() const
	{
		return m_SettingsValid && m_Simulation != nullptr && m_Simulation->IsInitialized();
	}
}
