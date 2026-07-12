#include "FluidRigidCoupling.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "FluidSimulation.h"
#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/AxisAlignedBox3.h"
#include "../CollisionPrimitive/Sphere3.h"
#include "../RigidBodyDynamics/RigidBody.h"

namespace Riemann
{
	namespace
	{
		constexpr float Pi = 3.14159265358979323846f;
		constexpr float MinimumScale = 1.0e-6f;
		constexpr int MaximumSamplesPerAxis = 16;

		struct ForceAtPoint
		{
			Vector3 Position;
			Vector3 Force;
		};

		bool IsFiniteVector(const Vector3& Value)
		{
			return std::isfinite(Value.x) && std::isfinite(Value.y) && std::isfinite(Value.z);
		}

		bool AreSettingsValid(const FluidRigidCouplingSettings& Settings)
		{
			return Settings.SamplesPerAxis >= 1 && Settings.SamplesPerAxis <= MaximumSamplesPerAxis
				&& std::isfinite(Settings.QuadraticDragCoefficient) && Settings.QuadraticDragCoefficient >= 0.0f
				&& std::isfinite(Settings.ViscousDragCoefficient) && Settings.ViscousDragCoefficient >= 0.0f
				&& std::isfinite(Settings.MaxLinearAcceleration) && Settings.MaxLinearAcceleration > 0.0f
				&& std::isfinite(Settings.MaxAngularAcceleration) && Settings.MaxAngularAcceleration > 0.0f
				&& std::isfinite(Settings.FeedbackRadiusScale) && Settings.FeedbackRadiusScale > 0.0f;
		}

		void BuildBoxSamples(const AxisAlignedBox3& Box, int SamplesPerAxis,
			std::vector<Vector3>* Samples)
		{
			Samples->clear();
			Samples->reserve((size_t)SamplesPerAxis * (size_t)SamplesPerAxis * (size_t)SamplesPerAxis);
			const Vector3 Size = Box.Max - Box.Min;
			for (int Z = 0; Z < SamplesPerAxis; ++Z)
			for (int Y = 0; Y < SamplesPerAxis; ++Y)
			for (int X = 0; X < SamplesPerAxis; ++X)
			{
				const Vector3 Fraction(((float)X + 0.5f) / (float)SamplesPerAxis,
					((float)Y + 0.5f) / (float)SamplesPerAxis,
					((float)Z + 0.5f) / (float)SamplesPerAxis);
				Samples->push_back(Box.Min + Size * Fraction);
			}
		}

		void BuildSphereSamples(const Sphere3& Sphere, int SamplesPerAxis,
			std::vector<Vector3>* Samples)
		{
			Samples->clear();
			Samples->reserve((size_t)SamplesPerAxis * (size_t)SamplesPerAxis * (size_t)SamplesPerAxis);
			for (int Z = 0; Z < SamplesPerAxis; ++Z)
			for (int Y = 0; Y < SamplesPerAxis; ++Y)
			for (int X = 0; X < SamplesPerAxis; ++X)
			{
				const Vector3 UnitPosition(
					2.0f * ((float)X + 0.5f) / (float)SamplesPerAxis - 1.0f,
					2.0f * ((float)Y + 0.5f) / (float)SamplesPerAxis - 1.0f,
					2.0f * ((float)Z + 0.5f) / (float)SamplesPerAxis - 1.0f);
				if (UnitPosition.SquareLength() <= 1.0f)
				{
					Samples->push_back(Sphere.Center + UnitPosition * Sphere.Radius);
				}
			}
			if (Samples->empty())
			{
				Samples->push_back(Sphere.Center);
			}
		}

		float CalculateBoxProjectedArea(const Vector3& Size, const Transform& ShapeTransform,
			const Vector3& Direction)
		{
			const Vector3 LocalDirection = ShapeTransform.WorldToLocalDirection(Direction);
			return std::fabs(LocalDirection.x) * Size.y * Size.z
				+ std::fabs(LocalDirection.y) * Size.x * Size.z
				+ std::fabs(LocalDirection.z) * Size.x * Size.y;
		}

		bool EvaluateCoupling(const FluidSimulation* Fluid,
			const FluidRigidCouplingSettings& Settings, const RigidBodyDynamic* Rigid,
			FluidRigidCouplingResult* Result, std::vector<ForceAtPoint>* PointForces)
		{
			if (Result == nullptr)
			{
				return false;
			}
			*Result = FluidRigidCouplingResult();
			if (Fluid == nullptr || !Fluid->IsInitialized() || !AreSettingsValid(Settings)
				|| Rigid == nullptr || !IsFiniteVector(Rigid->X) || !IsFiniteVector(Rigid->V)
				|| !IsFiniteVector(Rigid->W))
			{
				return false;
			}
			if (PointForces != nullptr) PointForces->clear();
			const FluidSimulationSettings& FluidSettings = Fluid->GetSettings();
			if (!IsFiniteVector(FluidSettings.Gravity) || !std::isfinite(FluidSettings.RestDensity)
				|| FluidSettings.RestDensity <= 0.0f || !std::isfinite(FluidSettings.DynamicViscosity)
				|| FluidSettings.DynamicViscosity < 0.0f)
			{
				return false;
			}

			std::vector<Vector3> LocalSamples;
			for (const Geometry* GeometryObject : Rigid->Geometries())
			{
				if (GeometryObject == nullptr)
				{
					continue;
				}

				Vector3 BoxSize = Vector3::Zero();
				float SphereRadius = 0.0f;
				float ShapeVolume = 0.0f;
				bool IsBox = false;
				if (const AxisAlignedBox3* Box = GeometryObject->GetShapeObj<AxisAlignedBox3>())
				{
					BoxSize = Box->Max - Box->Min;
					ShapeVolume = Box->CalculateVolume();
					if (!(BoxSize.x > 0.0f && BoxSize.y > 0.0f && BoxSize.z > 0.0f))
					{
						continue;
					}
					BuildBoxSamples(*Box, Settings.SamplesPerAxis, &LocalSamples);
					IsBox = true;
				}
				else if (const Sphere3* Sphere = GeometryObject->GetShapeObj<Sphere3>())
				{
					SphereRadius = Sphere->Radius;
					ShapeVolume = Sphere->CalculateVolume();
					if (!(SphereRadius > 0.0f))
					{
						continue;
					}
					BuildSphereSamples(*Sphere, Settings.SamplesPerAxis, &LocalSamples);
				}
				else
				{
					continue;
				}

				if (!std::isfinite(ShapeVolume) || ShapeVolume <= 0.0f || LocalSamples.empty())
				{
					continue;
				}
				const float SampleCount = (float)LocalSamples.size();
				const float SampleVolume = ShapeVolume / SampleCount;
				const float EquivalentRadius = std::cbrt(3.0f * ShapeVolume / (4.0f * Pi));
				const float ViscousLength = 6.0f * Pi * EquivalentRadius / SampleCount;
				const Transform ShapeTransform = Rigid->GetGeometryTransform(GeometryObject);
				Result->SampleCount += (int)LocalSamples.size();

				for (const Vector3& LocalPosition : LocalSamples)
				{
					const Vector3 Position = ShapeTransform * LocalPosition;
					const float Density = Fluid->SampleDensity(Position);
					if (!std::isfinite(Density) || Density <= std::numeric_limits<float>::min())
					{
						continue;
					}
					const Vector3 FluidVelocity = Fluid->SampleVelocity(Position);
					if (!IsFiniteVector(FluidVelocity))
					{
						continue;
					}

					const float Occupancy = std::min(1.0f, Density / FluidSettings.RestDensity);
					const Vector3 RelativePosition = Position - Rigid->X;
					const Vector3 BodyVelocity = Rigid->V + Rigid->W.Cross(RelativePosition);
					const Vector3 RelativeVelocity = FluidVelocity - BodyVelocity;
					const float RelativeSpeed = RelativeVelocity.Length();
					Vector3 DragForce = Vector3::Zero();
					if (RelativeSpeed > MinimumScale)
					{
						const Vector3 Direction = RelativeVelocity / RelativeSpeed;
						const float ProjectedArea = IsBox
							? CalculateBoxProjectedArea(BoxSize, ShapeTransform, Direction)
							: Pi * SphereRadius * SphereRadius;
						const float QuadraticScale = 0.5f * Density * Settings.QuadraticDragCoefficient
							* (ProjectedArea / SampleCount) * RelativeSpeed;
						const float ViscousScale = FluidSettings.DynamicViscosity
							* Settings.ViscousDragCoefficient * ViscousLength * Occupancy;
						DragForce = RelativeVelocity * (QuadraticScale + ViscousScale);
					}

					const float DisplacedMass = Density * SampleVolume;
					const Vector3 BuoyancyForce = -FluidSettings.Gravity * DisplacedMass;
					const Vector3 SampleForce = BuoyancyForce + DragForce;
					if (!IsFiniteVector(SampleForce))
					{
						return false;
					}

					Result->BuoyancyForce += BuoyancyForce;
					Result->DragForce += DragForce;
					Result->Force += SampleForce;
					Result->Torque += RelativePosition.Cross(SampleForce);
					Result->FeedbackPosition += Position * DisplacedMass;
					Result->DisplacedMass += DisplacedMass;
					Result->SubmergedVolume += SampleVolume * Occupancy;
					Result->ActiveSampleCount++;
					if (PointForces != nullptr) PointForces->push_back({ Position, SampleForce });
				}
			}

			if (!IsFiniteVector(Result->Force) || !IsFiniteVector(Result->Torque)
				|| !IsFiniteVector(Result->FeedbackPosition)
				|| !std::isfinite(Result->DisplacedMass) || !std::isfinite(Result->SubmergedVolume))
			{
				*Result = FluidRigidCouplingResult();
				if (PointForces != nullptr) PointForces->clear();
				return false;
			}
			Result->FeedbackPosition = Result->DisplacedMass > MinimumScale
				? Result->FeedbackPosition / Result->DisplacedMass : Rigid->X;

			float ClampScale = 1.0f;
			const float InverseMass = Rigid->GetInverseMass();
			if (!std::isfinite(InverseMass) || InverseMass < 0.0f)
			{
				return false;
			}
			const float LinearAcceleration = Result->Force.Length() * InverseMass;
			if (!std::isfinite(LinearAcceleration))
			{
				return false;
			}
			if (LinearAcceleration > Settings.MaxLinearAcceleration)
			{
				ClampScale = std::min(ClampScale, Settings.MaxLinearAcceleration / LinearAcceleration);
			}
			const Vector3 AngularAcceleration = Rigid->GetInverseInertia_WorldSpace() * Result->Torque;
			if (!IsFiniteVector(AngularAcceleration))
			{
				return false;
			}
			const float AngularAccelerationMagnitude = AngularAcceleration.Length();
			if (AngularAccelerationMagnitude > Settings.MaxAngularAcceleration)
			{
				ClampScale = std::min(ClampScale,
					Settings.MaxAngularAcceleration / AngularAccelerationMagnitude);
			}
			if (!std::isfinite(ClampScale) || ClampScale < 0.0f)
			{
				return false;
			}

			Result->ClampScale = ClampScale;
			Result->Clamped = ClampScale < 1.0f - MinimumScale;
			if (Result->Clamped)
			{
				Result->Force *= ClampScale;
				Result->Torque *= ClampScale;
				Result->BuoyancyForce *= ClampScale;
				Result->DragForce *= ClampScale;
				if (PointForces != nullptr)
				{
					for (ForceAtPoint& PointForce : *PointForces) PointForce.Force *= ClampScale;
				}
			}
			return true;
		}
	}

	FluidRigidCouplingSettings::FluidRigidCouplingSettings()
		: SamplesPerAxis(4)
		, QuadraticDragCoefficient(1.0f)
		, ViscousDragCoefficient(1.0f)
		, MaxLinearAcceleration(100.0f)
		, MaxAngularAcceleration(200.0f)
		, FeedbackRadiusScale(1.5f)
	{
	}

	FluidRigidCouplingResult::FluidRigidCouplingResult()
		: Force(Vector3::Zero())
		, Torque(Vector3::Zero())
		, BuoyancyForce(Vector3::Zero())
		, DragForce(Vector3::Zero())
		, FeedbackPosition(Vector3::Zero())
		, DisplacedMass(0.0f)
		, SubmergedVolume(0.0f)
		, ClampScale(1.0f)
		, SampleCount(0)
		, ActiveSampleCount(0)
		, Clamped(false)
	{
	}

	FluidRigidCoupling::FluidRigidCoupling(FluidSimulation* Fluid,
		const FluidRigidCouplingSettings& Settings)
		: m_Fluid(Fluid)
		, m_Settings(Settings)
		, m_DeltaTime(0.0f)
		, m_SettingsValid(AreSettingsValid(Settings))
		, m_LastFeedbackFailureCount(0)
		, m_TotalFeedbackFailureCount(0)
	{
	}

	void FluidRigidCoupling::Update(float DeltaTime)
	{
		m_LastFeedbackFailureCount = 0;
		if (m_Fluid && m_Fluid->IsInitialized())
		{
			for (const PendingImpulse& Feedback : m_PendingFeedback)
			{
				if (!m_Fluid->ApplyImpulse(Feedback.Position, Feedback.Radius, Feedback.Impulse)
					&& !m_Fluid->ApplyImpulse(Feedback.Position, 2.0f * Feedback.Radius,
						Feedback.Impulse))
				{
					++m_LastFeedbackFailureCount;
					++m_TotalFeedbackFailureCount;
				}
			}
		}
		m_PendingFeedback.clear();
		m_DeltaTime = std::isfinite(DeltaTime) && DeltaTime > 0.0f ? DeltaTime : 0.0f;
	}

	bool FluidRigidCoupling::ApplyForce(RigidBodyDynamic* Rigid)
	{
		FluidRigidCouplingResult Result;
		std::vector<ForceAtPoint> PointForces;
		if (!EvaluateCoupling(m_Fluid, m_Settings, Rigid, &Result, &PointForces))
		{
			return false;
		}

		Rigid->ApplyForce(Result.Force);
		Rigid->ApplyTorgue(Result.Torque);
		if (m_DeltaTime <= 0.0f || PointForces.empty())
		{
			return true;
		}

		const Vector3 CellSize = m_Fluid->GetCellSize();
		const float FeedbackRadius = std::max(m_Fluid->GetSettings().SmoothingRadius,
			std::max(CellSize.x, std::max(CellSize.y, CellSize.z)))
			* m_Settings.FeedbackRadiusScale;
		if (!std::isfinite(FeedbackRadius) || FeedbackRadius <= 0.0f)
		{
			return true;
		}
		m_PendingFeedback.reserve(m_PendingFeedback.size() + PointForces.size());
		for (const ForceAtPoint& PointForce : PointForces)
		{
			const Vector3 Impulse = -PointForce.Force * m_DeltaTime;
			if (IsFiniteVector(Impulse) && !Impulse.IsZero())
			{
				m_PendingFeedback.push_back({ PointForce.Position, Impulse, FeedbackRadius });
			}
		}
		return true;
	}

	bool FluidRigidCoupling::Evaluate(const RigidBodyDynamic* Rigid,
		FluidRigidCouplingResult* Result) const
	{
		return EvaluateCoupling(m_Fluid, m_Settings, Rigid, Result, nullptr);
	}

	bool FluidRigidCoupling::IsValid() const
	{
		return m_SettingsValid && m_Fluid != nullptr && m_Fluid->IsInitialized();
	}
}
