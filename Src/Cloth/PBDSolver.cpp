#include "PBDSolver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include "../Collision/GeometryObject.h"
#include "../RigidBodyDynamics/RigidBody.h"

namespace Riemann
{
	namespace
	{
		bool Finite(const Vector3& Value)
		{
			return std::isfinite(Value.x) && std::isfinite(Value.y) && std::isfinite(Value.z);
		}

		float NonNegative(float Value, float Default)
		{
			return std::isfinite(Value) && Value >= 0.0f ? Value : Default;
		}

		float InverseMass(RigidBody* Body)
		{
			RigidBodyDynamic* Dynamic = Body ? Body->CastDynamic() : nullptr;
			return Dynamic && !Dynamic->Freezing ? Dynamic->InvMass : 0.0f;
		}
		Vector3 WorldAnchor(RigidBody* Body, const Vector3& Anchor)
		{
			return Body ? Body->BodyLocalToWorld(Anchor) : Anchor;
		}

		Vector3 PointVelocity(RigidBody* Body, const Vector3& Point)
		{
			if (!Body || Body->mRigidType == RigidType::Static) return Vector3::Zero();
			RigidBodyDynamic* Dynamic = Body->CastDynamic();
			return Dynamic && Dynamic->Freezing ? Vector3::Zero() : Body->V + Body->W.Cross(Point - Body->X);
		}

		Vector3 RotationVector(Quaternion Rotation)
		{
			Rotation = Rotation.Unit();
			if (Rotation.w < 0.0f) Rotation = -Rotation;
			const Vector3 Vector(Rotation.x, Rotation.y, Rotation.z);
			const float Length = Vector.Length();
			return Length > 1.0e-8f ? Vector * (2.0f * std::atan2(Length, Rotation.w) / Length) : Vector * 2.0f;
		}

		void Rotate(RigidBody* Body, const Vector3& Rotation)
		{
			const float Angle = Rotation.Length();
			if (Angle <= 1.0e-8f) return;
			Quaternion Delta;
			Delta.FromRotationAxis(Rotation / Angle, Angle);
			Body->Q = (Delta * Body->Q).Unit();
		}
		float MaximumImpulse(RigidBody* Body)
		{
			RigidBodyDynamic* Dynamic = Body ? Body->CastDynamic() : nullptr;
			return Dynamic ? std::max(0.0f, Dynamic->MaxContactImpulse) : std::numeric_limits<float>::max();
		}
	}

	PBDSolver::PBDSolver(const PBDSolverSettings& Settings)
	{
		SetSettings(Settings);
	}

	void PBDSolver::SetSettings(const PBDSolverSettings& Settings)
	{
		if (!m_BackendConfigured || Settings.Backend != m_Settings.Backend
			|| Settings.AllowBackendFallback != m_Settings.AllowBackendFallback)
		{
			m_BackendConfigured = m_Backend.Configure(Settings.Backend, Settings.AllowBackendFallback);
			if (!m_BackendConfigured) throw std::runtime_error(m_Backend.GetStatistics().Message);
		}
		m_Settings = Settings;
		m_Settings.PositionIterations = std::max(1, std::min(256, Settings.PositionIterations));
		m_Settings.VelocityIterations = std::max(1, std::min(256, Settings.VelocityIterations));
		m_Settings.PenetrationSlop = NonNegative(Settings.PenetrationSlop, 0.001f);
		m_Settings.MaxPositionCorrection = std::max(1.0e-6f, NonNegative(Settings.MaxPositionCorrection, 0.2f));
		m_Settings.MaxAngularCorrection = std::max(1.0e-6f, std::min(0.5f, NonNegative(Settings.MaxAngularCorrection, 0.2f)));
		m_Settings.RestitutionThreshold = NonNegative(Settings.RestitutionThreshold, 1.0f);
	}

	void PBDSolver::ResetState()
	{
		m_Bodies.clear();
		m_Contacts.clear();
		m_Geometries.clear();
		m_Manifolds.clear();
		m_ManifoldPointers.clear();
		m_StepDt = 0.0f;
	}

	void PBDSolver::BeginStep(const std::vector<RigidBodyDynamic*>& Bodies, float Dt)
	{
		ResetState();
		if (!std::isfinite(Dt) || Dt <= 0.0f) return;
		m_StepDt = Dt;
		m_Bodies.reserve(Bodies.size());
		for (RigidBodyDynamic* Body : Bodies)
		{
			if (!Body) continue;
			BodyState State;
			State.Body = Body;
			State.PreviousPosition = State.PredictedPosition = Body->X;
			State.PreviousRotation = State.PredictedRotation = Body->Q;
			State.PredictedVelocity = Body->V;
			State.PredictedAngularVelocity = Body->W;
			m_Bodies.push_back(State);
		}
	}

	void PBDSolver::PredictPoses(float Dt)
	{
		if (!std::isfinite(Dt) || Dt <= 0.0f) return;
		for (BodyState& State : m_Bodies)
		{
			RigidBodyDynamic* Body = State.Body;
			State.PredictedVelocity = Body->V;
			State.PredictedAngularVelocity = Body->W;
			if (!Body->Sleeping && !Body->Freezing)
			{
				const float Fraction = Body->HadCCDContact ? Body->CCDMotionFraction : 1.0f;
				const Vector3 Velocity = Body->HadCCDContact ? Body->CCDPreSolveVelocity : Body->V;
				const Vector3 Angular = Body->HadCCDContact ? Body->CCDPreSolveAngularVelocity : Body->W;
				Body->X += Velocity * (Dt * Fraction);
				Rotate(Body, Angular * (Dt * Fraction));
			}
			State.PredictedPosition = Body->X;
			State.PredictedRotation = Body->Q;
			State.PreviousPosition = Body->X;
			State.PreviousRotation = Body->Q;
		}
	}

	Vector3 PBDSolver::GetPredictedPointVelocity(RigidBody* Body, const Vector3& Point) const
	{
		if (!Body || Body->mRigidType == RigidType::Static) return Vector3::Zero();
		RigidBodyDynamic* Dynamic = Body->CastDynamic();
		if (Dynamic && Dynamic->Freezing) return Vector3::Zero();
		for (const BodyState& State : m_Bodies)
		{
			if (State.Body == Body) return State.PredictedVelocity + State.PredictedAngularVelocity.Cross(Point - State.PredictedPosition);
		}
		return PointVelocity(Body, Point);
	}

	void PBDSolver::AddContacts(const std::vector<Geometry*>& Geometries, const std::vector<ContactManifold*>& Manifolds)
	{
		for (const ContactManifold* Manifold : Manifolds)
		{
			if (!Manifold || Manifold->indexA < 0 || Manifold->indexB < 0
				|| (size_t)Manifold->indexA >= Geometries.size() || (size_t)Manifold->indexB >= Geometries.size()) continue;
			Geometry* GeometryA = Geometries[(size_t)Manifold->indexA];
			Geometry* GeometryB = Geometries[(size_t)Manifold->indexB];
			if (!GeometryA || !GeometryB || GeometryA->IsTriggerEnabled() || GeometryB->IsTriggerEnabled()
				|| !GeometryA->IsSimulationEnabled() || !GeometryB->IsSimulationEnabled()) continue;
			RigidBody* A = GeometryA->GetParent<RigidBody>();
			RigidBody* B = GeometryB->GetParent<RigidBody>();
			if (A == B || InverseMass(A) + InverseMass(B) <= 1.0e-9f) continue;
			for (int Index = 0; Index < Manifold->NumContactPointCount; ++Index)
			{
				const Contact& ContactPoint = Manifold->ContactPoints[Index];
				if (!Finite(ContactPoint.Normal) || !Finite(ContactPoint.PositionWorldA) || !Finite(ContactPoint.PositionWorldB)) continue;
				const Vector3 Normal = ContactPoint.Normal.SafeUnit();
				if (Normal.SquareLength() < 0.5f) continue;
				bool Existing = false;
				for (const ContactState& State : m_Contacts)
				{
					if (State.GeometryA == GeometryA && State.GeometryB == GeometryB && State.Normal.Dot(Normal) > 0.98f
						&& (WorldAnchor(A, State.AnchorA) - ContactPoint.PositionWorldA).SquareLength()
						+ (WorldAnchor(B, State.AnchorB) - ContactPoint.PositionWorldB).SquareLength() < 0.002f)
					{
						Existing = true;
						break;
					}
				}
				if (Existing) continue;
				ContactState State;
				State.GeometryA = GeometryA; State.GeometryB = GeometryB;
				State.BodyA = A; State.BodyB = B;
				// Narrow-phase PositionLocal fields are world-space lever arms. Store
				// actual body-local anchors so rotation and compound COMs are respected.
				State.AnchorA = A ? A->WorldToBodyLocal(ContactPoint.PositionWorldA) : ContactPoint.PositionWorldA;
				State.AnchorB = B ? B->WorldToBodyLocal(ContactPoint.PositionWorldB) : ContactPoint.PositionWorldB;
				State.Normal = Normal;
				Normal.GetPerpVectors(State.Tangent, State.Binormal);
				State.InitialDepth = std::max(0.0f, ContactPoint.PenetrationDepth);
				const float Restitution = std::max(A ? A->GetRestitution() : 0.0f, B ? B->GetRestitution() : 0.0f);
				const float Speed = (GetPredictedPointVelocity(B, ContactPoint.PositionWorldB)
					- GetPredictedPointVelocity(A, ContactPoint.PositionWorldA)).Dot(Normal);
				State.RestitutionVelocity = Speed < -m_Settings.RestitutionThreshold ? -Restitution * Speed : 0.0f;
				State.DynamicFriction = std::sqrt(std::max(0.0f, (A ? A->GetFrictionDynamic() : 0.0f) * (B ? B->GetFrictionDynamic() : 0.0f)));
				State.StaticFriction = std::sqrt(std::max(0.0f, (A ? A->GetFrictionStatic() : 0.0f) * (B ? B->GetFrictionStatic() : 0.0f)));
				m_Contacts.push_back(State);
			}
		}
	}



	void PBDSolver::ExecuteBackend(float Dt, PBDBackendStage Stage)
	{
		if (m_Contacts.empty()) return;
		m_BackendBodies.clear();
		m_BackendContacts.clear();
		m_BackendBodyPointers.clear();
		std::unordered_map<RigidBody*, uint32_t> Indices;
		auto Vector = [](const Vector3& Value, float W = 0.0f) -> PBDBackendVector
		{
			return { Value.x, Value.y, Value.z, W };
		};
		auto BodyIndex = [&](RigidBody* Body) -> uint32_t
		{
			const auto Found = Indices.find(Body);
			if (Found != Indices.end()) return Found->second;
			const uint32_t Index = (uint32_t)m_BackendBodies.size();
			PBDBackendBody Packed = {};
			Packed.Rotation.w = 1.0f;
			Packed.LinearVelocity.w = std::numeric_limits<float>::max();
			if (Body)
			{
				RigidBodyDynamic* Dynamic = Body->CastDynamic();
				const bool Frozen = Dynamic && Dynamic->Freezing;
				Packed.Position = Vector(Body->X, InverseMass(Body));
				Packed.Rotation = { Body->Q.x, Body->Q.y, Body->Q.z, Body->Q.w };
				Packed.LinearVelocity = Vector(Body->V, MaximumImpulse(Body));
				Packed.AngularVelocity = Vector(Body->W);
				Packed.Flags.x = Dynamic && !Frozen ? 1 : 0;
				Packed.Flags.y = Body->mRigidType != RigidType::Static && !Frozen ? 1 : 0;
				if (Packed.Flags.x)
				{
					Packed.InverseInertia0 = { Body->InvInertia.mat[0][0], Body->InvInertia.mat[0][1], Body->InvInertia.mat[0][2], 0 };
					Packed.InverseInertia1 = { Body->InvInertia.mat[1][0], Body->InvInertia.mat[1][1], Body->InvInertia.mat[1][2], 0 };
					Packed.InverseInertia2 = { Body->InvInertia.mat[2][0], Body->InvInertia.mat[2][1], Body->InvInertia.mat[2][2], 0 };
				}
			}
			Indices.emplace(Body, Index);
			m_BackendBodies.push_back(Packed);
			m_BackendBodyPointers.push_back(Body);
			return Index;
		};
		m_BackendContacts.reserve(m_Contacts.size());
		for (const ContactState& State : m_Contacts)
		{
			PBDBackendContact Contact = {};
			Contact.Bodies.x = BodyIndex(State.BodyA);
			Contact.Bodies.y = BodyIndex(State.BodyB);
			Contact.AnchorA = Vector(State.BodyA ? State.AnchorA - State.BodyA->CM : State.AnchorA);
			Contact.AnchorB = Vector(State.BodyB ? State.AnchorB - State.BodyB->CM : State.AnchorB);
			Contact.Normal = Vector(State.Normal);
			Contact.Tangent = Vector(State.Tangent);
			Contact.Binormal = Vector(State.Binormal);
			Contact.Lambda = { State.PositionLambda, State.VelocityLambda, State.TangentLambda, State.BinormalLambda };
			Contact.Material = { State.RestitutionVelocity, State.DynamicFriction, State.StaticFriction, 0 };
			m_BackendContacts.push_back(Contact);
		}
		PBDBackendParameters Parameters;
		Parameters.DeltaTime = Dt;
		Parameters.Alpha = (float)std::min((double)std::numeric_limits<float>::max(), (double)GetContactCompliance() / ((double)Dt * Dt));
		Parameters.PenetrationSlop = m_Settings.PenetrationSlop;
		Parameters.MaxPositionCorrection = m_Settings.MaxPositionCorrection;
		Parameters.MaxAngularCorrection = m_Settings.MaxAngularCorrection;
		if (!m_Backend.Execute(m_BackendBodies, m_BackendContacts, Parameters, Stage))
		{
			m_BackendConfigured = false;
			throw std::runtime_error(m_Backend.GetStatistics().Message);
		}
		for (size_t Index = 0; Index < m_BackendBodies.size(); ++Index)
		{
			const PBDBackendBody& Packed = m_BackendBodies[Index];
			if (!Packed.Flags.z) continue;
			RigidBodyDynamic* Body = m_BackendBodyPointers[Index]->CastDynamic();
			Body->X = Vector3(Packed.Position.x, Packed.Position.y, Packed.Position.z);
			Body->Q = Quaternion(Packed.Rotation.x, Packed.Rotation.y, Packed.Rotation.z, Packed.Rotation.w);
			Body->V = Vector3(Packed.LinearVelocity.x, Packed.LinearVelocity.y, Packed.LinearVelocity.z);
			Body->W = Vector3(Packed.AngularVelocity.x, Packed.AngularVelocity.y, Packed.AngularVelocity.z);
			Body->Wakeup();
		}
		for (size_t Index = 0; Index < m_Contacts.size(); ++Index)
		{
			const PBDBackendVector& Lambda = m_BackendContacts[Index].Lambda;
			m_Contacts[Index].PositionLambda = Lambda.x;
			m_Contacts[Index].VelocityLambda = Lambda.y;
			m_Contacts[Index].TangentLambda = Lambda.z;
			m_Contacts[Index].BinormalLambda = Lambda.w;
		}
	}

	void PBDSolver::SolvePositions(float Dt)
	{
		if (!std::isfinite(Dt) || Dt <= 0.0f) return;
		for (BodyState& State : m_Bodies)
		{
			if (State.Body->Freezing) continue;
			// Joint projection occurs between contact iterations. Preserve its pose
			// correction separately from geometric contact depenetration.
			State.JointTranslation += State.Body->X - State.PreviousPosition;
			State.JointRotation = (State.Body->Q * State.PreviousRotation.Conjugate() * State.JointRotation).Unit();
		}
		ExecuteBackend(Dt, PBDBackendStage::Position);
		for (BodyState& State : m_Bodies)
		{
			State.PreviousPosition = State.Body->X;
			State.PreviousRotation = State.Body->Q;
		}
	}

	void PBDSolver::ReconstructVelocities(float Dt)
	{
		if (!std::isfinite(Dt) || Dt <= 0.0f) return;
		for (const BodyState& State : m_Bodies)
		{
			RigidBodyDynamic* Body = State.Body;
			if (Body->Freezing || Body->Sleeping) continue;
			// A CCD-truncated path retains its post-impact physical velocity. Joint
			// corrections contribute pose-derived velocities, while contact correction
			// uses the final Jacobian below to avoid nonlinear depenetration energy.
			Body->V = State.PredictedVelocity + State.JointTranslation / Dt;
			Body->W = State.PredictedAngularVelocity + RotationVector(State.JointRotation) / Dt;
		}
		ExecuteBackend(Dt, PBDBackendStage::Reconstruct);
	}

	void PBDSolver::SolveVelocities(float Dt)
	{
		if (!std::isfinite(Dt) || Dt <= 0.0f) return;
		ExecuteBackend(Dt, PBDBackendStage::Velocity);
	}

	void PBDSolver::EndStep(float Dt)
	{
		m_Geometries.clear(); m_Manifolds.clear(); m_ManifoldPointers.clear();
		if (!std::isfinite(Dt) || Dt <= 0.0f) return;
		auto GeometryIndex = [this](Geometry* GeometryObject) -> int
		{
			const auto Found = std::find(m_Geometries.begin(), m_Geometries.end(), GeometryObject);
			if (Found != m_Geometries.end()) return (int)(Found - m_Geometries.begin());
			m_Geometries.push_back(GeometryObject);
			return (int)m_Geometries.size() - 1;
		};
		for (const ContactState& State : m_Contacts)
		{
			const int A = GeometryIndex(State.GeometryA), B = GeometryIndex(State.GeometryB);
			ContactManifold* Manifold = nullptr;
			for (ContactManifold& Existing : m_Manifolds)
			{
				if (Existing.indexA == A && Existing.indexB == B && Existing.NumContactPointCount < MAX_CONTACT_POINTS)
				{
					Manifold = &Existing; break;
				}
			}
			if (!Manifold)
			{
				m_Manifolds.emplace_back();
				Manifold = &m_Manifolds.back();
				Manifold->indexA = A; Manifold->indexB = B;
			}
			Contact& Point = Manifold->ContactPoints[Manifold->NumContactPointCount++];
			Point.PositionWorldA = WorldAnchor(State.BodyA, State.AnchorA);
			Point.PositionWorldB = WorldAnchor(State.BodyB, State.AnchorB);
			Point.PositionLocalA = State.BodyA ? Point.PositionWorldA - State.BodyA->X : Point.PositionWorldA;
			Point.PositionLocalB = State.BodyB ? Point.PositionWorldB - State.BodyB->X : Point.PositionWorldB;
			Point.Normal = State.Normal; Point.Tangent = State.Tangent; Point.Binormal = State.Binormal;
			Point.PenetrationDepth = State.InitialDepth;
			Point.totalImpulseNormal = std::max(0.0f, State.PositionLambda / Dt + State.VelocityLambda);
			Point.totalImpulseTangent = State.TangentLambda;
			Point.totalImpulseBinormal = State.BinormalLambda;
		}
		for (ContactManifold& Manifold : m_Manifolds) m_ManifoldPointers.push_back(&Manifold);
	}

	float PBDSolver::GetContactPositionLambda(size_t Index) const
	{
		return Index < m_Contacts.size() ? m_Contacts[Index].PositionLambda : 0.0f;
	}

	bool PBDSolver::CanSleep(const RigidBodyDynamic* Body) const
	{
		if (!Body || m_StepDt <= 0.0f || Body->V.SquareLength() + Body->W.SquareLength() > 1.0e-4f) return false;
		for (const ContactState& State : m_Contacts)
		{
			if ((State.BodyA == Body || State.BodyB == Body) && State.PositionLambda / m_StepDt + State.VelocityLambda > 1.0e-7f) return true;
		}
		return false;
	}
}
