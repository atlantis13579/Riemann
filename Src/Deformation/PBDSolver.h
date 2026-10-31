#pragma once

#include <vector>

#include "../Maths/Quaternion.h"
#include "../Maths/Vector3.h"
#include "../RigidBodyDynamics/CollidingContact.h"
#include "PBDBackend.h"

namespace Riemann
{
	class Geometry;
	class RigidBody;
	class RigidBodyDynamic;

	struct PBDSolverSettings
	{
		int PositionIterations = 12;
		int VelocityIterations = 4;
		float PenetrationSlop = 0.001f;
		float MaxPositionCorrection = 0.2f;
		float MaxAngularCorrection = 0.2f;
		float RestitutionThreshold = 1.0f;
		SolverBackend Backend = SolverBackend::CPU;
		bool AllowBackendFallback = true;
	};

	// Rigid position projection with angular effective mass and a separate
	// restitution/friction pass. The world supplies collision candidates.
	class PBDSolver
	{
	public:
		explicit PBDSolver(const PBDSolverSettings& Settings = PBDSolverSettings());
		virtual ~PBDSolver() {}
		const PBDSolverSettings& GetSettings() const { return m_Settings; }
		void SetSettings(const PBDSolverSettings& Settings);
		SolverBackend GetBackend() const { return m_Backend.GetStatistics().Backend; }
		const PBDBackendStatistics& GetBackendStatistics() const { return m_Backend.GetStatistics(); }
		void ResetState();
		void BeginStep(const std::vector<RigidBodyDynamic*>& Bodies, float Dt);
		void PredictPoses(float Dt);
		void AddContacts(const std::vector<Geometry*>& Geometries, const std::vector<ContactManifold*>& Manifolds);
		void SolvePositions(float Dt);
		void ReconstructVelocities(float Dt);
		void SolveVelocities(float Dt);
		void EndStep(float Dt);
		size_t GetContactCount() const { return m_Contacts.size(); }
		float GetContactPositionLambda(size_t Index) const;
		virtual bool CanSleep(const RigidBodyDynamic* Body) const;
		const std::vector<Geometry*>& GetContactGeometries() const { return m_Geometries; }
		const std::vector<ContactManifold*>& GetContactManifolds() const { return m_ManifoldPointers; }

	protected:
		virtual float GetContactCompliance() const { return 0.0f; }

	private:
		struct BodyState
		{
			RigidBodyDynamic* Body = nullptr;
			Vector3 PreviousPosition, PredictedPosition, PredictedVelocity, PredictedAngularVelocity;
			Quaternion PreviousRotation, PredictedRotation;
			Vector3 JointTranslation = Vector3::Zero();
			Quaternion JointRotation = Quaternion::One();
		};
		struct ContactState
		{
			Geometry* GeometryA = nullptr;
			Geometry* GeometryB = nullptr;
			RigidBody* BodyA = nullptr;
			RigidBody* BodyB = nullptr;
			Vector3 AnchorA, AnchorB, Normal, Tangent, Binormal;
			float PositionLambda = 0.0f;
			float VelocityLambda = 0.0f;
			float TangentLambda = 0.0f;
			float BinormalLambda = 0.0f;
			float RestitutionVelocity = 0.0f;
			float DynamicFriction = 0.0f;
			float StaticFriction = 0.0f;
			float InitialDepth = 0.0f;
		};
		Vector3 GetPredictedPointVelocity(RigidBody* Body, const Vector3& Point) const;
		void ExecuteBackend(float Dt, PBDBackendStage Stage);

		PBDSolverSettings m_Settings;
		PBDBackend m_Backend;
		bool m_BackendConfigured = false;
		std::vector<PBDBackendBody> m_BackendBodies;
		std::vector<PBDBackendContact> m_BackendContacts;
		std::vector<RigidBody*> m_BackendBodyPointers;
		float m_StepDt = 0.0f;
		std::vector<BodyState> m_Bodies;
		std::vector<ContactState> m_Contacts;
		std::vector<Geometry*> m_Geometries;
		std::vector<ContactManifold> m_Manifolds;
		std::vector<ContactManifold*> m_ManifoldPointers;
	};
}