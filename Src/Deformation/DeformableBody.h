#pragma once

#include <vector>
#include <stdint.h>
#include "BarycentricAttachment.h"
#include "DeformableCollision.h"
#include "TetrahedralMaterial.h"
#include "TetrahedralAsset.h"
#include "ProjectiveDynamicsProjection.h"
#include "../RigidBodyDynamics/BroadPhase.h"

#include "../Maths/Index3.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	class Geometry;

	struct DeformableBodySettings
	{
		DeformableBodySettings();

		int Substeps;
		int SolverIterations;
		float EdgeCompliance;
		float VolumeCompliance;
		float Damping; // Velocity decay rate per second.
		Vector3 Gravity;
		float YieldStrain; // Elastic strain threshold; rigid edges do not yield.
		float PlasticCreep;
		float MaxPlasticStrain;
		bool GroundEnabled;
		float GroundHeight;
		float ParticleRadius;
		float Restitution;
		float Friction;
		DeformableCollisionSettings Collision;
		TetrahedralSolverSettings MaterialSolver;
		bool ProjectiveMaterials = false;
		ProjectiveDynamicsSettings ProjectiveSolver;
	};

	struct DeformableParticle
	{
		Vector3 Position;
		Vector3 PreviousPosition;
		Vector3 InitialPosition;
		Vector3 PinPosition;
		Vector3 Velocity;
		Vector3 Force;
		float InverseMass;
		bool Pinned;
	};

	struct DeformableEdge
	{
		int ParticleA;
		int ParticleB;
		float RestLength;
		float InitialRestLength;
		float Compliance;
		float TensionCompliance;
		float Lambda;
		bool Explicit = true, Enabled = true;
	};

	struct DeformableTetrahedron
	{
		int Particles[4];
		float RestVolume;
		float InitialRestVolume;
		float Compliance;
		float Lambda;
		std::array<int, 6> Edges = {};
		int Material = -1;
	};

	class DeformableBody
	{
		friend class DeformableStep;
		friend class DeformableCollision;
		friend class DeformableRigidCollision;
		friend class PhysicsWorld;
	public:
		explicit DeformableBody(const DeformableBodySettings& settings = DeformableBodySettings());
		~DeformableBody();
		DeformableBody(const DeformableBody&) = delete;
		DeformableBody& operator=(const DeformableBody&) = delete;
		void InitializeAsset(const TetrahedralAsset& Asset, const Transform& Pose = Transform::Identity());
		const TetrahedralAsset* GetPreparedAsset() const { return m_Asset.IsValid() ? &m_Asset : nullptr; }
		void UseEmbeddedRenderMesh(bool Enabled);
		bool IsEmbeddedRenderMeshEnabled() const { return m_Asset.IsValid() && m_EmbeddedRenderMesh; }

		int AddParticle(const Vector3& position, float inverseMass = 1.0f);
		int AddEdge(int particleA, int particleB, float compliance = -1.0f);
		void SetEdgeCompliance(int edge, float compression, float tension);
		// Tetrahedra also create their six elastic edges, sharing existing edges.
		int AddTetrahedron(int a, int b, int c, int d, float compliance = -1.0f);
		int AddMaterial(const TetrahedralMaterial& Material);
		void SetMaterial(int Index, const TetrahedralMaterial& Material);
		void SetProjectiveMaterialSolver(bool Enabled);
		const ProjectiveDynamicsStats& GetProjectiveMaterialStats() const;
		const std::vector<TetrahedralMaterial>& GetMaterials() const { return m_Materials; }
		void SetTetrahedronMaterial(int Tetrahedron, int Material);
		void ApplyMaterialDensities(double LegacyDensity = 1000);
		TetrahedralMaterialResponse GetTetrahedronMaterialResponse(int Tetrahedron) const;
		const TetrahedralRestShape& GetTetrahedronRestShape(int Tetrahedron) const;
		double GetMaterialEnergy() const;
		bool SetSurfaceTriangles(const std::vector<Index3>& triangles);
		bool PinParticle(int index, const Vector3& position);
		bool SetPinTarget(int index, const Vector3& position);
		bool UnpinParticle(int index);
		bool SetParticlePosition(int index, const Vector3& position, bool resetVelocity = true);
		bool SetParticleVelocity(int index, const Vector3& velocity);
		bool AddForce(int index, const Vector3& force);
		bool AddImpulse(int index, const Vector3& impulse);
		// The total impulse is distributed by normalized radial weights.
		int AddRadialImpulse(const Vector3& center, float radius, const Vector3& impulse);
		void Simulate(float dt);
		static void SimulateGroup(const std::vector<DeformableBody*>& Bodies, float Dt);
		uint64_t AttachToWorld(const BarycentricPoint& Point, const Vector3& Target, double Compliance = 0);
		uint64_t AttachToBody(const BarycentricPoint& Point, DeformableBody* Other, const BarycentricPoint& OtherPoint, double Compliance = 0);
		bool SetAttachmentTarget(uint64_t Id, const Vector3& Target);
		bool RemoveAttachment(uint64_t Id);
		size_t RemoveAttachments(const DeformableBody* Other);
		void ClearAttachments();
		size_t GetAttachmentCount() const { return m_Attachments.size(); }
		Maths::TVector3<double> GetAttachmentForce(uint64_t Id) const;
		Vector3 GetPointPosition(const BarycentricPoint& Point) const;
		// Restore the original shape and material state while retaining pin flags.
		void Reset();

		int GetParticleCount() const { return (int)m_Particles.size(); }
		const DeformableParticle& GetParticle(int index) const { return m_Particles[(size_t)index]; }
		const std::vector<DeformableParticle>& GetParticles() const { return m_Particles; }
		const std::vector<DeformableEdge>& GetEdges() const { return m_Edges; }
		const std::vector<DeformableTetrahedron>& GetTetrahedra() const { return m_Tetrahedra; }
		const std::vector<Index3>& GetSurfaceTriangles() const { return m_SurfaceTriangles; }
		const DeformableBodySettings& GetSettings() const { return m_Settings; }
		void SetCollisionSettings(const DeformableCollisionSettings& Settings);
		const DeformableCollisionStats& GetCollisionStats() const { return m_CollisionStats; }
		Geometry* GetRenderGeometry() const { return m_RenderGeometry; }
		float GetTetrahedronVolume(int index) const;

	private:
		static void SimulateGroup(const std::vector<DeformableBody*>& Bodies, float Dt, float FrameDt, bool First, bool Last);
		void RequireIdle() const;
		bool m_CoupledStepActive = false;
		BroadPhaseQuery m_CollisionQuery;
		size_t m_RigidAttachmentCount = 0;
		bool IsValidParticle(int index) const;
		float GetInverseMass(int index) const;
		void UpdatePlasticity(float dt);
		int AddEdgeInternal(int A, int B, float Compliance, bool Explicit);
		void RefreshElasticEdges();
		void SolveMaterials(float Dt);
		void BeginMaterialStep(float Dt);
		void SolveEdges(float dt);
		void SolveVolumes(float dt);
		void SolveGround();
		void UpdateRenderGeometry();
		void PrepareStep(float Dt);
		void FinishStep(float Dt);
		void DisconnectAttachments();
		struct Attachment
		{
			uint64_t Id = 0;
			BarycentricPoint Point, OtherPoint;
			DeformableBody* Other = nullptr;
			Vector3 Target = Vector3::Zero();
			double Compliance = 0;
			Maths::TVector3<double> Force = Maths::TVector3<double>(0);
		};

	private:
		DeformableBodySettings m_Settings;
		TetrahedralAsset m_Asset;
		bool m_EmbeddedRenderMesh = true;
		DeformableCollisionStats m_CollisionStats;
		std::vector<DeformableParticle> m_Particles;
		std::vector<DeformableEdge> m_Edges;
		std::vector<DeformableTetrahedron> m_Tetrahedra;
		std::vector<TetrahedralMaterial> m_Materials;
		struct MaterialElement { TetrahedralRestShape Shape; TetrahedralMaterialState State; };
		std::vector<std::unique_ptr<MaterialElement>> m_MaterialElements;
		std::vector<int> m_ActiveMaterialTetrahedra;
		struct ProjectiveMaterialData
		{
			ProjectiveDynamicsProjection Solver;
			std::vector<ProjectiveDynamicsParticle> Particles;
			std::vector<ProjectiveDynamicsElement> Elements;
		};
		std::unique_ptr<ProjectiveMaterialData> m_ProjectiveMaterials;
		std::vector<Index3> m_SurfaceTriangles;
		std::vector<unsigned char> m_GroundContacts;
		std::vector<float> m_GroundCorrections;
		std::vector<Attachment> m_Attachments;
		std::vector<DeformableBody*> m_AttachmentUsers;
		uint64_t m_NextAttachmentId = 1;
		Geometry* m_RenderGeometry = nullptr;
		std::vector<DeformationGradient::DVector> m_RenderNormals;
		bool m_RenderTopologyDirty = true;
	};
}
