#pragma once

#include <vector>
#include <memory>
#include <utility>

#include "../Maths/Vector3.h"
#include <stdexcept>
#include "../Maths/Index3.h"
#include "../Maths/Vector2.h"
#include "SurfaceContactConstraint.h"

namespace Riemann
{
	struct ClothMesh
	{
		std::vector<Vector3> Positions;
		std::vector<Index3> Triangles;
		std::vector<Vector2> MaterialCoordinates;
		std::vector<float> InverseMasses;
		std::vector<std::pair<int, int>> DistanceLinks;
	};
	struct ClothMaterial
	{
		bool StretchEnabled = true, ShearEnabled = true, BendingEnabled = true;
		float StretchUCompliance = 1e-5f, StretchVCompliance = 1e-5f, ShearCompliance = 1e-4f;
		float BendingCompliance = 0.01f;
		float MaxBendingStep = 0.78539816339f;
		bool TethersEnabled = false;
		float TetherCompliance = 0, TetherScale = 1;
		bool IsValid() const;
	};
	struct ClothWind
	{
		Vector3 Velocity = Vector3::Zero();
		float Density = 1.225f, NormalDrag = 0, TangentialDrag = 0;
		bool IsValid() const;
	};
	struct ClothAnimationDrive
	{
		bool Enabled = false, MovePins = true;
		float Compliance = 0.001f, Damping = 5;
		bool IsValid() const;
	};
	struct ClothTriangle
	{
		Index3 Particles;
		Vector3 GradientU, GradientV;
		float Area;
	};
	struct ClothEdge
	{
		int A, B;
		float Length;
	};
	struct ClothBend
	{
		int Particles[4];
		float RestAngle, Weight;
		Vector2 RestAreas;
	};
	class ClothTopology
	{
	public:
		explicit ClothTopology(const ClothMesh& Mesh);
		const std::vector<ClothTriangle>& GetTriangles() const { return m_Triangles; }
		const std::vector<ClothEdge>& GetEdges() const { return m_Edges; }
		const std::vector<ClothEdge>& GetDistanceLinks() const { return m_DistanceLinks; }
		const std::vector<ClothBend>& GetBends() const { return m_Bends; }
		const std::vector<float>& GetVertexAreas() const { return m_VertexAreas; }
		static bool EvaluateStrain(const ClothTriangle& Triangle, const Vector3 Positions[3], int Channel, float& Value, Vector3 Gradients[3]);
		static bool EvaluateBend(const ClothBend& Bend, const Vector3 Positions[4], float& Angle, Vector3 Gradients[4]);
	private:
		std::vector<ClothTriangle> m_Triangles;
		std::vector<ClothEdge> m_Edges;
		std::vector<ClothEdge> m_DistanceLinks;
		std::vector<ClothBend> m_Bends;
		std::vector<float> m_VertexAreas;
	};
}

namespace Riemann
{
	class ClothSystem;
	class Geometry;
	class RigidBody;
	struct ClothCollisionSettings
	{
		bool SurfaceContacts = false;
		bool SelfCollision = false, InterCollision = false;
		bool ExcludeSelfNeighbors = true;
		bool Continuous = false;
		int SweepIterations = 64, SweepRefinements = 64, ContinuousIterations = 16;
		int MaximumContactRefinements = 64;
		int MaximumContactRefinementDepth = 12;
		int MaximumAngularSubsteps = 64;
		float MaximumColliderAngle = 0.25f;
		float ContactTolerance = 1e-5f;
		SurfaceContactMaterial Material;
		bool IsValid() const;
	};
	struct ClothCollisionStats
	{
		size_t Candidates = 0, Contacts = 0;
		size_t SelfContacts = 0, InterContacts = 0;
		size_t ContinuousContacts = 0, UnresolvedSweeps = 0;
		int ContinuousPasses = 0;
		int Substeps = 0;
		int ContactRefinements = 0;
		double MaximumConstraintViolation = 0;
		double MaximumSurfacePenetration = 0;
		bool ProjectionLimitReached = false;
	};
	class ClothCollision
	{
	public:
		explicit ClothCollision(ClothSystem& Cloth);
		~ClothCollision();
		ClothCollision(const ClothCollision&) = delete;
		ClothCollision& operator=(const ClothCollision&) = delete;
		bool AddCollider(Geometry* Shape);
		bool AddCapsule(RigidBody* Body, float Radius, const Vector3& A, const Vector3& B, bool LegacySphere = false);
		bool RemoveCollider(const Geometry* Shape);
		bool RemoveColliders(RigidBody* Body, bool LegacySpheresOnly = false);
		size_t GetColliderCount() const;
		const ClothCollisionStats& GetStats() const;
		void BeginFrame();
		void BeginSubstep(float StartFraction, float EndFraction);
		void Solve(float Dt, bool SeparationOnly = false);
		void EndFrame(bool Commit = true);
	private:
		friend class ClothSystem;
		class SweepFailure final : public std::runtime_error
		{
		public:
			SweepFailure() : std::runtime_error("Cloth continuous collision exhausted its sweep refinement budget") {}
		};
		static void SolveInteractions(const std::vector<ClothCollision*>& Collisions, float Dt, bool Project = true, bool SeparationOnly = false);
		static bool FinishContinuous(const std::vector<ClothCollision*>& Collisions, float Dt);
		static void RecordRefinement(const std::vector<ClothCollision*>& Collisions);
		int RequiredSubsteps() const;
		struct Implementation;
		std::unique_ptr<Implementation> m_Impl;
	};
}

namespace Riemann
{
	class Geometry;
	class RigidBody;
	enum class ClothModel { DistanceLinks, TriangleStrain };

	struct ClothSettings
	{
		ClothSettings();

		int Rows;
		int Columns;
		int SolverIterations;
		float Width;
		float Height;
		float Thickness;
		float ParticleInverseMass;
		float Damping;
		Vector3 Gravity;
		ClothModel Model = ClothModel::DistanceLinks;
		int Substeps = 1;
		float SurfaceDensity = 1;
		ClothMaterial Material;
		ClothWind Wind;
		ClothAnimationDrive AnimationDrive;
		ClothCollisionSettings Collision;
	};

	struct ClothParticle
	{
		Vector3 Position;
		Vector3 PreviousPosition;
		Vector3 PinPosition;
		float InverseMass;
		bool Pinned;
	};

	class ClothSystem
	{
	public:
		ClothSystem(const ClothSettings& settings, const Vector3& center, const Vector3& horizontalAxis, const Vector3& verticalAxis);
		ClothSystem(const ClothSettings& Settings, const ClothMesh& Mesh);
		~ClothSystem();
		ClothSystem(const ClothSystem&) = delete;
		ClothSystem& operator=(const ClothSystem&) = delete;

		void Simulate(float dt);
		static void SimulateGroup(const std::vector<ClothSystem*>& Cloths, float Dt);
		bool PinParticle(int row, int column, const Vector3& worldPosition);
		bool UnpinParticle(int row, int column);
		bool PinParticle(int Index, const Vector3& WorldPosition);
		bool UnpinParticle(int Index);
		bool SetParticlePosition(int Index, const Vector3& Position, bool ResetVelocity = true);
		bool SetParticleVelocity(int Index, const Vector3& Velocity);
		Vector3 GetParticleVelocity(int Index) const;
		void SetMaterial(const ClothMaterial& Material);
		void SetModel(ClothModel Model);
		void SetSolverParameters(int Iterations, int Substeps, float Damping);
		void SetWind(const ClothWind& Wind);
		void SetAnimationDrive(const ClothAnimationDrive& Drive);
		void SetAnimationWeights(const std::vector<float>& Weights);
		void SetAnimationPose(const std::vector<Vector3>& WorldPositions, float BlendTime = 0);
		const std::vector<Vector3>& GetAnimationPose() const { return m_Animation; }
		const ClothSettings& GetSettings() const { return m_Settings; }
		const ClothTopology& GetTopology() const { return *m_Topology; }
		size_t GetTetherCount() const { return m_Tethers.size(); }
		bool AddSphereCollider(RigidBody* body, float radius, const Vector3& worldCenter);
		bool RemoveSphereCollider(RigidBody* body);
		// Shape data is copied; the body is borrowed. Re-register after shape edits.
		// Detached geometry and null-body capsules are static snapshots.
		// General colliders require Collision.SurfaceContacts; spheres also work in legacy mode.
		bool AddCollider(Geometry* Shape);
		bool AddCapsuleCollider(RigidBody* Body, float Radius, const Vector3& A, const Vector3& B);
		bool RemoveCollider(const Geometry* Shape);
		bool RemoveColliders(RigidBody* Body);
		void SetCollisionSettings(const ClothCollisionSettings& Settings);
		size_t GetColliderCount() const;
		const ClothCollisionStats& GetCollisionStats() const;

		int GetRows() const { return m_Settings.Rows; }
		int GetColumns() const { return m_Settings.Columns; }
		int GetParticleCount() const { return (int)m_Particles.size(); }
		const ClothParticle& GetParticle(int row, int column) const { return m_Particles[(size_t)ParticleIndex(row, column)]; }
		ClothParticle& GetParticle(int row, int column) { return m_Particles[(size_t)ParticleIndex(row, column)]; }
		const ClothParticle& GetParticle(int Index) const { return m_Particles.at(Index); }
		ClothParticle& GetParticle(int Index) { return m_Particles.at(Index); }
		Geometry* GetRenderGeometry() const { return m_RenderGeometry; }

	private:
		struct DistanceLink
		{
			int ParticleA;
			int ParticleB;
			float RestLength;
		};

		struct SphereCollider
		{
			RigidBody* Body;
			Vector3 LocalCenter;
			float Radius;
		};
		struct Tether { int Particle, Anchor; float Length, Lambda = 0; };

		int ParticleIndex(int row, int column) const { return row * m_Settings.Columns + column; }
		void SolveDistanceConstraints();
		void SolvePins();
		void SolveSphereCollisions();
		void BuildRenderMesh();
		void UpdateRenderMesh();
		void Initialize(const ClothMesh& Mesh);
		void ValidateSettings();
		static void SimulateBatch(const std::vector<ClothSystem*>& Cloths, float Dt);
		void PrepareStep(float Dt);
		void SolveIteration(float Dt, int Iteration);
		void SolveMaterial(float Dt);
		void SolveTethers(float Dt);
		void UpdateTethers();
		void SampleAnimation(float Dt);
		void SolveAnimation(float Dt);
		void ApplyWind(float Dt);
		float InverseMass(int Index) const;
		bool ValidIndex(int Index) const { return Index >= 0 && Index < GetParticleCount(); }

	private:
		ClothSettings m_Settings;
		std::vector<ClothParticle> m_Particles;
		std::vector<DistanceLink> m_Links;
		std::vector<SphereCollider> m_SphereColliders;
		Geometry* m_RenderGeometry;
		std::unique_ptr<ClothTopology> m_Topology;
		std::unique_ptr<ClothCollision> m_Collision;
		std::vector<Vector3> m_StrainMultipliers, m_AnimationMultipliers;
		std::vector<Vector3> m_StepVelocities;
		std::vector<float> m_BendMultipliers, m_AnimationWeights;
		std::vector<Tether> m_Tethers;
		std::vector<unsigned char> m_TetherAnchors;
		std::vector<Vector3> m_Animation, m_AnimationStart, m_AnimationEnd, m_AnimationVelocity;
		float m_AnimationTime = 0, m_AnimationDuration = 0, m_PreviousDt = 0;
		bool m_ResetAnimationVelocity = false;
	};
}
