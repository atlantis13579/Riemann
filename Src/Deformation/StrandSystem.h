#pragma once

#include <memory>
#include <vector>
#include <cstdint>
#include <stdexcept>
#include "../Maths/Transform.h"
#include "SurfaceContactConstraint.h"

namespace Riemann
{
	using StrandVector = Maths::TVector3<double>;
	struct StrandMatrix
	{
		double Values[3][3] = {};
		double* operator[](int I) { return Values[I]; }
		const double* operator[](int I) const { return Values[I]; }
		static StrandMatrix Identity();
		static StrandMatrix Diagonal(const StrandVector& V);
		static StrandMatrix Rotation(const Quaternion& Q);
		static StrandMatrix Cross(const StrandVector& V);
		StrandMatrix Transpose() const;
		StrandMatrix operator+(const StrandMatrix& B) const;
		StrandMatrix operator*(const StrandMatrix& B) const;
		StrandMatrix operator*(double Scale) const;
		StrandVector operator*(const StrandVector& V) const;
		bool Solve(const StrandVector& Right, StrandVector& Result, double RelativePivotTolerance) const;
	};
	struct StrandMaterial
	{
		double YoungModulus = 100000, PoissonRatio = 0.3, Density = 1000;
		double RadiusX = 0.01, RadiusY = 0.01, ShearAreaFactor = 1;
		bool IsValid() const;
		double Area() const;
		StrandVector StretchShearStiffness(double Length) const;
		StrandVector BendTwistStiffness(double Length) const;
		StrandVector DirectorInertia(double Length) const;
	};
	struct StrandNumerics
	{
		double MinimumLength = 1e-6, QuaternionTolerance = 1e-4;
		double RotationSeriesAngle = 1e-3, RelativePivotTolerance = 1e-14;
		double MaximumPositionCorrectionRatio = 0.5, MaximumAngularCorrection = 0.5;
		bool IsValid() const;
	};
	struct StrandResponse
	{
		StrandVector Value = StrandVector(0), Stiffness = StrandVector(0);
		StrandMatrix PositionA, PositionB, RotationA, RotationB;
		double Energy = 0;
	};
	class StrandElasticity
	{
	public:
		static std::vector<Quaternion> PrepareDirectors(const std::vector<Vector3>& Positions, const std::vector<Quaternion>& Directors,
			const Vector3& ReferenceNormal, double TangentTolerance, const StrandNumerics& Numerics = {});
		static Quaternion Increment(const StrandVector& Rotation);
		static Quaternion Interpolate(const Quaternion& A, const Quaternion& B, double Fraction);
		static StrandVector Log(const Quaternion& Rotation);
		static StrandResponse StretchShear(const Vector3& A, const Vector3& B, const Quaternion& Director,
			double RestLength, const StrandMaterial& Material, const StrandNumerics& Numerics = {});
		static StrandResponse BendTwist(const Quaternion& A, const Quaternion& B, const Quaternion& RestRelative,
			double DualLength, const StrandMaterial& Material, const StrandNumerics& Numerics = {});
	};
}

namespace Riemann
{
	class Geometry;
	class RigidBody;
	struct StrandCollisionSettings
	{
		bool Enabled=false, SelfCollision=false, InterGuideCollision=false, Continuous=true;
		double RadiusScale=1, Margin=0, ContactTolerance=1e-5, MaximumColliderAngle=0.25;
		int RootExclusionSegments=0, SweepIterations=64, SweepRefinements=64, SeparationIterations=24;
		int MaximumRefinements=64, MaximumRefinementDepth=12;
		SurfaceContactMaterial Material;
		bool IsValid() const;
	};
	struct StrandCollisionStats
	{
		uint64_t Candidates=0, Contacts=0, RigidContacts=0, SelfContacts=0, InterGuideContacts=0, ContinuousContacts=0;
		uint64_t PinnedContacts=0, UnresolvedSweeps=0, Substeps=0, Refinements=0;
		double MaximumPenetration=0, MaximumConstraintViolation=0;
		bool ProjectionLimitReached=false;
		void Accumulate(const StrandCollisionStats& Other);
	};
	struct StrandContactGuide
	{
		size_t Id=0;
		std::vector<SurfaceContactParticle> Particles;
		std::vector<Vector3> Start;
		std::vector<double> RestArc;
		double Radius=0;
		bool Rooted=false;
		StrandVector* RootImpulse=nullptr;
	};
	class StrandCollision
	{
	public:
		class RefinementNeeded : public std::runtime_error
		{ public: RefinementNeeded() : std::runtime_error("Strand contact interval needs refinement") {} };
		explicit StrandCollision(const StrandCollisionSettings& Settings);
		~StrandCollision();
		void SetSettings(const StrandCollisionSettings& Settings);
		bool AddCollider(Geometry* Shape);
		bool AddCapsule(RigidBody* Body,float Radius,const Vector3& A,const Vector3& B);
		bool RemoveCollider(const Geometry* Shape);
		void ClearColliders();
		void RemoveBody(RigidBody* Body);
		size_t GetColliderCount() const;
		const StrandCollisionStats& GetStats() const;
		void BeginFrame();
		int RequiredSubsteps(int Maximum) const;
		void BeginSubstep(float Begin,float End,const std::vector<StrandContactGuide>& Guides);
		void Solve(double Dt,bool SeparationOnly=false);
		bool Finish(double Dt);
		void EndSubstep();
		void RecordRefinement();
		void EndFrame(bool Commit);
	private:
		struct Storage;
		std::unique_ptr<Storage> m_Data;
	};
}

namespace Riemann
{
	class RigidBody;
	class PhysicsWorld;
	class Geometry;
	class StrandGroom;
	struct StrandGroomDefinition;
	struct GroomCurveState;
	enum class StrandSolver { SequentialBlocks, CoupledBanded };
	enum class StrandAlgorithm { DistanceConstraints, CosseratRods };
	struct StrandGuide
	{
		std::vector<Vector3> Positions;
		std::vector<Quaternion> Directors;
		Vector3 ReferenceNormal = Vector3::UnitX();
		StrandMaterial Material;
		bool PinRoot = true;
	};
	struct StrandSettings
	{
		StrandAlgorithm Algorithm = StrandAlgorithm::CosseratRods;
		StrandSolver Solver = StrandSolver::CoupledBanded;
		int Substeps = 4, SolverIterations = 24, MaximumSubsteps = 128;
		double LinearDamping = 1, AngularDamping = 1, MaximumRootAngle = 0.25, RestTangentTolerance = 1e-4;
		Vector3 Gravity = Vector3(0,-9.8f,0);
		StrandNumerics Numerics;
		StrandCollisionSettings Collision;
		bool IsValid() const;
	};
	struct StrandParticle
	{
		Vector3 Position = Vector3::Zero(), Velocity = Vector3::Zero();
		double InverseMass = 0;
		bool Pinned = false;
	};
	struct StrandDirector
	{
		Quaternion Orientation = Quaternion::One();
		StrandVector AngularMomentum = StrandVector(0), InverseInertia = StrandVector(0);
		bool Pinned = false;
	};
	struct StrandStats
	{
		int Substeps = 0;
		uint64_t ConstraintSolves = 0, FailedSteps = 0;
		size_t SimulatedGuides = 0, InterpolatedGuides = 0;
		double MaximumStretchShear = 0, MaximumBendTwist = 0, ElasticEnergy = 0;
		double Time = 0;
	};
	class StrandSystem
	{
	public:
		StrandSystem(const std::vector<StrandGuide>& Guides, const StrandSettings& Settings = {});
		~StrandSystem();
		StrandSystem(const StrandSystem&) = delete;
		StrandSystem& operator=(const StrandSystem&) = delete;
		void Simulate(float Dt);
		void Reset();
		void SetSolver(StrandSolver Solver);
		void SetAlgorithm(StrandAlgorithm Algorithm);
		void SetCollisionSettings(const StrandCollisionSettings& Settings);
		bool AddCollider(Geometry* Shape);
		bool AddCapsuleCollider(RigidBody* Body,float Radius,const Vector3& A,const Vector3& B);
		bool RemoveCollider(const Geometry* Shape);
		void ClearColliders();
		size_t GetColliderCount() const;
		const StrandCollisionStats& GetCollisionStats() const;
		void ConfigureGroom(const StrandGroomDefinition& Definition);
		void ClearGroom();
		const StrandGroom* GetGroom() const;
		const GroomCurveState& GetGroomStrand(size_t Index) const;
		void RefreshGroom();
		void SetGroomRootPosition(size_t Strand, const Vector3& Position);
		void AttachGroomRoot(size_t Strand, RigidBody* Body, const Vector3& WorldPosition);
		void UnpinGroomRoot(size_t Strand);
		Vector3 GetGroomRootPosition(size_t Strand) const;
		void SetGroomLod(size_t Level);
		void UpdateGroomDistance(double Distance);
		size_t GetGroomLod() const;
		bool IsGuideSimulated(size_t Guide) const;
		size_t GetGuideCount() const;
		uint64_t GetInstanceId() const;
		size_t GetParticleCount(size_t Guide) const;
		size_t GetDirectorCount(size_t Guide) const;
		const StrandParticle& GetParticle(size_t Guide, size_t Particle) const;
		const StrandDirector& GetDirector(size_t Guide, size_t Director) const;
		StrandVector GetAngularVelocity(size_t Guide, size_t Director) const;
		const StrandStats& GetStats() const;
		const StrandSettings& GetSettings() const;
		void SetParticlePosition(size_t Guide, size_t Particle, const Vector3& Position, bool ResetVelocity = true);
		void SetParticleVelocity(size_t Guide, size_t Particle, const Vector3& Velocity);
		void PinParticle(size_t Guide, size_t Particle, const Vector3& Position);
		void UnpinParticle(size_t Guide, size_t Particle);
		void PinDirector(size_t Guide, size_t Director, const Quaternion& Orientation);
		void UnpinDirector(size_t Guide, size_t Director);
		void AddForce(size_t Guide, size_t Particle, const Vector3& Force);
		void AddTorque(size_t Guide, size_t Director, const Vector3& Torque);
		void SetRootPose(size_t Guide, const Transform& Pose);
		void AttachRoot(size_t Guide, RigidBody* Body, const Transform& WorldPose);
		void DetachBody(RigidBody* Body);
		Transform GetRootPose(size_t Guide) const;
		StrandVector GetRootForce(size_t Guide) const;
		StrandVector GetRootTorque(size_t Guide) const;
	private:
		friend class PhysicsWorld;
		void SetWorld(PhysicsWorld* World);
		void BeginWorldFrame(float Dt);
		void SimulateWorldStep(float Dt);
		void EndWorldFrame(bool Success);
		void OnBodyRemoved(RigidBody* Body);
		void Step(float Dt, bool ConsumeLoads);
		struct Storage;
		std::unique_ptr<Storage> m_Data;
	};
}

namespace Riemann
{
	struct GroomInfluence { size_t Guide=0; double Weight=1; };
	struct GroomStrand
	{
		std::vector<Vector3> Positions;
		std::vector<Quaternion> Directors;
		Vector3 ReferenceNormal=Vector3::UnitX();
		uint32_t Group=0;
		std::vector<GroomInfluence> Influences;
		float RadiusX=0.003f, RadiusY=0.003f;
	};
	struct GroomLod
	{
		std::vector<size_t> Guides;
		double TransitionTime=0.15, Distance=0;
	};
	struct GroomInterpolation
	{
		size_t GuidesPerStrand=3;
		double RootDistancePower=2, CoincidentRootTolerance=1e-6, LodHysteresis=0.5;
		bool IsValid() const;
	};
	struct StrandGroomDefinition
	{
		std::vector<GroomStrand> Strands;
		std::vector<uint32_t> GuideGroups;
		std::vector<GroomLod> Lods;
		GroomInterpolation Interpolation;
	};
	struct GroomCurveState
	{
		std::vector<Vector3> Positions;
		std::vector<Quaternion> Directors;
	};
	struct GroomSample
	{
		size_t Guide=0, Particle=0, FrameA=0, FrameB=0;
		double PositionBlend=0, FrameBlend=0, Weight=1;
		Vector3 RestPosition;
		Quaternion RestDirector;
	};
	class StrandGroom
	{
	public:
		StrandGroom(const std::vector<StrandGuide>& Guides, const StrandGroomDefinition& Definition, const StrandSettings& Settings);
		~StrandGroom();
		size_t GetStrandCount() const;
		size_t GetLodCount() const;
		const GroomStrand& GetStrand(size_t Index) const;
		double GetStrandParameter(size_t Strand, size_t Particle) const;
		const GroomLod& GetLod(size_t Index) const;
		const GroomInterpolation& GetInterpolation() const;
		bool IsSimulated(size_t Lod, size_t Guide) const;
		double GetSimulationWeight(size_t Lod, size_t Guide) const;
		const std::vector<GroomSample>& GetGuideSamples(size_t Lod, size_t Guide, size_t Index, bool Director) const;
		void EvaluateGuide(size_t Lod, size_t Guide, const StrandSystem& System, GroomCurveState& Output) const;
		void EvaluateStrand(size_t Strand, const StrandSystem& System, GroomCurveState& Output) const;
	private:
		struct Storage;
		std::shared_ptr<const Storage> m_Data;
	};
}
