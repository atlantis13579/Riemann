#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "../Renderer/RiemannRenderer.h"
#include "../Src/Maths/Transform.h"
#include "../Src/Modules/Robotics/ArticulatedRobotController.h"
#include "../Src/Modules/Robotics/SpiderTraining.h"
#include "../Src/RigidBodyDynamics/RigidBody.h"
#include "../Src/Fluid/AerodynamicsSimulation.h"
#include "../Src/Destruction/DestructionSet.h"
#include "../Src/RigidBodyDynamics/PhysicsField.h"

namespace Riemann
{
	class DestructionSet;
	class Geometry;
	class PhysicsWorld;
	class ForceField;
	struct PhysicsWorldParam;
	class RopeSystem;
	class StaticMesh;
	class PositionConstraint;
	class VehicleDeformation;
	class DeformableBody;
	class VehicleDynamics;
	class SweepRobotController;
	class Ragdoll;
	enum class RagdollAlgorithm;
	class PhysicalAnimation;
	class ClothSystem;
	enum class RobotWalkMode { Policy, Sweep };
	enum class ClothModel;
	class StrandSystem;
	struct ClothSettings;
	struct ClothMesh;

	enum class ConstraintControlType
	{
		Fixed,
		Point,
		Distance,
		Hinge,
		Slider,
		Cone,
		SixDOF,
	};

	struct ConstraintControlState
	{
		size_t Index = 0;
		ConstraintControlType Type = ConstraintControlType::Fixed;
		std::string Name;
		bool Enabled = true;
		bool Broken = false;
		bool RepairRequested = false;
		float ReactionForce = 0.0f, ReactionTorque = 0.0f;
		float CurrentValue = 0.0f;
		float MinValue = 0.0f;
		float MaxValue = 0.0f;
		Vector3 LinearMin = Vector3::Zero();
		Vector3 LinearMax = Vector3::Zero();
		Vector3 AngularMinDegrees = Vector3::Zero();
		Vector3 AngularMaxDegrees = Vector3::Zero();
	};

	struct SceneObjectInstance
	{
		std::string Id;
		Geometry* GeometryPtr = nullptr;
		DestructionSet* DestructSet = nullptr;
		VehicleDeformation* Vehicle = nullptr;
		Vector4 Color = Vector4(0.72f, 0.76f, 0.82f, 1.0f);
		bool RenderBounds = false;
		bool RenderEnabled = true;
	};

	struct SceneFieldRegion
	{
		Vector3 Min = Vector3::Zero();
		Vector3 Max = Vector3::Zero();
		Vector3 Velocity = Vector3::Zero();
		float Density = 0.0f;
		bool SetDensity = false;
		bool SetVelocity = false;
	};

	struct SceneFieldSimulationDesc
	{
		std::string Type;
		std::string Method = "eulerian";
		std::string Backend = "cpu";
		bool AllowBackendFallback = true;
		bool RigidCoupling = false;
		Vector3 BoundsMin = Vector3(-5.0f, 0.0f, -3.0f);
		Vector3 BoundsMax = Vector3(5.0f, 6.0f, 3.0f);
		Vector3i Resolution = Vector3i(32, 24, 20);
		Vector3 Gravity = Vector3(0.0f, -9.81f, 0.0f);
		float RestDensity = 1000.0f;
		float AmbientDensity = 1.225f;
		float AmbientPressure = 101325.0f;
		float SpeedOfSound = 8.0f;
		float AdiabaticExponent = 1.4f;
		float DynamicViscosity = 0.02f;
		float ArtificialViscosity = 0.05f;
		float PressureStiffness = 2000.0f;
		float ParticleSpacing = 0.25f;
		float SmoothingRadius = 0.55f;
		float BoundaryRestitution = 0.1f;
		float BoundaryFriction = 0.05f;
		bool OpenTopBoundary = false;
		float MaxTimeStep = 1.0f / 120.0f;
		float AcousticCFL = 0.35f;
		int MaxSubsteps = 32;
		int PressureIterations = 48;
		int SurfaceSamplesPerAxis = 4;
		float DragCoefficient = 1.0f;
		float MaxLinearAcceleration = 20.0f;
		float MaxAngularAcceleration = 30.0f;
		float VisualizationThreshold = 0.08f;
		float InteractionRadius = 0.65f;
		float InteractionStrength = 18.0f;
		std::vector<SceneFieldRegion> Regions;
		AerodynamicsSimulationSettings Aerodynamics;
		float TracerExtinction = 5.5f;
		Vector3 TracerColor = Vector3(0.85f, 0.89f, 0.96f);

		bool IsConfigured() const { return !Type.empty(); }
	};

	class SceneWorld
	{
	public:
		struct SceneObjectDesc;

		SceneWorld();
		~SceneWorld();

		bool LoadFromFile(const std::string& fileName, std::string* errorMessage);
		bool LoadFromText(const std::string& text, const std::string& sourceName, std::string* errorMessage);
		void SetRagdollAlgorithmOverride(RagdollAlgorithm Algorithm) { m_RagdollAlgorithmOverride = Algorithm; m_HasRagdollAlgorithmOverride = true; }
		void ClearRagdollAlgorithmOverride() { m_HasRagdollAlgorithmOverride = false; }
		void Reset(const Vector3& gravity = Vector3(0.0f, -9.8f, 0.0f));
		void Step(float dt);
		void SetDistanceFieldDemo(bool Enabled) { m_DistanceFieldDemo = Enabled; }
		bool GetDistanceFieldDemo() const { return m_DistanceFieldDemo; }
		void SetAuthoredDestructionDemo(bool Enabled) { m_AuthoredDestructionDemo = Enabled; }
		bool GetAuthoredDestructionDemo() const { return m_AuthoredDestructionDemo; }
		void SetDestructionEventsDemo(bool Enabled) { m_DestructionEventsDemo = Enabled; }
		bool GetDestructionEventsDemo() const { return m_DestructionEventsDemo; }
		void SetDestructionCacheDemo(bool Enabled) { m_DestructionCacheDemo = Enabled; }
		bool GetDestructionCacheDemo() const { return m_DestructionCacheDemo; }
		void SetStrandDemo(bool Enabled) { m_StrandDemo=Enabled; }
		bool GetStrandDemo() const { return m_StrandDemo; }
		StrandSystem* GetDemoStrands() const;
		bool ApplyStrandDemo(float RootTwist, bool Coupled = true, size_t Lod = 0);
		bool ApplyStrandCollisionDemo(bool Enabled, bool Self, bool Continuous, bool Friction);
		size_t GetStrandDemoLod() const;
		float GetStrandDemoTwist() const { return m_StrandDemoTwist; }
		bool ApplyPhysicsFieldsDemo(bool Enabled, bool Frozen = false, bool StrainPulse = false, bool VelocityPulse = false);
		bool GetPhysicsFieldsDemo() const { return m_PhysicsFieldsDemo; }
		bool GetPhysicsFieldFreeze() const { return m_PhysicsFieldFreeze; }
		bool ApplyDestructionCacheDemo(DestructionPlaybackCommand Command, float SeekFraction = 0);
		bool IsDestructionCacheRecording() const { return !m_DestructionRecorders.empty(); }
		bool IsDestructionCachePlaying() const;
		double GetDestructionCacheTime() const;
		double GetDestructionCacheDuration() const;
		DestructionPlayback* GetDestructionPlayback(size_t Index) const;
		const std::vector<DestructionPlaybackEvent>& GetLastDestructionPlaybackEvents() const { return m_DestructionPlaybackEvents; }
		const std::vector<DestructionEvent>& GetLastDestructionEvents() const { return m_DestructionEvents; }
		uint64_t GetDestructionEventCount() const { return m_DestructionEventCount; }
		uint64_t GetDestructionDroppedEventCount() const { return m_DestructionDroppedEvents; }
		size_t GetDestructionSetCount() const { return m_DestructionSets.size(); }
		DestructionSet* GetDestructionSet(size_t Index) const { return Index < m_DestructionSets.size() ? m_DestructionSets[Index].get() : nullptr; }
		void SetCharacterDemo(bool Enabled, bool BoxShape) { m_CharacterDemo = Enabled; m_CharacterBox = BoxShape; }
		bool GetCharacterDemo() const { return m_CharacterDemo; }
		bool GetCharacterBox() const { return m_CharacterBox; }
		ArticulatedRobotController* GetWalkingRobot() const;
		RobotController* GetRobotDemoController() const;
		bool IsSpiderWalkingDemo() const { return m_SpiderWalkingDemo; }
		void SetRobotWalkMode(RobotWalkMode Mode) { m_RobotWalkMode = Mode; }
		RobotWalkMode GetRobotWalkMode() const { return m_RobotWalkMode; }
		void SetRobotDriveInput(float Forward, float Turn, float Strafe);
		void SetRobotAutoWalk(bool AutoWalk) { m_RobotAutoWalk = AutoWalk; }
		void BeginSpiderTrainingDisplay(size_t EnvironmentCount);
		void ShowSpiderTrainingSnapshot(const SpiderTrainingSnapshot& Snapshot, bool ShowAll, size_t Selected);
		Vector3 GetRobotDriveCommand() const { return m_RobotDriveCommand; }
		float GetRobotDriveHeading() const { return m_RobotDriveHeading; }
		// The next scene load starts a fresh policy and simulation with this selection.
		void SetWalkingRobotBackend(RobotDynamicsBackend Backend) { m_RobotBackend = Backend; }
		RobotDynamicsBackend GetWalkingRobotBackend() const { return m_RobotBackend; }
		SweepRobotController* GetDemoCharacter() const { return m_Character; }
		void SetVehicleDynamicsDemo(bool Enabled) { m_VehicleDynamicsDemo = Enabled; }
		bool GetVehicleDynamicsDemo() const { return m_VehicleDynamicsDemo; }
		void SetReducedRagdollDemo(bool Enabled) { m_ReducedRagdollDemo = Enabled; }
		bool GetReducedRagdollDemo() const { return m_ReducedRagdollDemo; }
		void SetPhysicalAnimationDemo(bool Enabled) { m_PhysicalAnimationDemo = Enabled; }
		bool GetPhysicalAnimationDemo() const { return m_PhysicalAnimationDemo; }
		PhysicalAnimation* GetDemoPhysicalAnimation() const;
		bool ApplyPhysicalAnimationDemo(bool Passive, float LeftArmStrength, float PhysicsBlend);
		void SetClothModelDemo(bool Enabled) { m_ClothModelDemo = Enabled; }
		bool GetClothModelDemo() const { return m_ClothModelDemo; }
		ClothSystem* GetDemoCloth() const;
		bool SetClothRigidInteraction(bool Enabled);
		bool SetClothSelfCollision(bool Enabled);
		bool ApplyClothParameters(ClothModel Model, int Iterations, int Substeps, float Damping,
			float StretchUCompliance, float StretchVCompliance, float ShearCompliance, float BendingCompliance);
		bool ApplyClothDemo(bool Wind, bool Animation, bool Tethers, bool SoftMaterial, bool SurfaceContacts = false, bool Friction = true,
			bool SelfCollision = false, bool InterCollision = false, bool Continuous = false,
			bool SoftBodyAttachments = false, bool CompliantAttachments = false, bool RigidAttachments = false, bool SoftBodyContacts = false, bool SoftRigidContacts = false,
			bool CalibratedSoftMaterial = false, bool StVKSoftMaterial = false, bool PreparedSoftAsset = false, bool EmbeddedSoftRender = true,
			bool CorotatedSoftMaterial = false, bool ProjectiveSoftMaterial = false);
		bool ApplyDeformableAttachmentDemo(bool Enabled, bool Compliant, bool Rigid = false);
		bool ApplyDeformableContactDemo(bool Enabled, bool Rigid = false, bool Prepared = false, bool Embedded = true);
		bool GetPreparedSoftAssetDemo() const;
		bool GetEmbeddedSoftRenderDemo() const;
		bool ApplyDeformableMaterialDemo(bool Enabled, bool StVK = false, bool Corotated = false, bool Projective = false);
		bool GetCorotatedSoftMaterialDemo() const;
		bool GetProjectiveSoftMaterialDemo() const;
		bool GetCalibratedSoftMaterialDemo() const;
		bool GetStVKSoftMaterialDemo() const;
		DeformableBody* GetDemoContactBody(size_t Index) const;
		RigidBodyDynamic* GetDemoContactRigid() const;
		bool GetSoftRigidContactDemo() const { return m_SoftRigidContactDemo; }
		RigidBodyDynamic* GetDemoAttachmentRigid() const;
		bool GetRigidAttachmentDemo() const { return m_RigidAttachmentDemo; }
		DeformableBody* GetDemoDeformableBody(size_t Index) const;
		bool GetCompliantAttachmentDemo() const { return m_CompliantAttachmentDemo; }
		ClothSystem* GetDemoClothPartner() const;
		bool GetClothDemoSoftMaterial() const { return m_ClothSoftMaterial; }
		VehicleDynamics* GetVehicleDynamics(int Index) const;

		Geometry* AddProjectileSphere(const std::string& id, const Vector3& position, float radius, const Vector3& velocity, const Vector4& color);
		int GetAerodynamicsRigidBodyCount() const;
		bool IsMouseControlGeometry(const Geometry* geometry) const;
		bool RayCastMouseControlGeometry(const Vector3& origin, const Vector3& direction, float maxDistance,
			Geometry** hitGeometry, Vector3* hitPoint) const;
		bool SelectMouseControlGeometry(Geometry* geometry);
		Vector3 GetMouseControlPosition() const { return m_MouseControlBody ? m_MouseControlBody->X : Vector3::Zero(); }
		bool SetMouseControlPosition(const Vector3& worldPosition, float deltaTime);
		bool SetMouseControlPosition(const Vector3& worldPosition, float deltaTime,
			const Vector3& boundsMin, const Vector3& boundsMax);
		bool SetMouseControlPosition(const Vector3& worldPosition, float deltaTime,
			const Vector3& boundsMin, const Vector3& boundsMax, const Transform& boundsTransform);
		void ReleaseMouseControl();
		bool RotateTank(const Vector3& center, const Quaternion& deltaRotation);
		std::vector<ConstraintControlState> GetConstraintControls() const;
		bool ApplyConstraintControl(const ConstraintControlState& state);
		std::vector<std::string> GetTerrainObjectIds() const;
		bool RayCastTerrain(const std::string& id, const Vector3& origin, const Vector3& direction,
			float maxDistance, Vector3* hitPoint, float* localHeight = nullptr) const;
		int EditTerrain(const std::string& id, const Vector3& worldCenter, float radius, float value, int mode, float strength = 1.0f);
		size_t GetVehicleCount() const { return m_Vehicles.size(); }
		VehicleDeformation* GetVehicle(int index);
		const VehicleDeformation* GetVehicle(int index) const;

		const std::vector<SceneObjectInstance>& GetObjects() const { return m_Objects; }
		uint32_t GetRenderRevision() const { return m_RenderRevision; }
		const CameraDesc& GetCamera() const { return m_Camera; }
		const DirectionalLightDesc& GetLight() const { return m_Light; }
		const std::string& GetName() const { return m_Name; }
		const SceneFieldSimulationDesc& GetFieldSimulation() const { return m_FieldSimulation; }
		PhysicsWorld* GetSimulation() { return m_World.get(); }
		const PhysicsWorld* GetSimulation() const { return m_World.get(); }
		Geometry* AddTriangleMeshObject(const std::string& id, const StaticMesh& mesh, const Transform& transform, RigidType bodyType, const Vector4& color, bool renderBounds);

	private:
		bool LoadParsedScene(const class JsonValue& root, const std::string& sourceName, std::string* errorMessage);
		void ResetWorld(const PhysicsWorldParam& param);
		bool CreateObject(const SceneObjectDesc& desc, std::string* errorMessage);
		void LoadRobotCourse(const std::string& Path);
		void CreateRobotLandmarks(bool Course);
		bool CreateWalkingRobot(bool Course, const std::string& Model, bool Landmarks, std::string* Error);
		void AddRobotPrimitive(size_t Link, std::unique_ptr<Geometry> Shape, const Vector4& Color);
		void CreateRobotPrimitives(const ArticulatedRobotSettings& Settings);
		void UpdateRobotDrive(float Dt);
		void SyncWalkingRobot(float Dt = 0);
		bool CreateCharacterDemo(const class JsonValue& Root, std::string* Error);
		bool CreateReducedRagdollDemo(const class JsonValue& Root, std::string* Error);
		bool CreatePhysicalAnimationDemo(const class JsonValue& Root, std::string* Error);
		void SamplePhysicalAnimationDemo(float Dt);
		void SyncPhysicalAnimationDemo();
		ClothSystem* CreateClothDemo(ClothSettings Settings, const Vector3& Center, const Vector3& Horizontal, const Vector3& Vertical, bool PinTop);
		void SampleClothDemo(float Dt);
		void PruneClothDemo();
		bool CreateVehicles(const class JsonValue& root, const std::string& sourceName, std::string* errorMessage);
		bool CreateDestructionBox(const SceneObjectDesc& desc, std::string* errorMessage);
		bool CreateDestructionGlass(const SceneObjectDesc& desc, std::string* errorMessage);
		Geometry* CreateGeometry(const SceneObjectDesc& desc, std::string* errorMessage) const;
		bool SyncDestructionObjects(bool force);
		bool SyncTerrainRevisions();
		void RegisterMouseControlGeometry(Geometry* geometry);
		void TouchRenderRevision() { ++m_RenderRevision; }

	private:
		struct DestructionRenderState
		{
			DestructionSet* Set = nullptr;
			std::string Id;
			Vector4 Color = Vector4(0.72f, 0.76f, 0.82f, 1.0f);
			bool RenderBounds = false;
			std::unique_ptr<Geometry> IntactGeometry;
			Transform IntactLocalTransform = Transform::Identity();
			bool RenderIntactUntilFractured = false;
			bool LastRenderedIntact = false;
			uint32_t LastRevision = 0xffffffffu;
			uint64_t LastPlaybackRevision = 0;
		};

	private:
		std::unique_ptr<PhysicsWorld> m_World;
		std::vector<std::unique_ptr<DestructionSet> > m_DestructionSets;
		std::vector<DestructionRenderState> m_DestructionRenderStates;
		std::vector<SceneObjectInstance> m_Objects;
		std::vector<std::pair<Geometry*, uint64_t>> m_TerrainRevisions;
		struct RopeSurfaceState
		{
			RopeSystem* Rope = nullptr;
			StrandSystem* Strands = nullptr;
			uint64_t StrandInstanceId = 0;
			bool GroomRender = false;
			size_t GroomIndex = 0;
			size_t GuideIndex = 0;
			Ragdoll* TendonOwner = nullptr;
			size_t TendonIndex = 0;
			std::unique_ptr<Geometry> Surface;
			std::vector<Vector3> LastPositions;
			std::vector<Quaternion> LastDirectors;
			Vector3 FirstNormal = Vector3::UnitX();
			float Radius = 0.05f;
			float RadiusY = 0;
			int Sides = 8;
		};
		void SyncRopeSurfaces();
		std::vector<RopeSurfaceState> m_RopeSurfaces;
		struct VehicleRenderState
		{
			struct Wheel
			{
				// Ownership passes to the physics world when the mount breaks.
				std::unique_ptr<Geometry> AttachedShape;
				Transform LocalPose;
				int MountNode = -1;
				std::vector<int> MountEdges;
			};
			std::unique_ptr<VehicleDeformation> Vehicle;
			VehicleDynamics* Dynamics = nullptr;
			std::vector<Wheel> Wheels;
			float WheelBreakStrain = 0;
			float WheelMass = 0;
			Vector3 PreviousPosition = Vector3::Zero();
			float WheelRadius = 0.4f;
			float WheelAngle = 0.0f;
			uint64_t LastRevision = ~uint64_t(0);
		};
		void SyncVehicles(float dt);
		std::vector<VehicleRenderState> m_Vehicles;
		CameraDesc m_Camera;
		DirectionalLightDesc m_Light;
		SceneFieldSimulationDesc m_FieldSimulation;
		std::string m_BaseDirectory;
		RagdollAlgorithm m_RagdollAlgorithmOverride;
		bool m_HasRagdollAlgorithmOverride = false;
		std::string m_Name;
		struct ConstraintInstance
		{
			std::string Name;
			PositionConstraint* Constraint = nullptr;
		};
		std::vector<ConstraintInstance> m_ConstraintInstances;
		std::vector<Geometry*> m_MouseControlGeometries;
		Geometry* m_MouseControlGeometry = nullptr;
		RigidBody* m_MouseControlBody = nullptr;
		RopeSystem* m_MouseControlRope = nullptr;
		int m_MouseControlRopeParticle = -1;
		uint32_t m_RenderRevision = 0;
		bool m_DistanceFieldDemo = false;
		bool m_AuthoredDestructionDemo = false;
		bool m_DestructionEventsDemo = false;
		bool m_DestructionCacheDemo = false;
		bool m_StrandDemo = false;
		StrandSystem* m_DemoStrands = nullptr;
		uint64_t m_DemoStrandInstanceId = 0;
		float m_StrandDemoTwist = 0;
		std::vector<Transform> m_StrandRootLocal;
		bool CreateStrandDemo();
		bool m_PhysicsFieldsDemo = false, m_PhysicsFieldFreeze = false;
		std::vector<uint64_t> m_DemoFieldIds;
		uint64_t m_DemoFreezeField = 0;
		ScalarField m_DemoFieldMask;
		Vector3 m_DemoFieldCenter = Vector3::Zero();
		double m_DestructionRecordingTime = 0;
		std::vector<std::unique_ptr<DestructionCacheRecorder>> m_DestructionRecorders;
		std::vector<DestructionCache> m_DestructionCaches;
		std::vector<std::unique_ptr<DestructionPlayback>> m_DestructionPlaybacks;
		std::vector<DestructionPlaybackEvent> m_DestructionPlaybackEvents;
		std::vector<DestructionEvent> m_DestructionEvents;
		uint64_t m_DestructionEventCount = 0, m_DestructionDroppedEvents = 0;
		bool m_CharacterDemo = false, m_CharacterBox = false;
		bool m_VehicleDynamicsDemo = false;
		bool m_ReducedRagdollDemo = false;
		bool m_PhysicalAnimationDemo = false;
		PhysicalAnimation* m_DemoAnimation = nullptr;
		std::vector<Transform> m_AnimationRestLocal;
		std::vector<Vector3> m_AnimationAnchors, m_AnimationAxes;
		struct PhysicalAnimationRenderState
		{
			size_t Bone = 0;
			Transform LocalTransform;
			std::unique_ptr<Geometry> Surface;
		};
		std::vector<PhysicalAnimationRenderState> m_AnimationSurfaces;
		double m_PhysicalAnimationTime = 0;
		bool m_ClothModelDemo = false;
		bool m_ClothSoftMaterial = false;
		bool m_CompliantAttachmentDemo = false;
		bool m_RigidAttachmentDemo = false;
		GeometryHandle m_DemoAttachmentHandle;
		Geometry* m_DemoAttachmentGeometry = nullptr;
		ForceField* m_DemoAttachmentGravity = nullptr;
		std::vector<std::pair<DeformableBody*, Geometry*>> m_DemoSoftBodies;
		std::vector<std::pair<DeformableBody*, Geometry*>> m_DemoContactBodies;
		bool m_SoftRigidContactDemo = false;
		GeometryHandle m_DemoContactHandle;
		Geometry* m_DemoContactGeometry = nullptr;
		ForceField* m_DemoContactGravity = nullptr;
		ClothSystem* m_DemoCloth = nullptr;
		Geometry* m_DemoClothGeometry = nullptr;
		std::vector<Vector3> m_ClothRestPose;
		ClothSystem* m_DemoClothPartner = nullptr;
		Geometry* m_DemoClothPartnerGeometry = nullptr;
		std::vector<Vector3> m_ClothPartnerRestPose;
		std::unique_ptr<ClothMesh> m_ClothDemoMesh;
		double m_ClothAnimationTime = 0;
		std::vector<Ragdoll*> m_MechanismDemoDrivers;
		double m_MechanismDemoTime = 0;
		ArticulatedRobotController* m_WalkingRobot = nullptr;
		SweepRobotController* m_SweepWalkingRobot = nullptr;
		std::unique_ptr<FeatherstoneSolver> m_SweepRobotPose;
		RobotWalkMode m_RobotWalkMode = RobotWalkMode::Policy;
		bool m_SpiderWalkingDemo = false, m_RobotAutoWalk = false;
		Vector3 m_RobotDriveInput = Vector3::Zero(), m_RobotDriveCommand = Vector3::Zero();
		Vector3 m_SweepRobotPrevious = Vector3::Zero();
		float m_RobotDriveHeading = 0, m_SweepRobotPhase = 0;
		struct RobotSurface
		{
			size_t Link;
			Transform Local;
			std::unique_ptr<Geometry> Shape;
			size_t Environment = 0;
			size_t ObjectIndex = 0;
		};
		size_t m_TrainingBuildIndex = 0;
		bool m_TrainingDisplay = false;
		RobotDynamicsBackend m_RobotBackend = RobotDynamicsBackend::SparseJointSpaceDynamics;
		std::vector<RobotSurface> m_RobotSurfaces;
		SweepRobotController* m_Character = nullptr;
		RigidBody* m_CharacterPlatform = nullptr;
		Vector3 m_CharacterPlatformOrigin = Vector3::Zero();
		float m_CharacterDemoTime = 0.0f;
	};
}
