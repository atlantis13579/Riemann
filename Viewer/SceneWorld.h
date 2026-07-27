#pragma once

#include <memory>
#include <string>
#include <vector>

#include "../Renderer/RiemannRenderer.h"
#include "../Src/Maths/Transform.h"
#include "../Src/RigidBodyDynamics/RigidBody.h"

namespace Riemann
{
	class DestructionSet;
	class Geometry;
	class PhysicsWorld;
	struct PhysicsWorldParam;
	class RopeSystem;
	class StaticMesh;
	class PositionConstraint;

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
		void Reset(const Vector3& gravity = Vector3(0.0f, -9.8f, 0.0f));
		void Step(float dt);

		Geometry* AddProjectileSphere(const std::string& id, const Vector3& position, float radius, const Vector3& velocity, const Vector4& color);
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

		const std::vector<SceneObjectInstance>& GetObjects() const { return m_Objects; }
		uint32_t GetRenderRevision() const { return m_RenderRevision; }
		const CameraDesc& GetCamera() const { return m_Camera; }
		const DirectionalLightDesc& GetLight() const { return m_Light; }
		const std::string& GetName() const { return m_Name; }
		const SceneFieldSimulationDesc& GetFieldSimulation() const { return m_FieldSimulation; }
		PhysicsWorld* GetSimulation() { return m_World.get(); }
		Geometry* AddTriangleMeshObject(const std::string& id, const StaticMesh& mesh, const Transform& transform, RigidType bodyType, const Vector4& color, bool renderBounds);

	private:
		bool LoadParsedScene(const class JsonValue& root, const std::string& sourceName, std::string* errorMessage);
		void ResetWorld(const PhysicsWorldParam& param);
		bool CreateObject(const SceneObjectDesc& desc, std::string* errorMessage);
		bool CreateDestructionBox(const SceneObjectDesc& desc, std::string* errorMessage);
		bool CreateDestructionGlass(const SceneObjectDesc& desc, std::string* errorMessage);
		Geometry* CreateGeometry(const SceneObjectDesc& desc, std::string* errorMessage) const;
		bool SyncDestructionObjects(bool force);
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
		};

	private:
		std::unique_ptr<PhysicsWorld> m_World;
		std::vector<std::unique_ptr<DestructionSet> > m_DestructionSets;
		std::vector<DestructionRenderState> m_DestructionRenderStates;
		std::vector<SceneObjectInstance> m_Objects;
		CameraDesc m_Camera;
		DirectionalLightDesc m_Light;
		SceneFieldSimulationDesc m_FieldSimulation;
		std::string m_BaseDirectory;
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
	};
}
