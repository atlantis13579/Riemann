#pragma once

#include <memory>
#include <string>
#include <vector>

#include "../Renderer/RiemannRenderer.h"
#include "../Src/Maths/Transform.h"
#include "../Src/RigidBodyDynamics/RigidBody.h"

namespace Riemann
{
	class Geometry;
	class PhysicsWorld;
	class StaticMesh;

	struct SceneObjectInstance
	{
		std::string Id;
		Geometry* GeometryPtr = nullptr;
		Vector4 Color = Vector4(0.72f, 0.76f, 0.82f, 1.0f);
		bool RenderBounds = false;
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

		Geometry* AddProjectileSphere(const std::string& id, const Vector3& position, float radius, const Vector3& acceleration, const Vector4& color);

		const std::vector<SceneObjectInstance>& GetObjects() const { return m_Objects; }
		const CameraDesc& GetCamera() const { return m_Camera; }
		const DirectionalLightDesc& GetLight() const { return m_Light; }
		PhysicsWorld* GetSimulation() { return m_World.get(); }
		Geometry* AddTriangleMeshObject(const std::string& id, const StaticMesh& mesh, const Transform& transform, RigidType bodyType, const Vector4& color, bool renderBounds);

	private:
		bool LoadParsedScene(const class JsonValue& root, const std::string& sourceName, std::string* errorMessage);
		bool CreateObject(const SceneObjectDesc& desc, std::string* errorMessage);
		Geometry* CreateGeometry(const SceneObjectDesc& desc, std::string* errorMessage) const;

	private:
		std::unique_ptr<PhysicsWorld> m_World;
		std::vector<SceneObjectInstance> m_Objects;
		CameraDesc m_Camera;
		DirectionalLightDesc m_Light;
		std::string m_BaseDirectory;
	};
}
