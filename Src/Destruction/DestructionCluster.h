#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

#include "../Maths/Box3.h"
#include "../Maths/Transform.h"

namespace Riemann
{
	class DestructionSet;
	class Geometry;
	class RigidBodyDynamic;
	class PhysicsWorld;
	struct RigidBodyParam;

	class DestructionCluster
	{
	public:
		DestructionCluster() = default;
		DestructionCluster(uint32_t InID, int InLevel, const std::vector<int>& InSourceIndices);

		static DestructionCluster Create(DestructionSet& DestructSet, int Level, const std::vector<int>& Indices);

		uint32_t GetID() const { return mID; }
		int GetLevel() const { return mLevel; }
		bool IsStatic() const { return bIsStatic; }
		size_t Size() const { return mSourceIndices.size(); }
		float Volume() const { return std::max(mLocalBounds.GetVolume(), 0.0f); }

		const std::vector<int>& GetSourceIndices() const { return mSourceIndices; }
		int GetSourceIndexMin() const { return mSourceIndexMin; }
		const Box3& GetLocalBounds() const { return mLocalBounds; }
		const Vector3& GetCenterOfMass() const { return mCenterOfMass; }
		RigidBodyDynamic* GetRigidBody() const { return mRigidBody; }
		const std::vector<Geometry*>& GetGeometries() const { return mGeometries; }

		bool Contains(int ChunkIndex) const;
		bool GetLocalTransform(int ChunkIndex, Maths::Transform& OutTransform) const;
		bool GetWorldTransform(int ChunkIndex, Maths::Transform& OutTransform) const;
		bool GetWorldPosition(int ChunkIndex, Vector3& Position) const;
		Geometry* GetGeometryByChunkIndex(int ChunkIndex) const;
		int GetChunkIndexByGeometry(const Geometry* Geom) const;
		Maths::Transform GetClusterWorldTransform() const;

		RigidBodyDynamic* BuildRigidBody(PhysicsWorld& Simulation, const RigidBodyParam& Param, const std::vector<Geometry*>& Geometries);
		RigidBodyDynamic* BuildRigidBodyFromChunkBounds(PhysicsWorld& Simulation, const DestructionSet& DestructSet, const RigidBodyParam& Param);
		void ReleaseRigidBodyBinding();

		void SetID(uint32_t ID) { mID = ID; }
		void SetLevel(int Level) { mLevel = Level; }
		void SetStatic(bool bStatic) { bIsStatic = bStatic; }
		void SetBounds(const Box3& Bounds) { mLocalBounds = Bounds; }
		void SetCenterOfMass(const Vector3& CenterOfMass) { mCenterOfMass = CenterOfMass; }
		void SetSourceIndices(const std::vector<int>& Indices);
		void SetLocalTransforms(const std::vector<Maths::Transform>& Transforms) { mLocalTransforms = Transforms; }

	private:
		uint32_t mID = 0;
		int mLevel = 0;
		bool bIsStatic = false;
		Box3 mLocalBounds = Box3::Empty();
		Vector3 mCenterOfMass = Vector3::Zero();
		std::vector<int> mSourceIndices;
		int mSourceIndexMin = -1;
		std::vector<Maths::Transform> mLocalTransforms;
		RigidBodyDynamic* mRigidBody = nullptr;
		std::vector<Geometry*> mGeometries;
	};

}	// namespace Riemann
