
#include "DestructionCluster.h"

#include <algorithm>
#include <cmath>

#include "DestructionSet.h"
#include "../Collision/GeometryObject.h"
#include "../RigidBodyDynamics/RigidBody.h"
#include "../RigidBodyDynamics/PhysicsWorld.h"

namespace Riemann
{
	namespace
	{
		int GetClusterLevelByVolume(const DestructionSet& DestructSet, const DestructionCluster& Cluster)
		{
			if (Cluster.Size() <= 1)
			{
				return 7;
			}

			const float ClusterVolume = std::max(Cluster.Volume(), 1.0f);
			const float TotalVolume = std::max(DestructSet.GetBounds().GetVolume(), 1.0f);
			const int Level = (int)(std::log(TotalVolume / ClusterVolume) / std::log(2.0f));
			return std::max(0, std::min(Level, 7));
		}

		Maths::Transform CalculateGeometryClusterWorldTransform(const std::vector<Geometry*>& Geometries, const Vector3& FallbackCenter)
		{
			Vector3 Center = Vector3::Zero();
			float TotalMass = 0.0f;
			Geometry* FirstGeometry = nullptr;

			for (Geometry* Geom : Geometries)
			{
				if (!Geom)
				{
					continue;
				}

				if (!FirstGeometry)
				{
					FirstGeometry = Geom;
				}

				const MassParameters* Mass = Geom->GetMassParameters();
				const float ShapeMass = Mass ? std::max(Mass->Mass, 1e-3f) : 1.0f;
				const Vector3 ShapeCenter = Geom->GetWorldTransform()->LocalToWorld(Mass ? Mass->CenterOfMass : Vector3::Zero());
				Center += ShapeCenter * ShapeMass;
				TotalMass += ShapeMass;
			}

			Maths::Transform WorldTransform;
			WorldTransform.pos = TotalMass > 1e-6f ? Center / TotalMass : FallbackCenter;
			WorldTransform.quat = Geometries.size() == 1 && FirstGeometry ? FirstGeometry->GetWorldRotation() : Quaternion::One();
			return WorldTransform;
		}

		void OverrideGeometryMass(Geometry* Geom, const DestructionChunk& Chunk)
		{
			if (!Geom)
			{
				return;
			}

			MassParameters* Mass = Geom->GetMassParameters();
			if (!Mass)
			{
				return;
			}

			const float OldMass = std::max(Mass->Mass, 1e-3f);
			const float NewMass = std::max(Chunk.Mass, 1e-3f);
			Mass->InertiaMat *= NewMass / OldMass;
			Mass->Mass = NewMass;
			Mass->Volume = std::max(Chunk.Volume, 1e-3f);
		}
	}

	DestructionCluster::DestructionCluster(uint32_t InID, int InLevel, const std::vector<int>& InSourceIndices)
		: mID(InID)
		, mLevel(InLevel)
	{
		SetSourceIndices(InSourceIndices);
	}

	DestructionCluster DestructionCluster::Create(DestructionSet& DestructSet, int Level, const std::vector<int>& Indices)
	{
		DestructionCluster Cluster(DestructSet.AllocClusterID(), Level, Indices);
		if (Indices.empty())
		{
			return Cluster;
		}

		const std::vector<DestructionChunk>& Chunks = DestructSet.GetChunks();
		Box3 Bounds = Box3::Empty();
		float TotalMass = 0.0f;
		Vector3 Center = Vector3::Zero();
		bool bStatic = false;
		for (int Index : Indices)
		{
			if (Index < 0 || (size_t)Index >= Chunks.size())
			{
				continue;
			}

			const DestructionChunk& Chunk = Chunks[(size_t)Index];
			Bounds.Encapsulate(Chunk.Bounds);
			Center += Chunk.Centroid * Chunk.Mass;
			TotalMass += Chunk.Mass;
			bStatic = bStatic || Chunk.IsStatic;
		}

		TotalMass = std::max(TotalMass, 1e-3f);
		Center /= TotalMass;
		Cluster.SetBounds(Bounds);
		Cluster.SetCenterOfMass(Center);
		Cluster.SetStatic(bStatic);

		std::vector<Maths::Transform> LocalTransforms;
		LocalTransforms.reserve(Indices.size());
		for (int Index : Indices)
		{
			Maths::Transform Transform;
			if (Index >= 0 && (size_t)Index < Chunks.size())
			{
				Transform.pos = Chunks[(size_t)Index].Centroid - Center;
			}
			LocalTransforms.push_back(Transform);
		}
		Cluster.SetLocalTransforms(LocalTransforms);
		if (Level < 0)
		{
			Cluster.SetLevel(GetClusterLevelByVolume(DestructSet, Cluster));
		}
		return Cluster;
	}

	bool DestructionCluster::Contains(int ChunkIndex) const
	{
		return std::find(mSourceIndices.begin(), mSourceIndices.end(), ChunkIndex) != mSourceIndices.end();
	}

	bool DestructionCluster::GetLocalTransform(int ChunkIndex, Maths::Transform& OutTransform) const
	{
		for (size_t Index = 0; Index < mSourceIndices.size(); ++Index)
		{
			if (mSourceIndices[Index] == ChunkIndex)
			{
				if (Index < mLocalTransforms.size())
				{
					OutTransform = mLocalTransforms[Index];
				}
				else
				{
					OutTransform = Maths::Transform::Identity();
				}
				return true;
			}
		}
		return false;
	}

	bool DestructionCluster::GetWorldTransform(int ChunkIndex, Maths::Transform& OutTransform) const
	{
		Maths::Transform LocalTransform;
		if (!GetLocalTransform(ChunkIndex, LocalTransform))
		{
			return false;
		}
		const Maths::Transform ClusterWorldTransform = GetClusterWorldTransform();
		OutTransform = ClusterWorldTransform * LocalTransform;
		return true;
	}

	bool DestructionCluster::GetWorldPosition(int ChunkIndex, Vector3& Position) const
	{
		Maths::Transform WorldTransform;
		if (!GetWorldTransform(ChunkIndex, WorldTransform))
		{
			return false;
		}
		Position = WorldTransform.pos;
		return true;
	}

	Geometry* DestructionCluster::GetGeometryByChunkIndex(int ChunkIndex) const
	{
		for (size_t Index = 0; Index < mSourceIndices.size(); ++Index)
		{
			if (mSourceIndices[Index] == ChunkIndex)
			{
				return Index < mGeometries.size() ? mGeometries[Index] : nullptr;
			}
		}
		return nullptr;
	}

	int DestructionCluster::GetChunkIndexByGeometry(const Geometry* Geom) const
	{
		if (!Geom)
		{
			return -1;
		}

		for (size_t Index = 0; Index < mGeometries.size() && Index < mSourceIndices.size(); ++Index)
		{
			if (mGeometries[Index] == Geom)
			{
				return mSourceIndices[Index];
			}
		}
		return -1;
	}

	Maths::Transform DestructionCluster::GetClusterWorldTransform() const
	{
		if (mRigidBody)
		{
			return Maths::Transform(mRigidBody->X, mRigidBody->Q);
		}
		return Maths::Transform(mCenterOfMass);
	}

	RigidBodyDynamic* DestructionCluster::BuildRigidBody(PhysicsWorld& Simulation, const RigidBodyParam& Param, const std::vector<Geometry*>& Geometries)
	{
		if (mSourceIndices.empty() || Geometries.size() != mSourceIndices.size())
		{
			return nullptr;
		}

		for (Geometry* Geom : Geometries)
		{
			if (!Geom)
			{
				return nullptr;
			}
		}

		Maths::Transform ClusterWorldTransform = CalculateGeometryClusterWorldTransform(Geometries, mCenterOfMass);
		mCenterOfMass = ClusterWorldTransform.pos;
		mGeometries = Geometries;
		mLocalTransforms.clear();
		mLocalTransforms.reserve(Geometries.size());

		Box3 Bounds = Box3::Empty();
		for (Geometry* Geom : Geometries)
		{
			const Maths::Transform LocalTransform = ClusterWorldTransform.TransformInv(*Geom->GetWorldTransform());
			mLocalTransforms.push_back(LocalTransform);
			Bounds.Encapsulate(Geom->GetBoundingVolume_WorldSpace());
		}
		mLocalBounds = Bounds;

		RigidBodyParam ClusterParam = Param;
		ClusterParam.rigidType = RigidType::Dynamic;
		RigidBody* Body = Simulation.CreateRigidBody(Geometries, ClusterParam, ClusterWorldTransform);
		mRigidBody = Body ? Body->CastDynamic() : nullptr;
		if (mRigidBody && bIsStatic)
		{
			mRigidBody->Freeze();
			mRigidBody->InvMass = 0.0f;
			mRigidBody->InvInertia = Matrix3::Zero();
		}
		return mRigidBody;
	}

	RigidBodyDynamic* DestructionCluster::BuildRigidBodyFromChunkBounds(PhysicsWorld& Simulation, const DestructionSet& DestructSet, const RigidBodyParam& Param)
	{
		if (mSourceIndices.empty())
		{
			return nullptr;
		}

		const std::vector<DestructionChunk>& Chunks = DestructSet.GetChunks();
		std::vector<Geometry*> Geometries;
		Geometries.reserve(mSourceIndices.size());
		for (int Index : mSourceIndices)
		{
			if (Index < 0 || (size_t)Index >= Chunks.size())
			{
				for (Geometry* Geom : Geometries)
				{
					GeometryFactory::DeleteGeometry(Geom);
				}
				return nullptr;
			}

			const DestructionChunk& Chunk = Chunks[(size_t)Index];
			Geometry* Geom = GeometryFactory::CreateOBB(Chunk.Bounds.GetCenter(), Chunk.Bounds.GetExtent());
			OverrideGeometryMass(Geom, Chunk);
			Geometries.push_back(Geom);
		}

		RigidBodyDynamic* Body = BuildRigidBody(Simulation, Param, Geometries);
		if (!Body)
		{
			for (Geometry* Geom : Geometries)
			{
				GeometryFactory::DeleteGeometry(Geom);
			}
		}
		return Body;
	}

	void DestructionCluster::ReleaseRigidBodyBinding()
	{
		mRigidBody = nullptr;
		mGeometries.clear();
	}

	void DestructionCluster::SetSourceIndices(const std::vector<int>& Indices)
	{
		mSourceIndices = Indices;
		if (mSourceIndices.empty())
		{
			mSourceIndexMin = -1;
		}
		else
		{
			mSourceIndexMin = *std::min_element(mSourceIndices.begin(), mSourceIndices.end());
		}
	}

}	// namespace Riemann
