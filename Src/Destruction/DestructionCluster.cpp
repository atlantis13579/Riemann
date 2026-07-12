
#include "DestructionCluster.h"

#include <algorithm>
#include <cmath>

#include "DestructionSet.h"
#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/ConvexMesh.h"
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
				const Vector3 ShapeCenter = Geom->GetTransform()->LocalToWorld(Mass ? Mass->CenterOfMass : Vector3::Zero());
				Center += ShapeCenter * ShapeMass;
				TotalMass += ShapeMass;
			}

			Maths::Transform WorldTransform;
			WorldTransform.pos = TotalMass > 1e-6f ? Center / TotalMass : FallbackCenter;
			WorldTransform.quat = Geometries.size() == 1 && FirstGeometry ? FirstGeometry->GetRotation() : Quaternion::One();
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

		Geometry* CreateGeometryForChunk(const DestructionChunk& Chunk)
		{
			if (Chunk.CollisionConvex && Chunk.CollisionConvex->GetNumVertices() >= 4 && Chunk.CollisionConvex->GetNumFaces() >= 4)
			{
				Geometry* Geom = GeometryFactory::CreateConvexMesh();
				ConvexMesh* Convex = Geom ? Geom->GetShapeObj<ConvexMesh>() : nullptr;
				if (Convex)
				{
					Convex->SetConvexData(
						Chunk.CollisionConvex->Vertices, Chunk.CollisionConvex->NumVertices,
						Chunk.CollisionConvex->Faces, Chunk.CollisionConvex->NumFaces,
						nullptr, 0,
						Chunk.CollisionConvex->Indices, Chunk.CollisionConvex->NumIndices,
						false);
					Geom->UpdateVolumeProperties();
					Geom->SetTransform(Chunk.Centroid, Quaternion::One());
					return Geom;
				}

				GeometryFactory::DeleteGeometry(Geom);
			}

			return GeometryFactory::CreateOBB(Chunk.Bounds.GetCenter(), Chunk.Bounds.GetExtent());
		}

		Geometry* CreateClusterCollisionProxy(const std::vector<Geometry*>& Geometries, const Vector3& LocalOrigin, const Box3& Bounds)
		{
			std::vector<Vector3> Points;
			for (Geometry* Geom : Geometries)
			{
				if (!Geom)
				{
					continue;
				}

				const Maths::Transform& WorldTransform = *Geom->GetTransform();
				const ConvexMesh* Convex = Geom->GetShapeType() == PrimitiveType::CONVEX_MESH
					? Geom->GetShapeObj<ConvexMesh>()
					: nullptr;
				if (Convex && Convex->Vertices && Convex->NumVertices >= 4)
				{
					for (uint16_t VertexIndex = 0; VertexIndex < Convex->NumVertices; ++VertexIndex)
					{
						Points.push_back(WorldTransform.LocalToWorld(Convex->Vertices[VertexIndex]));
					}
					continue;
				}

				Vector3 Corners[8];
				const Box3 WorldBounds = Geom->GetBounds();
				Box3::GetVertices(WorldBounds.Min, WorldBounds.Max, Corners);
				Points.insert(Points.end(), Corners, Corners + 8);
			}

			if (Points.size() >= 4)
			{
				Geometry* Proxy = GeometryFactory::CreateConvexMesh();
				ConvexMesh* ProxyConvex = Proxy ? Proxy->GetShapeObj<ConvexMesh>() : nullptr;
				const float Tolerance = std::max(Bounds.MaxDim() * 1e-5f, 1e-6f);
				if (ProxyConvex && ProxyConvex->BuildFromPoints(Points.data(), (int)Points.size(), LocalOrigin, Tolerance))
				{
					Proxy->UpdateVolumeProperties();
					Proxy->SetTransform(LocalOrigin, Quaternion::One());
					return Proxy;
				}
				GeometryFactory::DeleteGeometry(Proxy);
			}

			return GeometryFactory::CreateOBB(Bounds.GetCenter(), Bounds.GetExtent());
		}
	}

	DestructionCluster::DestructionCluster(DestructionSet* InDestructSet, int InLevel, const std::vector<int>& InSourceIndices)
		: mLevel(InLevel)
		, mDestructSet(InDestructSet)
	{
		mID = InDestructSet->AllocClusterID();
		SetSourceIndices(InSourceIndices);
	}

	DestructionCluster DestructionCluster::Create(DestructionSet& DestructSet, int Level, const std::vector<int>& Indices)
	{
		assert(!Indices.empty());
		DestructionCluster Cluster(&DestructSet, Level, Indices);

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
		const std::vector<Geometry*>& Geometries = GetGeometries();
		for (size_t Index = 0; Index < mSourceIndices.size(); ++Index)
		{
			if (mSourceIndices[Index] == ChunkIndex)
			{
				return Index < Geometries.size() ? Geometries[Index] : nullptr;
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

		const std::vector<Geometry*>& Geometries = GetGeometries();
		for (size_t Index = 0; Index < Geometries.size() && Index < mSourceIndices.size(); ++Index)
		{
			if (Geometries[Index] == Geom)
			{
				return mSourceIndices[Index];
			}
		}
		return -1;
	}

	const std::vector<Geometry*>& DestructionCluster::GetGeometries() const
	{
		static const std::vector<Geometry*> EmptyGeometries;
		return mRigidBody ? mRigidBody->Geometries() : EmptyGeometries;
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
		mLocalTransforms.clear();
		mLocalTransforms.reserve(Geometries.size());

		Box3 Bounds = Box3::Empty();
		for (Geometry* Geom : Geometries)
		{
			const Maths::Transform LocalTransform = ClusterWorldTransform.TransformInv(*Geom->GetTransform());
			mLocalTransforms.push_back(LocalTransform);
			Bounds.Encapsulate(Geom->GetBounds());
		}
		mLocalBounds = Bounds;

		const bool bUseCollisionProxy = Geometries.size() > 1 && mDestructSet
			&& mDestructSet->GetDestructionConstants().EnableClusterCollisionProxy;
		for (Geometry* Geom : Geometries)
		{
			Geom->SetQueryEnabled(!bUseCollisionProxy);
			Geom->SetSimulationEnabled(!bUseCollisionProxy);
		}

		std::vector<Geometry*> BodyGeometries = Geometries;
		Geometry* CollisionProxy = nullptr;
		if (bUseCollisionProxy)
		{
			CollisionProxy = CreateClusterCollisionProxy(Geometries, ClusterWorldTransform.pos, Bounds);
			if (CollisionProxy)
			{
				MassParameters* ProxyMass = CollisionProxy->GetMassParameters();
				ProxyMass->Mass = 0.0f;
				ProxyMass->Volume = 0.0f;
				ProxyMass->CenterOfMass = Vector3::Zero();
				ProxyMass->InertiaMat = Matrix3::Zero();
				BodyGeometries.push_back(CollisionProxy);
			}
		}

		RigidBodyParam ClusterParam = Param;
		ClusterParam.rigidType = RigidType::Dynamic;
		RigidBody* Body = Simulation.CreateRigidBody(BodyGeometries, ClusterParam, ClusterWorldTransform, false);
		mRigidBody = Body ? Body->CastDynamic() : nullptr;
		mCollisionProxy = mRigidBody ? CollisionProxy : nullptr;
		if (!mRigidBody && CollisionProxy)
		{
			GeometryFactory::DeleteGeometry(CollisionProxy);
		}
		if (mRigidBody)
		{
			mRigidBody->Parent = mDestructSet;
		}
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
			Geometry* Geom = CreateGeometryForChunk(Chunk);
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
		if (mRigidBody && mRigidBody->Parent == mDestructSet)
		{
			mRigidBody->Parent = nullptr;
		}
		mRigidBody = nullptr;
		mCollisionProxy = nullptr;
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
