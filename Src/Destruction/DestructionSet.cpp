
#include "DestructionSet.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>
#include <queue>
#include <random>
#include <set>

#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/ConvexMesh.h"
#include "../Geometry/DynamicMesh.h"
#include "../RigidBodyDynamics/PhysicsWorld.h"
#include "Fracture.h"
#include "StrainSolver.h"

namespace Riemann
{
	namespace
	{
		uint32_t HashClusterRevisionValue(uint32_t Hash, uint32_t Value)
		{
			Hash ^= Value;
			Hash *= 16777619u;
			return Hash;
		}

		uint32_t HashClusterRevisionPointer(uint32_t Hash, const void* Ptr)
		{
			const uint64_t Value = (uint64_t)reinterpret_cast<uintptr_t>(Ptr);
			Hash = HashClusterRevisionValue(Hash, (uint32_t)Value);
			Hash = HashClusterRevisionValue(Hash, (uint32_t)(Value >> 32));
			return Hash;
		}

		bool IntersectSphereAABB(const Vector3& Center, float Radius, const Box3& Bounds)
		{
			float DistanceSquared = 0.0f;
			const float Coordinates[3] = { Center.x, Center.y, Center.z };
			const float MinValues[3] = { Bounds.Min.x, Bounds.Min.y, Bounds.Min.z };
			const float MaxValues[3] = { Bounds.Max.x, Bounds.Max.y, Bounds.Max.z };
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				float Delta = 0.0f;
				if (Coordinates[Axis] < MinValues[Axis])
				{
					Delta = MinValues[Axis] - Coordinates[Axis];
				}
				else if (Coordinates[Axis] > MaxValues[Axis])
				{
					Delta = Coordinates[Axis] - MaxValues[Axis];
				}
				DistanceSquared += Delta * Delta;
			}
			return DistanceSquared <= Radius * Radius;
		}

		bool IsValidBounds(const Box3& Bounds)
		{
			return Bounds.Min.x <= Bounds.Max.x && Bounds.Min.y <= Bounds.Max.y && Bounds.Min.z <= Bounds.Max.z;
		}

		bool IsFiniteVector(const Vector3& Value)
		{
			return std::isfinite(Value.x) && std::isfinite(Value.y) && std::isfinite(Value.z);
		}

		Box3 CalculateMeshBounds(const DynamicMesh& Mesh)
		{
			Box3 Bounds = Box3::Empty();
			for (int VertexIndex = 0; VertexIndex < Mesh.GetVertexCount(); ++VertexIndex)
			{
				if (Mesh.IsVertexFast(VertexIndex))
				{
					Bounds.Encapsulate(Mesh.GetVertex(VertexIndex));
				}
			}
			return Bounds;
		}

		std::vector<Vector3> CollectMeshPoints(const DynamicMesh& Mesh)
		{
			std::vector<Vector3> Points;
			Points.reserve(Mesh.GetVertexCount());
			for (int VertexIndex = 0; VertexIndex < Mesh.GetVertexCount(); ++VertexIndex)
			{
				if (Mesh.IsVertexFast(VertexIndex))
				{
					Points.push_back(Mesh.GetVertex(VertexIndex));
				}
			}
			return Points;
		}

		int CountValidTriangles(const DynamicMesh& Mesh)
		{
			int Count = 0;
			for (int TriangleIndex = 0; TriangleIndex < Mesh.GetTriangleCount(); ++TriangleIndex)
			{
				Count += Mesh.IsTriangleFast(TriangleIndex) ? 1 : 0;
			}
			return Count;
		}

		bool BuildCompactMeshCopy(const DynamicMesh& SourceMesh, DynamicMesh& CompactMesh)
		{
			CompactMesh.Clear();
			std::vector<int> VertexMap((size_t)SourceMesh.GetVertexCount(), -1);
			for (int TriangleIndex = 0; TriangleIndex < SourceMesh.GetTriangleCount(); ++TriangleIndex)
			{
				if (!SourceMesh.IsTriangleFast(TriangleIndex))
				{
					continue;
				}
				const Index3 Triangle = SourceMesh.GetTriangle(TriangleIndex);
				if (Triangle[0] < 0 || Triangle[1] < 0 || Triangle[2] < 0
					|| (size_t)Triangle[0] >= VertexMap.size() || (size_t)Triangle[1] >= VertexMap.size() || (size_t)Triangle[2] >= VertexMap.size())
				{
					return false;
				}
				if (Triangle[0] == Triangle[1] || Triangle[0] == Triangle[2] || Triangle[1] == Triangle[2])
				{
					return false;
				}
				int CompactTriangle[3] = { -1, -1, -1 };
				for (int CornerIndex = 0; CornerIndex < 3; ++CornerIndex)
				{
					const int SourceVertex = Triangle[CornerIndex];
					if (!SourceMesh.IsVertex(SourceVertex))
					{
						return false;
					}
					int& CompactVertex = VertexMap[(size_t)SourceVertex];
					if (CompactVertex < 0)
					{
						const Vector3 Position = SourceMesh.GetVertex(SourceVertex);
						if (!IsFiniteVector(Position))
						{
							return false;
						}
						CompactVertex = CompactMesh.AppendVertex(Position);
					}
					CompactTriangle[CornerIndex] = CompactVertex;
				}
				if (CompactMesh.AppendTriangle(CompactTriangle[0], CompactTriangle[1], CompactTriangle[2]) < 0)
				{
					return false;
				}
			}

			if (CountValidTriangles(CompactMesh) < 4)
			{
				CompactMesh.Clear();
				return false;
			}
			CompactMesh.BuildBounds();
			CompactMesh.CalculateWeightAverageNormals();
			return true;
		}

		bool IsClosedMesh(const DynamicMesh& Mesh)
		{
			for (int EdgeIndex = 0; EdgeIndex < Mesh.GetEdgeCount(); ++EdgeIndex)
			{
				if (Mesh.IsEdgeFast(EdgeIndex) && Mesh.IsBoundaryEdge(EdgeIndex))
				{
					return false;
				}
			}
			return Mesh.GetEdgeCount() > 0;
		}

		bool CalculateMeshMassProperties(const DynamicMesh& Mesh, float& Volume, Vector3& Centroid)
		{
			const Vector3 Reference = Mesh.GetBounds().GetCenter();
			if (!IsFiniteVector(Reference))
			{
				return false;
			}
			double SignedVolume = 0.0;
			double WeightedX = 0.0;
			double WeightedY = 0.0;
			double WeightedZ = 0.0;
			for (int TriangleIndex = 0; TriangleIndex < Mesh.GetTriangleCount(); ++TriangleIndex)
			{
				if (!Mesh.IsTriangleFast(TriangleIndex))
				{
					continue;
				}

				const Index3 Triangle = Mesh.GetTriangle(TriangleIndex);
				if (!Mesh.IsVertex(Triangle[0]) || !Mesh.IsVertex(Triangle[1]) || !Mesh.IsVertex(Triangle[2]))
				{
					return false;
				}
				const Vector3 A = Mesh.GetVertex(Triangle[0]) - Reference;
				const Vector3 B = Mesh.GetVertex(Triangle[1]) - Reference;
				const Vector3 C = Mesh.GetVertex(Triangle[2]) - Reference;
				if (!IsFiniteVector(A) || !IsFiniteVector(B) || !IsFiniteVector(C))
				{
					return false;
				}

				const double CrossX = (double)B.y * (double)C.z - (double)B.z * (double)C.y;
				const double CrossY = (double)B.z * (double)C.x - (double)B.x * (double)C.z;
				const double CrossZ = (double)B.x * (double)C.y - (double)B.y * (double)C.x;
				const double TetraVolume = ((double)A.x * CrossX + (double)A.y * CrossY + (double)A.z * CrossZ) / 6.0;
				const double TetraCentroidX = ((double)A.x + (double)B.x + (double)C.x) * 0.25;
				const double TetraCentroidY = ((double)A.y + (double)B.y + (double)C.y) * 0.25;
				const double TetraCentroidZ = ((double)A.z + (double)B.z + (double)C.z) * 0.25;
				SignedVolume += TetraVolume;
				WeightedX += TetraCentroidX * TetraVolume;
				WeightedY += TetraCentroidY * TetraVolume;
				WeightedZ += TetraCentroidZ * TetraVolume;
			}

			if (!std::isfinite(SignedVolume) || std::fabs(SignedVolume) <= 1e-9)
			{
				return false;
			}

			Centroid = Vector3(
				Reference.x + (float)(WeightedX / SignedVolume),
				Reference.y + (float)(WeightedY / SignedVolume),
				Reference.z + (float)(WeightedZ / SignedVolume));
			Volume = (float)std::fabs(SignedVolume);
			if (!IsFiniteVector(Centroid) || !std::isfinite(Volume) || Volume <= 1e-6f)
			{
				return false;
			}

			const Box3 Bounds = Mesh.GetBounds();
			const float Tolerance = std::max(Bounds.MaxDim() * 1e-3f, 1e-4f);
			return Centroid.x >= Bounds.Min.x - Tolerance && Centroid.x <= Bounds.Max.x + Tolerance
				&& Centroid.y >= Bounds.Min.y - Tolerance && Centroid.y <= Bounds.Max.y + Tolerance
				&& Centroid.z >= Bounds.Min.z - Tolerance && Centroid.z <= Bounds.Max.z + Tolerance;
		}

		float GetMeshVolume(const DynamicMesh& Mesh)
		{
			float Volume = 0.0f;
			Vector3 Centroid;
			if (CalculateMeshMassProperties(Mesh, Volume, Centroid))
			{
				return Volume;
			}
			return std::max(Mesh.GetBounds().GetVolume(), 1e-6f);
		}

		float BoundsDistanceSquared(const Box3& A, const Box3& B)
		{
			float DistanceSquared = 0.0f;
			const float AMin[3] = { A.Min.x, A.Min.y, A.Min.z };
			const float AMax[3] = { A.Max.x, A.Max.y, A.Max.z };
			const float BMin[3] = { B.Min.x, B.Min.y, B.Min.z };
			const float BMax[3] = { B.Max.x, B.Max.y, B.Max.z };
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				float Distance = 0.0f;
				if (AMax[Axis] < BMin[Axis])
				{
					Distance = BMin[Axis] - AMax[Axis];
				}
				else if (BMax[Axis] < AMin[Axis])
				{
					Distance = AMin[Axis] - BMax[Axis];
				}
				DistanceSquared += Distance * Distance;
			}
			return DistanceSquared;
		}

		float PointBoundsDistanceSquared(const Vector3& Point, const Box3& Bounds)
		{
			float DistanceSquared = 0.0f;
			const float Coordinates[3] = { Point.x, Point.y, Point.z };
			const float Minimum[3] = { Bounds.Min.x, Bounds.Min.y, Bounds.Min.z };
			const float Maximum[3] = { Bounds.Max.x, Bounds.Max.y, Bounds.Max.z };
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				const float Delta = Coordinates[Axis] < Minimum[Axis]
					? Minimum[Axis] - Coordinates[Axis]
					: (Coordinates[Axis] > Maximum[Axis] ? Coordinates[Axis] - Maximum[Axis] : 0.0f);
				DistanceSquared += Delta * Delta;
			}
			return DistanceSquared;
		}

		void InitializeChunkFromBounds(DestructionChunk& Chunk, const Box3& Bounds, float Mass, bool bStatic, const std::string& Name)
		{
			Chunk.Name = Name;
			Chunk.Bounds = Bounds;
			Chunk.Centroid = Bounds.GetCenter();
			Chunk.Volume = std::max(Bounds.GetVolume(), 1e-3f);
			Chunk.Mass = std::max(Mass, 1e-3f);
			Chunk.Density = Chunk.Mass / Chunk.Volume;
			Chunk.IsStatic = bStatic;

			DestructionBondList TempList;
			TempList.InitCollapseStrength(Bounds);
			Chunk.CollapseStrength = TempList.Strength;
		}

		std::shared_ptr<ConvexMesh> BuildChunkCollisionConvex(const DynamicMesh& Mesh, const Vector3& LocalOrigin, const Box3& Bounds)
		{
			std::vector<Vector3> Points = CollectMeshPoints(Mesh);
			if (Points.size() < 4)
			{
				return std::shared_ptr<ConvexMesh>();
			}

			const float Tolerance = std::max(Bounds.MaxDim() * 1e-5f, 1e-6f);
			std::shared_ptr<ConvexMesh> Convex(new ConvexMesh());
			if (!Convex->BuildFromPoints(Points.data(), (int)Points.size(), LocalOrigin, Tolerance))
			{
				return std::shared_ptr<ConvexMesh>();
			}
			return Convex;
		}

		struct PreservedChunkBodyState
		{
			Vector3 LinearVelocity = Vector3::Zero();
			Vector3 AngularVelocity = Vector3::Zero();
			MorionType MotionType = MorionType::Discrete;
			float LinearDamping = 0.999f;
			float AngularDamping = 0.999f;
			float MaxContactImpulse = 10.0f;
			float SleepThreshold = 0.01f;
			float FreezeThreshold = 0.01f;
			float Restitution = 0.0f;
			bool DisableGravity = false;
			bool Sleeping = false;
			bool WasStaticCluster = false;
			bool Valid = false;
		};

		class HierarchicalFractureBuilder
		{
		public:
			explicit HierarchicalFractureBuilder(const DestructionFractureSettings& Settings)
				: mSettings(Settings)
				, mRandom((unsigned int)std::max(Settings.RandomSeed, 0))
				, mMaximumChunks(std::max(Settings.MaxChunkCount, 1))
			{
			}

			bool Build(const DynamicMesh& SourceMesh)
			{
				DynamicMesh CompactSourceMesh;
				float SourceVolume = 0.0f;
				Vector3 SourceCentroid;
				if (!BuildCompactMeshCopy(SourceMesh, CompactSourceMesh) || !IsValidBounds(CompactSourceMesh.GetBounds())
					|| !IsClosedMesh(CompactSourceMesh)
					|| !CalculateMeshMassProperties(CompactSourceMesh, SourceVolume, SourceCentroid))
				{
					return false;
				}

				mReservedLeafCount = 1;
				mRoot = BuildNode(CompactSourceMesh, -1, 0, true);
				if (mRoot < 0 || mLeafMeshes.empty())
				{
					return false;
				}
				return mSettings.Levels.empty() || mNodes[(size_t)mRoot].Children.size() >= 2;
			}

			const std::vector<DynamicMesh>& GetLeafMeshes() const { return mLeafMeshes; }
			const std::vector<DestructionHierarchyNode>& GetNodes() const { return mNodes; }
			const std::vector<std::vector<std::pair<int, int>>>& GetBoundaryBonds() const { return mBoundaryBonds; }
			const std::vector<int>& GetLeafNodes() const { return mLeafNodes; }
			int GetRoot() const { return mRoot; }

		private:
			int RandomInt(int Minimum, int Maximum)
			{
				if (Maximum <= Minimum)
				{
					return Minimum;
				}
				std::uniform_int_distribution<int> Distribution(Minimum, Maximum);
				return Distribution(mRandom);
			}

			float RandomFloat(float Minimum, float Maximum)
			{
				std::uniform_real_distribution<float> Distribution(Minimum, Maximum);
				return Distribution(mRandom);
			}

			int NextSeed()
			{
				return RandomInt(1, std::numeric_limits<int>::max());
			}

			bool ValidatePieces(const DynamicMesh& SourceMesh, const std::vector<FracturePiece>& Pieces, int MinimumPieceCount) const
			{
				if (Pieces.size() < (size_t)std::max(MinimumPieceCount, 2))
				{
					return false;
				}

				float SourceVolume = 0.0f;
				Vector3 SourceCentroid;
				if (!CalculateMeshMassProperties(SourceMesh, SourceVolume, SourceCentroid))
				{
					return false;
				}
				float PieceVolume = 0.0f;
				for (const FracturePiece& Piece : Pieces)
				{
					DynamicMesh CompactPiece;
					if (!BuildCompactMeshCopy(Piece.Mesh, CompactPiece) || !IsValidBounds(CompactPiece.GetBounds())
						|| !IsClosedMesh(CompactPiece))
					{
						return false;
					}
					for (int VertexIndex = 0; VertexIndex < CompactPiece.GetVertexCount(); ++VertexIndex)
					{
						if (CompactPiece.IsVertexFast(VertexIndex) && !IsFiniteVector(CompactPiece.GetVertex(VertexIndex)))
						{
							return false;
						}
					}
					float ThisPieceVolume = 0.0f;
					Vector3 ThisPieceCentroid;
					if (!CalculateMeshMassProperties(CompactPiece, ThisPieceVolume, ThisPieceCentroid))
					{
						return false;
					}
					if (mSettings.MinChunkVolume > 0.0f && ThisPieceVolume < mSettings.MinChunkVolume)
					{
						return false;
					}
					PieceVolume += ThisPieceVolume;
				}

				const float MaximumDimension = std::max(SourceMesh.GetBounds().MaxDim(), 1e-3f);
				const float RelativeGrout = std::max(0.0f, std::min(mSettings.Grout / MaximumDimension, 0.10f));
				const float MinimumRetainedVolume = std::max(0.55f, 0.70f - RelativeGrout * 1.5f);
				return std::isfinite(PieceVolume) && PieceVolume >= SourceVolume * MinimumRetainedVolume
					&& PieceVolume <= SourceVolume * 1.35f;
			}

			bool CutNode(const DynamicMesh& SourceMesh, int Depth, std::vector<FracturePiece>& Pieces)
			{
				if (Depth < 0 || (size_t)Depth >= mSettings.Levels.size())
				{
					return false;
				}

				const DestructionFractureLevel& Level = mSettings.Levels[(size_t)Depth];
				const int MinimumPieces = std::max(std::min(Level.MinPieceCount, Level.MaxPieceCount), 2);
				const int MaximumPieces = std::max(std::max(Level.MinPieceCount, Level.MaxPieceCount), MinimumPieces);
				const int RemainingBudget = mMaximumChunks - mReservedLeafCount + 1;
				const float SourceVolume = GetMeshVolume(SourceMesh);
				int VolumeLimitedPieces = RemainingBudget;
				if (mSettings.MinChunkVolume > 0.0f)
				{
					const double VolumeRatio = (double)SourceVolume / (double)mSettings.MinChunkVolume;
					if (std::isnan(VolumeRatio) || VolumeRatio <= 0.0)
					{
						return false;
					}
					if (VolumeRatio < (double)RemainingBudget)
					{
						VolumeLimitedPieces = std::max((int)std::floor(std::max(VolumeRatio, 0.0)), 0);
					}
				}
				const int MaximumAllowedPieces = std::min(RemainingBudget, VolumeLimitedPieces);
				if (MaximumAllowedPieces < 2)
				{
					return false;
				}

				const int AllowedMinimumPieces = std::min(MinimumPieces, MaximumAllowedPieces);
				const int AllowedMaximumPieces = std::min(MaximumPieces, MaximumAllowedPieces);
				PlanarCutOptions CutOptions;
				CutOptions.SnapTolerance = std::max(SourceMesh.GetBounds().MaxDim() * 1e-5f, 1e-6f);
				CutOptions.BoundsPaddingScale = 0.20f;
				CutOptions.Grout = std::max(mSettings.Grout, 0.0f);
				CutOptions.WeldSharedEdges = true;
				CutOptions.MinTriangleCount = 4;

				const int RetryCount = std::max(mSettings.CutRetryCount, 1);
				std::vector<int> PieceCountCandidates;
				const int InitialPieceCount = RandomInt(AllowedMinimumPieces, AllowedMaximumPieces);
				PieceCountCandidates.push_back(InitialPieceCount);
				for (int Candidate = AllowedMaximumPieces; Candidate >= AllowedMinimumPieces; --Candidate)
				{
					if (Candidate != InitialPieceCount)
					{
						PieceCountCandidates.push_back(Candidate);
					}
				}
				for (int Attempt = 0; Attempt < RetryCount; ++Attempt)
				{
					const int PieceCount = PieceCountCandidates[(size_t)Attempt % PieceCountCandidates.size()];
					const int Seed = NextSeed();
					CutOptions.RandomSeed = Seed;
					Pieces.clear();
					const bool bCut = Level.UseClusteredVoronoi
						? Fracture::ClusterVoronoiFracture(SourceMesh, Pieces, PieceCount, Seed, CutOptions)
						: Fracture::VoronoiFracture3D(SourceMesh, Pieces, PieceCount, Seed, CutOptions);
					if (bCut && Pieces.size() <= (size_t)RemainingBudget
						&& ValidatePieces(SourceMesh, Pieces, AllowedMinimumPieces))
					{
						return true;
					}
				}

				Pieces.clear();
				return false;
			}

			void FinalizeLeaf(int NodeIndex, const DynamicMesh& Mesh)
			{
				const int LeafIndex = (int)mLeafMeshes.size();
				mLeafMeshes.push_back(Mesh);
				mLeafBounds.push_back(Mesh.GetBounds());
				mLeafNodes.push_back(NodeIndex);
				mNodes[(size_t)NodeIndex].ChunkIndices.assign(1, LeafIndex);
				mNodes[(size_t)NodeIndex].IsBreakable = false;
			}

			std::pair<int, int> FindClosestLeafPair(int NodeA, int NodeB) const
			{
				std::pair<int, int> Result(-1, -1);
				float BestDistance = FLT_MAX;
				const std::vector<int>& LeavesA = mNodes[(size_t)NodeA].ChunkIndices;
				const std::vector<int>& LeavesB = mNodes[(size_t)NodeB].ChunkIndices;
				for (int LeafA : LeavesA)
				{
					for (int LeafB : LeavesB)
					{
						if (LeafA < 0 || LeafB < 0 || (size_t)LeafA >= mLeafBounds.size() || (size_t)LeafB >= mLeafBounds.size())
						{
							continue;
						}
						const float Distance = BoundsDistanceSquared(mLeafBounds[(size_t)LeafA], mLeafBounds[(size_t)LeafB]);
						if (Distance < BestDistance)
						{
							BestDistance = Distance;
							Result = std::make_pair(LeafA, LeafB);
						}
					}
				}
				return Result;
			}

			void BuildBoundaryBonds(int NodeIndex, const std::vector<FracturePiece>& Pieces)
			{
				const std::vector<int>& Children = mNodes[(size_t)NodeIndex].Children;
				const int ChildCount = (int)Children.size();
				if (ChildCount < 2)
				{
					return;
				}

				std::vector<int> Parents((size_t)ChildCount);
				for (int ChildIndex = 0; ChildIndex < ChildCount; ++ChildIndex)
				{
					Parents[(size_t)ChildIndex] = ChildIndex;
				}
				auto FindRoot = [&Parents](int Node)
				{
					int Root = Node;
					while (Parents[(size_t)Root] != Root)
					{
						Root = Parents[(size_t)Root];
					}
					while (Parents[(size_t)Node] != Node)
					{
						const int Next = Parents[(size_t)Node];
						Parents[(size_t)Node] = Root;
						Node = Next;
					}
					return Root;
				};

				std::set<std::pair<int, int>> ChildEdges;
				auto AddChildEdge = [&ChildEdges, &Parents, &FindRoot](int A, int B)
				{
					if (A == B)
					{
						return;
					}
					if (A > B)
					{
						std::swap(A, B);
					}
					ChildEdges.insert(std::make_pair(A, B));
					const int RootA = FindRoot(A);
					const int RootB = FindRoot(B);
					if (RootA != RootB)
					{
						Parents[(size_t)RootB] = RootA;
					}
				};

				for (int PieceIndex = 0; PieceIndex < ChildCount && (size_t)PieceIndex < Pieces.size(); ++PieceIndex)
				{
					for (int NeighborIndex : Pieces[(size_t)PieceIndex].NeighborPieceIndices)
					{
						if (NeighborIndex >= 0 && NeighborIndex < ChildCount)
						{
							AddChildEdge(PieceIndex, NeighborIndex);
						}
					}
				}

				while (true)
				{
					float BestDistance = FLT_MAX;
					int BestA = -1;
					int BestB = -1;
					for (int A = 0; A < ChildCount; ++A)
					{
						for (int B = A + 1; B < ChildCount; ++B)
						{
							if (FindRoot(A) == FindRoot(B))
							{
								continue;
							}
							const float Distance = BoundsDistanceSquared(mNodes[(size_t)Children[(size_t)A]].Bounds,
								mNodes[(size_t)Children[(size_t)B]].Bounds);
							if (Distance < BestDistance)
							{
								BestDistance = Distance;
								BestA = A;
								BestB = B;
							}
						}
					}
					if (BestA < 0 || BestB < 0)
					{
						break;
					}
					AddChildEdge(BestA, BestB);
				}

				std::set<std::pair<int, int>> UniqueLeafBonds;
				for (const std::pair<int, int>& Edge : ChildEdges)
				{
					std::pair<int, int> LeafPair = FindClosestLeafPair(Children[(size_t)Edge.first], Children[(size_t)Edge.second]);
					if (LeafPair.first < 0 || LeafPair.second < 0)
					{
						continue;
					}
					if (LeafPair.first > LeafPair.second)
					{
						std::swap(LeafPair.first, LeafPair.second);
					}
					UniqueLeafBonds.insert(LeafPair);
				}
				mBoundaryBonds[(size_t)NodeIndex].assign(UniqueLeafBonds.begin(), UniqueLeafBonds.end());
			}

			int BuildNode(const DynamicMesh& Mesh, int Parent, int Depth, bool bAllowCut)
			{
				// Planar cuts may leave sparse element IDs, so compact every piece before another recursive cut.
				DynamicMesh CompactMesh;
				if (!BuildCompactMeshCopy(Mesh, CompactMesh))
				{
					return -1;
				}
				const DynamicMesh* UseMesh = &CompactMesh;
				DestructionHierarchyNode Node;
				Node.Parent = Parent;
				Node.Level = Depth;
				Node.Bounds = UseMesh->GetBounds();
				const int NodeIndex = (int)mNodes.size();
				mNodes.push_back(Node);
				mBoundaryBonds.emplace_back();

				const float MeshVolume = GetMeshVolume(*UseMesh);
				const bool bCanAttemptCut = bAllowCut && (size_t)Depth < mSettings.Levels.size()
					&& MeshVolume >= std::max(mSettings.MinChunkVolume, 0.0f) * 2.0f
					&& mReservedLeafCount < mMaximumChunks;
				std::vector<FracturePiece> Pieces;
				if (!bCanAttemptCut || !CutNode(*UseMesh, Depth, Pieces))
				{
					FinalizeLeaf(NodeIndex, *UseMesh);
					return NodeIndex;
				}

				mReservedLeafCount += (int)Pieces.size() - 1;
				const DestructionFractureLevel& Level = mSettings.Levels[(size_t)Depth];
				const float Variation = std::max(0.0f, std::min(Level.StrengthVariation, 0.95f));
				mNodes[(size_t)NodeIndex].BreakThreshold = std::max(Level.BreakThreshold, 0.0f)
					* (1.0f + RandomFloat(-Variation, Variation));
				mNodes[(size_t)NodeIndex].IsBreakable = true;

				std::vector<int> PieceOrder(Pieces.size());
				std::vector<float> PieceVolumes(Pieces.size(), 0.0f);
				for (size_t PieceIndex = 0; PieceIndex < Pieces.size(); ++PieceIndex)
				{
					PieceOrder[PieceIndex] = (int)PieceIndex;
					PieceVolumes[PieceIndex] = GetMeshVolume(Pieces[PieceIndex].Mesh);
				}
				std::sort(PieceOrder.begin(), PieceOrder.end(), [&PieceVolumes](int A, int B)
				{
					const float VolumeA = PieceVolumes[(size_t)A];
					const float VolumeB = PieceVolumes[(size_t)B];
					return VolumeA == VolumeB ? A < B : VolumeA > VolumeB;
				});

				std::vector<bool> Preserve(Pieces.size(), false);
				const int PreserveCount = std::min(std::max(Level.PreserveLargestPieces, 0), (int)Pieces.size());
				for (int PreserveIndex = 0; PreserveIndex < PreserveCount; ++PreserveIndex)
				{
					Preserve[(size_t)PieceOrder[(size_t)PreserveIndex]] = true;
				}

				const float RecurseProbability = std::max(0.0f, std::min(Level.ChildFractureProbability, 1.0f));
				for (size_t PieceIndex = 0; PieceIndex < Pieces.size(); ++PieceIndex)
				{
					const bool bRecurse = !Preserve[PieceIndex] && RandomFloat(0.0f, 1.0f) < RecurseProbability;
					const int ChildNode = BuildNode(Pieces[PieceIndex].Mesh, NodeIndex, Depth + 1, bRecurse);
					if (ChildNode < 0)
					{
						return -1;
					}
					mNodes[(size_t)NodeIndex].Children.push_back(ChildNode);
					const std::vector<int>& ChildLeaves = mNodes[(size_t)ChildNode].ChunkIndices;
					mNodes[(size_t)NodeIndex].ChunkIndices.insert(mNodes[(size_t)NodeIndex].ChunkIndices.end(),
						ChildLeaves.begin(), ChildLeaves.end());
				}

				BuildBoundaryBonds(NodeIndex, Pieces);
				return NodeIndex;
			}

		private:
			const DestructionFractureSettings& mSettings;
			std::mt19937 mRandom;
			int mMaximumChunks = 1;
			int mReservedLeafCount = 1;
			int mRoot = -1;
			std::vector<DynamicMesh> mLeafMeshes;
			std::vector<Box3> mLeafBounds;
			std::vector<int> mLeafNodes;
			std::vector<DestructionHierarchyNode> mNodes;
			std::vector<std::vector<std::pair<int, int>>> mBoundaryBonds;
		};
	}

	static float GetBodyMassForContact(const RigidBody* Body)
	{
		if (Body == nullptr || Body->GetInverseMass() <= kMinimumInvMass)
		{
			return 1.0f;
		}
		return 1.0f / Body->GetInverseMass();
	}

	static Vector3 BuildContactMomentum(const PhysicsContactInfo& ContactInfo, const PhysicsContactPoint& Point)
	{
		Vector3 Momentum = Point.Impulse;
		if (Momentum.SquareLength() > 1e-6f)
		{
			return Momentum;
		}

		const Vector3 VelocityA = ContactInfo.BodyA ? ContactInfo.BodyA->GetLinearVelocity() : Vector3::Zero();
		const Vector3 VelocityB = ContactInfo.BodyB ? ContactInfo.BodyB->GetLinearVelocity() : Vector3::Zero();
		const float ClosingSpeed = std::max(0.0f, -(VelocityB - VelocityA).Dot(Point.Normal));
		const float EffectiveMass = std::min(GetBodyMassForContact(ContactInfo.BodyA), GetBodyMassForContact(ContactInfo.BodyB));
		return Point.Normal * (ClosingSpeed * EffectiveMass);
	}

	DestructionConstants::DestructionConstants()
	{
		Levels[0] = { 10.0f, 3 };
		Levels[1] = { 100.0f, 2 };
		Levels[2] = { 1000.0f, 2 };
		Levels[3] = { 1000.0f, 2 };
		Levels[4] = { 1000.0f, 2 };
		Levels[5] = { 1000.0f, 2 };
		Levels[6] = { 1000.0f, 2 };
		Levels[7] = { 1000.0f, 2 };
	}

	DestructionSet::DestructionSet() = default;

	DestructionSet::DestructionSet(PhysicsWorld* Simulation)
	{
		if (Simulation)
		{
			Simulation->AddDestructionSet(this);
		}
	}

	DestructionSet::DestructionSet(PhysicsWorld& Simulation)
	{
		Simulation.AddDestructionSet(this);
	}

	DestructionSet::~DestructionSet()
	{
		ClearClusters();
		if (mPhysicsWorld)
		{
			mPhysicsWorld->RemoveDestructionSet(this);
			mPhysicsWorld = nullptr;
		}
	}

	int DestructionSet::AddChunk(const Box3& Bounds, float Mass, bool bStatic, const std::string& Name)
	{
		if (!mHierarchyNodes.empty())
		{
			mPartitionQueue.clear();
			mPartitionTicksUntilProcess = 0;
			mHierarchyNodes.clear();
			mHierarchyBoundaryBonds.clear();
			mChunkHierarchyLeaves.clear();
			mHierarchyRoot = -1;
		}
		DestructionChunk Chunk;
		InitializeChunkFromBounds(Chunk, Bounds, Mass, bStatic, Name);

		const int Index = (int)mChunks.size();
		mChunks.push_back(Chunk);
		mGraph.Resize(mChunks.size());
		RecalculateBounds();
		return Index;
	}

	int DestructionSet::AddChunk(const DynamicMesh& Mesh, float Mass, bool bStatic, const std::string& Name)
	{
		if (!mHierarchyNodes.empty())
		{
			mPartitionQueue.clear();
			mPartitionTicksUntilProcess = 0;
			mHierarchyNodes.clear();
			mHierarchyBoundaryBonds.clear();
			mChunkHierarchyLeaves.clear();
			mHierarchyRoot = -1;
		}
		Box3 Bounds = CalculateMeshBounds(Mesh);
		if (!IsValidBounds(Bounds))
		{
			Bounds = Mesh.GetBounds();
		}

		DestructionChunk Chunk;
		InitializeChunkFromBounds(Chunk, Bounds, Mass, bStatic, Name);
		float MeshVolume = 0.0f;
		Vector3 MeshCentroid;
		if (CalculateMeshMassProperties(Mesh, MeshVolume, MeshCentroid))
		{
			Chunk.Centroid = MeshCentroid;
			Chunk.Volume = std::max(MeshVolume, 1e-3f);
			Chunk.Density = Chunk.Mass / Chunk.Volume;
		}
		if (IsValidBounds(Bounds))
		{
			Chunk.CollisionConvex = BuildChunkCollisionConvex(Mesh, Chunk.Centroid, Bounds);
		}

		const int Index = (int)mChunks.size();
		mChunks.push_back(Chunk);
		mGraph.Resize(mChunks.size());
		RecalculateBounds();
		return Index;
	}

	bool DestructionSet::BuildHierarchicalFracture(const DynamicMesh& SourceMesh, const DestructionFractureSettings& Settings,
		float TotalMass, bool bStatic, const std::string& Name)
	{
		HierarchicalFractureBuilder Builder(Settings);
		if (!Builder.Build(SourceMesh))
		{
			return false;
		}

		const std::vector<DynamicMesh>& LeafMeshes = Builder.GetLeafMeshes();
		const std::vector<DestructionHierarchyNode>& Hierarchy = Builder.GetNodes();
		const std::vector<std::vector<std::pair<int, int>>>& BoundaryBonds = Builder.GetBoundaryBonds();
		const std::vector<int>& LeafNodes = Builder.GetLeafNodes();
		if (LeafMeshes.empty() || LeafNodes.size() != LeafMeshes.size() || BoundaryBonds.size() != Hierarchy.size())
		{
			return false;
		}

		Clear();
		for (size_t LeafIndex = 0; LeafIndex < LeafMeshes.size(); ++LeafIndex)
		{
			std::string ChunkName = Name;
			if (!ChunkName.empty())
			{
				ChunkName += "_piece_" + std::to_string(LeafIndex);
			}
			AddChunk(LeafMeshes[LeafIndex], 1.0f, bStatic, ChunkName);
		}

		float TotalVolume = 0.0f;
		for (const DestructionChunk& Chunk : mChunks)
		{
			TotalVolume += std::max(Chunk.Volume, 1e-3f);
		}
		const float UseTotalMass = std::max(TotalMass, 1e-3f);
		for (DestructionChunk& Chunk : mChunks)
		{
			Chunk.Mass = UseTotalMass * std::max(Chunk.Volume, 1e-3f) / std::max(TotalVolume, 1e-3f);
			Chunk.Density = Chunk.Mass / std::max(Chunk.Volume, 1e-3f);
		}

		mHierarchyNodes = Hierarchy;
		mHierarchyBoundaryBonds = BoundaryBonds;
		mChunkHierarchyLeaves = LeafNodes;
		mHierarchyRoot = Builder.GetRoot();
		mHierarchyDamageAttenuation = std::max(0.0f, std::min(Settings.DamageAttenuation, 1.0f));
		mHierarchyNeighborDamageFraction = std::max(0.0f, std::min(Settings.NeighborDamageFraction, 1.0f));

		int BondIndex = 0;
		for (size_t NodeIndex = 0; NodeIndex < mHierarchyBoundaryBonds.size(); ++NodeIndex)
		{
			const float NodeStrength = NodeIndex < mHierarchyNodes.size()
				? std::max(mHierarchyNodes[NodeIndex].BreakThreshold, 1e-3f)
				: 1.0f;
			for (const std::pair<int, int>& BondPair : mHierarchyBoundaryBonds[NodeIndex])
			{
				if (AddBond(BondPair.first, BondPair.second, DestructionBondType::Connect,
					std::max(Settings.BondStrength, 1e-3f) * NodeStrength, BondIndex))
				{
					++BondIndex;
				}
			}
		}

		RecalculateBounds();
		RebuildClustersFromGraph();
		if (mChunks.size() > 1 && mClusters.size() != 1)
		{
			Clear();
			return false;
		}
		return true;
	}

	void DestructionSet::SetChunks(const std::vector<DestructionChunk>& Chunks)
	{
		std::vector<DestructionChunk> NewChunks = Chunks;
		Clear();
		mChunks = std::move(NewChunks);
		for (DestructionChunk& Chunk : mChunks)
		{
			if (Chunk.Volume <= 0.0f)
			{
				Chunk.Volume = std::max(Chunk.Bounds.GetVolume(), 1e-3f);
			}
			if (Chunk.Mass <= 0.0f)
			{
				Chunk.Mass = std::max(Chunk.Volume * Chunk.Density, 1e-3f);
			}
			const float Tolerance = std::max(Chunk.Bounds.MaxDim() * 1e-3f, 1e-4f);
			const bool bCentroidInBounds = Chunk.CollisionConvex && IsFiniteVector(Chunk.Centroid)
				&& Chunk.Centroid.x >= Chunk.Bounds.Min.x - Tolerance && Chunk.Centroid.x <= Chunk.Bounds.Max.x + Tolerance
				&& Chunk.Centroid.y >= Chunk.Bounds.Min.y - Tolerance && Chunk.Centroid.y <= Chunk.Bounds.Max.y + Tolerance
				&& Chunk.Centroid.z >= Chunk.Bounds.Min.z - Tolerance && Chunk.Centroid.z <= Chunk.Bounds.Max.z + Tolerance;
			if (!bCentroidInBounds)
			{
				Chunk.Centroid = Chunk.Bounds.GetCenter();
			}
		}
		mGraph.Resize(mChunks.size());
		RecalculateBounds();
	}

	void DestructionSet::Clear()
	{
		DisableStressSolver();
		ClearClusters();
		mGraph.Clear();
		mBounds = Box3::Empty();
		mChunks.clear();
		mHierarchyNodes.clear();
		mHierarchyBoundaryBonds.clear();
		mChunkHierarchyLeaves.clear();
		mHierarchyRoot = -1;
		mHierarchyDamageAttenuation = 0.6f;
		mHierarchyNeighborDamageFraction = 0.15f;
		mMomentumQueue.clear();
		mPartitionQueue.clear();
		mPartitionTicksUntilProcess = 0;
		mBrokenIndices.clear();
		for (int Type = 0; Type < (int)DestructionEventType::Count; ++Type)
		{
			mDestructionInfo[Type].clear();
		}
		bHasDestructionInfo = false;
		mNextClusterID = 1;
	}

	bool DestructionSet::AddBond(int NodeA, int NodeB, DestructionBondType Type, float Strength, int BondIndex)
	{
		return mGraph.AddBond(NodeA, NodeB, Type, Strength, BondIndex);
	}

	bool DestructionSet::SetBondStrength(int NodeA, int NodeB, float Strength)
	{
		return mGraph.SetBondStrength(NodeA, NodeB, Strength);
	}

	void DestructionSet::RemoveAllBonds()
	{
		ClearClusters();
		mHierarchyNodes.clear();
		mHierarchyBoundaryBonds.clear();
		mChunkHierarchyLeaves.clear();
		mHierarchyRoot = -1;
		mGraph.Clear();
		mGraph.Resize(mChunks.size());
		mStressImpulses.clear();
		if (mStressSolver) mStressSolver->ResetGraph(mGraph);
	}

	void DestructionSet::BuildClusterRigidBody(DestructionCluster& Cluster)
	{
		if (mPhysicsWorld == nullptr)
		{
			return;
		}

		RigidBodyParam Param;
		Param.rigidType = RigidType::Dynamic;
		Cluster.BuildRigidBodyFromChunkBounds(*mPhysicsWorld, *this, Param);
	}

	void DestructionSet::ReleaseClusterRigidBody(DestructionCluster& Cluster, bool bReleaseGeometries)
	{
		RigidBodyDynamic* Body = Cluster.GetRigidBody();
		if (Body == nullptr)
		{
			return;
		}

		if (mPhysicsWorld == nullptr)
		{
			Cluster.ReleaseRigidBodyBinding();
			return;
		}

		mPhysicsWorld->RemoveRigidBody(Body);
		Geometry* CollisionProxy = Cluster.GetCollisionProxy();
		Cluster.ReleaseRigidBodyBinding();
		if (bReleaseGeometries)
		{
			Body->ReleaseGeometries();
		}
		else
		{
			GeometryFactory::DeleteGeometry(CollisionProxy);
			Body->mGeometries.clear();
			Body->mGeometryHandles.clear();
		}
		delete Body;
	}

	void DestructionSet::ReleaseClusterRigidBodies(bool bReleaseGeometries)
	{
		for (std::map<uint32_t, DestructionCluster>::value_type& Entry : mClusters)
		{
			ReleaseClusterRigidBody(Entry.second, bReleaseGeometries);
		}
	}

	uint32_t DestructionSet::AddCluster(int Level, const std::vector<int>& Indices)
	{
		DestructionCluster Cluster = DestructionCluster::Create(*this, Level, Indices);
		const uint32_t ID = Cluster.GetID();
		BuildClusterRigidBody(Cluster);
		mClusters[ID] = Cluster;
		return ID;
	}

	bool DestructionSet::RemoveCluster(uint32_t ClusterID)
	{
		std::map<uint32_t, DestructionCluster>::iterator It = mClusters.find(ClusterID);
		if (It == mClusters.end())
		{
			return false;
		}
		ReleaseClusterRigidBody(It->second, true);
		mClusters.erase(It);
		return true;
	}

	void DestructionSet::ClearClusters()
	{
		mMomentumQueue.clear();
		mPartitionQueue.clear();
		mPartitionTicksUntilProcess = 0;
		ReleaseClusterRigidBodies(true);
		mClusters.clear();
	}

	void DestructionSet::BreakHierarchyCrossChildBonds(int HierarchyNode)
	{
		if (HierarchyNode < 0 || (size_t)HierarchyNode >= mHierarchyNodes.size())
		{
			return;
		}
		const DestructionHierarchyNode& Node = mHierarchyNodes[(size_t)HierarchyNode];
		std::vector<int> ChildOwner(mChunks.size(), -1);
		for (size_t ChildOffset = 0; ChildOffset < Node.Children.size(); ++ChildOffset)
		{
			const int ChildIndex = Node.Children[ChildOffset];
			if (ChildIndex < 0 || (size_t)ChildIndex >= mHierarchyNodes.size())
			{
				continue;
			}
			for (int ChunkIndex : mHierarchyNodes[(size_t)ChildIndex].ChunkIndices)
			{
				if (ChunkIndex >= 0 && (size_t)ChunkIndex < ChildOwner.size())
				{
					ChildOwner[(size_t)ChunkIndex] = (int)ChildOffset;
				}
			}
		}

		std::set<std::pair<int, int>> BondsToBreak;
		for (int ChunkIndex : Node.ChunkIndices)
		{
			if (!mGraph.IsValidNode(ChunkIndex) || ChunkIndex < 0 || (size_t)ChunkIndex >= ChildOwner.size())
			{
				continue;
			}
			const int Owner = ChildOwner[(size_t)ChunkIndex];
			for (const Bond& ExistingBond : mGraph[(size_t)ChunkIndex].Bonds)
			{
				const int Other = ExistingBond.GetOther(ChunkIndex);
				if (Other < 0 || (size_t)Other >= ChildOwner.size() || ChildOwner[(size_t)Other] < 0
					|| ChildOwner[(size_t)Other] == Owner)
				{
					continue;
				}
				BondsToBreak.insert(std::minmax(ChunkIndex, Other));
			}
		}
		for (const std::pair<int, int>& BondPair : BondsToBreak)
		{
			mGraph.BreakBond(BondPair.first, BondPair.second);
		}
	}

	void DestructionSet::ReconcileHierarchyWithGraph()
	{
		for (size_t NodeIndex = 0; NodeIndex < mHierarchyNodes.size() && NodeIndex < mHierarchyBoundaryBonds.size(); ++NodeIndex)
		{
			DestructionHierarchyNode& Node = mHierarchyNodes[NodeIndex];
			const std::vector<std::pair<int, int>>& BoundaryBonds = mHierarchyBoundaryBonds[NodeIndex];
			if (!Node.IsBreakable || BoundaryBonds.empty())
			{
				continue;
			}

			bool bBoundaryWasBroken = Node.IsBroken;
			for (const std::pair<int, int>& BondPair : BoundaryBonds)
			{
				bBoundaryWasBroken = bBoundaryWasBroken || !mGraph.HasBond(BondPair.first, BondPair.second);
			}
			if (!bBoundaryWasBroken)
			{
				continue;
			}

			Node.IsBroken = true;
			BreakHierarchyCrossChildBonds((int)NodeIndex);
		}
	}

	void DestructionSet::RebuildClustersFromGraph()
	{
		ReconcileHierarchyWithGraph();
		std::map<int, Geometry*> ExistingGeometries;
		std::vector<PreservedChunkBodyState> PreservedStates(mChunks.size());
		if (mPhysicsWorld != nullptr)
		{
			for (std::map<uint32_t, DestructionCluster>::value_type& Entry : mClusters)
			{
				DestructionCluster& Cluster = Entry.second;
				const std::vector<int>& SourceIndices = Cluster.GetSourceIndices();
				const std::vector<Geometry*>& Geometries = Cluster.GetGeometries();
				const size_t Count = std::min(SourceIndices.size(), Geometries.size());
				RigidBodyDynamic* Body = Cluster.GetRigidBody();
				for (size_t Index = 0; Index < Count; ++Index)
				{
					if (Geometries[Index] != nullptr)
					{
						Geometry* Geom = Geometries[Index];
						Transform WorldTransform = *Geom->GetTransform();
						if (Body)
						{
							WorldTransform = Body->GetGeometryTransform(Geom);
							Geom->SetTransform(WorldTransform.pos, WorldTransform.quat);
						}
						Geom->SetParent(nullptr);

						const int ChunkIndex = SourceIndices[Index];
						ExistingGeometries[ChunkIndex] = Geom;
						if (Body && ChunkIndex >= 0 && (size_t)ChunkIndex < PreservedStates.size())
						{
							PreservedChunkBodyState& State = PreservedStates[(size_t)ChunkIndex];
							const MassParameters* Mass = Geom->GetMassParameters();
							const Vector3 WorldCenterOfMass = WorldTransform.LocalToWorld(Mass ? Mass->CenterOfMass : Vector3::Zero());
							State.LinearVelocity = Body->GetLinearVelocity() + Body->GetAngularVelocity().Cross(WorldCenterOfMass - Body->X);
							State.AngularVelocity = Body->GetAngularVelocity();
							State.MotionType = Body->mMotionType;
							State.LinearDamping = Body->LinearDamping;
							State.AngularDamping = Body->AngularDamping;
							State.MaxContactImpulse = Body->MaxContactImpulse;
							State.SleepThreshold = Body->SleepThreshold;
							State.FreezeThreshold = Body->FreezeThreshold;
							State.Restitution = Body->GetRestitution();
							State.DisableGravity = Body->DisableGravity;
							State.Sleeping = Body->Sleeping;
							State.WasStaticCluster = Cluster.IsStatic();
							State.Valid = true;
						}
					}
				}
			}
		}

		ReleaseClusterRigidBodies(false);
		mClusters.clear();
		BitSet Visited(mChunks.size());

		for (size_t Node = 0; Node < mChunks.size(); ++Node)
		{
			if (Visited.get(Node) || mChunks[Node].IsFree)
			{
				continue;
			}

			std::vector<int> Component;
			std::queue<int> Queue;
			Queue.push((int)Node);
			Visited.insert(Node);
			while (!Queue.empty())
			{
				const int Current = Queue.front();
				Queue.pop();
				Component.push_back(Current);
				if (!mGraph.IsValidNode(Current))
				{
					continue;
				}
				for (const Bond& ExistingBond : mGraph[(size_t)Current].Bonds)
				{
					const int Other = ExistingBond.GetOther(Current);
					if (Other >= 0 && (size_t)Other < mChunks.size() && !Visited.get((size_t)Other) && !mChunks[(size_t)Other].IsFree)
					{
						Visited.insert((size_t)Other);
						Queue.push(Other);
					}
				}
			}

			if (!Component.empty())
			{
				DestructionCluster Cluster = DestructionCluster::Create(*this, -1, Component);
				const uint32_t ID = Cluster.GetID();
				if (mPhysicsWorld != nullptr)
				{
					std::vector<Geometry*> Geometries;
					Geometries.reserve(Component.size());
					bool bHasExistingGeometries = true;
					for (int ChunkIndex : Component)
					{
						std::map<int, Geometry*>::iterator It = ExistingGeometries.find(ChunkIndex);
						if (It == ExistingGeometries.end() || It->second == nullptr)
						{
							bHasExistingGeometries = false;
							break;
						}
						Geometries.push_back(It->second);
					}

					RigidBodyParam Param;
					Param.rigidType = RigidType::Dynamic;
					RigidBodyDynamic* NewBody = nullptr;
					if (bHasExistingGeometries)
					{
						NewBody = Cluster.BuildRigidBody(*mPhysicsWorld, Param, Geometries);
					}
					if (NewBody)
					{
						for (int ChunkIndex : Component)
						{
							ExistingGeometries.erase(ChunkIndex);
						}
					}
					else
					{
						NewBody = Cluster.BuildRigidBodyFromChunkBounds(*mPhysicsWorld, *this, Param);
					}

					if (NewBody && !Cluster.IsStatic())
					{
						Vector3 LinearVelocity = Vector3::Zero();
						Vector3 AngularVelocity = Vector3::Zero();
						float TotalMass = 0.0f;
						const PreservedChunkBodyState* FirstState = nullptr;
						bool bAllSleeping = true;
						bool bAnyWasStaticCluster = false;
						for (int ChunkIndex : Component)
						{
							if (ChunkIndex < 0 || (size_t)ChunkIndex >= PreservedStates.size())
							{
								continue;
							}
							const PreservedChunkBodyState& State = PreservedStates[(size_t)ChunkIndex];
							if (!State.Valid)
							{
								continue;
							}
							const float Mass = std::max(mChunks[(size_t)ChunkIndex].Mass, 1e-3f);
							LinearVelocity += State.LinearVelocity * Mass;
							AngularVelocity += State.AngularVelocity * Mass;
							TotalMass += Mass;
							FirstState = FirstState ? FirstState : &State;
							bAllSleeping = bAllSleeping && State.Sleeping;
							bAnyWasStaticCluster = bAnyWasStaticCluster || State.WasStaticCluster;
						}

						if (TotalMass > 0.0f && FirstState)
						{
							NewBody->SetLinearVelocity(LinearVelocity / TotalMass);
							NewBody->SetAngularVelocity(AngularVelocity / TotalMass);
							NewBody->mMotionType = FirstState->MotionType;
							NewBody->LinearDamping = FirstState->LinearDamping;
							NewBody->AngularDamping = FirstState->AngularDamping;
							NewBody->MaxContactImpulse = FirstState->MaxContactImpulse;
							NewBody->SleepThreshold = FirstState->SleepThreshold;
							NewBody->FreezeThreshold = FirstState->FreezeThreshold;
							NewBody->SetRestitution(FirstState->Restitution);
							NewBody->DisableGravity = FirstState->DisableGravity;
							NewBody->SolverV = NewBody->GetLinearVelocity();
							NewBody->SolverW = NewBody->GetAngularVelocity();
							if (bAllSleeping && !bAnyWasStaticCluster)
							{
								NewBody->Sleeping = true;
							}
							else
							{
								NewBody->Wakeup();
							}
						}
					}
				}
				mClusters[ID] = Cluster;
			}
		}

		for (std::map<int, Geometry*>::value_type& Entry : ExistingGeometries)
		{
			GeometryFactory::DeleteGeometry(Entry.second);
		}
	}

	std::vector<std::vector<int>> DestructionSet::FindConnectedComponents(const std::vector<int>& Indices) const
	{
		BitSet Allowed(mChunks.size());
		BitSet Visited(mChunks.size());
		for (int Index : Indices)
		{
			if (Index >= 0 && (size_t)Index < mChunks.size())
			{
				Allowed.insert((size_t)Index);
			}
		}

		std::vector<std::vector<int>> Components;
		for (int Index : Indices)
		{
			if (Index < 0 || (size_t)Index >= mChunks.size() || Visited.get((size_t)Index))
			{
				continue;
			}

			std::vector<int> Component;
			std::queue<int> Queue;
			Queue.push(Index);
			Visited.insert((size_t)Index);
			while (!Queue.empty())
			{
				const int Current = Queue.front();
				Queue.pop();
				Component.push_back(Current);
				if (!mGraph.IsValidNode(Current))
				{
					continue;
				}
				for (const Bond& ExistingBond : mGraph[(size_t)Current].Bonds)
				{
					const int Other = ExistingBond.GetOther(Current);
					if (Other >= 0 && (size_t)Other < mChunks.size() && Allowed.get((size_t)Other) && !Visited.get((size_t)Other))
					{
						Visited.insert((size_t)Other);
						Queue.push(Other);
					}
				}
			}
			Components.push_back(Component);
		}
		return Components;
	}

	bool DestructionSet::PartitionClusterOnce(const std::vector<int>& Indices, int Level, float Strength, std::vector<std::vector<int>>& Components)
	{
		Components.clear();
		if (Indices.size() <= 1 || mConstants.PartitionAlgorithm != DestructionPartitionAlgorithm::Metis)
		{
			return false;
		}

		Box3 Bounds = Box3::Empty();
		for (int Index : Indices)
		{
			if (Index >= 0 && (size_t)Index < mChunks.size())
			{
				Bounds.Encapsulate(mChunks[(size_t)Index].Bounds);
			}
		}
		if (Bounds.GetVolume() <= mConstants.MinClusterVolume)
		{
			return false;
		}

		const int ClampedLevel = std::max(0, std::min(Level, 7));
		const DestructionClusterLevelConstants& LevelConstants = mConstants.Levels[ClampedLevel];
		if (Strength < LevelConstants.BreakThreshold)
		{
			return false;
		}
		if (mGraph.BreakGraph(Indices, LevelConstants.PartitionSize) != DestructionBreakStatus::Success)
		{
			return false;
		}

		Components = FindConnectedComponents(Indices);
		if (Components.size() <= 1)
		{
			Components.clear();
			return false;
		}
		return true;
	}

	void DestructionSet::QueuePartitionChildren(const std::vector<std::vector<int>>& Components, int Level, float Strength, const Vector3& Momentum, const Vector3& Position, const Vector3& Normal)
	{
		for (const std::vector<int>& Component : Components)
		{
			if (Component.size() <= 1)
			{
				continue;
			}

			PendingPartition Pending;
			Pending.Indices = Component;
			Pending.Level = Level;
			Pending.Strength = Strength;
			Pending.Momentum = Momentum;
			Pending.Position = Position;
			Pending.Normal = Normal;
			mPartitionQueue.push_back(Pending);
		}
	}

	bool DestructionSet::MergePendingPartition(const DestructionCluster& Cluster, float Strength, const Vector3& Momentum, const Vector3& Position, const Vector3& Normal)
	{
		for (PendingPartition& Pending : mPartitionQueue)
		{
			if (Pending.Indices.empty() || !Cluster.Contains(Pending.Indices.front()))
			{
				continue;
			}

			const float ExistingStrength = Pending.Momentum.Length();
			const float AddedStrength = Momentum.Length();
			const float TotalStrength = ExistingStrength + AddedStrength;
			if (TotalStrength > 1e-6f)
			{
				Pending.Position = (Pending.Position * ExistingStrength + Position * AddedStrength) / TotalStrength;
				Pending.Normal = Pending.Normal * ExistingStrength + Normal * AddedStrength;
				Pending.Normal.SafeNormalize();
			}
			Pending.Momentum += Momentum;
			Pending.Strength = std::max(Strength, Pending.Momentum.Length());
			return true;
		}
		return false;
	}

	bool DestructionSet::MergePendingHierarchyPartition(int HierarchyNode, int ImpactChunkIndex, float Strength,
		const Vector3& Momentum, const Vector3& Position, const Vector3& Normal)
	{
		for (PendingPartition& Pending : mPartitionQueue)
		{
			if (Pending.HierarchyNode != HierarchyNode)
			{
				continue;
			}

			const float ExistingStrength = Pending.Momentum.Length();
			const float AddedStrength = Momentum.Length();
			const float TotalStrength = ExistingStrength + AddedStrength;
			if (TotalStrength > 1e-6f)
			{
				Pending.Position = (Pending.Position * ExistingStrength + Position * AddedStrength) / TotalStrength;
				Pending.Normal = Pending.Normal * ExistingStrength + Normal * AddedStrength;
				Pending.Normal.SafeNormalize();
			}
			if (Pending.ImpactChunkIndex < 0 || AddedStrength >= ExistingStrength)
			{
				Pending.ImpactChunkIndex = ImpactChunkIndex;
			}
			Pending.Momentum += Momentum;
			Pending.Strength = std::max(Strength, Pending.Momentum.Length());
			return true;
		}
		return false;
	}

	int DestructionSet::FindActiveHierarchyNode(int ChunkIndex) const
	{
		if (ChunkIndex < 0 || (size_t)ChunkIndex >= mChunkHierarchyLeaves.size())
		{
			return -1;
		}

		int NodeIndex = mChunkHierarchyLeaves[(size_t)ChunkIndex];
		if (NodeIndex < 0 || (size_t)NodeIndex >= mHierarchyNodes.size())
		{
			return -1;
		}
		while (true)
		{
			const int Parent = mHierarchyNodes[(size_t)NodeIndex].Parent;
			if (Parent < 0 || (size_t)Parent >= mHierarchyNodes.size() || mHierarchyNodes[(size_t)Parent].IsBroken)
			{
				break;
			}
			NodeIndex = Parent;
		}
		return NodeIndex;
	}

	bool DestructionSet::BreakHierarchyNode(int HierarchyNode, float Strength)
	{
		if (HierarchyNode < 0 || (size_t)HierarchyNode >= mHierarchyNodes.size()
			|| (size_t)HierarchyNode >= mHierarchyBoundaryBonds.size())
		{
			return false;
		}

		DestructionHierarchyNode& Node = mHierarchyNodes[(size_t)HierarchyNode];
		if (!Node.IsBreakable || Node.IsBroken || Node.Children.size() < 2 || Strength < Node.BreakThreshold)
		{
			return false;
		}

		BreakHierarchyCrossChildBonds(HierarchyNode);
		Node.IsBroken = true;
		return true;
	}

	void DestructionSet::QueueHierarchyChildren(int HierarchyNode, int ImpactChunkIndex, float Strength,
		const Vector3& Momentum, const Vector3& Position, const Vector3& Normal)
	{
		if (HierarchyNode < 0 || (size_t)HierarchyNode >= mHierarchyNodes.size())
		{
			return;
		}

		const DestructionHierarchyNode& Parent = mHierarchyNodes[(size_t)HierarchyNode];
		for (int ChildIndex : Parent.Children)
		{
			if (ChildIndex < 0 || (size_t)ChildIndex >= mHierarchyNodes.size())
			{
				continue;
			}
			const DestructionHierarchyNode& Child = mHierarchyNodes[(size_t)ChildIndex];
			if (!Child.IsBreakable || Child.IsBroken || Child.Children.size() < 2)
			{
				continue;
			}

			const bool bContainsImpact = std::find(Child.ChunkIndices.begin(), Child.ChunkIndices.end(), ImpactChunkIndex)
				!= Child.ChunkIndices.end();
			const float Propagation = mHierarchyDamageAttenuation
				* (bContainsImpact ? 1.0f : mHierarchyNeighborDamageFraction);
			const float ChildStrength = Strength * Propagation;
			if (Propagation <= 0.0f || ChildStrength < Child.BreakThreshold)
			{
				continue;
			}

			int ChildImpactChunk = bContainsImpact ? ImpactChunkIndex : -1;
			if (ChildImpactChunk < 0)
			{
				float BestDistance = FLT_MAX;
				for (int ChunkIndex : Child.ChunkIndices)
				{
					const float Distance = PointBoundsDistanceSquared(Position, GetChunkWorldBounds(ChunkIndex));
					if (Distance < BestDistance)
					{
						BestDistance = Distance;
						ChildImpactChunk = ChunkIndex;
					}
				}
			}

			PendingPartition Pending;
			Pending.Indices = Child.ChunkIndices;
			Pending.HierarchyNode = ChildIndex;
			Pending.ImpactChunkIndex = ChildImpactChunk;
			Pending.Level = Child.Level;
			Pending.Strength = ChildStrength;
			Pending.Momentum = Momentum * Propagation;
			Pending.Position = Position;
			Pending.Normal = Normal;
			mPartitionQueue.push_back(Pending);
		}
	}

	bool DestructionSet::ApplyHierarchyMomentum(int Index, const Vector3& Momentum, const Vector3& Position, const Vector3& Normal)
	{
		const int HierarchyNode = FindActiveHierarchyNode(Index);
		if (HierarchyNode < 0 || (size_t)HierarchyNode >= mHierarchyNodes.size())
		{
			return false;
		}

		const float Strength = Momentum.Length();
		if (MergePendingHierarchyPartition(HierarchyNode, Index, Strength, Momentum, Position, Normal))
		{
			return true;
		}
		const std::vector<int> SourceIndices = mHierarchyNodes[(size_t)HierarchyNode].ChunkIndices;
		if (!BreakHierarchyNode(HierarchyNode, Strength))
		{
			return false;
		}

		QueueHierarchyChildren(HierarchyNode, Index, Strength, Momentum, Position, Normal);
		RebuildClustersFromGraph();
		ApplyFractureSeparation(SourceIndices, Momentum, Position, Normal, 1.0f);
		return true;
	}

	void DestructionSet::ProcessPartitionQueue()
	{
		if (mPartitionQueue.empty())
		{
			mPartitionTicksUntilProcess = 0;
			return;
		}
		if (mPartitionTicksUntilProcess > 0)
		{
			--mPartitionTicksUntilProcess;
			return;
		}

		const int ProcessCount = std::min(std::max(mConstants.PartitionProcessPerTick, 1), (int)mPartitionQueue.size());
		std::vector<PendingPartition> Completed;
		for (int CommandIndex = 0; CommandIndex < ProcessCount; ++CommandIndex)
		{
			PendingPartition Pending = mPartitionQueue.front();
			mPartitionQueue.pop_front();
			if (Pending.HierarchyNode >= 0)
			{
				if (BreakHierarchyNode(Pending.HierarchyNode, Pending.Strength))
				{
					QueueHierarchyChildren(Pending.HierarchyNode, Pending.ImpactChunkIndex, Pending.Strength,
						Pending.Momentum, Pending.Position, Pending.Normal);
					Completed.push_back(Pending);
				}
				continue;
			}

			std::vector<std::vector<int>> Components;
			if (!PartitionClusterOnce(Pending.Indices, Pending.Level, Pending.Strength, Components))
			{
				continue;
			}

			QueuePartitionChildren(Components, Pending.Level + 1, Pending.Strength, Pending.Momentum, Pending.Position, Pending.Normal);
			Completed.push_back(Pending);
		}

		if (!Completed.empty())
		{
			RebuildClustersFromGraph();
			const float SeparationAttenuation = std::max(0.0f, std::min(mConstants.PartitionSeparationAttenuation, 1.0f));
			for (const PendingPartition& Pending : Completed)
			{
				const float SeparationScale = std::pow(SeparationAttenuation, (float)std::max(Pending.Level, 0));
				ApplyFractureSeparation(Pending.Indices, Pending.Momentum, Pending.Position, Pending.Normal, SeparationScale);
			}
		}
		mPartitionTicksUntilProcess = std::max(mConstants.PartitionProcessIntervalTicks - 1, 0);
	}

	void DestructionSet::ApplyFractureSeparation(const std::vector<int>& Indices, const Vector3& Momentum, const Vector3& Position, const Vector3& Normal, float SeparationScale)
	{
		if (mConstants.ImpulseBreakStrength <= 0.0f || SeparationScale <= 0.0f)
		{
			return;
		}

		BitSet Affected(mChunks.size());
		for (int Index : Indices)
		{
			if (Index >= 0 && (size_t)Index < mChunks.size())
			{
				Affected.insert((size_t)Index);
			}
		}

		struct SeparatedBody
		{
			RigidBodyDynamic* Body = nullptr;
			Vector3 Direction = Vector3::Zero();
			float Mass = 0.0f;
		};
		std::vector<SeparatedBody> Bodies;
		float TotalMass = 0.0f;
		Vector3 FractureCenter = Vector3::Zero();
		for (std::map<uint32_t, DestructionCluster>::value_type& Entry : mClusters)
		{
			DestructionCluster& Cluster = Entry.second;
			bool bAffected = false;
			for (int Index : Cluster.GetSourceIndices())
			{
				if (Index >= 0 && (size_t)Index < mChunks.size() && Affected.get((size_t)Index))
				{
					bAffected = true;
					break;
				}
			}
			RigidBodyDynamic* Body = bAffected ? Cluster.GetRigidBody() : nullptr;
			if (!Body || Cluster.IsStatic() || Body->GetInverseMass() <= kMinimumInvMass)
			{
				continue;
			}

			SeparatedBody Item;
			Item.Body = Body;
			Item.Mass = 1.0f / Body->GetInverseMass();
			TotalMass += Item.Mass;
			FractureCenter += Body->X * Item.Mass;
			Bodies.push_back(Item);
		}
		if (Bodies.size() <= 1 || TotalMass <= 0.0f)
		{
			return;
		}
		FractureCenter /= TotalMass;
		for (SeparatedBody& Item : Bodies)
		{
			Item.Direction = Item.Body->X - FractureCenter;
			if (Item.Direction.SafeNormalize() == 0.0f)
			{
				Item.Direction = Item.Body->X - Position;
				if (Item.Direction.SafeNormalize() == 0.0f)
				{
					Item.Direction = Momentum;
					if (Item.Direction.SafeNormalize() == 0.0f)
					{
						Item.Direction = Normal;
						Item.Direction.SafeNormalize();
					}
				}
			}
		}

		const float ImpactSpeed = Momentum.Length() / TotalMass;
		const float BaseSeparationSpeed = std::max(0.4f, std::min(std::max(ImpactSpeed, 0.4f) * mConstants.ImpulseBreakStrength, 2.0f));
		const float SeparationSpeed = BaseSeparationSpeed * SeparationScale;
		Vector3 MeanVelocity = Vector3::Zero();
		for (const SeparatedBody& Item : Bodies)
		{
			MeanVelocity += Item.Direction * (SeparationSpeed * Item.Mass);
		}
		MeanVelocity /= TotalMass;

		Vector3 ImpactDirection = Momentum;
		if (ImpactDirection.SafeNormalize() == 0.0f)
		{
			ImpactDirection = Normal;
			ImpactDirection.SafeNormalize();
		}
		for (SeparatedBody& Item : Bodies)
		{
			Item.Body->AddLinearVelocity(Item.Direction * SeparationSpeed - MeanVelocity);
			Item.Body->AddAngularVelocity(ImpactDirection.Cross(Item.Direction) * (SeparationSpeed * 0.35f));
			Item.Body->Wakeup();
		}
	}

	Box3 DestructionSet::GetChunkWorldBounds(int Index) const
	{
		if (Index < 0 || (size_t)Index >= mChunks.size())
		{
			return Box3::Empty();
		}
		const DestructionCluster* Cluster = FindClusterContaining(Index);
		if (Cluster)
		{
			Geometry* Geom = Cluster->GetGeometryByChunkIndex(Index);
			RigidBodyDynamic* Body = Cluster->GetRigidBody();
			if (Geom && Body)
			{
				return Body->GetGeometryBounds(Geom);
			}
		}
		return mChunks[(size_t)Index].Bounds;
	}

	DestructionCluster* DestructionSet::FindClusterContaining(int ChunkIndex)
	{
		for (std::map<uint32_t, DestructionCluster>::value_type& Entry : mClusters)
		{
			if (Entry.second.Contains(ChunkIndex))
			{
				return &Entry.second;
			}
		}
		return nullptr;
	}

	const DestructionCluster* DestructionSet::FindClusterContaining(int ChunkIndex) const
	{
		for (std::map<uint32_t, DestructionCluster>::const_reference Entry : mClusters)
		{
			if (Entry.second.Contains(ChunkIndex))
			{
				return &Entry.second;
			}
		}
		return nullptr;
	}

	DestructionCluster* DestructionSet::GetCluster(uint32_t ClusterID)
	{
		std::map<uint32_t, DestructionCluster>::iterator It = mClusters.find(ClusterID);
		return It == mClusters.end() ? nullptr : &It->second;
	}

	const DestructionCluster* DestructionSet::GetCluster(uint32_t ClusterID) const
	{
		std::map<uint32_t, DestructionCluster>::const_iterator It = mClusters.find(ClusterID);
		return It == mClusters.end() ? nullptr : &It->second;
	}

	std::vector<DestructionCluster*> DestructionSet::GetClusters()
	{
		std::vector<DestructionCluster*> Clusters;
		Clusters.reserve(mClusters.size());
		for (std::map<uint32_t, DestructionCluster>::value_type& Entry : mClusters)
		{
			Clusters.push_back(&Entry.second);
		}
		return Clusters;
	}

	uint32_t DestructionSet::GetClusterRevision() const
	{
		uint32_t Hash = 2166136261u;
		Hash = HashClusterRevisionValue(Hash, (uint32_t)mChunks.size());
		Hash = HashClusterRevisionValue(Hash, (uint32_t)mClusters.size());

		for (size_t Index = 0; Index < mChunks.size(); ++Index)
		{
			const DestructionChunk& Chunk = mChunks[Index];
			uint32_t Flags = 0;
			Flags |= Chunk.IsStatic ? 1u : 0u;
			Flags |= Chunk.IsFree ? 2u : 0u;
			Flags |= Chunk.IsSmashed ? 4u : 0u;
			Hash = HashClusterRevisionValue(Hash, (uint32_t)Index);
			Hash = HashClusterRevisionValue(Hash, Flags);
		}

		for (std::map<uint32_t, DestructionCluster>::const_reference Entry : mClusters)
		{
			const DestructionCluster& Cluster = Entry.second;
			Hash = HashClusterRevisionValue(Hash, Entry.first);
			Hash = HashClusterRevisionValue(Hash, Cluster.GetID());
			Hash = HashClusterRevisionValue(Hash, (uint32_t)Cluster.GetLevel());
			Hash = HashClusterRevisionValue(Hash, Cluster.IsStatic() ? 1u : 0u);
			Hash = HashClusterRevisionValue(Hash, (uint32_t)Cluster.GetSourceIndexMin());
			Hash = HashClusterRevisionPointer(Hash, Cluster.GetRigidBody());

			const std::vector<int>& SourceIndices = Cluster.GetSourceIndices();
			const std::vector<Geometry*>& Geometries = Cluster.GetGeometries();
			Hash = HashClusterRevisionValue(Hash, (uint32_t)SourceIndices.size());
			Hash = HashClusterRevisionValue(Hash, (uint32_t)Geometries.size());
			for (size_t Index = 0; Index < SourceIndices.size(); ++Index)
			{
				Hash = HashClusterRevisionValue(Hash, (uint32_t)SourceIndices[Index]);
				Hash = HashClusterRevisionPointer(Hash, Index < Geometries.size() ? Geometries[Index] : nullptr);
			}
		}
		return Hash;
	}

	void DestructionSet::BreakBondEx(int Index)
	{
		if (!mGraph.IsValidNode(Index) || mGraph.IsBroken(Index))
		{
			return;
		}

		mGraph.BreakBonds(Index);
		MarkFree(Index);
		RebuildClustersFromGraph();
	}

	bool DestructionSet::ApplyMomentum(int Index, const Vector3& Momentum, const Vector3& Position, const Vector3& Normal)
	{
		if (Index < 0 || (size_t)Index >= mChunks.size())
		{
			return false;
		}
		if (!mHierarchyNodes.empty())
		{
			return ApplyHierarchyMomentum(Index, Momentum, Position, Normal);
		}

		const float Strength = Momentum.Length();
		bool bChanged = false;
		DestructionCluster* Cluster = FindClusterContaining(Index);
		if (Cluster && Cluster->Size() <= 1)
		{
			return false;
		}
		if (Cluster && mConstants.PartitionAlgorithm == DestructionPartitionAlgorithm::Metis)
		{
			if (MergePendingPartition(*Cluster, Strength, Momentum, Position, Normal))
			{
				return true;
			}

			const std::vector<int> SourceIndices = Cluster->GetSourceIndices();
			std::vector<std::vector<int>> Components;
			if (PartitionClusterOnce(SourceIndices, Cluster->GetLevel(), Strength, Components))
			{
				QueuePartitionChildren(Components, Cluster->GetLevel() + 1, Strength, Momentum, Position, Normal);
				RebuildClustersFromGraph();
				ApplyFractureSeparation(SourceIndices, Momentum, Position, Normal, 1.0f);
				return true;
			}
		}

		std::vector<int> Collapse;
		const std::vector<int>* SourceIndices = Cluster ? &Cluster->GetSourceIndices() : nullptr;
		if (SourceIndices)
		{
			for (int ChunkIndex : *SourceIndices)
			{
				if (ChunkIndex < 0 || (size_t)ChunkIndex >= mChunks.size())
				{
					continue;
				}
				DestructionChunk& Chunk = mChunks[(size_t)ChunkIndex];
				if (Chunk.CollapseStrength > mConstants.ImpulseCollapseStrength || Chunk.IsStatic || Chunk.IsSmashed)
				{
					continue;
				}
				if (IntersectSphereAABB(Position, mConstants.ImpulseRadius, GetChunkWorldBounds(ChunkIndex)))
				{
					Collapse.push_back(ChunkIndex);
				}
			}
		}
		else
		{
			DestructionChunk& Chunk = mChunks[(size_t)Index];
			if (Chunk.CollapseStrength <= mConstants.ImpulseCollapseStrength && !Chunk.IsStatic && !Chunk.IsSmashed)
			{
				Collapse.push_back(Index);
			}
		}

		if (!Collapse.empty())
		{
			AddDestructionInfo(DestructionEventType::SelfContact, Collapse);
			bChanged = true;
		}

		std::vector<std::pair<int, int>> ShockBreaks;
		mGraph.ShockPropagation(Index, Strength * mConstants.DamagePropagationStrength, mConstants.DamagePropagationAttenuation, ShockBreaks);
		for (const std::pair<int, int>& Pair : ShockBreaks)
		{
			mGraph.BreakBond(Pair.first, Pair.second);
			bChanged = true;
		}
		if (bChanged)
		{
			RebuildClustersFromGraph();
		}
		return bChanged;
	}

	void DestructionSet::AddMomentumQueue(int Index, const Vector3& Momentum, const Vector3& Position, const Vector3& Normal)
	{
		if (Momentum.Length() < 1e-3f)
		{
			return;
		}
		for (DestructionMomentum& ExistingMomentum : mMomentumQueue)
		{
			if (ExistingMomentum.Index == Index)
			{
				const float ExistingStrength = ExistingMomentum.Momentum.Length();
				const float AddedStrength = Momentum.Length();
				const float TotalStrength = ExistingStrength + AddedStrength;
				if (TotalStrength > 1e-6f)
				{
					ExistingMomentum.Position = (ExistingMomentum.Position * ExistingStrength + Position * AddedStrength) / TotalStrength;
					ExistingMomentum.Normal = ExistingMomentum.Normal * ExistingStrength + Normal * AddedStrength;
					ExistingMomentum.Normal.SafeNormalize();
				}
				ExistingMomentum.Momentum += Momentum;
				return;
			}
		}

		DestructionMomentum Info;
		Info.Index = Index;
		Info.Momentum = Momentum;
		Info.Position = Position;
		Info.Normal = Normal;
		mMomentumQueue.push_back(Info);
	}

	void DestructionSet::ProcessMomentumQueue()
	{
		int Count = std::max(mConstants.ImpulseProcessPerTick, 1);
		while (Count-- > 0 && !mMomentumQueue.empty())
		{
			const DestructionMomentum Info = mMomentumQueue.front();
			mMomentumQueue.erase(mMomentumQueue.begin());
			ApplyMomentum(Info.Index, Info.Momentum, Info.Position, Info.Normal);
		}
	}

	void DestructionSet::AddDestructionInfo(DestructionEventType Type, const std::vector<int>& Indices)
	{
		const int TypeIndex = (int)Type;
		if (TypeIndex < 0 || TypeIndex >= (int)DestructionEventType::Count)
		{
			return;
		}
		if (mDestructionInfo[TypeIndex].size() != mChunks.size())
		{
			mDestructionInfo[TypeIndex].resize(mChunks.size());
		}
		for (int Index : Indices)
		{
			if (Index >= 0 && (size_t)Index < mChunks.size() && !mChunks[(size_t)Index].IsStatic)
			{
				mDestructionInfo[TypeIndex].insert((size_t)Index);
				bHasDestructionInfo = true;
			}
		}
	}

	uint32_t DestructionSet::ProcessDestructionInfo()
	{
		if (!bHasDestructionInfo)
		{
			return 0;
		}
		bHasDestructionInfo = false;

		uint32_t Flags = 0;
		for (int TypeIndex = 0; TypeIndex < (int)DestructionEventType::Count; ++TypeIndex)
		{
			std::vector<uint32_t> Indices = mDestructionInfo[TypeIndex].to_vector();
			mDestructionInfo[TypeIndex].clear();
			if (Indices.empty())
			{
				continue;
			}

			Flags |= (1u << TypeIndex);
			for (uint32_t Index : Indices)
			{
				if (Index < mChunks.size())
				{
					if (TypeIndex == (int)DestructionEventType::SelfContact ||
						TypeIndex == (int)DestructionEventType::SelfCollapse ||
						TypeIndex == (int)DestructionEventType::Destruct)
					{
						mChunks[Index].IsSmashed = true;
					}
					MarkFree((int)Index);
				}
			}
		}
		RebuildClustersFromGraph();
		return Flags;
	}

	void DestructionSet::DoFractures(const DestructionFractureBuffer& Commands)
	{
		bool bChanged = false;
		for (int BondIndex : Commands.BondFractures)
		{
			bChanged = BreakBondByIndex(BondIndex) || bChanged;
		}
		if (bChanged)
		{
			RebuildClustersFromGraph();
		}
	}

	void DestructionSet::OnContact(const PhysicsContactInfo& ContactInfo)
	{
		for (int PointIndex = 0; PointIndex < ContactInfo.NumContactPoints; ++PointIndex)
		{
			const PhysicsContactPoint& Point = ContactInfo.ContactPoints[PointIndex];
			const Vector3 Momentum = BuildContactMomentum(ContactInfo, Point);
			if (Momentum.SquareLength() <= 1e-6f)
			{
				continue;
			}

			QueueContactMomentum(ContactInfo.BodyA, ContactInfo.GeomA, Point, -Momentum, -Point.Normal);
			QueueContactMomentum(ContactInfo.BodyB, ContactInfo.GeomB, Point, Momentum, Point.Normal);
		}
	}

	BitSet DestructionSet::FindUnsupported() const
	{
		BitSet Supported(mChunks.size());
		std::vector<int> Seeds;
		for (size_t Index = 0; Index < mChunks.size(); ++Index)
		{
			if (mChunks[Index].IsStatic)
			{
				Seeds.push_back((int)Index);
			}
		}

		if (!Seeds.empty())
		{
			Supported = mGraph.FindSupported(Seeds, nullptr);
		}

		BitSet Unsupported(mChunks.size());
		for (size_t Index = 0; Index < mChunks.size(); ++Index)
		{
			if (!mChunks[Index].IsStatic && !mChunks[Index].IsFree && !Supported.get(Index))
			{
				Unsupported.insert(Index);
			}
		}
		return Unsupported;
	}

	std::vector<int> DestructionSet::PruneUnsupported()
	{
		BitSet Unsupported = FindUnsupported();
		std::vector<uint32_t> UnsupportedIndices = Unsupported.to_vector();
		std::vector<int> Result;
		Result.reserve(UnsupportedIndices.size());
		for (uint32_t Index : UnsupportedIndices)
		{
			MarkFree((int)Index);
			Result.push_back((int)Index);
		}
		if (!Result.empty())
		{
			RebuildClustersFromGraph();
		}
		return Result;
	}

	void DestructionSet::Update(float DeltaTime)
	{
		ProcessPartitionQueue();
		ProcessMomentumQueue();
		ProcessDestructionInfo();
		if (mStressSolver && std::isfinite(DeltaTime) && DeltaTime > 0.0f)
		{
			EnsureStressBondIndices();
			mStressSolver->SyncGraph(BuildStressGraph());
			mStressSolver->SetAllNodesInfo(*this);
			for (const DestructionMomentum& load : mStressImpulses)
			{
				if (load.Index >= 0 && (size_t)load.Index < mChunks.size()
					&& !mChunks[load.Index].IsFree && !mChunks[load.Index].IsSmashed)
					mStressSolver->AddForceAtPosition(load.Index, load.Position, load.Momentum);
			}
			mStressImpulses.clear();
			for (size_t i = 0; i < mChunks.size(); ++i)
			{
				if (mChunks[i].IsFree || mChunks[i].IsSmashed) continue;
				const DestructionCluster* cluster = FindClusterContaining((int)i);
				Transform chunkTransform;
				const Vector3 gravity = cluster && cluster->GetWorldTransform((int)i, chunkTransform)
					? chunkTransform.WorldToLocalDirection(mStressGravity) : mStressGravity;
				mStressSolver->AddForce((int)i, gravity, StrainForceMode::Acceleration);
			}
			mStressSolver->Update(DeltaTime);
			DestructionFractureBuffer commands;
			if (mStressSolver->HasConverged()) mStressSolver->GenerateFractureCommands(commands);
			if (!commands.BondFractures.empty())
			{
				DoFractures(commands);
				mStressSolver->SyncGraph(BuildStressGraph());
			}
		}
	}

	ConnectionGraph DestructionSet::BuildStressGraph() const
	{
		ConnectionGraph graph = mGraph;
		for (size_t i = 0; i < mChunks.size(); ++i)
		{
			if (mChunks[i].IsFree || mChunks[i].IsSmashed) graph.BreakBonds((int)i);
		}
		return graph;
	}

	void DestructionSet::EnableStressSolver(const StrainSolverSettings& Settings, const Vector3& WorldGravity)
	{
		EnsureStressBondIndices();
		mStressSolver.reset(new StrainSolver(BuildStressGraph(), Settings));
		mStressSolver->SetAllNodesInfo(*this);
		mStressGravity = IsFiniteVector(WorldGravity) ? WorldGravity : Vector3::Zero();
		mStressImpulses.clear();
	}

	void DestructionSet::DisableStressSolver()
	{
		mStressSolver.reset();
		mStressImpulses.clear();
	}

	bool DestructionSet::AddStressImpulse(int Index, const Vector3& WorldPosition, const Vector3& WorldImpulse)
	{
		if (!mStressSolver || Index < 0 || (size_t)Index >= mChunks.size()
			|| !IsFiniteVector(WorldPosition) || !IsFiniteVector(WorldImpulse)
			|| mChunks[Index].IsFree || mChunks[Index].IsSmashed) return false;
		DestructionMomentum load;
		load.Index = Index;
		const DestructionCluster* cluster = FindClusterContaining(Index);
		Transform transform;
		if (cluster && cluster->GetWorldTransform(Index, transform))
		{
			load.Position = transform.WorldToLocal(WorldPosition) + mChunks[Index].Centroid;
			load.Momentum = transform.WorldToLocalDirection(WorldImpulse);
		}
		else
		{
			load.Position = WorldPosition;
			load.Momentum = WorldImpulse;
		}
		mStressImpulses.push_back(load);
		return true;
	}

	void DestructionSet::EnsureStressBondIndices()
	{
		std::set<int> used;
		for (size_t i = 0; i < mGraph.Size(); ++i)
			for (const Bond& bond : mGraph[i].Bonds)
				if (bond.bond_index >= 0) used.insert(bond.bond_index);
		int next = 0;
		for (size_t i = 0; i < mGraph.Size(); ++i)
		{
			for (Bond& bond : mGraph[i].Bonds)
			{
				const int other = bond.GetOther((int)i);
				if (bond.bond_index >= 0 || other < 0 || (size_t)other >= mGraph.Size() || (int)i >= other) continue;
				while (used.count(next)) ++next;
				bond.bond_index = next;
				used.insert(next);
				for (Bond& reverse : mGraph[other].Bonds)
					if (reverse.GetOther(other) == (int)i) reverse.bond_index = next;
			}
		}
	}

	bool DestructionSet::HasClusterRigidBodies() const
	{
		for (std::map<uint32_t, DestructionCluster>::const_reference Entry : mClusters)
		{
			if (Entry.second.GetRigidBody())
			{
				return true;
			}
		}
		return false;
	}

	DestructionCluster* DestructionSet::FindClusterByRigidBody(const RigidBody* Body)
	{
		if (Body == nullptr || Body->Parent != this)
		{
			return nullptr;
		}

		for (std::map<uint32_t, DestructionCluster>::value_type& Entry : mClusters)
		{
			if (Entry.second.GetRigidBody() == Body)
			{
				return &Entry.second;
			}
		}
		return nullptr;
	}

	void DestructionSet::QueueContactMomentum(RigidBody* Body, Geometry* Geom, const PhysicsContactPoint& Point, const Vector3& Momentum, const Vector3& Normal)
	{
		if (Geom == nullptr)
		{
			return;
		}

		DestructionCluster* Cluster = FindClusterByRigidBody(Body);
		if (Cluster == nullptr)
		{
			return;
		}

		int ChunkIndex = Cluster->GetChunkIndexByGeometry(Geom);
		if (ChunkIndex < 0)
		{
			float BestDistance = FLT_MAX;
			for (int CandidateIndex : Cluster->GetSourceIndices())
			{
				const float Distance = PointBoundsDistanceSquared(Point.Position, GetChunkWorldBounds(CandidateIndex));
				if (Distance < BestDistance)
				{
					BestDistance = Distance;
					ChunkIndex = CandidateIndex;
				}
			}
		}
		if (ChunkIndex < 0)
		{
			return;
		}

		if (mStressSolver) AddStressImpulse(ChunkIndex, Point.Position, Momentum);
		else AddMomentumQueue(ChunkIndex, Momentum, Point.Position, Normal);
	}

	std::vector<int> DestructionSet::QueryChunksRadius(const Vector3& Center, float Radius) const
	{
		std::vector<int> Result;
		for (size_t Index = 0; Index < mChunks.size(); ++Index)
		{
			if (IntersectSphereAABB(Center, Radius, GetChunkWorldBounds((int)Index)))
			{
				Result.push_back((int)Index);
			}
		}
		return Result;
	}

	void DestructionSet::RecalculateBounds()
	{
		mBounds = Box3::Empty();
		for (const DestructionChunk& Chunk : mChunks)
		{
			mBounds.Encapsulate(Chunk.Bounds);
		}
	}

	DestructionSet::Stats DestructionSet::GetStats() const
	{
		Stats Result;
		for (const DestructionChunk& Chunk : mChunks)
		{
			if (Chunk.IsStatic)
			{
				++Result.StaticCount;
			}
			else if (Chunk.IsSmashed)
			{
				++Result.SmashedCount;
			}
			else if (Chunk.IsFree)
			{
				++Result.FreeCount;
			}
			else
			{
				++Result.NormalCount;
			}
		}
		return Result;
	}

	std::vector<float> DestructionSet::GetMasses() const
	{
		std::vector<float> Masses;
		Masses.reserve(mChunks.size());
		for (const DestructionChunk& Chunk : mChunks)
		{
			Masses.push_back(Chunk.Mass);
		}
		return Masses;
	}

	float DestructionSet::GetVolume(int Index) const
	{
		if (Index < 0 || (size_t)Index >= mChunks.size())
		{
			return 0.0f;
		}
		return mChunks[(size_t)Index].Volume;
	}

	void DestructionSet::MarkFree(int Index)
	{
		if (Index < 0 || (size_t)Index >= mChunks.size())
		{
			return;
		}
		if (!mHierarchyNodes.empty())
		{
			// Removing a hierarchy leaf must open every authored boundary that depended on it.
			mGraph.BreakBonds(Index);
		}
		mChunks[(size_t)Index].IsFree = true;
		if (std::find(mBrokenIndices.begin(), mBrokenIndices.end(), Index) == mBrokenIndices.end())
		{
			mBrokenIndices.push_back(Index);
		}
	}

	bool DestructionSet::BreakBondByIndex(int BondIndex)
	{
		if (BondIndex < 0)
		{
			return false;
		}
		for (size_t Node = 0; Node < mGraph.Size(); ++Node)
		{
			for (const Bond& ExistingBond : mGraph[Node].Bonds)
			{
				if (ExistingBond.bond_index == BondIndex)
				{
					mGraph.BreakBond((int)Node, ExistingBond.GetOther((int)Node));
					return true;
				}
			}
		}
		return false;
	}

}	// namespace Riemann
