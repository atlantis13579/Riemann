
#include "DestructionSet.h"

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <climits>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <random>
#include <set>
#include <stdexcept>
#include <tuple>

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
				// Recursive pieces are convex. Plane clipping avoids coplanar mesh-boolean
				// degeneracies when a generated cap is cut again at the next level.
				CutOptions.ClipConvexCells = true;
				CutOptions.AssumeConvexSource = true;

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

	static constexpr float MinimumContactMomentumSquared = 1e-6f;
	static Vector3 BuildContactMomentum(const PhysicsContactInfo& ContactInfo, const PhysicsContactPoint& Point, bool& Estimated)
	{
		Vector3 Momentum = Point.Impulse;
		Estimated = !(Momentum.SquareLength() > MinimumContactMomentumSquared);
		if (!Estimated)
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

	bool DestructionConstants::IsValid() const
	{
		for (float V : {ContactOffset,ImpulseCollapseStrength,ImpulseBreakStrength,ImpulseRadius,PartitionSeparationAttenuation,
			IdleTime,DamagePropagationStrength,DamagePropagationAttenuation,MinClusterVolume})
			if (!std::isfinite(V) || V < 0) return false;
		if (!std::isfinite(RestOffset) || RestOffset > ContactOffset || ImpulseProcessPerTick < 1 || PartitionProcessPerTick < 1
			|| PartitionProcessIntervalTicks < 1 || (PartitionAlgorithm != DestructionPartitionAlgorithm::None && PartitionAlgorithm != DestructionPartitionAlgorithm::Metis)) return false;
		for (const auto& L : Levels) if (!std::isfinite(L.BreakThreshold) || L.BreakThreshold < 0 || L.PartitionSize < 1) return false;
		return true;
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
		if (mAsset.IsValid()) throw std::logic_error("Re-author the destruction asset to change its chunk topology");
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
		if (mAsset.IsValid()) throw std::logic_error("Re-author the destruction asset to change its chunk topology");
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
		mAsset = DestructionAsset(); mAssetPlacement = Transform::Identity();
		mEventUpdateIndex = 0; mEventDeltaTime = 0; ResetEventState();
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
		CollectEvents();
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
		RefreshEventKinematics();
		ReleaseClusterRigidBody(It->second, true);
		mClusters.erase(It);
		CollectEvents();
		return true;
	}

	void DestructionSet::ClearClusters()
	{
		RefreshEventKinematics();
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
		RefreshEventKinematics();
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
		CollectEvents();
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
		EventCauseScope Cause(*this,DestructionEventCause::Partition);
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
		EventCauseScope Cause(*this,DestructionEventCause::Command);
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
		EventCauseScope Cause(*this,DestructionEventCause::Momentum);
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
		EventCauseScope Cause(*this,DestructionEventCause::LegacyCommand);
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

	uint32_t DestructionSet::ApplyBondStrain(const std::vector<DestructionBondStrain>& Strains)
	{
		// External strain is a sampled demand, rather than accumulated damage.
		// Directed records describe one physical bond; use the larger capacity.
		EventCauseScope Cause(*this,DestructionEventCause::Field);
		const size_t PreviousBreaks = mGraph.GetBrokenBonds().size();
		std::set<std::pair<int,int>> Broken;
		for (const auto& S : Strains)
		{
			if (!mGraph.IsValidNode(S.NodeA) || !mGraph.IsValidNode(S.NodeB) || S.NodeA == S.NodeB || !std::isfinite(S.Value) || S.Value < 0)
				throw std::invalid_argument("Invalid external bond strain");
			const auto* A = mGraph.FindBond(S.NodeA,S.NodeB); const auto* B = mGraph.FindBond(S.NodeB,S.NodeA);
			if (!A && !B) continue;
			if ((A && (!std::isfinite(A->strength) || A->strength < 0)) || (B && (!std::isfinite(B->strength) || B->strength < 0)))
				throw std::invalid_argument("Invalid bond strain capacity");
			const float Capacity = std::max(A ? A->strength : 0,B ? B->strength : 0);
			if (!std::isfinite(Capacity) || Capacity < 0) throw std::invalid_argument("Invalid bond strain capacity");
			if (S.Value > 0 && S.Value >= Capacity) Broken.insert(std::minmax(S.NodeA,S.NodeB));
		}
		for (const auto& Pair : Broken) mGraph.BreakBond(Pair.first,Pair.second);
		if (!Broken.empty()) RebuildClustersFromGraph();
		return (uint32_t)(mGraph.GetBrokenBonds().size()-PreviousBreaks);
	}

	void DestructionSet::DoFractures(const DestructionFractureBuffer& Commands)
	{
		EventCauseScope Cause(*this,DestructionEventCause::Command);
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
			bool Estimated;
			const Vector3 Momentum = BuildContactMomentum(ContactInfo, Point, Estimated);
			if (Momentum.SquareLength() <= MinimumContactMomentumSquared && !mEvents)
			{
				continue;
			}

			QueueContactMomentum(ContactInfo.BodyA, ContactInfo.GeomA, Point, -Momentum, -Point.Normal, Estimated);
			QueueContactMomentum(ContactInfo.BodyB, ContactInfo.GeomB, Point, Momentum, Point.Normal, Estimated);
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
		EventCauseScope Cause(*this,DestructionEventCause::Unsupported);
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
		++mEventUpdateIndex;
		mEventDeltaTime = std::isfinite(DeltaTime) && DeltaTime > 0 ? DeltaTime : 0;
		RefreshEventKinematics();
		ProcessPartitionQueue();
		ProcessMomentumQueue();
		ProcessDestructionInfo();
		if (mStressSolver && std::isfinite(DeltaTime) && DeltaTime > 0.0f)
		{
			EventCauseScope Cause(*this,DestructionEventCause::Stress);
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
		CollectEvents();
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

	void DestructionSet::QueueContactMomentum(RigidBody* Body, Geometry* Geom, const PhysicsContactPoint& Point, const Vector3& Momentum, const Vector3& Normal, bool ImpulseEstimated)
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

		if (mEvents) mEvents->Contact(*this,*Cluster,ChunkIndex,Point,Momentum,Normal,ImpulseEstimated);
		if (Momentum.SquareLength() <= MinimumContactMomentumSquared) return;
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

namespace Riemann
{
	/* Destruction authoring, instances and future cache identity
	   -------------------------------------------------------
	   Capture freezes rest geometry, directed bonds, support flags, fracture
	   hierarchy, authoring constants and stress configuration. It rejects pending
	   loads or damage: this is an authored asset, not a checkpoint of a live actor.
	   Runtime clusters, fracture queues and solver history belong to each instance.
	   The asset exposes metadata without mutable ConvexMesh pointers. Geometry is
	   deeply copied because the legacy chunk API exposes mutable collision data.

	   Placement bakes a rigid rotation into local hull vertices and face normals,
	   and transforms centroids, bounds and bond frames. Thus rotated boxes remain
	   exact convex boxes instead of becoming their larger world AABBs. Source
	   inertia tensors and custom stress geometry transform with the asset; stress
	   gravity remains a world acceleration. Cluster construction then uses the
	   ordinary live path, retaining its filtering and mass/inertia policy.

	   A field-wise FNV-1a fingerprint uses little-endian scalar encodings, never
	   pointers, padding or unordered traversal. Canonical node/bond ordering makes
	   copies compatible with later simulation caches. Chunk array indices are
	   stable authored identities; cluster identifiers remain per-instance state.
	   Cache frames and external events can therefore refer to chunks without
	   borrowing live rigid-body or geometry pointers. Disk/DCC asset import is a
	   separate producer; existing procedural scenes exercise this in-memory path. */
	namespace
	{
		bool Finite(const Vector3& P) { return std::isfinite(P.x) && std::isfinite(P.y) && std::isfinite(P.z); }
		bool Positive(float V) { return std::isfinite(V) && V > 0; }
		bool NonNegative(float V) { return std::isfinite(V) && V >= 0; }
		bool BoundsValid(const Box3& B, float Minimum)
		{
			const auto S = B.GetSize(); return Finite(B.Min) && Finite(B.Max) && Finite(S) && S.x > Minimum && S.y > Minimum && S.z > Minimum;
		}
		struct AssetHash
		{
			static constexpr uint64_t Offset = 14695981039346656037ull, Prime = 1099511628211ull;
			uint64_t Value = Offset;
			void Integer(uint64_t V) { for (int I = 0; I < 8; ++I) { Value ^= (V >> (I * 8)) & 255; Value *= Prime; } }
			void Scalar(float V) { if (V == 0) V = 0; uint32_t Bits; std::memcpy(&Bits,&V,sizeof(Bits)); Integer(Bits); }
			void Vector(const Vector3& V) { Scalar(V.x); Scalar(V.y); Scalar(V.z); }
			void Bounds(const Box3& B) { Vector(B.Min); Vector(B.Max); }
			void String(const std::string& S) { Integer(S.size()); for (unsigned char C : S) { Value ^= C; Value *= Prime; } }
		};
		void ValidateConstants(const DestructionConstants& C)
		{
			if (!C.IsValid()) throw std::invalid_argument("Invalid destruction asset constants");
		}
		std::shared_ptr<ConvexMesh> CopyHull(const ConvexMesh& Source, const Quaternion& Rotation)
		{
			auto Hull = std::make_shared<ConvexMesh>();
			Hull->SetConvexData(Source.Vertices,Source.NumVertices,Source.Faces,Source.NumFaces,
				reinterpret_cast<uint16_t*>(Source.Edges),Source.NumEdges,Source.Indices,Source.NumIndices,false);
			for (uint16_t I = 0; I < Hull->NumVertices; ++I) Hull->Vertices[I] = Rotation * Hull->Vertices[I];
			for (uint16_t I = 0; I < Hull->NumFaces; ++I) Hull->Faces[I].plane.Normal = Rotation * Hull->Faces[I].plane.Normal;
			Hull->Bounds = Hull->ComputeBoundingVolume(); return Hull;
		}
		size_t OrientHull(ConvexMesh& Hull)
		{
			// ConvexHull3/ConvexMesh use polygon winding opposite to the outward
			// plane normal. Normalize on owned data before hashing or instancing.
			using DVector = Maths::TVector3<double>;
			size_t Changed = 0;
			for (uint16_t I = 0; I < Hull.NumFaces; ++I)
			{
				const auto& F = Hull.Faces[I]; const auto& P = Hull.Vertices[Hull.Indices[F.first]];
				const DVector Origin(P.x,P.y,P.z); DVector Area(0);
				for (int J = 1; J + 1 < F.numVerties; ++J)
				{
					const auto& A = Hull.Vertices[Hull.Indices[F.first+J]]; const auto& B = Hull.Vertices[Hull.Indices[F.first+J+1]];
					Area += (DVector(A.x,A.y,A.z)-Origin).Cross(DVector(B.x,B.y,B.z)-Origin);
				}
				const auto& N = F.plane.Normal; const double Orientation = Area.Dot(DVector(N.x,N.y,N.z));
				if (!std::isfinite(Orientation) || Orientation == 0) throw std::invalid_argument("Degenerate destruction hull polygon");
				if (Orientation > 0) { std::reverse(Hull.Indices+F.first,Hull.Indices+F.first+F.numVerties); ++Changed; }
			}
			return Changed;
		}
		void ValidateHull(const ConvexMesh& H, const DestructionAssetSettings& Settings, bool Oriented = true)
		{
			if (!H.Vertices || !H.Faces || !H.Indices || H.NumVertices < 4 || H.NumVertices > (unsigned)std::numeric_limits<uint8_t>::max() + 1
				|| H.NumFaces < 4 || !BoundsValid(H.ComputeBoundingVolume(),Settings.MinimumLength)) throw std::invalid_argument("Invalid destruction asset hull");
			const float Tolerance = std::max(Settings.MinimumLength,H.ComputeBoundingVolume().MaxDim() * Settings.RelativeGeometryTolerance);
			std::map<std::pair<int,int>,std::pair<int,int>> Edges; std::set<int> Used;
			for (uint16_t I = 0; I < H.NumVertices; ++I) if (!Finite(H.Vertices[I])) throw std::invalid_argument("Non-finite destruction hull vertex");
			for (uint16_t I = 0; I < H.NumFaces; ++I)
			{
				const auto& F = H.Faces[I]; const auto& P = F.plane;
				if (F.numVerties < 3 || (unsigned)F.first + F.numVerties > H.NumIndices || !Finite(P.Normal) || !std::isfinite(P.D)
					|| std::abs(P.Normal.SquareLength() - 1) > Settings.QuaternionTolerance) throw std::invalid_argument("Invalid destruction hull face");
				for (uint16_t J = 0; J < H.NumVertices; ++J) if (P.Normal.Dot(H.Vertices[J]) + P.D > Tolerance) throw std::invalid_argument("Non-convex destruction hull");
				std::set<int> FaceVertices;
				for (int J = 0; J < F.numVerties; ++J)
				{
					const int A = H.Indices[F.first + J], B = H.Indices[F.first + (J + 1) % F.numVerties];
					if (A >= H.NumVertices || B >= H.NumVertices || !FaceVertices.insert(A).second || std::abs(P.Normal.Dot(H.Vertices[A]) + P.D) > Tolerance)
						throw std::invalid_argument("Invalid destruction hull polygon");
					auto& E = Edges[{std::min(A,B),std::max(A,B)}]; ++E.first; E.second += A < B ? 1 : -1; Used.insert(A);
				}
			}
			for (const auto& E : Edges) if (E.second.first != 2 || (Oriented && E.second.second != 0))
				throw std::invalid_argument("Invalid destruction hull edge " + std::to_string(E.first.first) + "/" + std::to_string(E.first.second)
					+ " incidence=" + std::to_string(E.second.first) + " orientation=" + std::to_string(E.second.second));
			if (Used.size() != H.NumVertices) throw std::invalid_argument("Unused destruction hull vertex");
		}
		void HashConstants(AssetHash& H, const DestructionConstants& C)
		{
			for (uint32_t V : {C.FilterData.v0,C.FilterData.v1,C.FilterData.v2,C.FilterData.v3}) H.Integer(V);
			for (float V : {C.ContactOffset,C.RestOffset,C.ImpulseCollapseStrength,C.ImpulseBreakStrength,C.ImpulseRadius,C.PartitionSeparationAttenuation,
				C.IdleTime,C.DamagePropagationStrength,C.DamagePropagationAttenuation,C.MinClusterVolume}) H.Scalar(V);
			H.Integer(C.ImpulseProcessPerTick); H.Integer(C.PartitionProcessPerTick); H.Integer(C.PartitionProcessIntervalTicks);
			H.Integer(C.EnableClusterCollisionProxy); H.Integer((int)C.PartitionAlgorithm); H.Integer(C.EnableSupportGroup); H.Integer(C.EnableHingeJoint); H.Integer(C.DebugLog);
			for (const auto& L : C.Levels) { H.Scalar(L.BreakThreshold); H.Integer(L.PartitionSize); }
		}
	}
	bool DestructionAssetSettings::IsValid() const
	{
		return Positive(RelativeGeometryTolerance) && RelativeGeometryTolerance < 1 && Positive(MinimumLength) && Positive(QuaternionTolerance) && QuaternionTolerance < 1;
	}
	struct DestructionAsset::Storage
	{
		DestructionAssetSettings Settings;
		DestructionAssetStats Stats;
		std::vector<DestructionAssetChunk> Metadata;
		std::vector<DestructionChunk> Chunks;
		ConnectionGraph Graph;
		DestructionConstants Constants;
		std::vector<DestructionHierarchyNode> Hierarchy;
		std::vector<std::vector<std::pair<int,int>>> BoundaryBonds;
		std::vector<int> Leaves;
		int Root = -1;
		float DamageAttenuation = 0, NeighborDamageFraction = 0;
		bool StressEnabled = false;
		StrainSolverSettings StressSettings;
		Vector3 StressGravity = Vector3::Zero();
		std::vector<Matrix3> Inertia;
		struct StressBond { StrainBondInfo Info; bool CustomGeometry = false; };
		std::vector<StressBond> StressBonds;
		std::vector<int> DisabledStressBonds;
	};
	DestructionAsset DestructionAsset::Capture(const DestructionSet& Source, const DestructionAssetSettings& Settings)
	{
		if (!Settings.IsValid() || Source.mChunks.empty() || Source.mChunks.size() > INT_MAX || Source.mGraph.Size() != Source.mChunks.size()
			|| (Source.mPhysicsWorld && Source.mPhysicsWorld->IsSimulating())) throw std::invalid_argument("Invalid destruction authoring source");
		if (!Source.mMomentumQueue.empty() || !Source.mPartitionQueue.empty() || !Source.mStressImpulses.empty() || Source.bHasDestructionInfo
			|| !Source.mGraph.GetBrokenNodes().empty() || !Source.mGraph.GetBrokenBonds().empty()) throw std::invalid_argument("Capture destruction assets before loading or damage");
		ValidateConstants(Source.mConstants);
		auto D = std::make_shared<Storage>(); D->Settings = Settings; D->Constants = Source.mConstants; D->Graph = Source.mGraph;
		AssetHash Hash; constexpr uint64_t SchemaVersion = 1; Hash.Integer(SchemaVersion); HashConstants(Hash,D->Constants);
		Hash.Scalar(Settings.RelativeGeometryTolerance); Hash.Scalar(Settings.MinimumLength); Hash.Scalar(Settings.QuaternionTolerance); Hash.Integer(Source.mChunks.size());
		for (const auto& C : Source.mChunks)
		{
			if (C.IsFree || C.IsSmashed || !BoundsValid(C.Bounds,Settings.MinimumLength) || !Finite(C.Centroid) || !Positive(C.Volume)
				|| !Positive(C.Mass) || !Positive(C.Density) || !NonNegative(C.CollapseStrength)) throw std::invalid_argument("Invalid authored destruction chunk");
			DestructionChunk Copy = C;
			if (C.CollisionConvex) { ValidateHull(*C.CollisionConvex,Settings,false); Copy.CollisionConvex = CopyHull(*C.CollisionConvex,Quaternion::One()); }
			else
			{
				Vector3 Corners[8]; Box3::GetVertices(C.Bounds.Min,C.Bounds.Max,Corners); Copy.CollisionConvex = std::make_shared<ConvexMesh>();
				if (!Copy.CollisionConvex->BuildFromPoints(Corners,8,C.Centroid,Settings.MinimumLength)) throw std::invalid_argument("Cannot prepare destruction box hull");
			}
			D->Stats.ReorientedFaces += OrientHull(*Copy.CollisionConvex); ValidateHull(*Copy.CollisionConvex,Settings);
			DestructionAssetChunk Metadata; Metadata.Name = C.Name; Metadata.Bounds = C.Bounds; Metadata.Centroid = C.Centroid;
			Metadata.Volume = C.Volume; Metadata.Mass = C.Mass; Metadata.Density = C.Density; Metadata.CollapseStrength = C.CollapseStrength; Metadata.IsStatic = C.IsStatic;
			D->Metadata.push_back(Metadata); D->Chunks.push_back(Copy); D->Stats.TotalMass += C.Mass; D->Stats.TotalVolume += C.Volume; D->Stats.Bounds.Encapsulate(C.Bounds);
			Hash.String(C.Name); Hash.Bounds(C.Bounds); Hash.Vector(C.Centroid); Hash.Scalar(C.Volume); Hash.Scalar(C.Mass); Hash.Scalar(C.Density); Hash.Scalar(C.CollapseStrength); Hash.Integer(C.IsStatic);
			const auto& H = *Copy.CollisionConvex; Hash.Integer(H.NumVertices); Hash.Integer(H.NumFaces); Hash.Integer(H.NumIndices);
			for (uint16_t I = 0; I < H.NumVertices; ++I) Hash.Vector(H.Vertices[I]);
			for (uint16_t I = 0; I < H.NumFaces; ++I) { Hash.Vector(H.Faces[I].plane.Normal); Hash.Scalar(H.Faces[I].plane.D); Hash.Integer(H.Faces[I].first); Hash.Integer(H.Faces[I].numVerties); }
			for (uint16_t I = 0; I < H.NumIndices; ++I) Hash.Integer(H.Indices[I]);
		}
		if (!std::isfinite(D->Stats.TotalMass) || D->Stats.TotalMass > std::numeric_limits<float>::max() || !std::isfinite(D->Stats.TotalVolume)) throw std::overflow_error("Destruction asset totals overflow");
		std::set<std::pair<int,int>> Pairs; std::map<int,std::pair<int,int>> Identities;
		for (size_t I = 0; I < D->Graph.Size(); ++I)
		{
			auto& Node = D->Graph[I]; if (!NonNegative(Node.Strength)) throw std::invalid_argument("Invalid destruction node strength");
			std::sort(Node.Bonds.begin(),Node.Bonds.end(),[](const Bond& A,const Bond& B) { return A.v1 < B.v1; });
			Hash.Scalar(Node.Strength); Hash.Integer(Node.SupportGroupID); Hash.Integer(Node.IsSupport); Hash.Integer(Node.Bonds.size()); std::set<int> Neighbors;
			for (const auto& B : Node.Bonds)
			{
				const auto Type = B.GetDestructionType();
				if (B.v0 != (int)I || B.v1 < 0 || (size_t)B.v1 >= D->Chunks.size() || B.v1 == B.v0 || !Neighbors.insert(B.v1).second
					|| Type == DestructionBondType::Invalid || !NonNegative(B.strength) || B.bond_index < -1 || !Finite(B.frame.Origin)
					|| !Finite(B.frame.Normal) || !Finite(B.frame.Tangent) || !Finite(B.frame.Binormal)) throw std::invalid_argument("Invalid authored destruction bond");
				const std::pair<int,int> Pair(std::min(B.v0,B.v1),std::max(B.v0,B.v1)); Pairs.insert(Pair);
				if (B.bond_index >= 0) { auto Insert = Identities.emplace(B.bond_index,Pair); if (!Insert.second && Insert.first->second != Pair) throw std::invalid_argument("Duplicate destruction bond identity"); }
				Hash.Integer(B.v0); Hash.Integer(B.v1); Hash.Integer((int)Type); Hash.Integer(B.bond_index); Hash.Scalar(B.strength);
				Hash.Vector(B.frame.Origin); Hash.Vector(B.frame.Normal); Hash.Vector(B.frame.Tangent); Hash.Vector(B.frame.Binormal);
			}
		}
		D->Graph.RefreshBondCache(); D->Stats.BondCount = Pairs.size();
		D->Hierarchy = Source.mHierarchyNodes; D->BoundaryBonds = Source.mHierarchyBoundaryBonds; D->Leaves = Source.mChunkHierarchyLeaves; D->Root = Source.mHierarchyRoot;
		D->DamageAttenuation = Source.mHierarchyDamageAttenuation; D->NeighborDamageFraction = Source.mHierarchyNeighborDamageFraction;
		if (!NonNegative(D->DamageAttenuation) || !NonNegative(D->NeighborDamageFraction) || D->DamageAttenuation > 1 || D->NeighborDamageFraction > 1) throw std::invalid_argument("Invalid hierarchical damage policy");
		Hash.Scalar(D->DamageAttenuation); Hash.Scalar(D->NeighborDamageFraction); Hash.Integer(D->Root); Hash.Integer(D->Hierarchy.size());
		if (D->Hierarchy.empty())
		{ if (D->Root != -1 || !D->BoundaryBonds.empty() || !D->Leaves.empty()) throw std::invalid_argument("Incomplete destruction hierarchy"); }
		else
		{
			if (D->Hierarchy.size() > INT_MAX || D->Root < 0 || (size_t)D->Root >= D->Hierarchy.size() || D->Hierarchy[D->Root].Parent != -1
				|| D->BoundaryBonds.size() != D->Hierarchy.size() || D->Leaves.size() != D->Chunks.size()) throw std::invalid_argument("Invalid destruction hierarchy root");
			std::vector<int> Stack = {D->Root}; std::set<int> Visited;
			while (!Stack.empty())
			{
				const int I = Stack.back(); Stack.pop_back(); if (!Visited.insert(I).second) throw std::invalid_argument("Cyclic destruction hierarchy");
				const auto& N = D->Hierarchy[I]; if (N.IsBroken || N.Level < 0 || !NonNegative(N.BreakThreshold) || N.ChunkIndices.empty() || !BoundsValid(N.Bounds,Settings.MinimumLength)) throw std::invalid_argument("Invalid authored hierarchy node");
				std::set<int> Chunks(N.ChunkIndices.begin(),N.ChunkIndices.end()), Children;
				if (Chunks.size() != N.ChunkIndices.size() || *Chunks.begin() < 0 || (size_t)*Chunks.rbegin() >= D->Chunks.size()) throw std::invalid_argument("Invalid hierarchy chunk membership");
				std::set<int> ChildChunks;
				for (int Child : N.Children)
				{
					if (Child < 0 || (size_t)Child >= D->Hierarchy.size() || !Children.insert(Child).second || D->Hierarchy[Child].Parent != I || D->Hierarchy[Child].Level <= N.Level) throw std::invalid_argument("Invalid hierarchy child");
					for (int C : D->Hierarchy[Child].ChunkIndices) if (!ChildChunks.insert(C).second) throw std::invalid_argument("Overlapping hierarchy children");
					Stack.push_back(Child);
				}
				if (!N.Children.empty() && ChildChunks != Chunks) throw std::invalid_argument("Incomplete hierarchy partition");
				if (N.Children.empty() && (Chunks.size() != 1 || D->Leaves[*Chunks.begin()] != I)) throw std::invalid_argument("Invalid hierarchy leaf mapping");
				std::set<std::pair<int,int>> Boundaries;
				for (auto Pair : D->BoundaryBonds[I])
				{
					if (Pair.first > Pair.second) std::swap(Pair.first,Pair.second);
					if (Pair.first == Pair.second || !Chunks.count(Pair.first) || !Chunks.count(Pair.second) || !Pairs.count(Pair) || !Boundaries.insert(Pair).second) throw std::invalid_argument("Invalid hierarchy boundary bond");
				}
			}
			if (Visited.size() != D->Hierarchy.size() || D->Hierarchy[D->Root].ChunkIndices.size() != D->Chunks.size()) throw std::invalid_argument("Disconnected destruction hierarchy");
		}
		for (const auto& N : D->Hierarchy)
		{
			Hash.Integer(N.Parent); Hash.Integer(N.Level); Hash.Bounds(N.Bounds); Hash.Scalar(N.BreakThreshold); Hash.Integer(N.IsBreakable);
			Hash.Integer(N.Children.size()); for (int I : N.Children) Hash.Integer(I); Hash.Integer(N.ChunkIndices.size()); for (int I : N.ChunkIndices) Hash.Integer(I);
		}
		for (const auto& PairsAtNode : D->BoundaryBonds) { Hash.Integer(PairsAtNode.size()); for (auto Pair : PairsAtNode) { Hash.Integer(Pair.first); Hash.Integer(Pair.second); } }
		for (int I : D->Leaves) Hash.Integer(I); D->Stats.HierarchyNodes = D->Hierarchy.size();
		D->StressEnabled = Source.mStressSolver != nullptr; Hash.Integer(D->StressEnabled);
		if (D->StressEnabled)
		{
			const auto& Solver = *Source.mStressSolver;
			if (Solver.mFrameCount != 0 || Solver.mNodes.size() != D->Chunks.size()) throw std::invalid_argument("Capture stress authoring before simulation");
			D->StressSettings = Solver.GetSettings(); D->StressGravity = Source.mStressGravity;
			const auto& S = D->StressSettings;
			Hash.Scalar(S.Hardness); Hash.Scalar(S.StressLinearFactor); Hash.Scalar(S.StressAngularFactor); Hash.Scalar(S.RelativeTolerance); Hash.Scalar(S.AbsoluteTolerance);
			Hash.Integer(S.BondIterationsPerFrame); Hash.Integer(S.GraphReductionLevel); Hash.Integer(S.MaxIterations); Hash.Integer(S.WarmStart); Hash.Vector(D->StressGravity);
			Hash.Integer(Solver.mNodes.size());
			for (const auto& N : Solver.mNodes)
			{
				if (N.Impulse.SquareLength() != 0 || N.AngularImpulse.SquareLength() != 0 || N.Force.SquareLength() != 0 || N.Torque.SquareLength() != 0) throw std::invalid_argument("Pending stress loads are not authoring data");
				D->Inertia.push_back(N.Info.Inertia); for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) Hash.Scalar(N.Info.Inertia(I,J));
			}
			std::set<int> Active;
			for (const auto& B : Solver.mRuntimeBonds)
			{
				D->StressBonds.push_back({B.Info,B.CustomGeometry}); Active.insert(B.Info.BondIndex);
			}
			std::sort(D->StressBonds.begin(),D->StressBonds.end(),[](const Storage::StressBond& A,const Storage::StressBond& B) { return A.Info.BondIndex < B.Info.BondIndex; });
			Hash.Integer(D->StressBonds.size());
			for (const auto& B : D->StressBonds)
			{
				Hash.Integer(B.Info.BondIndex); Hash.Scalar(B.Info.Health); Hash.Integer(B.CustomGeometry);
				if (B.CustomGeometry) { Hash.Vector(B.Info.Centroid); Hash.Vector(B.Info.Normal); Hash.Scalar(B.Info.Area); }
			}
			for (const auto& Identity : Identities) if (!Active.count(Identity.first)) D->DisabledStressBonds.push_back(Identity.first);
			Hash.Integer(D->DisabledStressBonds.size()); for (int I : D->DisabledStressBonds) Hash.Integer(I);
		}
		D->Stats.Fingerprint = Hash.Value; DestructionAsset Asset; Asset.m_Data = std::move(D); return Asset;
	}
	const DestructionAsset::Storage& DestructionAsset::Data() const
	{ if (!m_Data) throw std::logic_error("Unprepared destruction asset"); return *m_Data; }
	const std::vector<DestructionAssetChunk>& DestructionAsset::GetChunks() const { return Data().Metadata; }
	const DestructionAssetStats& DestructionAsset::GetStats() const { return Data().Stats; }
	void DestructionSet::InitializeAsset(const DestructionAsset& Asset, const Transform& Placement)
	{
		if (!mChunks.empty() || !mClusters.empty() || !mGraph.Empty() || mStressSolver || (mPhysicsWorld && mPhysicsWorld->IsSimulating())) throw std::logic_error("Initialize destruction assets on empty idle sets");
		const auto& D = Asset.Data(); const auto& Q = Placement.quat;
		const double Norm = (double)Q.x*Q.x + (double)Q.y*Q.y + (double)Q.z*Q.z + (double)Q.w*Q.w;
		if (!Finite(Placement.pos) || !std::isfinite(Norm) || std::abs(Norm - 1) > D.Settings.QuaternionTolerance) throw std::invalid_argument("Invalid destruction asset placement");
		Transform Pose(Placement.pos,Q.Unit()); DestructionSet Staged;
		Staged.mConstants = D.Constants; Staged.mGraph = D.Graph; Staged.mHierarchyNodes = D.Hierarchy; Staged.mHierarchyBoundaryBonds = D.BoundaryBonds;
		Staged.mChunkHierarchyLeaves = D.Leaves; Staged.mHierarchyRoot = D.Root; Staged.mHierarchyDamageAttenuation = D.DamageAttenuation; Staged.mHierarchyNeighborDamageFraction = D.NeighborDamageFraction;
		for (const auto& C : D.Chunks)
		{
			auto Copy = C; Copy.Centroid = Pose.LocalToWorld(C.Centroid); Copy.CollisionConvex = CopyHull(*C.CollisionConvex,Pose.quat);
			const auto LocalBounds = Copy.CollisionConvex->Bounds; Copy.Bounds = Box3(LocalBounds.Min + Copy.Centroid,LocalBounds.Max + Copy.Centroid);
			if (!Finite(Copy.Centroid) || !BoundsValid(Copy.Bounds,D.Settings.MinimumLength)) throw std::overflow_error("Destruction placement loses geometry precision");
			const double Size = Copy.Bounds.MaxDim(); if ((double)Copy.Mass * Size * Size > std::numeric_limits<float>::max()) throw std::overflow_error("Destruction inertia exceeds simulation precision");
			Staged.mChunks.push_back(std::move(Copy));
		}
		for (auto& Node : Staged.mGraph.GetBondLists()) for (auto& B : Node.Bonds)
		{
			B.frame.Origin = Pose.LocalToWorld(B.frame.Origin); B.frame.Normal = Pose.LocalToWorldDirection(B.frame.Normal);
			B.frame.Tangent = Pose.LocalToWorldDirection(B.frame.Tangent); B.frame.Binormal = Pose.LocalToWorldDirection(B.frame.Binormal);
			if (!Finite(B.frame.Origin)) throw std::overflow_error("Destruction bond placement overflow");
		}
		Staged.mGraph.RefreshBondCache();
		for (auto& N : Staged.mHierarchyNodes) { N.Bounds = Box3::Transform(N.Bounds,Pose.pos,Pose.quat); if (!BoundsValid(N.Bounds,D.Settings.MinimumLength)) throw std::overflow_error("Hierarchy placement overflow"); }
		Staged.RecalculateBounds();
		if (D.StressEnabled)
		{
			Staged.EnableStressSolver(D.StressSettings,D.StressGravity); const auto R = Pose.quat.ToRotationMatrix3();
			for (size_t I = 0; I < D.Inertia.size(); ++I) if (!Staged.mStressSolver->SetNodeInertia((int)I,R*D.Inertia[I]*R.Transpose())) throw std::invalid_argument("Invalid placed stress inertia");
			for (const auto& B : D.StressBonds)
			{
				if (!Staged.mStressSolver->SetBondHealth(B.Info.BondIndex,B.Info.Health)) throw std::invalid_argument("Invalid authored stress health");
				if (B.CustomGeometry && !Staged.mStressSolver->SetBondGeometry(B.Info.BondIndex,Pose.LocalToWorld(B.Info.Centroid),Pose.LocalToWorldDirection(B.Info.Normal),B.Info.Area)) throw std::invalid_argument("Invalid authored stress geometry");
			}
			for (int I : D.DisabledStressBonds) Staged.mStressSolver->RemoveBond(I);
		}
		mChunks.swap(Staged.mChunks); std::swap(mGraph,Staged.mGraph); mHierarchyNodes.swap(Staged.mHierarchyNodes); mHierarchyBoundaryBonds.swap(Staged.mHierarchyBoundaryBonds);
		mChunkHierarchyLeaves.swap(Staged.mChunkHierarchyLeaves); mHierarchyRoot = Staged.mHierarchyRoot; mHierarchyDamageAttenuation = Staged.mHierarchyDamageAttenuation;
		mHierarchyNeighborDamageFraction = Staged.mHierarchyNeighborDamageFraction; mConstants = Staged.mConstants; mBounds = Staged.mBounds;
		mStressSolver.swap(Staged.mStressSolver); mStressGravity = Staged.mStressGravity; mAsset = Asset; mAssetPlacement = Pose;
		try { RebuildClustersFromGraph(); } catch (...) { Clear(); throw; }
	}
}


namespace Riemann
{
	/* External destruction observations
	   ---------------------------------
	   Damage commands and notifications have different lifetimes. Legacy bitsets
	   still drive destruction; this optional stream observes committed topology
	   and copies contact data. Polling between world steps transfers ownership to
	   the consumer, so processing an event cannot invalidate a solver's pointers.
	   Events contain no borrowed bodies, geometry, contact arrays or asset data.

	   Instance identity never uses an address. Clear advances a generation before
	   chunk indices can be reused. An asset fingerprint identifies authored data;
	   runtime cluster IDs are meaningful only with their instance and generation.
	   Membership, rather than newly allocated cluster IDs, detects actual splits.
	   Rebuilding unchanged topology therefore emits no artificial break events.
	   A physical bond produces one notification. Its endpoint order, type and
	   strength describe the first retained directed record, including one-way
	   support edges; the ordered pair used for deduplication is only an index.

	   Topology payloads contain chunk states immediately BEFORE the rebuild;
	   fragments describe membership AFTER it. Pose maps the chunk's collision-local
	   vertices into world space. LinearVelocity is measured at Pose.pos, including
	   the parent's angular contribution. Refresh these values before old bodies are
	   released, including for chunks removed by a legacy destruction command.
	   Contact Impulse follows the legacy damage input: ImpulseEstimated identifies
	   its closing-speed fallback when the solver supplied no usable impulse.

	   The bounded queue retains the earliest matching events and drops new events
	   when full. Sequence advances before filtering/dropping; counters and gaps make
	   loss observable. Consumers requiring lossless recording must drain frequently
	   and reject recordings with dropped/invalid observations. Polling does not
	   change the baseline, and exhausted queues cannot replay an old fracture.
	   Reconfiguration starts a new baseline and discards the pending queue. The
	   contact threshold belongs to notification settings independently of the
	   legacy minimum damage momentum; a zero threshold also reports resting points. */
	namespace
	{
		bool EventFinite(const Vector3& P) { return std::isfinite(P.x) && std::isfinite(P.y) && std::isfinite(P.z); }
		bool Valid(const DestructionEventChunk& C)
		{
			const auto& Q = C.Pose.quat;
			return EventFinite(C.Pose.pos) && EventFinite(C.LinearVelocity) && EventFinite(C.AngularVelocity)
				&& std::isfinite(C.Mass) && C.Mass > 0 && std::isfinite(Q.x) && std::isfinite(Q.y) && std::isfinite(Q.z) && std::isfinite(Q.w);
		}
	}
	bool DestructionEventSettings::IsValid() const
	{
		return !(KindMask & ~AllKinds) && MaximumQueuedEvents > 0 && std::isfinite(MinimumContactImpulse) && MinimumContactImpulse >= 0;
	}
	struct DestructionEvents::Storage
	{
		using Pair = std::pair<int,int>;
		struct Snapshot
		{
			std::vector<DestructionEventChunk> Chunks;
			std::map<Pair,Bond> Bonds;
			std::map<uint32_t,std::vector<int>> Clusters;
			std::vector<bool> BrokenHierarchy;
		};
		DestructionEventSettings Settings;
		DestructionEventStats Counters;
		Snapshot Previous;
		std::deque<DestructionEvent> Queue;
		static void ReadRigidState(const DestructionCluster& Cluster, DestructionEventChunk& C)
		{
			const auto* Body = Cluster.GetRigidBody(); Cluster.GetWorldTransform(C.ChunkIndex,C.Pose); C.HasRigidBody = Body != nullptr;
			if (Body)
			{
				C.AngularVelocity = Body->GetAngularVelocity();
				C.LinearVelocity = Body->GetLinearVelocity() + C.AngularVelocity.Cross(C.Pose.pos - Body->X);
			}
		}
		static void ReadKinematics(const DestructionSet& Set, std::vector<DestructionEventChunk>& Chunks)
		{
			for (const auto& Entry : Set.mClusters)
			{
				const auto& Cluster = Entry.second;
				for (int I : Cluster.GetSourceIndices()) if (I >= 0 && (size_t)I < Chunks.size())
					ReadRigidState(Cluster,Chunks[I]);
			}
		}
		static Snapshot Read(const DestructionSet& Set)
		{
			Snapshot S; S.Chunks.reserve(Set.mChunks.size());
			for (size_t I = 0; I < Set.mChunks.size(); ++I)
			{
				const auto& C = Set.mChunks[I]; DestructionEventChunk State;
				State.ChunkIndex = (int)I; State.Pose = Transform(C.Centroid); State.Mass = C.Mass;
				State.IsStatic = C.IsStatic; State.IsFree = C.IsFree; State.IsSmashed = C.IsSmashed; S.Chunks.push_back(State);
			}
			for (const auto& Entry : Set.mClusters)
			{
				auto Members = Entry.second.GetSourceIndices(); std::sort(Members.begin(),Members.end());
				for (int I : Members) if (I >= 0 && (size_t)I < S.Chunks.size()) S.Chunks[I].ClusterId = Entry.first;
				S.Clusters.emplace(Entry.first,std::move(Members));
			}
			ReadKinematics(Set,S.Chunks);
			for (size_t I = 0; I < Set.mGraph.Size(); ++I) for (const auto& B : Set.mGraph[I].Bonds)
			{
				const int Other = B.GetOther((int)I);
				if (Other >= 0 && (size_t)Other < Set.mChunks.size() && I < Set.mChunks.size()) S.Bonds.emplace(std::minmax((int)I,Other),B);
			}
			for (const auto& N : Set.mHierarchyNodes) S.BrokenHierarchy.push_back(N.IsBroken);
			return S;
		}
		void Emit(DestructionSet& Set, DestructionEvent Event)
		{
			++Counters.Generated;
			if (!Set.mNextEventSequence) { ++Counters.Invalid; return; }
			Event.Sequence = Set.mNextEventSequence++; Event.InstanceId = Set.mInstanceId; Event.Generation = Set.mGeneration;
			Event.AssetFingerprint = Set.mAsset.IsValid() ? Set.mAsset.GetStats().Fingerprint : 0;
			Event.Cause = Event.Kind == DestructionEventKind::Contact ? DestructionEventCause::Contact : Set.mEventCause;
			Event.UpdateIndex = Set.mEventUpdateIndex;
			Event.DeltaTime = Set.mEventDeltaTime;
			if (Set.mPhysicsWorld)
			{
				const auto& Step = Set.mPhysicsWorld->GetStepInfo(); Event.WorldTick = Set.mPhysicsWorld->GetTick();
				Event.DeltaTime = Step.DeltaTime; Event.SubstepIndex = Step.SubstepIndex; Event.SubstepCount = Step.SubstepCount;
			}
			if (!EventFinite(Event.Position) || !EventFinite(Event.Normal) || !EventFinite(Event.Impulse)
				|| !std::isfinite(Event.BondStrength) || Event.BondStrength < 0
				|| !std::all_of(Event.Chunks.begin(),Event.Chunks.end(),Valid)) { ++Counters.Invalid; return; }
			const double ImpulseSize = std::hypot(std::hypot((double)Event.Impulse.x,Event.Impulse.y),Event.Impulse.z);
			if (!(Settings.KindMask & (1u << (uint32_t)Event.Kind))
				|| (Event.Kind == DestructionEventKind::Contact && ImpulseSize < Settings.MinimumContactImpulse)) { ++Counters.Filtered; return; }
			if (Queue.size() >= Settings.MaximumQueuedEvents) { ++Counters.Dropped; return; }
			Queue.push_back(std::move(Event));
		}
		DestructionEvent ForChunks(DestructionEventKind Kind, const std::vector<int>& Members) const
		{
			DestructionEvent E; E.Kind = Kind;
			for (int I : Members) if (I >= 0 && (size_t)I < Previous.Chunks.size()) E.Chunks.push_back(Previous.Chunks[I]);
			if (!E.Chunks.empty()) E.Position = E.Chunks.front().Pose.pos;
			return E;
		}
	};
	DestructionEvents::DestructionEvents(const DestructionSet& Set, const DestructionEventSettings& Settings) : m_Data(new Storage)
	{ m_Data->Settings = Settings; m_Data->Previous = Storage::Read(Set); }
	DestructionEvents::~DestructionEvents() = default;
	void DestructionEvents::Reset(const DestructionSet& Set)
	{ m_Data->Previous = Storage::Read(Set); m_Data->Queue.clear(); m_Data->Counters = {}; }
	void DestructionEvents::RefreshKinematics(const DestructionSet& Set)
	{ Storage::ReadKinematics(Set,m_Data->Previous.Chunks); }
	void DestructionEvents::Collect(DestructionSet& Set)
	{
		auto& D = *m_Data; auto Current = Storage::Read(Set); const auto& Before = D.Previous;
		if (Current.Chunks.size() != Before.Chunks.size()) { Set.ResetEventState(); return; }
		for (const auto& Entry : Before.Bonds) if (!Current.Bonds.count(Entry.first))
		{
			const auto& Bond = Entry.second; const int A = Bond.v0, B = Bond.v1;
			auto E = D.ForChunks(DestructionEventKind::BondBroken,{A,B}); E.NodeA = A; E.NodeB = B;
			E.BondIndex = Bond.bond_index; E.BondType = Bond.GetDestructionType(); E.BondStrength = Bond.strength;
			const auto& PA = Before.Chunks[A].Pose; const auto& PB = Before.Chunks[B].Pose;
			E.Position = (PA.LocalToWorld(Bond.frame.Origin-Set.mChunks[A].Centroid) + PB.LocalToWorld(Bond.frame.Origin-Set.mChunks[B].Centroid)) * 0.5f;
			E.Normal = PA.LocalToWorldDirection(Bond.frame.Normal); D.Emit(Set,std::move(E));
		}
		for (size_t I = 0; I < Current.BrokenHierarchy.size() && I < Before.BrokenHierarchy.size(); ++I)
			if (Current.BrokenHierarchy[I] && !Before.BrokenHierarchy[I])
			{
				auto E = D.ForChunks(DestructionEventKind::HierarchyBroken,Set.mHierarchyNodes[I].ChunkIndices);
				E.HierarchyNode = (int)I; D.Emit(Set,std::move(E));
			}
		for (size_t I = 0; I < Current.Chunks.size(); ++I)
		{
			if (Current.Chunks[I].IsFree && !Before.Chunks[I].IsFree) D.Emit(Set,D.ForChunks(DestructionEventKind::ChunkReleased,{(int)I}));
			if (Current.Chunks[I].IsSmashed && !Before.Chunks[I].IsSmashed) D.Emit(Set,D.ForChunks(DestructionEventKind::ChunkSmashed,{(int)I}));
		}
		std::set<std::vector<int>> Membership; for (const auto& C : Current.Clusters) Membership.insert(C.second);
		for (const auto& C : Before.Clusters) if (!Membership.count(C.second))
		{
			auto E = D.ForChunks(DestructionEventKind::ClusterChanged,C.second); E.PreviousClusterId = C.first;
			std::map<uint32_t,std::vector<int>> Fragments;
			for (int I : C.second) if (I >= 0 && (size_t)I < Current.Chunks.size()) Fragments[Current.Chunks[I].ClusterId].push_back(I);
			for (auto& F : Fragments) E.Fragments.push_back({F.first,std::move(F.second)});
			D.Emit(Set,std::move(E));
		}
		D.Previous = std::move(Current);
	}
	void DestructionEvents::Contact(DestructionSet& Set, const DestructionCluster& Cluster, int Chunk, const PhysicsContactPoint& Point, const Vector3& Impulse, const Vector3& Normal, bool ImpulseEstimated)
	{
		auto& D = *m_Data;
		if (Chunk < 0 || (size_t)Chunk >= Set.mChunks.size()) return;
		const auto& Source = Set.mChunks[Chunk]; DestructionEventChunk C; C.ChunkIndex = Chunk; C.ClusterId = Cluster.GetID();
		C.Pose = Transform(Source.Centroid); C.Mass = Source.Mass; C.IsStatic = Source.IsStatic; C.IsFree = Source.IsFree; C.IsSmashed = Source.IsSmashed;
		Storage::ReadRigidState(Cluster,C);
		DestructionEvent E; E.Kind = DestructionEventKind::Contact; E.Position = Point.Position; E.Normal = Normal; E.Impulse = Impulse;
		E.ImpulseEstimated = ImpulseEstimated;
		E.Chunks.push_back(C); D.Emit(Set,std::move(E));
	}
	std::vector<DestructionEvent> DestructionEvents::Take()
	{
		std::vector<DestructionEvent> Out; Out.reserve(m_Data->Queue.size());
		while (!m_Data->Queue.empty()) { Out.push_back(std::move(m_Data->Queue.front())); m_Data->Queue.pop_front(); }
		return Out;
	}
	DestructionEventStats DestructionEvents::Stats() const
	{ auto Out = m_Data->Counters; Out.Pending = m_Data->Queue.size(); return Out; }
	uint64_t DestructionSet::AllocateInstanceId()
	{
		static std::atomic<uint64_t> Next{1}; auto Value = Next.load(std::memory_order_relaxed);
		for (;;)
		{
			if (!Value || Value == std::numeric_limits<uint64_t>::max()) throw std::overflow_error("Destruction instance identity exhausted");
			if (Next.compare_exchange_weak(Value,Value+1,std::memory_order_relaxed)) return Value;
		}
	}
	void DestructionSet::ConfigureEvents(const DestructionEventSettings& Settings)
	{
		if (mPhysicsWorld && mPhysicsWorld->IsSimulating()) throw std::logic_error("Configure destruction events between world steps");
		if (!Settings.IsValid()) throw std::invalid_argument("Invalid destruction event settings");
		std::unique_ptr<DestructionEvents> Staged(Settings.Enabled ? new DestructionEvents(*this,Settings) : nullptr);
		if (mEventRevision == std::numeric_limits<uint64_t>::max()) throw std::overflow_error("Destruction event revision exhausted");
		mEvents.swap(Staged); mEventSettings = Settings; ++mEventRevision;
	}
	DestructionEventStats DestructionSet::GetEventStats() const { return mEvents ? mEvents->Stats() : DestructionEventStats(); }
	std::vector<DestructionEvent> DestructionSet::TakeEvents()
	{
		if (mPhysicsWorld && mPhysicsWorld->IsSimulating()) throw std::logic_error("Poll destruction events between world steps");
		return mEvents ? mEvents->Take() : std::vector<DestructionEvent>();
	}
	void DestructionSet::RefreshEventKinematics() { if (mEvents) mEvents->RefreshKinematics(*this); }
	void DestructionSet::CollectEvents() { if (mEvents) mEvents->Collect(*this); }
	void DestructionSet::ResetEventState()
	{
		if (mGeneration == std::numeric_limits<uint64_t>::max() || mEventRevision == std::numeric_limits<uint64_t>::max()) throw std::overflow_error("Destruction generation exhausted");
		++mGeneration; ++mEventRevision; if (mEvents) mEvents->Reset(*this);
	}
	DestructionSet::EventCauseScope::EventCauseScope(DestructionSet& Set, DestructionEventCause Cause) : mSet(Set), mPrevious(Set.mEventCause)
	{ if (Set.mEventCause == DestructionEventCause::GraphEdit) Set.mEventCause = Cause; }
	DestructionSet::EventCauseScope::~EventCauseScope() { mSet.mEventCause = mPrevious; }
}


namespace Riemann
{
	/* Evaluated destruction trajectories
	   ---------------------------------
	   An authored asset supplies immutable rest geometry. A cache records evaluated
	   chunk flags, cluster motion, graph/hierarchy damage and copied external events.
	   This is a trajectory cache: queued
	   damage, stress iterates and contact warm starts are not simulation checkpoints.
	   DestructionPlayback consumes these samples without rerunning fracture.

	   Recording uses the source asset placement as its coordinate frame. Asset
	   instancing bakes placement rotation S into collision vertices. Consequently a
	   geometry rotation G becomes S^-1 G S in the cache, rather than just S^-1 G.
	   Replaying at rotation T uses T Gcache T^-1 on the T-baked collision geometry.
	   Points include translation; velocity, normals and impulses only rotate.

	   Each cluster is anchored to its smallest chunk index. Its other local poses
	   come from the stored body-local transforms, avoiding pose-dependent roundoff
	   and arbitrary center-of-mass frames. Interpolate the anchor and reconstruct
	   every member rigidly. Independently lerping each chunk position would shrink
	   a rotating compound. Topology and flags hold the lower sample until the next
	   key, including at fracture boundaries; an anchor retired at that key is held.

	   Identical topology shares one immutable record. Frame/time/payload limits
	   fail before appending. Payload accounting covers schema objects and their
	   logical array contents, excluding allocator overhead and the shared asset.
	   Legacy mutable chunk geometry is compared to an independent prepared instance
	   on every capture; an asset handle alone cannot prove compatibility. Named
	   length/direction tolerances accommodate repeated float placement normalization.

	   Event delivery remains owned by the caller: drain once and pass that batch
	   to both recorder and gameplay. Accepted-event counts, source identity,
	   generation, stream revision and sequence ranges reject missing/duplicated
	   batches, reconfiguration and queue loss. Configured filters are intentional.
	   Events are observed at the containing capture boundary, with original tick
	   and substep metadata retained. Queries use (After, Through], so adjacent
	   playback intervals neither duplicate nor skip a boundary event. */
	namespace
	{
		bool CacheFinite(const Vector3& P) { return std::isfinite(P.x) && std::isfinite(P.y) && std::isfinite(P.z); }
		bool Near(const Vector3& A, const Vector3& B, double Tolerance)
		{ return std::hypot(std::hypot((double)A.x-B.x,(double)A.y-B.y),(double)A.z-B.z) <= Tolerance; }
		bool RotationValid(const Quaternion& Q, float Tolerance)
		{
			const double N = (double)Q.x*Q.x + (double)Q.y*Q.y + (double)Q.z*Q.z + (double)Q.w*Q.w;
			return std::isfinite(N) && std::abs(N-1) <= Tolerance;
		}
		void RequirePose(const Transform& P, float Tolerance)
		{ if (!CacheFinite(P.pos) || !RotationValid(P.quat,Tolerance)) throw std::invalid_argument("Invalid destruction cache pose"); }
		Transform CanonicalPose(const Transform& P, const Transform& Placement)
		{ return Transform(Placement.WorldToLocal(P.pos),(Placement.quat.Conjugate()*P.quat*Placement.quat).Unit()); }
		Transform PlacedPose(const Transform& P, const Transform& Placement)
		{ return Transform(Placement.LocalToWorld(P.pos),(Placement.quat*P.quat*Placement.quat.Conjugate()).Unit()); }
		bool SameFrame(const BondFrame& A, const BondFrame& B)
		{ return A.Origin == B.Origin && A.Normal == B.Normal && A.Tangent == B.Tangent && A.Binormal == B.Binormal; }
		bool NearFrame(const BondFrame& A, const BondFrame& B, float LengthTolerance, float DirectionTolerance)
		{ return Near(A.Origin,B.Origin,LengthTolerance) && Near(A.Normal,B.Normal,DirectionTolerance) && Near(A.Tangent,B.Tangent,DirectionTolerance) && Near(A.Binormal,B.Binormal,DirectionTolerance); }
		bool SameBond(const Bond& A, const Bond& B)
		{ return A.v0 == B.v0 && A.v1 == B.v1 && A.type == B.type && A.strength == B.strength && A.bond_index == B.bond_index && SameFrame(A.frame,B.frame); }
		bool SameGraph(const ConnectionGraph& A, const ConnectionGraph& B)
		{
			if (A.Size() != B.Size() || A.GetBrokenNodes() != B.GetBrokenNodes() || A.GetBrokenBonds() != B.GetBrokenBonds()) return false;
			for (size_t I = 0; I < A.Size(); ++I)
			{
				const auto& X = A[I]; const auto& Y = B[I];
				if (X.Strength != Y.Strength || X.IsSupport != Y.IsSupport || X.SupportGroupID != Y.SupportGroupID || X.Bonds.size() != Y.Bonds.size()) return false;
				for (size_t J = 0; J < X.Bonds.size(); ++J) if (!SameBond(X.Bonds[J],Y.Bonds[J])) return false;
			}
			return true;
		}
		bool SameTopology(const DestructionCacheTopology& A, const DestructionCacheTopology& B)
		{
			if (!SameGraph(A.Graph,B.Graph) || A.BrokenHierarchy != B.BrokenHierarchy || A.Clusters.size() != B.Clusters.size()) return false;
			for (size_t I = 0; I < A.Clusters.size(); ++I)
			{
				const auto& X = A.Clusters[I]; const auto& Y = B.Clusters[I];
				if (X.SourceId != Y.SourceId || X.Level != Y.Level || X.IsStatic != Y.IsStatic || X.ChunkIndices != Y.ChunkIndices || X.LocalPoses != Y.LocalPoses) return false;
			}
			return true;
		}
		bool SameConstants(const DestructionConstants& A, const DestructionConstants& B)
		{
			if (std::tie(A.FilterData.v0,A.FilterData.v1,A.FilterData.v2,A.FilterData.v3,A.ContactOffset,A.RestOffset,
				A.ImpulseCollapseStrength,A.ImpulseBreakStrength,A.ImpulseRadius,A.ImpulseProcessPerTick,A.PartitionProcessPerTick,
				A.PartitionProcessIntervalTicks,A.PartitionSeparationAttenuation,A.EnableClusterCollisionProxy,A.PartitionAlgorithm,
				A.IdleTime,A.DamagePropagationStrength,A.DamagePropagationAttenuation,A.MinClusterVolume,A.EnableSupportGroup,A.EnableHingeJoint,A.DebugLog)
				!= std::tie(B.FilterData.v0,B.FilterData.v1,B.FilterData.v2,B.FilterData.v3,B.ContactOffset,B.RestOffset,
				B.ImpulseCollapseStrength,B.ImpulseBreakStrength,B.ImpulseRadius,B.ImpulseProcessPerTick,B.PartitionProcessPerTick,
				B.PartitionProcessIntervalTicks,B.PartitionSeparationAttenuation,B.EnableClusterCollisionProxy,B.PartitionAlgorithm,
				B.IdleTime,B.DamagePropagationStrength,B.DamagePropagationAttenuation,B.MinClusterVolume,B.EnableSupportGroup,B.EnableHingeJoint,B.DebugLog)) return false;
			for (size_t I = 0; I < sizeof(A.Levels)/sizeof(A.Levels[0]); ++I)
				if (A.Levels[I].BreakThreshold != B.Levels[I].BreakThreshold || A.Levels[I].PartitionSize != B.Levels[I].PartitionSize) return false;
			return true;
		}
		bool SameHull(const ConvexMesh& A, const ConvexMesh& B, float LengthTolerance, float DirectionTolerance)
		{
			if (A.NumVertices != B.NumVertices || A.NumFaces != B.NumFaces || A.NumEdges != B.NumEdges || A.NumIndices != B.NumIndices
				|| !A.Vertices || !A.Faces || !A.Indices || (A.NumEdges && !A.Edges)) return false;
			for (size_t I = 0; I < A.NumVertices; ++I) if (!Near(A.Vertices[I],B.Vertices[I],LengthTolerance)) return false;
			for (size_t I = 0; I < A.NumIndices; ++I) if (A.Indices[I] != B.Indices[I]) return false;
			for (size_t I = 0; I < A.NumEdges; ++I) if (A.Edges[I].s != B.Edges[I].s || A.Edges[I].e != B.Edges[I].e) return false;
			for (size_t I = 0; I < A.NumFaces; ++I)
			{
				const auto& X = A.Faces[I]; const auto& Y = B.Faces[I];
				if (X.first != Y.first || X.numVerties != Y.numVerties || !Near(X.plane.Normal,Y.plane.Normal,DirectionTolerance)
					|| !std::isfinite(X.plane.D) || std::abs((double)X.plane.D-Y.plane.D) > LengthTolerance) return false;
			}
			return true;
		}
		struct PayloadCounter
		{
			size_t Size = 0, Limit;
			explicit PayloadCounter(size_t Maximum) : Limit(Maximum) {}
			void Add(size_t Count, size_t Stride)
			{
				if (Stride && Count > (Limit-Size)/Stride) throw std::length_error("Destruction cache payload limit exceeded");
				Size += Count*Stride;
			}
			void Event(const DestructionEvent& E)
			{
				Add(1,sizeof(E)); Add(E.Chunks.size(),sizeof(DestructionEventChunk)); Add(E.Fragments.size(),sizeof(DestructionEventFragment));
				for (const auto& F : E.Fragments) Add(F.ChunkIndices.size(),sizeof(int));
			}
			void Topology(const DestructionCacheTopology& T)
			{
				Add(1,sizeof(T)); Add(T.Clusters.size(),sizeof(DestructionCacheCluster)); Add(T.BrokenHierarchy.size(),sizeof(bool));
				Add(T.Graph.Size(),sizeof(DestructionBondList)); Add(T.Graph.GetBrokenNodes().size(),sizeof(int)); Add(T.Graph.GetBrokenBonds().size(),sizeof(std::pair<int,int>));
				for (const auto& N : T.Graph.GetBondLists()) Add(N.Bonds.size(),sizeof(Bond));
				for (const auto& C : T.Clusters) { Add(C.ChunkIndices.size(),sizeof(int)); Add(C.LocalPoses.size(),sizeof(Transform)); }
			}
		};
		uint64_t Accepted(const DestructionEventStats& S)
		{
			uint64_t N = S.Generated;
			for (uint64_t Part : {S.Filtered,S.Dropped,S.Invalid}) { if (Part > N) throw std::invalid_argument("Invalid destruction event counters"); N -= Part; }
			return N;
		}
		void TransformEvent(DestructionEvent& E, const Transform& P, bool ToCanonical)
		{
			auto Point = [&](const Vector3& V) { return ToCanonical ? P.WorldToLocal(V) : P.LocalToWorld(V); };
			auto Direction = [&](const Vector3& V) { return ToCanonical ? P.WorldToLocalDirection(V) : P.LocalToWorldDirection(V); };
			E.Position = Point(E.Position); E.Normal = Direction(E.Normal); E.Impulse = Direction(E.Impulse);
			for (auto& C : E.Chunks)
			{
				C.Pose = ToCanonical ? CanonicalPose(C.Pose,P) : PlacedPose(C.Pose,P);
				C.LinearVelocity = Direction(C.LinearVelocity); C.AngularVelocity = Direction(C.AngularVelocity);
			}
		}
		void RequireEventGeometry(const DestructionEvent& E, float QuaternionTolerance)
		{
			if (!CacheFinite(E.Position) || !CacheFinite(E.Normal) || !CacheFinite(E.Impulse)) throw std::overflow_error("Destruction event placement overflow");
			for (const auto& C : E.Chunks)
			{
				RequirePose(C.Pose,QuaternionTolerance);
				if (!CacheFinite(C.LinearVelocity) || !CacheFinite(C.AngularVelocity)) throw std::overflow_error("Destruction event velocity overflow");
			}
		}
	}
	bool DestructionCacheSettings::IsValid() const
	{
		return MaximumFrames > 0 && MaximumPayloadBytes > 0 && std::isfinite(QuaternionTolerance) && QuaternionTolerance > 0 && QuaternionTolerance < 1
			&& std::isfinite(RelativeGeometryTolerance) && RelativeGeometryTolerance > 0 && RelativeGeometryTolerance < 1
			&& std::isfinite(MinimumGeometryTolerance) && MinimumGeometryTolerance > 0;
	}
	struct DestructionCache::Storage
	{
		DestructionAsset Asset;
		DestructionConstants Constants;
		DestructionCacheSettings Settings;
		DestructionEventSettings EventSettings;
		DestructionCacheStats Stats;
		uint64_t SourceInstance = 0, SourceGeneration = 0;
		std::vector<DestructionCacheFrame> Frames;
	};
	struct DestructionCacheRecorder::Storage
	{
		std::shared_ptr<DestructionCache::Storage> Data;
		std::unique_ptr<DestructionSet> Reference;
		Transform Placement;
		PhysicsWorld* World = nullptr;
		uint64_t EventRevision = 0, LastSequence = 0;
		DestructionEventStats LastEvents;
		void ValidateSource(const DestructionSet& Source) const
		{
			if (Source.GetInstanceId() != Data->SourceInstance || Source.GetGeneration() != Data->SourceGeneration
				|| !Source.GetAuthoredAsset() || Source.GetAuthoredAsset()->GetStats().Fingerprint != Data->Asset.GetStats().Fingerprint
				|| Source.GetAssetPlacement() != Placement || Source.GetPhysicsWorld() != World) throw std::invalid_argument("Destruction cache source identity changed");
			if (World && World->IsSimulating()) throw std::logic_error("Record destruction caches between world steps");
			if (Source.mChunks.size() != Reference->mChunks.size() || Source.mGraph.Size() != Source.mChunks.size()
				|| !SameConstants(Source.mConstants,Data->Constants) || Source.mHierarchyRoot != Reference->mHierarchyRoot
				|| Source.mHierarchyNodes.size() != Reference->mHierarchyNodes.size()) throw std::invalid_argument("Destruction cache authoring changed");
			for (size_t I = 0; I < Source.mChunks.size(); ++I)
			{
				const auto& C = Source.mChunks[I]; const auto& R = Reference->mChunks[I];
				const float Tolerance = std::max(Data->Settings.MinimumGeometryTolerance,R.Bounds.MaxDim()*Data->Settings.RelativeGeometryTolerance);
				if (C.Name != R.Name || !Near(C.Bounds.Min,R.Bounds.Min,Tolerance) || !Near(C.Bounds.Max,R.Bounds.Max,Tolerance) || !Near(C.Centroid,R.Centroid,Tolerance)
					|| C.Mass != R.Mass || C.Volume != R.Volume || C.Density != R.Density || C.CollapseStrength != R.CollapseStrength
					|| !C.CollisionConvex || !SameHull(*C.CollisionConvex,*R.CollisionConvex,Tolerance,Data->Settings.RelativeGeometryTolerance)) throw std::invalid_argument("Destruction cache rest geometry changed");
			}
			for (size_t I = 0; I < Source.mHierarchyNodes.size(); ++I)
			{
				const auto& N = Source.mHierarchyNodes[I]; const auto& R = Reference->mHierarchyNodes[I];
				const float Tolerance = std::max(Data->Settings.MinimumGeometryTolerance,R.Bounds.MaxDim()*Data->Settings.RelativeGeometryTolerance);
				if (N.Parent != R.Parent || N.Level != R.Level || N.Children != R.Children || N.ChunkIndices != R.ChunkIndices
					|| !Near(N.Bounds.Min,R.Bounds.Min,Tolerance) || !Near(N.Bounds.Max,R.Bounds.Max,Tolerance) || N.BreakThreshold != R.BreakThreshold || N.IsBreakable != R.IsBreakable)
					throw std::invalid_argument("Destruction cache hierarchy authoring changed");
			}
			std::map<int,std::pair<int,int>> Identities;
			for (size_t I = 0; I < Source.mGraph.Size(); ++I)
			{
				const auto& N = Source.mGraph[I];
				if (!std::isfinite(N.Strength) || N.Strength < 0) throw std::invalid_argument("Invalid cached graph strength");
				std::set<int> Neighbors;
				for (const auto& B : N.Bonds)
				{
					const int Other = B.GetOther((int)I); const auto* R = Reference->mGraph.FindBond((int)I,Other);
					const float Tolerance = std::max(Data->Settings.MinimumGeometryTolerance,Reference->mChunks[I].Bounds.MaxDim()*Data->Settings.RelativeGeometryTolerance);
					if (B.v0 != (int)I || B.v1 != Other || Other == (int)I || !Neighbors.insert(Other).second || !R || B.type != R->type
						|| !NearFrame(B.frame,R->frame,Tolerance,Data->Settings.RelativeGeometryTolerance)
						|| B.bond_index < -1 || (R->bond_index >= 0 && B.bond_index != R->bond_index) || !std::isfinite(B.strength) || B.strength < 0)
						throw std::invalid_argument("Cached bonds must retain authored identity and geometry");
					if (B.bond_index >= 0)
					{
						const std::pair<int,int> Pair = std::minmax((int)I,Other); auto Found = Identities.emplace(B.bond_index,Pair);
						if (!Found.second && Found.first->second != Pair) throw std::invalid_argument("Ambiguous cached bond identity");
					}
				}
			}
		}
		DestructionEventStats ValidateEvents(const DestructionSet& Source, const std::vector<DestructionEvent>& Events) const
		{
			if (!Data->Settings.RecordEvents) { if (!Events.empty()) throw std::invalid_argument("This destruction cache does not record events"); return {}; }
			const auto Current = Source.GetEventStats();
			if (Source.GetEventRevision() != EventRevision || Current.Pending || Current.Dropped != LastEvents.Dropped || Current.Invalid != LastEvents.Invalid
				|| Current.Generated < LastEvents.Generated || Current.Filtered < LastEvents.Filtered || Accepted(Current) < Accepted(LastEvents)
				|| Accepted(Current)-Accepted(LastEvents) != Events.size()) throw std::invalid_argument("Destruction cache event batch is incomplete or its stream changed");
			uint64_t Sequence = LastSequence;
			for (const auto& E : Events)
			{
				if (E.InstanceId != Data->SourceInstance || E.Generation != Data->SourceGeneration || E.AssetFingerprint != Data->Asset.GetStats().Fingerprint
					|| E.Sequence <= Sequence || !Source.mNextEventSequence || E.Sequence >= Source.mNextEventSequence
					|| (uint32_t)E.Kind >= (uint32_t)DestructionEventKind::Count || (uint32_t)E.Cause > (uint32_t)DestructionEventCause::Field
					|| !CacheFinite(E.Position) || !CacheFinite(E.Normal) || !CacheFinite(E.Impulse) || !std::isfinite(E.DeltaTime) || E.DeltaTime < 0
					|| E.SubstepCount < 1 || E.SubstepIndex < 0 || E.SubstepIndex >= E.SubstepCount) throw std::invalid_argument("Invalid cached destruction event");
				for (const auto& C : E.Chunks)
				{
					if (C.ChunkIndex < 0 || (size_t)C.ChunkIndex >= Source.mChunks.size() || !CacheFinite(C.LinearVelocity) || !CacheFinite(C.AngularVelocity)
						|| !std::isfinite(C.Mass) || C.Mass <= 0) throw std::invalid_argument("Invalid cached event chunk");
					RequirePose(C.Pose,Data->Settings.QuaternionTolerance);
				}
				std::set<int> FragmentChunks;
				for (const auto& F : E.Fragments) for (int I : F.ChunkIndices)
					if (I < 0 || (size_t)I >= Source.mChunks.size() || !FragmentChunks.insert(I).second) throw std::invalid_argument("Invalid cached event fragment");
				Sequence = E.Sequence;
			}
			return Current;
		}
		DestructionCacheFrame Read(const DestructionSet& Source) const
		{
			DestructionCacheFrame F; auto T = std::make_shared<DestructionCacheTopology>(); F.Topology = T;
			F.SourceWorldTick = World ? World->GetTick() : 0; F.Chunks.reserve(Source.mChunks.size());
			for (size_t I = 0; I < Source.mChunks.size(); ++I)
			{
				const auto& C = Source.mChunks[I]; DestructionEventChunk S; S.ChunkIndex = (int)I; S.Mass = C.Mass;
				S.IsStatic = C.IsStatic; S.IsFree = C.IsFree; S.IsSmashed = C.IsSmashed;
				S.Pose = Transform(Placement.WorldToLocal(C.Centroid));
				if (!Data->Frames.empty() && C.IsFree) S.Pose = Data->Frames.back().Chunks[I].Pose;
				F.Chunks.push_back(S);
			}
			std::vector<bool> Seen(Source.mChunks.size(),false);
			for (const auto& Entry : Source.mClusters)
			{
				const auto& C = Entry.second; DestructionCacheCluster Group; Group.SourceId = Entry.first; Group.Level = C.GetLevel(); Group.IsStatic = C.IsStatic();
				if (!C.GetRigidBody()) throw std::invalid_argument("Record destruction clusters with live world bodies");
				const auto& Indices = C.GetSourceIndices(); const auto& Locals = C.GetLocalTransforms();
				if (Indices.empty() || Indices.size() != Locals.size()) throw std::invalid_argument("Cached cluster has incomplete local transforms");
				std::vector<size_t> Order(Indices.size()); std::iota(Order.begin(),Order.end(),0);
				std::sort(Order.begin(),Order.end(),[&](size_t A,size_t B) { return Indices[A] < Indices[B]; });
				const int Anchor = Indices[Order.front()]; Transform AnchorLocal = Locals[Order.front()];
				const Transform AnchorWorld = C.GetClusterWorldTransform()*AnchorLocal;
				RequirePose(AnchorWorld,Data->Settings.QuaternionTolerance); RequirePose(AnchorLocal,Data->Settings.QuaternionTolerance);
				AnchorLocal.quat = AnchorLocal.quat.Unit(); DestructionCacheClusterState State; State.Pose = CanonicalPose(AnchorWorld,Placement);
				if (const auto* Body = C.GetRigidBody())
				{
					State.AngularVelocity = Placement.WorldToLocalDirection(Body->W);
					State.LinearVelocity = Placement.WorldToLocalDirection(Body->V+Body->W.Cross(AnchorWorld.pos-Body->X));
					State.FrictionDynamic = Body->GetFrictionDynamic(); State.FrictionStatic = Body->GetFrictionStatic(); State.Restitution = Body->GetRestitution();
					State.Sleeping = Body->Sleeping; State.Frozen = Body->Freezing;
				}
				if (!CacheFinite(State.LinearVelocity) || !CacheFinite(State.AngularVelocity) || !std::isfinite(State.FrictionDynamic) || State.FrictionDynamic < 0
					|| !std::isfinite(State.FrictionStatic) || State.FrictionStatic < 0 || !std::isfinite(State.Restitution) || State.Restitution < 0)
					throw std::invalid_argument("Invalid cached cluster state");
				// Index the parallel source arrays once; repeated chunk-ID lookups
				// would make capture quadratic for a large intact cluster.
				for (size_t Offset : Order)
				{
					const int I = Indices[Offset];
					if (I < 0 || (size_t)I >= F.Chunks.size() || Seen[I] || F.Chunks[I].IsFree) throw std::invalid_argument("Invalid cached cluster membership");
					Seen[I] = true; Transform Local = Locals[Offset]; Group.ChunkIndices.push_back(I);
					RequirePose(Local,Data->Settings.QuaternionTolerance); Local.quat = Local.quat.Unit();
					Local = AnchorLocal.TransformInv(Local); Local.pos = Placement.WorldToLocalDirection(Local.pos);
					Local.quat = (Placement.quat.Conjugate()*Local.quat*Placement.quat).Unit();
					if (I == Anchor) Local = Transform::Identity(); Group.LocalPoses.push_back(Local);
					auto& S = F.Chunks[I]; S.ClusterId = Entry.first; S.HasRigidBody = C.GetRigidBody() != nullptr; S.Pose = State.Pose*Local;
					S.LinearVelocity = State.LinearVelocity+State.AngularVelocity.Cross(S.Pose.pos-State.Pose.pos); S.AngularVelocity = State.AngularVelocity;
					RequirePose(S.Pose,Data->Settings.QuaternionTolerance);
					if (!CacheFinite(S.LinearVelocity)) throw std::overflow_error("Cached chunk velocity overflow");
				}
				T->Clusters.push_back(std::move(Group)); F.Clusters.push_back(State);
			}
			for (size_t I = 0; I < Seen.size(); ++I) if (!Seen[I] && !F.Chunks[I].IsFree) throw std::invalid_argument("Active cached chunk has no cluster");
			T->Graph = Source.mGraph;
			for (auto& N : T->Graph.GetBondLists())
			{
				for (auto& B : N.Bonds)
				{
					B.frame.Origin = Placement.WorldToLocal(B.frame.Origin); B.frame.Normal = Placement.WorldToLocalDirection(B.frame.Normal);
					B.frame.Tangent = Placement.WorldToLocalDirection(B.frame.Tangent); B.frame.Binormal = Placement.WorldToLocalDirection(B.frame.Binormal);
				}
				std::sort(N.Bonds.begin(),N.Bonds.end(),[](const Bond& A,const Bond& B) { return std::tie(A.v0,A.v1,A.bond_index) < std::tie(B.v0,B.v1,B.bond_index); });
			}
			T->Graph.RefreshBondCache(); for (const auto& N : Source.mHierarchyNodes) T->BrokenHierarchy.push_back(N.IsBroken);
			return F;
		}
	};
	DestructionCacheRecorder::DestructionCacheRecorder(const DestructionSet& Source, const DestructionCacheSettings& Settings) : m_Data(new Storage)
	{
		if (!Settings.IsValid() || !Source.GetAuthoredAsset() || !Source.GetPhysicsWorld() || !Source.mConstants.IsValid()) throw std::invalid_argument("Destruction caching requires valid settings and a live prepared asset instance");
		if (Source.GetPhysicsWorld() && Source.GetPhysicsWorld()->IsSimulating()) throw std::logic_error("Begin destruction recording between world steps");
		auto& S = *m_Data; S.Data = std::make_shared<DestructionCache::Storage>(); S.Data->Asset = *Source.GetAuthoredAsset(); S.Data->Settings = Settings;
		S.Data->Constants = Source.mConstants; S.Data->EventSettings = Source.GetEventSettings(); S.Data->SourceInstance = Source.GetInstanceId(); S.Data->SourceGeneration = Source.GetGeneration();
		S.Placement = Source.GetAssetPlacement(); S.World = Source.GetPhysicsWorld(); S.EventRevision = Source.GetEventRevision(); S.LastEvents = Source.GetEventStats();
		if (Settings.RecordEvents && (!Source.GetEventSettings().Enabled || S.LastEvents.Pending || !Source.mNextEventSequence)) throw std::invalid_argument("Begin recording with an enabled, drained event stream");
		S.LastSequence = Source.mNextEventSequence ? Source.mNextEventSequence-1 : 0;
		S.Reference.reset(new DestructionSet); S.Reference->InitializeAsset(S.Data->Asset,S.Placement); Record(0,Source);
	}
	DestructionCacheRecorder::~DestructionCacheRecorder() = default;
	void DestructionCacheRecorder::Record(double Time, const DestructionSet& Source, const std::vector<DestructionEvent>& Events)
	{
		if (!m_Data) throw std::logic_error("Destruction recording has finished");
		auto& S = *m_Data; auto& D = *S.Data;
		if (!std::isfinite(Time) || Time < 0 || (!D.Frames.empty() && Time <= D.Frames.back().Time)) throw std::invalid_argument("Cache capture times must increase strictly from zero");
		if (D.Frames.size() >= D.Settings.MaximumFrames) throw std::length_error("Destruction cache frame limit exceeded");
		S.ValidateSource(Source); const auto EventsNow = S.ValidateEvents(Source,Events); auto F = S.Read(Source); F.Time = Time;
		if (!D.Frames.empty() && F.SourceWorldTick < D.Frames.back().SourceWorldTick) throw std::invalid_argument("Destruction cache world clock regressed");
		bool NewTopology = D.Frames.empty() || !SameTopology(*D.Frames.back().Topology,*F.Topology);
		if (!NewTopology) F.Topology = D.Frames.back().Topology;
		F.Events = Events; for (auto& E : F.Events) { TransformEvent(E,S.Placement,true); RequireEventGeometry(E,D.Settings.QuaternionTolerance); }
		PayloadCounter Bytes(D.Settings.MaximumPayloadBytes); Bytes.Add(D.Stats.PayloadBytes,1);
		Bytes.Add(1,sizeof(F)); Bytes.Add(F.Chunks.size(),sizeof(DestructionEventChunk)); Bytes.Add(F.Clusters.size(),sizeof(DestructionCacheClusterState));
		for (const auto& E : F.Events) Bytes.Event(E); if (NewTopology) Bytes.Topology(*F.Topology);
		D.Frames.push_back(std::move(F)); D.Stats.FrameCount = D.Frames.size(); D.Stats.TopologyCount += NewTopology;
		D.Stats.EventCount += Events.size(); D.Stats.PayloadBytes = Bytes.Size; D.Stats.Duration = Time;
		S.LastEvents = EventsNow; S.LastSequence = Source.mNextEventSequence ? Source.mNextEventSequence-1 : 0;
	}
	DestructionCacheStats DestructionCacheRecorder::GetStats() const
	{ if (!m_Data) throw std::logic_error("Destruction recording has finished"); return m_Data->Data->Stats; }
	DestructionCache DestructionCacheRecorder::Finish()
	{ if (!m_Data) throw std::logic_error("Destruction recording has finished"); DestructionCache C; C.m_Data = m_Data->Data; m_Data.reset(); return C; }
	const DestructionCache::Storage& DestructionCache::Data() const
	{ if (!m_Data) throw std::logic_error("Unprepared destruction cache"); return *m_Data; }
	const DestructionAsset& DestructionCache::GetAsset() const { return Data().Asset; }
	const DestructionConstants& DestructionCache::GetConstants() const { return Data().Constants; }
	const DestructionCacheSettings& DestructionCache::GetSettings() const { return Data().Settings; }
	const DestructionEventSettings& DestructionCache::GetEventSettings() const { return Data().EventSettings; }
	const DestructionCacheStats& DestructionCache::GetStats() const { return Data().Stats; }
	const DestructionCacheFrame& DestructionCache::GetFrame(size_t Index) const { return Data().Frames.at(Index); }
	uint64_t DestructionCache::GetSourceInstanceId() const { return Data().SourceInstance; }
	uint64_t DestructionCache::GetSourceGeneration() const { return Data().SourceGeneration; }
	DestructionCacheSample DestructionCache::Sample(double Time, const Transform& Placement) const
	{
		const auto& D = Data(); RequirePose(Placement,D.Settings.QuaternionTolerance);
		if (!std::isfinite(Time)) throw std::invalid_argument("Invalid destruction sample time");
		DestructionCacheSample Out; Out.Time = std::max(0.0,std::min(Time,D.Stats.Duration));
		const auto Next = std::upper_bound(D.Frames.begin(),D.Frames.end(),Out.Time,[](double T,const DestructionCacheFrame& F) { return T < F.Time; });
		Out.LowerFrame = (size_t)(Next-D.Frames.begin()-1); Out.UpperFrame = std::min(Out.LowerFrame+1,D.Frames.size()-1);
		const auto& A = D.Frames[Out.LowerFrame]; const auto& B = D.Frames[Out.UpperFrame];
		Out.Interpolation = B.Time > A.Time ? (float)((Out.Time-A.Time)/(B.Time-A.Time)) : 0;
		Out.Topology = A.Topology; Out.Chunks = A.Chunks; Out.Clusters = A.Clusters;
		for (size_t I = 0; I < A.Topology->Clusters.size(); ++I)
		{
			const auto& G = A.Topology->Clusters[I]; const int Anchor = G.ChunkIndices.front(); const auto& Start = A.Chunks[Anchor]; const auto& End = B.Chunks[Anchor];
			const float Alpha = End.IsFree ? 0 : Out.Interpolation; auto& C = Out.Clusters[I];
			C.Pose.pos = Start.Pose.pos*(1-Alpha)+End.Pose.pos*Alpha; C.Pose.quat = Quaternion::Slerp(Start.Pose.quat,End.Pose.quat,Alpha).Unit();
			C.LinearVelocity = Start.LinearVelocity*(1-Alpha)+End.LinearVelocity*Alpha; C.AngularVelocity = Start.AngularVelocity*(1-Alpha)+End.AngularVelocity*Alpha;
			for (size_t J = 0; J < G.ChunkIndices.size(); ++J)
			{
				auto& Chunk = Out.Chunks[G.ChunkIndices[J]]; Chunk.Pose = C.Pose*G.LocalPoses[J];
				Chunk.AngularVelocity = C.AngularVelocity; Chunk.LinearVelocity = C.LinearVelocity+C.AngularVelocity.Cross(Chunk.Pose.pos-C.Pose.pos);
			}
		}
		for (auto& C : Out.Chunks)
		{
			C.Pose = PlacedPose(C.Pose,Placement); C.LinearVelocity = Placement.LocalToWorldDirection(C.LinearVelocity); C.AngularVelocity = Placement.LocalToWorldDirection(C.AngularVelocity);
			RequirePose(C.Pose,D.Settings.QuaternionTolerance); if (!CacheFinite(C.LinearVelocity) || !CacheFinite(C.AngularVelocity)) throw std::overflow_error("Placed cache velocity overflow");
		}
		for (auto& C : Out.Clusters)
		{ C.Pose = PlacedPose(C.Pose,Placement); C.LinearVelocity = Placement.LocalToWorldDirection(C.LinearVelocity); C.AngularVelocity = Placement.LocalToWorldDirection(C.AngularVelocity); }
		return Out;
	}
	std::vector<DestructionCacheEvent> DestructionCache::GetEvents(double After, double Through, const Transform& Placement) const
	{
		const auto& D = Data(); RequirePose(Placement,D.Settings.QuaternionTolerance);
		if (!std::isfinite(After) || !std::isfinite(Through) || After > Through) throw std::invalid_argument("Invalid destruction event time interval");
		std::vector<DestructionCacheEvent> Out;
		auto First = std::upper_bound(D.Frames.begin(),D.Frames.end(),After,[](double T,const DestructionCacheFrame& F) { return T < F.Time; });
		for (; First != D.Frames.end() && First->Time <= Through; ++First) for (const auto& Source : First->Events)
		{
			auto E = Source; TransformEvent(E,Placement,false);
			RequireEventGeometry(E,D.Settings.QuaternionTolerance);
			Out.push_back({First->Time,std::move(E)});
		}
		return Out;
	}
}


namespace Riemann
{
	/* World playback of evaluated destruction trajectories
	   ---------------------------------------------------
	   The cache supplies topology and rigid cluster poses; playback never executes
	   fracture or restores solver state. It evaluates a recorded trajectory.
	   Each recorded cluster becomes one infinite-mass kinematic compound, anchored
	   at its first chunk. This preserves rigid motion without integrating it twice.
	   Placement rotation is already baked into the private rest hulls. Sampling
	   supplies geometry-world poses in that basis; compound construction converts
	   those to anchor-local poses. Kinematics retain zero inverse mass and CM = 0.

	   The world advances players BEFORE acquiring substep body/contact references.
	   Topology replacement stages new compounds, then unregisters and deletes old
	   ones. Bodies belong to the world while registered; reset/destruction detaches
	   players before disposing of scene state. Individual external body removal is
	   observed too, invalidating that binding before the caller may delete it.

	   Seek is a teleport: it pauses, clears pending events, advances the replay epoch
	   and invalidates contact caches. Pause and the clip endpoint zero velocities.
	   Forward steps deliver capture-boundary events in (previous time, current time].
	   Source event identity remains intact inside an explicit playback-id/epoch
	   wrapper. Bounded queues drop newest events and expose a cumulative loss count.
	   Recorded velocities are transformed/scaled with playback rate; they are not
	   impulses to replay. Cached bodies may push live bodies, but cannot be fractured
	   by them. Dynamic handoff requires a separate state-transfer contract.
	   Contact response uses evaluated poses. Swept kinematic target trajectories
	   are not supplied by this driver; live dynamic CCD sees the evaluated hulls.
	*/
	namespace { std::atomic<uint64_t> NextPlaybackId{1}; }
	bool DestructionPlaybackSettings::IsValid() const
	{
		return std::isfinite(Rate) && Rate > 0 && Rate <= std::numeric_limits<float>::max() && MaximumQueuedEvents > 0;
	}
	DestructionPlayback::DestructionPlayback(PhysicsWorld& World, const DestructionCache& Cache,
		const Transform& Placement, const DestructionPlaybackSettings& Settings)
		: m_World(&World), m_Cache(Cache), m_Placement(Placement), m_Settings(Settings)
	{
		RequireIdle();
		if (!Settings.IsValid()) throw std::invalid_argument("Invalid destruction playback settings");
		auto Initial = Cache.Sample(0,Placement);
		m_Rest.reset(new DestructionSet);
		m_Rest->InitializeAsset(Cache.GetAsset(),Placement);
		m_InstanceId = NextPlaybackId.fetch_add(1,std::memory_order_relaxed);
		try
		{
			Apply(std::move(Initial),false);
			World.m_DestructionPlaybacks.push_back(this);
		}
		catch (...) { ClearBodies(m_Bodies); throw; }
	}
	DestructionPlayback::~DestructionPlayback() { Detach(); }
	void DestructionPlayback::RequireIdle() const
	{
		if (m_World && m_World->IsSimulating()) throw std::logic_error("Edit destruction playback between world steps");
	}
	void DestructionPlayback::ClearBodies(std::vector<Binding>& Bodies)
	{
		std::vector<Binding> Removed; Removed.swap(Bodies);
		for (auto& Entry : Removed) if (Entry.Body)
		{
			m_World->RemoveRigidBody(Entry.Body);
			Entry.Body->ReleaseGeometries(); delete Entry.Body;
		}
	}
	void DestructionPlayback::Detach()
	{
		RequireIdle();
		if (!m_World) return;
		m_Playing = false; ClearBodies(m_Bodies);
		auto& Players = m_World->m_DestructionPlaybacks;
		Players.erase(std::remove(Players.begin(),Players.end(),this),Players.end());
		m_World = nullptr; m_Geometries.clear(); m_Topology.reset(); m_Events.clear(); ++m_Revision;
	}
	Geometry* DestructionPlayback::GetChunkGeometry(int Chunk) const
	{
		return Chunk >= 0 && (size_t)Chunk < m_Geometries.size() ? m_Geometries[Chunk] : nullptr;
	}
	RigidBody* DestructionPlayback::GetBody(size_t Index) const
	{
		return Index < m_Bodies.size() ? m_Bodies[Index].Body : nullptr;
	}
	void DestructionPlayback::StopVelocities()
	{
		for (auto& Entry : m_Bodies) if (Entry.Body) Entry.Body->V = Entry.Body->W = Vector3::Zero();
	}
	void DestructionPlayback::SetPlaying(bool Playing)
	{
		RequireIdle();
		if (!m_World) throw std::logic_error("Destruction playback is detached");
		m_Playing = Playing && GetTime() < m_Cache.GetStats().Duration;
		if (!m_Playing) StopVelocities();
	}
	void DestructionPlayback::Seek(double Time)
	{
		RequireIdle();
		if (!m_World) throw std::logic_error("Destruction playback is detached");
		auto Sample = m_Cache.Sample(Time,m_Placement);
		Apply(std::move(Sample),false);
		m_Playing = false; m_Events.clear(); ++m_Epoch;
		m_World->RefreshCollisionFiltering();
		m_World->SynchronizeSceneQueries();
	}
	std::vector<DestructionPlaybackEvent> DestructionPlayback::TakeEvents()
	{
		RequireIdle(); std::vector<DestructionPlaybackEvent> Result; Result.swap(m_Events); return Result;
	}
	void DestructionPlayback::OnBodyRemoved(RigidBody* Body)
	{
		for (auto& Entry : m_Bodies) if (Entry.Body == Body)
		{
			for (int Chunk : Entry.Chunks) m_Geometries[Chunk] = nullptr;
			Entry.Body = nullptr; m_Topology.reset(); m_Playing = false; StopVelocities(); ++m_Revision;
		}
	}
	void DestructionPlayback::Rebuild(const DestructionCacheSample& Sample)
	{
		std::vector<const DestructionEventChunk*> Chunks(m_Rest->GetChunks().size(),nullptr);
		for (const auto& Chunk : Sample.Chunks) Chunks[Chunk.ChunkIndex] = &Chunk;
		std::vector<Binding> Prepared;
		std::vector<Geometry*> Geometries(Chunks.size(),nullptr);
		Prepared.reserve(Sample.Topology->Clusters.size());
		try
		{
			for (size_t I = 0; I < Sample.Topology->Clusters.size(); ++I)
			{
				const auto& Cluster = Sample.Topology->Clusters[I];
				std::vector<std::unique_ptr<Geometry>> Owned;
				std::vector<Geometry*> Shapes;
				for (int Chunk : Cluster.ChunkIndices)
				{
					const auto& Hull = *m_Rest->GetChunks()[Chunk].CollisionConvex;
					std::unique_ptr<Geometry> Shape(GeometryFactory::CreateConvexMesh());
					Shape->GetShapeObj<ConvexMesh>()->SetConvexData(Hull.Vertices,Hull.NumVertices,Hull.Faces,Hull.NumFaces,
						nullptr,0,Hull.Indices,Hull.NumIndices,false);
					Shape->UpdateVolumeProperties();
					Shape->SetTransform(Chunks[Chunk]->Pose.pos,Chunks[Chunk]->Pose.quat);
					Shape->SetFilterData(m_Cache.GetConstants().FilterData);
					Shape->SetContactOffsets(m_Cache.GetConstants().ContactOffset,m_Cache.GetConstants().RestOffset);
					Shape->SetQueryEnabled(m_Settings.QueryEnabled); Shape->SetSimulationEnabled(m_Settings.CollisionEnabled);
					Geometries[Chunk] = Shape.get(); Shapes.push_back(Shape.get()); Owned.push_back(std::move(Shape));
				}
				Prepared.push_back({nullptr,Cluster.ChunkIndices});
				RigidBodyParam P; P.rigidType = RigidType::Kinematic;
				auto* Body = m_World->CreateRigidBody(Shapes,P,Sample.Clusters[I].Pose);
				Prepared.back().Body = Body;
				for (auto& Shape : Owned) Shape.release();
				Body->Parent = this; Body->CollisionGroup = this; Body->EnableCollisionWithinGroup = false; Body->SetGuid(Cluster.SourceId);
			}
		}
		catch (...) { ClearBodies(Prepared); throw; }
		ClearBodies(m_Bodies); m_Bodies.swap(Prepared); m_Geometries.swap(Geometries);
		m_Topology = Sample.Topology; ++m_Revision;
	}
	void DestructionPlayback::Apply(DestructionCacheSample Sample, bool Moving)
	{
		if (Sample.Topology != m_Topology) Rebuild(Sample);
		for (size_t I = 0; I < m_Bodies.size(); ++I)
		{
			auto* Body = m_Bodies[I].Body; const auto& State = Sample.Clusters[I];
			Body->X = State.Pose.pos; Body->Q = State.Pose.quat;
			Body->V = Moving ? State.LinearVelocity * (float)m_Settings.Rate : Vector3::Zero();
			Body->W = Moving ? State.AngularVelocity * (float)m_Settings.Rate : Vector3::Zero();
			Body->SetFriction(State.FrictionDynamic,State.FrictionStatic); Body->SetRestitution(State.Restitution);
		}
		m_Sample = std::move(Sample);
		m_World->SynchronizeSceneQueries();
	}
	void DestructionPlayback::Step(float Dt)
	{
		if (!m_Playing) return;
		const double Next = std::min(m_Cache.GetStats().Duration,GetTime()+Dt*m_Settings.Rate);
		auto Events = m_Cache.GetEvents(GetTime(),Next,m_Placement);
		Apply(m_Cache.Sample(Next,m_Placement),true);
		for (auto& Event : Events)
		{
			if (m_Events.size() == m_Settings.MaximumQueuedEvents) { ++m_DroppedEvents; continue; }
			m_Events.push_back({m_InstanceId,m_Epoch,std::move(Event)});
		}
		if (Next == m_Cache.GetStats().Duration) { m_Playing = false; StopVelocities(); }
	}
}
