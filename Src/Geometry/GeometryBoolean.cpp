#include "GeometryBoolean.h"

#include <assert.h>
#include <functional>
#include <map>
#include <set>
#include <vector>

#include "../CollisionPrimitive/Segment3.h"
#include "../CollisionPrimitive/Triangle3.h"
#include "../Core/Base.h"
#include "../Core/DynamicArray.h"
#include "../Core/PriorityQueue.h"
#include "../Maths/Maths.h"
#include "DynamicMesh.h"
#include "SparseOctree.h"
#include "SparseSpatialHash.h"
#include "WindingNumber.h"

namespace Riemann
{
	namespace
	{

	enum class VertexLocationType
	{
		Unknown = -1,
		Vertex = 0,
		Edge = 1,
		Face = 2
	};

	struct PointOnMesh
	{
		Vector3 Pos;
		VertexLocationType Type = VertexLocationType::Unknown;
		int ElemID = -1;
	};

	struct SegmentElements
	{
		int BaseTID;
		int PtOnMeshIdx[2];
	};

	enum class SurfacePointType
	{
		Vertex = 0,
		Edge = 1,
		Triangle = 2
	};

	struct MeshSurfacePoint
	{
		int ElementID;
		Vector3 BaryCoord;
		SurfacePointType Point;

		MeshSurfacePoint() : ElementID(-1)
		{
		}
		MeshSurfacePoint(int TriangleID, const Vector3& BaryCoord) : ElementID(TriangleID), BaryCoord(BaryCoord), Point(SurfacePointType::Triangle)
		{
		}
		MeshSurfacePoint(int EdgeID, float FirstCoordWt) : ElementID(EdgeID), BaryCoord(FirstCoordWt, 1 - FirstCoordWt, 0), Point(SurfacePointType::Edge)
		{
		}
		MeshSurfacePoint(int VertexID) : ElementID(VertexID), BaryCoord(1, 0, 0), Point(SurfacePointType::Vertex)
		{
		}

		Vector3 Pos(const DynamicMesh* Mesh) const
		{
			if (Point == SurfacePointType::Vertex)
			{
				return Mesh->GetVertex(ElementID);
			}
			else if (Point == SurfacePointType::Edge)
			{
				Vector3 EA, EB;
				Mesh->GetEdgeV(ElementID, EA, EB);
				return BaryCoord[0] * EA + BaryCoord[1] * EB;
			}
			else
			{
				Vector3 TA, TB, TC;
				Mesh->GetTriVertices(ElementID, TA, TB, TC);
				return BaryCoord[0] * TA + BaryCoord[1] * TB + BaryCoord[2] * TC;
			}
		}
	};

	class MeshSurfacePath
	{
	public:
		DynamicMesh* Mesh;
		std::vector<std::pair<MeshSurfacePoint, int>> Path;
		bool Closed;

	public:
		MeshSurfacePath(DynamicMesh* Mesh) : Mesh(Mesh), Closed(false)
		{
		}
		virtual ~MeshSurfacePath() {}

		bool IsConnected() const
		{
			int Idx = 1, LastIdx = 0;
			if (Closed)
			{
				LastIdx = (int)Path.size() - 1;
				Idx = 0;
			}
			for (; Idx < Path.size(); LastIdx = Idx++)
			{
				int WalkingOnTri = Path[LastIdx].second;
				if (!Mesh->IsTriangle(WalkingOnTri))
				{
					return false;
				}
				int Inds[2] = { LastIdx, Idx };
				for (int IndIdx = 0; IndIdx < 2; IndIdx++)
				{
					const MeshSurfacePoint& P = Path[Inds[IndIdx]].first;
					switch (P.Point)
					{
					case SurfacePointType::Triangle:
						if (P.ElementID != WalkingOnTri)
						{
							return false;
						}
						break;
					case SurfacePointType::Edge:
						if (!Mesh->GetEdgeT(P.ElementID).Contains(WalkingOnTri))
						{
							return false;
						}
						break;
					case SurfacePointType::Vertex:
						if (!Mesh->GetTriangle(WalkingOnTri).Contains(P.ElementID))
						{
							return false;
						}
						break;
					}
				}
			}
			return true;
		}

		bool IsClosed() const
		{
			return Closed;
		}

		void Reset()
		{
			Path.clear();
			Closed = false;
		}

		bool AddViaPlanarWalk(int StartTri, int StartVID, Vector3 StartPt, int EndTri, int EndVertID, Vector3 EndPt,
			Vector3 WalkPlaneNormal, std::function<Vector3(const DynamicMesh*, int)> VertexToPosnFn = nullptr,
			bool AllowBackwardsSearch = true, float AcceptEndPtOutsideDist = 1e-6f,
			float PtOnPlaneThresholdSq = 1e-4f, float BackwardsTolerance = 1e-5f)
		{
			if (!VertexToPosnFn)
			{
				VertexToPosnFn = [](const DynamicMesh* MeshArg, int VertexID)
				{
					return MeshArg->GetVertex(VertexID);
				};
			}
			return WalkMeshPlanar(Mesh, StartTri, StartVID, StartPt, EndTri, EndVertID, EndPt, WalkPlaneNormal, VertexToPosnFn,
				AllowBackwardsSearch, AcceptEndPtOutsideDist, PtOnPlaneThresholdSq, Path, BackwardsTolerance);
		}

		bool EmbedSimplePath(bool UpdatePath, std::vector<int>& PathVertices, bool DoNotDuplicateFirstVertexID = true, float SnapElementThresholdSq = 1e-4f)
		{
			int InitialPathIdx = (int)PathVertices.size();
			if (Path.empty())
			{
				return true;
			}

			int PathNum = (int)Path.size();
			const MeshSurfacePoint& OrigEndPt = Path[PathNum - 1].first;

			int StartProcessIdx = 0, EndSimpleProcessIdx = PathNum - 1;
			bool NeedsEndpointSpecialProcess = false;
			if (PathNum > 1 && OrigEndPt.Point == SurfacePointType::Triangle)
			{
				EndSimpleProcessIdx = PathNum - 2;
				NeedsEndpointSpecialProcess = true;
			}
			MeshSurfacePoint EndPtUpdated = Path.back().first;
			Vector3 EndPtPos = OrigEndPt.Pos(Mesh);

			if (Path[0].first.Point == SurfacePointType::Triangle)
			{
				PokeTriangleInfo PokeInfo;
				Mesh->PokeTriangle(Path[0].first.ElementID, Path[0].first.BaryCoord, PokeInfo);
				if (EndPtUpdated.Point == SurfacePointType::Triangle && Path[0].first.ElementID == EndPtUpdated.ElementID)
				{
					EndPtUpdated = RelocateTrianglePointAfterRefinement(Mesh, EndPtPos, { PokeInfo.NewTriangles.a, PokeInfo.NewTriangles.b, PokeInfo.OriginalTriangle }, SnapElementThresholdSq);
				}
				PathVertices.push_back(PokeInfo.NewVertex);
				StartProcessIdx = 1;
			}
			for (int PathIdx = StartProcessIdx; PathIdx <= EndSimpleProcessIdx; PathIdx++)
			{
				if (!(Path[PathIdx].first.Point != SurfacePointType::Triangle))
				{
					return false;
				}
				const MeshSurfacePoint& Pt = Path[PathIdx].first;
				if (Pt.Point == SurfacePointType::Edge)
				{
					assert(Mesh->IsEdge(Pt.ElementID));
					EdgeSplitInfo SplitInfo;
					Mesh->SplitEdge(Pt.ElementID, SplitInfo, Pt.BaryCoord[0]);
					PathVertices.push_back(SplitInfo.NewVertex);
					if (EndPtUpdated.Point == SurfacePointType::Triangle && SplitInfo.OriginalTriangles.Contains(EndPtUpdated.ElementID))
					{
						std::vector<int> TriInds = { EndPtUpdated.ElementID };
						if (SplitInfo.OriginalTriangles.a == EndPtUpdated.ElementID)
						{
							TriInds.push_back(SplitInfo.NewTriangles.a);
						}
						else
						{
							TriInds.push_back(SplitInfo.NewTriangles.b);
						}
						EndPtUpdated = RelocateTrianglePointAfterRefinement(Mesh, EndPtPos, TriInds, SnapElementThresholdSq);
					}
					else if (PathIdx != PathNum - 1 && EndPtUpdated.Point == SurfacePointType::Edge && Pt.ElementID == EndPtUpdated.ElementID)
					{
						assert(false);
					}
				}
				else
				{
					assert(Pt.Point == SurfacePointType::Vertex);
					assert(Mesh->IsVertex(Pt.ElementID));
					if (!DoNotDuplicateFirstVertexID || PathVertices.size() != InitialPathIdx || 0 == PathVertices.size() || PathVertices.back() != Pt.ElementID)
					{
						PathVertices.push_back(Pt.ElementID);
					}
				}
			}

			if (NeedsEndpointSpecialProcess)
			{
				if (EndPtUpdated.Point == SurfacePointType::Triangle)
				{
					PokeTriangleInfo PokeInfo;
					Mesh->PokeTriangle(EndPtUpdated.ElementID, EndPtUpdated.BaryCoord, PokeInfo);
					PathVertices.push_back(PokeInfo.NewVertex);
				}
				else if (EndPtUpdated.Point == SurfacePointType::Edge)
				{
					EdgeSplitInfo SplitInfo;
					Mesh->SplitEdge(EndPtUpdated.ElementID, SplitInfo, EndPtUpdated.BaryCoord[0]);
					PathVertices.push_back(SplitInfo.NewVertex);
				}
				else
				{
					if (PathVertices.size() == 0 || PathVertices.back() != EndPtUpdated.ElementID)
					{
						PathVertices.push_back(EndPtUpdated.ElementID);
					}
				}
			}

			if (UpdatePath)
			{
				assert(false);
			}

			return true;
		}

	private:
		struct IndexDistance
		{
			int Index;
			float PathLength;
			float DistanceToEnd;
			bool operator<(const IndexDistance& Other) const
			{
				return PathLength + DistanceToEnd < Other.PathLength + Other.DistanceToEnd;
			}
		};

		static bool WalkMeshPlanar(const DynamicMesh* Mesh, int StartTri, int StartVID, Vector3 StartPt, int EndTri, int EndVertID, Vector3 EndPt, Vector3 WalkPlaneNormal, std::function<Vector3(const DynamicMesh*, int)> VertexToPosnFn,
			bool AllowBackwardsSearch, float AcceptEndPtOutsideDist, float PtOnPlaneThresholdSq, std::vector<std::pair<MeshSurfacePoint, int>>& WalkedPath, float BackwardsTolerance)
		{
			auto SetTriVertPositions = [&VertexToPosnFn, &Mesh](Index3 TriVertIDs, Triangle3& Tri)
			{
				Tri.v0 = VertexToPosnFn(Mesh, TriVertIDs.a);
				Tri.v1 = VertexToPosnFn(Mesh, TriVertIDs.b);
				Tri.v2 = VertexToPosnFn(Mesh, TriVertIDs.c);
			};

			auto PtInsideTri = [](const Vector3& BaryCoord, float BaryThreshold = 1e-6f)
			{
				return BaryCoord[0] >= -BaryThreshold && BaryCoord[1] >= -BaryThreshold && BaryCoord[2] >= -BaryThreshold;
			};
            (void)PtInsideTri;

			struct WalkIndices
			{
				Vector3 Position;
				int WalkedFromPt;
				int	WalkingOnTri;

				WalkIndices() : WalkedFromPt(-1), WalkingOnTri(-1)
				{}

				WalkIndices(Vector3 Position, int FromPt, int OnTri) : Position(Position), WalkedFromPt(FromPt), WalkingOnTri(OnTri)
				{}
			};

			std::vector<std::pair<MeshSurfacePoint, WalkIndices>> ComputedPointsAndSources;
			PriorityQueue<IndexDistance> UnexploredEnds;

			std::set<int> ExploredTriangles, CrossedVertices;

			int BestKnownEnd = -1;

			Triangle3 CurrentTri;
			Index3 StartTriVertIDs = Mesh->GetTriangle(StartTri);
			SetTriVertPositions(StartTriVertIDs, CurrentTri);
			Triangle3::PointDistanceQueryResult CurrentTriDist;
			int StartVIDIndex = -1;
			if (StartVID != -1)
			{
				StartVIDIndex = StartTriVertIDs.IndexOf(StartVID);
			}
			if (StartVIDIndex == -1)
			{
				Triangle3::PointDistanceQueryResult info = CurrentTri.PointDistanceQuery(StartPt);
                (void)info;
				ComputedPointsAndSources.emplace_back(MeshSurfacePoint(StartTri, CurrentTriDist.BaryCoords), WalkIndices(StartPt, -1, StartTri));
			}
			else
			{
				CurrentTriDist.BaryCoords = Vector3::Zero();
				CurrentTriDist.BaryCoords[StartVIDIndex] = 1.0;
				CurrentTriDist.ClosestPoint = StartPt;
				ComputedPointsAndSources.emplace_back(MeshSurfacePoint(StartTri, CurrentTriDist.BaryCoords), WalkIndices(StartPt, -1, StartTri));
			}

			Vector3 ForwardsDirection = EndPt - StartPt;

			int CurrentEnd = 0;
			float CurrentPathLength = 0;
			float CurrentDistanceToEnd = ForwardsDirection.Length();

			UnexploredEnds.push({ 0, CurrentPathLength, CurrentDistanceToEnd });

			int IterCountSafety = 0;
			int NumTriangles = Mesh->GetTriangleCount();
			while (true)
			{
				if (!(IterCountSafety++ < NumTriangles * 2))
				{
					return false;
				}
				if (UnexploredEnds.size())
				{
					IndexDistance TopEndWithDistance = UnexploredEnds.pop();

					CurrentEnd = TopEndWithDistance.Index;
					CurrentPathLength = TopEndWithDistance.PathLength;
					CurrentDistanceToEnd = TopEndWithDistance.DistanceToEnd;
				}
				else
				{
					return false;
				}

				MeshSurfacePoint FromPt = ComputedPointsAndSources[CurrentEnd].first;
				WalkIndices CurrentWalk = ComputedPointsAndSources[CurrentEnd].second;
				int TriID = CurrentWalk.WalkingOnTri;
				assert(Mesh->IsTriangle(TriID));
				Index3 TriVertIDs = Mesh->GetTriangle(TriID);
				SetTriVertPositions(TriVertIDs, CurrentTri);

				if (EndVertID >= 0 && TriVertIDs.Contains(EndVertID))
				{
					int FromEnd = CurrentEnd;
					CurrentEnd = (int)ComputedPointsAndSources.size();
					ComputedPointsAndSources.emplace_back(MeshSurfacePoint(EndVertID), WalkIndices(EndPt, FromEnd, TriID));
					BestKnownEnd = CurrentEnd;
					break;
				}

				bool OnEndTri = EndTri == TriID;
				bool ComputedEndPtOnTri = false;
				if (EndVertID < 0 && EndTri == -1)
				{
					CurrentTriDist = CurrentTri.PointDistanceQuery(EndPt);
					ComputedEndPtOnTri = true;
					float DistSq = CurrentTriDist.SqrDistance;
					if (DistSq < AcceptEndPtOutsideDist)
					{
						OnEndTri = true;
					}
				}

				if (OnEndTri)
				{
					if (!ComputedEndPtOnTri)
					{
						ComputedEndPtOnTri = true;
						CurrentTriDist = CurrentTri.PointDistanceQuery(EndPt);
					}
					int FromEnd = CurrentEnd;
					CurrentEnd = (int)ComputedPointsAndSources.size();
					ComputedPointsAndSources.emplace_back(MeshSurfacePoint(TriID, CurrentTriDist.BaryCoords), WalkIndices(EndPt, FromEnd, TriID));

					BestKnownEnd = CurrentEnd;
					break;
				}

				if (ExploredTriangles.count(TriID) > 0)
				{
					continue;
				}
				ExploredTriangles.insert(TriID);

				float SignDist[3];
				int Side[3];
				int InitialComputedPointsNum = (int)ComputedPointsAndSources.size();
				for (int TriSubIdx = 0; TriSubIdx < 3; TriSubIdx++)
				{
					float SD = (CurrentTri[TriSubIdx] - StartPt).Dot(WalkPlaneNormal);
					SignDist[TriSubIdx] = SD;
					if (fabs(SD) <= PtOnPlaneThresholdSq)
					{
						// Cross through a vertex on the walking plane.
						Side[TriSubIdx] = 0;
						int CandidateVertID = TriVertIDs[TriSubIdx];
						if (FromPt.Point != SurfacePointType::Vertex || CandidateVertID != FromPt.ElementID)
						{
							MeshSurfacePoint SurfPt(CandidateVertID);
							WalkIndices WalkInds(CurrentTri[TriSubIdx], CurrentEnd, -1);
							bool IsForward = ForwardsDirection.Dot(CurrentTri[TriSubIdx] - StartPt) >= -BackwardsTolerance;
							if ((AllowBackwardsSearch || IsForward) && !CrossedVertices.count(CandidateVertID))
							{

								CrossedVertices.insert(CandidateVertID);

								std::vector<int> VtxTriangles = Mesh->GetVexTriangles(CandidateVertID);
								for (int NbrTriID : VtxTriangles)
								{
									if (NbrTriID != TriID)
									{
										Index3 NbrTriVertIDs = Mesh->GetTriangle(NbrTriID);
										Triangle3 NbrTri;
										SetTriVertPositions(NbrTriVertIDs, NbrTri);
										int SignsMultiplied = 1;
										for (int NbrTriSubIdx = 0; NbrTriSubIdx < 3; NbrTriSubIdx++)
										{
											if (NbrTriVertIDs[NbrTriSubIdx] == CandidateVertID)
											{
												continue;
											}
											float NbrSD = (NbrTri[NbrTriSubIdx] - StartPt).Dot(WalkPlaneNormal);
											int NbrSign = fabsf(NbrSD) <= PtOnPlaneThresholdSq ? 0 : NbrSD > 0 ? 1 : -1;
											SignsMultiplied *= NbrSign;
										}
										if (SignsMultiplied < 1)
										{
											WalkInds.WalkingOnTri = NbrTriID;
											ComputedPointsAndSources.emplace_back(SurfPt, WalkInds);
										}
									}
								}
							}
						}
					}
					else
					{
						Side[TriSubIdx] = SD > 0 ? 1 : -1;
					}
				}
				Index3 TriEdgeIDs = Mesh->GetTriEdges(TriID);
				for (int TriSubIdx = 0; TriSubIdx < 3; TriSubIdx++)
				{
					int NextSubIdx = (TriSubIdx + 1) % 3;
					if (Side[TriSubIdx] * Side[NextSubIdx] < 0)
					{
						// Cross an edge on the walking plane.
						int CandidateEdgeID = TriEdgeIDs[TriSubIdx];
						if (FromPt.Point != SurfacePointType::Edge || CandidateEdgeID != FromPt.ElementID)
						{
							float CrossingT = SignDist[TriSubIdx] / (SignDist[TriSubIdx] - SignDist[NextSubIdx]);
							Vector3 CrossingP = (1.0f - CrossingT) * CurrentTri[TriSubIdx] + CrossingT * CurrentTri[NextSubIdx];
							const DynamicMesh::Edge edge = Mesh->GetEdge(CandidateEdgeID);
							if (edge.Vert[0] != TriVertIDs[TriSubIdx])
							{
								CrossingT = 1 - CrossingT;
							}
							int CrossToTriID = edge.Tri[0];
							if (CrossToTriID == TriID)
							{
								CrossToTriID = edge.Tri[1];
							}
							if (CrossToTriID == -1)
							{
								continue;
							}
							bool IsForward = ForwardsDirection.Dot(CrossingP - StartPt) >= -BackwardsTolerance;
							if (!AllowBackwardsSearch && !IsForward)
							{
								continue;
							}
							ComputedPointsAndSources.emplace_back(MeshSurfacePoint(CandidateEdgeID, CrossingT), WalkIndices(CrossingP, CurrentEnd, CrossToTriID));
						}
					}
				}

				const Vector3& PreviousPathPoint = ComputedPointsAndSources[CurrentEnd].second.Position;
				for (int NewComputedPtIdx = InitialComputedPointsNum; NewComputedPtIdx < ComputedPointsAndSources.size(); NewComputedPtIdx++)
				{
					const Vector3& CurrentPathPoint = ComputedPointsAndSources[NewComputedPtIdx].second.Position;
					float PathLength = CurrentPathLength + (PreviousPathPoint - CurrentPathPoint).Length();
					float DistanceToEnd = (EndPt - CurrentPathPoint).Length();

					bool IsForward = ForwardsDirection.Dot(CurrentPathPoint - StartPt) >= -BackwardsTolerance;
					if (AllowBackwardsSearch || IsForward)
					{
						UnexploredEnds.push({ NewComputedPtIdx, PathLength, DistanceToEnd });
					}
				}
			}


			int TrackedPtIdx = BestKnownEnd;
			int SafetyIdxBacktrack = 0;
			std::vector<int> AcceptedIndices;
			while (TrackedPtIdx > -1)
			{
				if (!(SafetyIdxBacktrack++ < 2 * ComputedPointsAndSources.size()))
				{
					return false;
				}
				AcceptedIndices.push_back(TrackedPtIdx);
				TrackedPtIdx = ComputedPointsAndSources[TrackedPtIdx].second.WalkedFromPt;
			}
			WalkedPath.clear();
			for (int IdxIdx = (int)AcceptedIndices.size() - 1; IdxIdx >= 0; IdxIdx--)
			{
				WalkedPath.emplace_back(ComputedPointsAndSources[AcceptedIndices[IdxIdx]].first, ComputedPointsAndSources[AcceptedIndices[IdxIdx]].second.WalkingOnTri);
			}

			if (WalkedPath.size() && WalkedPath[0].first.Point == SurfacePointType::Triangle)
			{
				MeshSurfacePoint& SurfacePt = WalkedPath[0].first;

				if (StartVIDIndex > -1 && SurfacePt.BaryCoord[StartVIDIndex] == 1.0)
				{
					Index3 TriVertIDs = Mesh->GetTriangle(SurfacePt.ElementID);
					SurfacePt.ElementID = TriVertIDs[StartVIDIndex];
					SurfacePt.Point = SurfacePointType::Vertex;
				}
				else
				{
					RefineSurfacePtFromTriangleToSubElement(Mesh, SurfacePt.Pos(Mesh), SurfacePt, PtOnPlaneThresholdSq);
				}
				if (WalkedPath.size() > 1 &&
					SurfacePt.Point != SurfacePointType::Triangle &&
					SurfacePt.Point == WalkedPath[1].first.Point &&
					SurfacePt.ElementID == WalkedPath[1].first.ElementID)
				{
					if (SurfacePt.Point == SurfacePointType::Edge)
					{
						WalkedPath[1].first.BaryCoord = SurfacePt.BaryCoord;
					}
					WalkedPath.erase(WalkedPath.begin());
				}
			}
			if (WalkedPath.size() && WalkedPath.back().first.Point == SurfacePointType::Triangle)
			{
				MeshSurfacePoint& SurfacePt = WalkedPath.back().first;
				RefineSurfacePtFromTriangleToSubElement(Mesh, SurfacePt.Pos(Mesh), SurfacePt, PtOnPlaneThresholdSq);
				if (WalkedPath.size() > 1 &&
					SurfacePt.Point != SurfacePointType::Triangle &&
					SurfacePt.Point == WalkedPath[WalkedPath.size() - 2].first.Point &&
					SurfacePt.ElementID == WalkedPath[WalkedPath.size() - 2].first.ElementID)
				{
					if (SurfacePt.Point == SurfacePointType::Edge)
					{
						WalkedPath[WalkedPath.size() - 2].first.BaryCoord = SurfacePt.BaryCoord;
					}
					WalkedPath.pop_back();
				}
			}

			return true;
		}

		static void RefineSurfacePtFromTriangleToSubElement(const DynamicMesh* Mesh, Vector3 Pos, MeshSurfacePoint& SurfacePt, float SnapElementThresholdSq)
		{
			if (!(SurfacePt.Point == SurfacePointType::Triangle))
			{
				return;
			}
			int TriID = SurfacePt.ElementID;

			Index3 TriVertIDs = Mesh->GetTriangle(TriID);
			int BestSubIdx = -1;
			float BestElementDistSq = 0;
			for (int VertSubIdx = 0; VertSubIdx < 3; VertSubIdx++)
			{
				float DistSq = (Pos - Mesh->GetVertex(TriVertIDs[VertSubIdx])).SquareLength();
				if (DistSq <= SnapElementThresholdSq && (BestSubIdx == -1 || DistSq < BestElementDistSq))
				{
					BestSubIdx = VertSubIdx;
					BestElementDistSq = DistSq;
				}
			}

			if (BestSubIdx > -1)
			{
				SurfacePt.ElementID = TriVertIDs[BestSubIdx];
				SurfacePt.Point = SurfacePointType::Vertex;
				return;
			}

			Index3 TriEdgeIDs = Mesh->GetTriEdges(TriID);

			assert(BestSubIdx == -1);
			float BestEdgeParam = 0;
			for (int EdgeSubIdx = 0; EdgeSubIdx < 3; EdgeSubIdx++)
			{
				int EdgeID = TriEdgeIDs[EdgeSubIdx];
				Vector3 EPosA, EPosB;
				Mesh->GetEdgeV(EdgeID, EPosA, EPosB);
				Segment3 EdgeSeg(EPosA, EPosB);
				float DistSq = EdgeSeg.SqrDistanceToPoint(Pos);
				if (DistSq <= SnapElementThresholdSq && (BestSubIdx == -1 || DistSq < BestElementDistSq))
				{
					BestSubIdx = EdgeSubIdx;
					BestElementDistSq = DistSq;
					BestEdgeParam = EdgeSeg.ProjectUnitRange(Pos);
				}
			}

			if (BestSubIdx > -1)
			{
				SurfacePt.ElementID = TriEdgeIDs[BestSubIdx];
				SurfacePt.Point = SurfacePointType::Edge;
				SurfacePt.BaryCoord = Vector3(BestEdgeParam, 1 - BestEdgeParam, 0);
				return;
			}
		}

		static MeshSurfacePoint RelocateTrianglePointAfterRefinement(const DynamicMesh* Mesh, const Vector3& PosInVertexCoordSpace, const std::vector<int>& TriIDs, float SnapElementThresholdSq)
		{
			float BestTriDistSq = 0;
			Vector3 BestBaryCoords;
			int BestTriID = -1;
			for (int TriID : TriIDs)
			{
				assert(Mesh->IsTriangle(TriID));
				Index3 TriVertIDs = Mesh->GetTriangle(TriID);
				Triangle3 Tri(Mesh->GetVertex(TriVertIDs.a), Mesh->GetVertex(TriVertIDs.b), Mesh->GetVertex(TriVertIDs.c));
				Triangle3::PointDistanceQueryResult info = Tri.PointDistanceQuery(PosInVertexCoordSpace);
				float DistSq = info.SqrDistance;
				if (BestTriID == -1 || DistSq < BestTriDistSq)
				{
					BestTriID = TriID;
					BestTriDistSq = DistSq;
					BestBaryCoords = info.BaryCoords;
				}
			}

			assert(Mesh->IsTriangle(BestTriID));
			MeshSurfacePoint SurfacePt(BestTriID, BestBaryCoords);
			RefineSurfacePtFromTriangleToSubElement(Mesh, PosInVertexCoordSpace, SurfacePt, SnapElementThresholdSq);
			return SurfacePt;
		}


	};

	struct CutWorkingInfo
	{
		CutWorkingInfo(DynamicMesh* WorkingMesh, float SnapTolerance) : Mesh(WorkingMesh), SnapToleranceSq(SnapTolerance* SnapTolerance)
		{
			Init(WorkingMesh);
		}

		static Vector3 GetDegenTriangleEdgeDirection(const DynamicMesh* Mesh, int TID, Vector3 DefaultDir = Vector3::UnitZ())
		{
			Vector3 V[3];
			Mesh->GetTriVertices(TID, V[0], V[1], V[2]);
			for (int Prev = 2, Idx = 0; Idx < 3; Prev = Idx++)
			{
				Vector3 E = V[Idx] - V[Prev];
				if (E.Normalize())
				{
					return E;
				}
			}
			return DefaultDir;
		}

		void Init(DynamicMesh* WorkingMesh)
		{
			for (int i = 0; i < WorkingMesh->GetTriangleCount(); ++i)
			{
				if (!WorkingMesh->IsTriangleFast(i))
				{
					continue;
				}

				BaseFaceNormals.push_back(WorkingMesh->GetTriNormal(i));
			}

			FaceVertices.clear();
			EdgeVertices.clear();
			IntersectionVerts.clear();
			Segments.clear();
		}

		DynamicMesh* Mesh;
		float SnapToleranceSq;
		std::multimap<int, int> FaceVertices;
		std::multimap<int, int> EdgeVertices;
		std::vector<Vector3> BaseFaceNormals;
		std::vector<PointOnMesh> IntersectionVerts;
		std::vector<SegmentElements> Segments;

		void AddSegments(const IntersectionsQueryResult& Intersections, int WhichSide)
		{
			size_t SegStart = Segments.size();
			Segments.resize(SegStart + Intersections.Segments.size());

			// Classify each segment endpoint on the current mesh.
			for (size_t SegIdx = 0, SegCount = Intersections.Segments.size(); SegIdx < SegCount; SegIdx++)
			{
				const IntersectionsQueryResult::SegmentIntersection& Seg = Intersections.Segments[SegIdx];
				SegmentElements& SegToEls = Segments[SegStart + SegIdx];
				SegToEls.BaseTID = Seg.TriangleID[WhichSide];

				Triangle3 Tri;
				Mesh->GetTriVertices(SegToEls.BaseTID, Tri.v0, Tri.v1, Tri.v2);
				Index3 TriVIDs = Mesh->GetTriangle(SegToEls.BaseTID);
				int PrevOnEdgeIdx = -1;
				Vector3 PrevOnEdgePos;
				for (int SegPtIdx = 0; SegPtIdx < 2; SegPtIdx++)
				{
					int NewPtIdx = (int)IntersectionVerts.size();
					IntersectionVerts.push_back({});
					PointOnMesh& PtOnMesh = IntersectionVerts.back();
					PtOnMesh.Pos = Seg.Point[SegPtIdx];
					SegToEls.PtOnMeshIdx[SegPtIdx] = NewPtIdx;

					int OnVertexIdx = OnVertex(Tri, PtOnMesh.Pos);
					if (OnVertexIdx > -1)
					{
						PtOnMesh.Type = VertexLocationType::Vertex;
						PtOnMesh.ElemID = TriVIDs[OnVertexIdx];
						continue;
					}

					int OnEdgeIdx = OnEdge(Tri, PtOnMesh.Pos);
					if (OnEdgeIdx > -1)
					{
						// Degenerate segments can lie on a triangle edge.
						if (PrevOnEdgeIdx == OnEdgeIdx && (PrevOnEdgePos - PtOnMesh.Pos).SquareLength() < SnapToleranceSq)
						{
							int OnEdgeReplaceIdx = OnEdgeWithSkip(Tri, PtOnMesh.Pos, OnEdgeIdx);
							if (OnEdgeReplaceIdx > -1)
							{
								OnEdgeIdx = OnEdgeReplaceIdx;
							}
						}
						PtOnMesh.Type = VertexLocationType::Edge;
						PtOnMesh.ElemID = Mesh->GetTriEdges(SegToEls.BaseTID, OnEdgeIdx);

						assert(PtOnMesh.ElemID > -1);
						EdgeVertices.emplace(PtOnMesh.ElemID, NewPtIdx);

						PrevOnEdgeIdx = OnEdgeIdx;
						PrevOnEdgePos = PtOnMesh.Pos;

						continue;
					}

					PtOnMesh.Type = VertexLocationType::Face;
					PtOnMesh.ElemID = SegToEls.BaseTID;
					FaceVertices.emplace(PtOnMesh.ElemID, NewPtIdx);
				}
			}
		}

		void InsertFaceVertices()
		{
			Triangle3 Tri;
			std::vector<int> PtIndices;

			while (FaceVertices.size() > 0)
			{
				int TID = -1, PtIdx = -1;

				for (std::pair<int, int> TIDToPtIdx : FaceVertices)
				{
					TID = TIDToPtIdx.first;
					PtIdx = TIDToPtIdx.second;
					break;
				}
				PtIndices.clear();

				int nums = (int)FaceVertices.count(TID);
				auto it = FaceVertices.find(TID);
				while (nums--)
				{
					PtIndices.push_back(it->second);
					++it;
				}

				PointOnMesh& Pt = IntersectionVerts[PtIdx];

				Mesh->GetTriVertices(TID, Tri.v0, Tri.v1, Tri.v2);

				Vector3 BaryCoords = Tri.BarycentricCoods(Pt.Pos);
				PokeTriangleInfo PokeInfo;
				EMeshResult Result = Mesh->PokeTriangle(TID, BaryCoords, PokeInfo);
				assert(Result == EMeshResult::Ok);
				int PokeVID = PokeInfo.NewVertex;
				Mesh->SetVertex(PokeVID, Pt.Pos);
				Pt.ElemID = PokeVID;
				Pt.Type = VertexLocationType::Vertex;

				FaceVertices.erase(TID);
				Index3 PokeTriangles(TID, PokeInfo.NewTriangles.a, PokeInfo.NewTriangles.b);
				if (PtIndices.size() > 1)
				{
					for (int RelocatePtIdx : PtIndices)
					{
						if (PtIdx == RelocatePtIdx)
						{
							continue;
						}

						PointOnMesh& RelocatePt = IntersectionVerts[RelocatePtIdx];
						UpdateFromPoke(RelocatePt, PokeInfo.NewVertex, PokeInfo.NewEdges, PokeTriangles);
						if (RelocatePt.Type == VertexLocationType::Edge)
						{
							assert(RelocatePt.ElemID > -1);
							EdgeVertices.emplace(RelocatePt.ElemID, RelocatePtIdx);
						}
						else if (RelocatePt.Type == VertexLocationType::Face)
						{
							assert(RelocatePt.ElemID > -1);
							FaceVertices.emplace(RelocatePt.ElemID, RelocatePtIdx);
						}
					}
				}
			}
		}

		void InsertEdgeVertices()
		{
			Triangle3 Tri;
			std::vector<int> PtIndices;

			while (EdgeVertices.size() > 0)
			{
				int EID = -1, PtIdx = -1;

				for (std::pair<int, int> EIDToPtIdx : EdgeVertices)
				{
					EID = EIDToPtIdx.first;
					PtIdx = EIDToPtIdx.second;
					break;
				}

				PtIndices.clear();

				int nums = (int)EdgeVertices.count(EID);
				auto it = EdgeVertices.find(EID);
				while (nums--)
				{
					PtIndices.push_back(it->second);
					++it;
				}

				PointOnMesh& Pt = IntersectionVerts[PtIdx];

				Vector3 EA, EB;
				Mesh->GetEdgeV(EID, EA, EB);
				Segment3 Seg(EA, EB);
				float SplitParam = Seg.ProjectUnitRange(Pt.Pos);

				Index2 SplitTris = Mesh->GetEdgeT(EID);
                (void)SplitTris;
				EdgeSplitInfo SplitInfo;
				EMeshResult Result = Mesh->SplitEdge(EID, SplitInfo, SplitParam);
				assert(Result == EMeshResult::Ok);

				Mesh->SetVertex(SplitInfo.NewVertex, Pt.Pos);
				Pt.ElemID = SplitInfo.NewVertex;
				Pt.Type = VertexLocationType::Vertex;

				EdgeVertices.erase(EID);
				if (PtIndices.size() > 1)
				{
					Index2 SplitEdges{ SplitInfo.OriginalEdge, SplitInfo.NewEdges.a };
					for (int RelocatePtIdx : PtIndices)
					{
						if (PtIdx == RelocatePtIdx)
						{
							continue;
						}

						PointOnMesh& RelocatePt = IntersectionVerts[RelocatePtIdx];
						UpdateFromSplit(RelocatePt, SplitInfo.NewVertex, SplitEdges);
						if (RelocatePt.Type == VertexLocationType::Edge)
						{
							assert(RelocatePt.ElemID > -1);
							EdgeVertices.emplace(RelocatePt.ElemID, RelocatePtIdx);
						}
					}
				}
			}
		}

		bool ConnectEdges(std::vector<int>* VertexChains = nullptr, std::vector<int>* SegmentToChain = nullptr)
		{
			std::vector<int> EmbeddedPath;

			bool Success = true;

			assert(VertexChains || !SegmentToChain);

			if (SegmentToChain)
			{
				SegmentToChain->resize(Segments.size());
				for (int& ChainIdx : *SegmentToChain)
				{
					ChainIdx = -1;
				}
			}

			for (size_t SegIdx = 0, NumSegs = Segments.size(); SegIdx < NumSegs; SegIdx++)
			{
				SegmentElements& Seg = Segments[SegIdx];
				if (Seg.PtOnMeshIdx[0] == Seg.PtOnMeshIdx[1])
				{
					continue;
				}
				PointOnMesh& PtA = IntersectionVerts[Seg.PtOnMeshIdx[0]];
				PointOnMesh& PtB = IntersectionVerts[Seg.PtOnMeshIdx[1]];
				if (!(PtA.Type == VertexLocationType::Vertex && PtB.Type == VertexLocationType::Vertex && PtA.ElemID != -1 && PtB.ElemID != -1))
				{
					Success = false;
					continue;
				}
				if (PtA.ElemID == PtB.ElemID)
				{
					if (VertexChains)
					{
						if (SegmentToChain)
						{
							(*SegmentToChain)[SegIdx] = (int)VertexChains->size();
						}
						VertexChains->push_back(1);
						VertexChains->push_back(PtA.ElemID);
					}
					continue;
				}


				int EID = Mesh->FindEdge(PtA.ElemID, PtB.ElemID);
				if (EID != -1)
				{
					if (VertexChains)
					{
						if (SegmentToChain)
						{
							(*SegmentToChain)[SegIdx] = (int)VertexChains->size();
						}
						VertexChains->push_back(2);
						VertexChains->push_back(PtA.ElemID);
						VertexChains->push_back(PtB.ElemID);
					}
					continue;
				}

				MeshSurfacePath SurfacePath(Mesh);
				int StartTID = Mesh->GetVtxSingleTriangle(PtA.ElemID);
				Vector3 WalkPlaneNormal = BaseFaceNormals[Seg.BaseTID].Cross(PtB.Pos - PtA.Pos);
				if (WalkPlaneNormal.Normalize() == 0)
				{
					if ((PtA.Pos - PtB.Pos).SquareLength() > SnapToleranceSq)
					{

						continue;
					}

					WalkPlaneNormal = GetDegenTriangleEdgeDirection(Mesh, StartTID);
					if (!(WalkPlaneNormal.Normalize() > 0))
					{

						continue;
					}
				}

				bool WalkSuccess = SurfacePath.AddViaPlanarWalk(StartTID, PtA.ElemID,
					Mesh->GetVertex(PtA.ElemID), -1, PtB.ElemID,
					Mesh->GetVertex(PtB.ElemID), WalkPlaneNormal, nullptr, false, 1e-6f, sqrtf(SnapToleranceSq), 0.001f);
				if (!WalkSuccess)
				{
					SurfacePath.Reset();
					WalkSuccess = SurfacePath.AddViaPlanarWalk(StartTID, PtA.ElemID,
						Mesh->GetVertex(PtA.ElemID), -1, PtB.ElemID,
						Mesh->GetVertex(PtB.ElemID), WalkPlaneNormal, nullptr, true, 1e-6f, sqrtf(SnapToleranceSq), 0.001f);
				}
				if (!WalkSuccess)
				{
					Success = false;
				}
				else
				{
					EmbeddedPath.clear();
					if (SurfacePath.EmbedSimplePath(false, EmbeddedPath, false, SnapToleranceSq))
					{
						assert(EmbeddedPath.size() > 0 && EmbeddedPath[0] == PtA.ElemID);
						if (VertexChains)
						{
							if (SegmentToChain)
							{
								(*SegmentToChain)[SegIdx] = (int)VertexChains->size();
							}
							VertexChains->push_back((int)EmbeddedPath.size());
							VertexChains->insert(VertexChains->end(), EmbeddedPath.begin(), EmbeddedPath.end());
						}
					}
					else
					{
						Success = false;
					}
				}
			}

			return Success;
		}

		void UpdateFromSplit(PointOnMesh& Pt, int SplitVertex, const Index2& SplitEdges)
		{
			if ((Pt.Pos - Mesh->GetVertex(SplitVertex)).SquareLength() <= SnapToleranceSq)
			{
				Pt.Type = VertexLocationType::Vertex;
				Pt.ElemID = SplitVertex;
				return;
			}

			int EdgeIdx = ClosestEdge(SplitEdges, Pt.Pos, SnapToleranceSq);
			if (EdgeIdx < 0)
			{
				EdgeIdx = ClosestEdge(SplitEdges, Pt.Pos, FLT_MAX);
			}
			assert(EdgeIdx > -1 && EdgeIdx < 2 && SplitEdges[EdgeIdx]>-1);
			Pt.Type = VertexLocationType::Edge;
			Pt.ElemID = SplitEdges[EdgeIdx];
		}

		void UpdateFromPoke(PointOnMesh& Pt, int PokeVertex, const Index3& PokeEdges, const Index3& PokeTris)
		{
			if ((Pt.Pos - Mesh->GetVertex(PokeVertex)).SquareLength() < SnapToleranceSq)
			{
				Pt.Type = VertexLocationType::Vertex;
				Pt.ElemID = PokeVertex;
				return;
			}

			int EdgeIdx = OnEdge(PokeEdges, Pt.Pos, SnapToleranceSq);
			if (EdgeIdx > -1)
			{
				Pt.Type = VertexLocationType::Edge;
				Pt.ElemID = PokeEdges[EdgeIdx];
				return;
			}

			for (int j = 0; j < 3; ++j)
			{
				if (IsInTriangle(PokeTris[j], Pt.Pos))
				{
					assert(Pt.Type == VertexLocationType::Face);
					Pt.ElemID = PokeTris[j];
					return;
				}
			}

			EdgeIdx = OnEdge(PokeEdges, Pt.Pos, FLT_MAX);
			Pt.Type = VertexLocationType::Edge;
			Pt.ElemID = PokeEdges[EdgeIdx];
		}

		int OnVertex(const Triangle3& Tri, const Vector3& V)
		{
			float BestDSq = SnapToleranceSq;
			int BestIdx = -1;
			for (int SubIdx = 0; SubIdx < 3; SubIdx++)
			{
				float DSq = (Tri[SubIdx] - V).SquareLength();
				if (DSq < BestDSq)
				{
					BestIdx = SubIdx;
					BestDSq = DSq;
				}
			}
			return BestIdx;
		}

		int OnEdge(const Triangle3& Tri, const Vector3& V)
		{
			float BestDSq = SnapToleranceSq;
			int BestIdx = -1;
			for (int Idx = 0; Idx < 3; Idx++)
			{
				Segment3 Seg( Tri[Idx], Tri[(Idx + 1) % 3] );
				float DSq = Seg.SqrDistanceToPoint(V);
				if (DSq < BestDSq)
				{
					BestDSq = DSq;
					BestIdx = Idx;
				}
			}
			return BestIdx;
		}


		int OnEdgeWithSkip(const Triangle3& Tri, const Vector3& V, int SkipIdx)
		{
			float BestDSq = SnapToleranceSq;
			int BestIdx = -1;
			for (int Idx = 0; Idx < 3; Idx++)
			{
				if (Idx == SkipIdx)
				{
					continue;
				}
				Segment3 Seg( Tri[Idx], Tri[(Idx + 1) % 3] );
				const float DSq = Seg.SqrDistanceToPoint(V);
				if (DSq < BestDSq)
				{
					BestDSq = DSq;
					BestIdx = Idx;
				}
			}
			return BestIdx;
		}

		int ClosestEdge(Index2 EIDs, const Vector3& Pos, float BestDSq)
		{
			int BestIdx = -1;
			for (int Idx = 0; Idx < 2; Idx++)
			{
				int EID = EIDs[Idx];
				if (!Mesh->IsEdge(EID))
				{
					continue;
				}
				Index2 EVIDs = Mesh->GetEdgeV(EID);
				Segment3 Seg(Mesh->GetVertex(EVIDs.a), Mesh->GetVertex(EVIDs.b));
				const float DSq = Seg.SqrDistanceToPoint(Pos);
				if (DSq < BestDSq)
				{
					BestDSq = DSq;
					BestIdx = Idx;
				}
			}
			return BestIdx;
		}

		int OnEdge(Index3 EIDs, const Vector3& Pos, float BestDSq)
		{
			int BestIdx = -1;
			for (int Idx = 0; Idx < 3; Idx++)
			{
				int EID = EIDs[Idx];
				Index2 EVIDs = Mesh->GetEdgeV(EID);
				Segment3 Seg(Mesh->GetVertex(EVIDs.a), Mesh->GetVertex(EVIDs.b));
				float DSq = Seg.SqrDistanceToPoint(Pos);
				if (DSq < BestDSq)
				{
					BestDSq = DSq;
					BestIdx = Idx;
				}
			}
			return BestIdx;
		}

		bool IsInTriangle(int TID, const Vector3& Pos)
		{
			Triangle3 Tri;
			Mesh->GetTriVertices(TID, Tri.v0, Tri.v1, Tri.v2);
			Vector3 bary = Tri.BarycentricCoods(Pos);
			return (bary.x >= 0 && bary.y >= 0 && bary.z >= 0
				&& bary.x < 1 && bary.y <= 1 && bary.z <= 1);
		}
	};

	class GeometryCut
	{
	public:
		DynamicMesh* Mesh[2];

		GeometryCut(DynamicMesh* MeshA, DynamicMesh* MeshB)
			: Mesh{ MeshA, MeshB }
		{
		}

		bool Compute(const IntersectionsQueryResult& Intersections)
		{
			bool Success = true;

			int MeshesToProcess = MutuallyCut ? 2 : 1;
			for (int MeshIdx = 0; MeshIdx < MeshesToProcess; MeshIdx++)
			{
				CutWorkingInfo WorkingInfo(Mesh[MeshIdx], SnapTolerance);
				WorkingInfo.AddSegments(Intersections, MeshIdx);
				WorkingInfo.InsertFaceVertices();
				WorkingInfo.InsertEdgeVertices();

				bool Connected = WorkingInfo.ConnectEdges(&VertexChains[MeshIdx], &SegmentToChain[MeshIdx]);
				if (!Connected)
				{
					Success = false;
				}
			}

			return Success;
		}

		float SnapTolerance = 1e-4f;
		bool MutuallyCut = true;
		bool CutCoplanar = false;

		std::vector<int> VertexChains[2];
		std::vector<int> SegmentToChain[2];
	};

	class LocalPlanarSimplify
	{
	public:
		float SimplificationAngleTolerance = 0.1f;
		float TryToImproveTriQualityThreshold = 0.25f;
		bool PreserveTriangleGroups = true;
		bool PreserveVertexUVs = true;
		bool PreserveOverlayUVs = true;
		float UVDistortTolerance = 1e-6f;
		bool PreserveVertexNormals = true;
		float NormalDistortTolerance = .01f;

		void SimplifyAlongEdges(DynamicMesh& Mesh, std::set<int>& InOutEdges, std::function<void(const EdgeCollapseInfo&)> ProcessCollapse = nullptr) const
		{
			float DotTolerance = cosf(SimplificationAngleTolerance * DEG_TO_RAD);

			// The stricter flat-only path is useful for cuts made inside flat triangles.
			const bool OnlySimplifyWhereFlat = false;

			// Iterate a copy so collapses can edit the edge set.
			std::vector<int> CutBoundaryEdgesArray = ToVector<int>(InOutEdges);

			int NumCollapses = 0, CollapseIters = 0;
			int MaxCollapseIters = 1;
			while (CollapseIters < MaxCollapseIters)
			{
				int LastNumCollapses = NumCollapses;
				for (int EID : CutBoundaryEdgesArray)
				{
					if (!Mesh.IsEdge(EID))
					{
						continue;
					}

					DynamicMesh::Edge& e = Mesh.GetEdge(EID);

					bool Developable[2]{ false, false };
					Vector3 FlatNormals[2]{ Vector3::Zero(), Vector3::Zero() };
					bool Flat[2]{ false, false };
					int NumDevelopable = 0;
					for (int VIdx = 0; VIdx < 2; VIdx++)
					{
						if (OnlySimplifyWhereFlat)
						{
							Flat[VIdx] = Developable[VIdx] = IsFlat(Mesh, e.Vert[VIdx], DotTolerance, FlatNormals[VIdx]);
						}
						else
						{
							Developable[VIdx] = IsDevelopableAlongEdge(Mesh, EID, e.Vert[VIdx], DotTolerance, FlatNormals[VIdx], Flat[VIdx]);
						}

						if (Developable[VIdx])
						{
							NumDevelopable++;
						}
					}

					if (NumDevelopable == 0)
					{
						continue;
					}

					for (int RemoveVIdx = 0; RemoveVIdx < 2; RemoveVIdx++)
					{
						if (!Developable[RemoveVIdx])
						{
							continue;
						}
						int KeepVIdx = 1 - RemoveVIdx;
						Vector3 RemoveVPos = Mesh.GetVertex(e.Vert[RemoveVIdx]);
						Vector3 KeepVPos = Mesh.GetVertex(e.Vert[KeepVIdx]);
						Vector3 EdgeDir = KeepVPos - RemoveVPos;
						if (EdgeDir.SafeNormalize() == 0)
						{
							break;
						}

						bool HasBadEdge = false;
						int RemoveV = e.Vert[RemoveVIdx];
						int KeepV = e.Vert[KeepVIdx];
						int SourceEID = EID;

						HasBadEdge = HasBadEdge || CollapseWouldHurtTriangleQuality(
							Mesh, FlatNormals[RemoveVIdx], RemoveV, KeepV, KeepVPos, TryToImproveTriQualityThreshold, !Flat[RemoveVIdx]);

						HasBadEdge = HasBadEdge || CollapseWouldChangeShapeOrUVs(
							Mesh, InOutEdges, DotTolerance,
							SourceEID, RemoveV, RemoveVPos, KeepV, KeepVPos, EdgeDir, PreserveTriangleGroups,
							true, PreserveVertexUVs, PreserveOverlayUVs, UVDistortTolerance * UVDistortTolerance,
							PreserveVertexNormals, cosf(NormalDistortTolerance * DEG_TO_RAD));

						if (HasBadEdge)
						{
							continue;
						}

						EdgeCollapseInfo CollapseInfo;
						EMeshResult CollapseResult = Mesh.CollapseEdge(KeepV, RemoveV, 0, CollapseInfo);
						if (CollapseResult == EMeshResult::Ok)
						{
							if (ProcessCollapse)
							{
								ProcessCollapse(CollapseInfo);
							}
							NumCollapses++;
							InOutEdges.erase(CollapseInfo.CollapsedEdge);
							InOutEdges.erase(CollapseInfo.RemovedEdges[0]);
							if (CollapseInfo.RemovedEdges[1] != -1)
							{
								InOutEdges.erase(CollapseInfo.RemovedEdges[1]);
							}
						}
						break;
					}
				}

				CutBoundaryEdgesArray = ToVector<int>(InOutEdges);

				if (NumCollapses == LastNumCollapses)
				{
					break;
				}

				CollapseIters++;
			}

		}

		static bool IsFlat(const DynamicMesh& Mesh, int VID, float DotTolerance, Vector3& OutFirstNormal)
		{
			bool HasFirst = false;
			bool IsFlatVertex = true;
			Mesh.EnumerateVertexTriangles(VID, [&Mesh, DotTolerance, &OutFirstNormal, &HasFirst, &IsFlatVertex](int TID)
				{
					if (!IsFlatVertex)
					{
						return;
					}
					Vector3 Normal = Mesh.GetTriNormal(TID);
					if (!HasFirst)
					{
						OutFirstNormal = Normal;
						HasFirst = true;
					}
					else
					{
						IsFlatVertex = IsFlatVertex && Normal.Dot(OutFirstNormal) >= DotTolerance;
					}
				});
			return IsFlatVertex;
		}

		static bool IsDevelopableAlongEdge(const DynamicMesh& Mesh, int EID, int VID, float DotTolerance, Vector3& NormalA, bool& IsFlatVertex)
		{
			Index2 EdgeT = Mesh.GetEdgeT(EID);
			NormalA = Mesh.GetTriNormal(EdgeT.a);
			IsFlatVertex = true;
			Vector3 NormalB;
			if (EdgeT.b != -1)
			{
				NormalB = Mesh.GetTriNormal(EdgeT.b);
				if (NormalA.Dot(NormalB) < DotTolerance)
				{
					IsFlatVertex = false;
				}
			}
			bool IsDevelopable = true;
			Mesh.EnumerateVertexTriangles(VID, [&IsDevelopable, EdgeT, &Mesh, NormalA, NormalB, DotTolerance, IsFlatVertex](int TID)
				{
					if (!IsDevelopable || EdgeT.Contains(TID))
					{
						return;
					}
					Vector3 Normal = Mesh.GetTriNormal(TID);
					IsDevelopable = Normal.Dot(NormalA) >= DotTolerance;
					if (!IsDevelopable && !IsFlatVertex)
					{
						IsDevelopable = Normal.Dot(NormalB) >= DotTolerance;
					}
				});
			IsFlatVertex = IsFlatVertex && IsDevelopable;

			return IsDevelopable;
		}

		static bool CollapseWouldHurtTriangleQuality(
			const DynamicMesh& Mesh, const Vector3& RemoveVNormal,
			int RemoveV, int KeepV, const Vector3& KeepVPos,
			float TryToImproveTriQualityThreshold, bool HasMultipleNormals = false)
		{
			float WorstQualityNewTriangle = FLT_MAX;

			bool IsHurt = false;
			Mesh.EnumerateVertexTriangles(RemoveV,
				[&Mesh, &IsHurt, &KeepVPos, RemoveV, KeepV, &RemoveVNormal,
				TryToImproveTriQualityThreshold, &WorstQualityNewTriangle, HasMultipleNormals](int TID)
				{
					if (IsHurt)
					{
						return;
					}
					Index3 Tri = Mesh.GetTriangle(TID);
					Vector3 TriNormal = HasMultipleNormals ? Mesh.GetTriNormal(TID) : RemoveVNormal;
					Vector3 Verts[3];
					for (int Idx = 0; Idx < 3; Idx++)
					{
						int VID = Tri[Idx];
						if (VID == KeepV)
						{
							return;
						}
						else if (VID == RemoveV)
						{
							Verts[Idx] = KeepVPos;
						}
						else
						{
							Verts[Idx] = Mesh.GetVertex(Tri[Idx]);
						}
					}
					Vector3 Edge1(Verts[1] - Verts[0]);
					Vector3 Edge2(Verts[2] - Verts[0]);
					Vector3 VCross(Edge2.Cross(Edge1));

					float EdgeFlipTolerance = 1e-5f;
					float Area2 = VCross.Normalize();
					if (TryToImproveTriQualityThreshold > 0)
					{
						Vector3 Edge3(Verts[2] - Verts[1]);
						float MaxLenSq = Maths::Max(Edge1.SquareLength(), Edge2.SquareLength(), Edge3.SquareLength());
						float Quality = Area2 / (MaxLenSq + 1e-6f);
						WorstQualityNewTriangle = Maths::Min(Quality, WorstQualityNewTriangle);
					}

					IsHurt = VCross.Dot(TriNormal) <= EdgeFlipTolerance;
				}
			);

			// Convert the quality estimate to an aspect-ratio-like threshold.
			if (!IsHurt && WorstQualityNewTriangle * 2.0f * INV_SQRT_3 < TryToImproveTriQualityThreshold)
			{
				IsHurt = true;
				Mesh.EnumerateVertexTriangles(RemoveV, [&Mesh, &IsHurt, WorstQualityNewTriangle](int TID)
					{
						if (!IsHurt)
						{
							return;
						}
						Vector3 A, B, C;
						Mesh.GetTriVertices(TID, A, B, C);
						Vector3 E1 = B - A, E2 = C - A, E3 = C - B;
						float Area2 = E1.Cross(E2).Length();
						float MaxLenSq = Maths::Max(E1.SquareLength(), E2.SquareLength(), E3.SquareLength());
						float Quality = Area2 / (MaxLenSq + 1e-6f);
						if (Quality < WorstQualityNewTriangle)
						{
							IsHurt = false;
						}
					}
				);
			}
			return IsHurt;
		}

		static int FindEdgeOtherVertex(const Index2& EdgeVerts, int VertexID)
		{
			if (EdgeVerts.a == VertexID)		return EdgeVerts.b;
			else if (EdgeVerts.b == VertexID)	return EdgeVerts.a;
			else								return -1;
		}

		static bool CollapseWouldChangeShapeOrUVs(
			const DynamicMesh& Mesh, const std::set<int>& PathEdgeSet, float DotTolerance, int SourceEID,
			int RemoveV, const Vector3& RemoveVPos, int KeepV, const Vector3& KeepVPos, const Vector3& EdgeDir,
			bool PreserveTriangleGroups, bool PreserveUVsForMesh, bool PreserveVertexUVs, bool PreserveOverlayUVs,
			float UVEqualThresholdSq, bool PreserveVertexNormals, float NormalEqualCosThreshold)
		{
			// The removed vertex must have one opposite path edge and no protected seam changes.
			bool HasBadEdge = false;

			int OpposedEdge = -1;
			Index2 EdgeT = Mesh.GetEdgeT(SourceEID);
			int SourceGroupID = Mesh.GetTriangleGroup(EdgeT.a);
			int OtherGroupID = -1;
			constexpr bool AllowTwoGroups = true;
			if (AllowTwoGroups && EdgeT.b != -1)
			{
				OtherGroupID = Mesh.GetTriangleGroup(EdgeT.b);
			}

			Mesh.EnumerateVertexEdges(RemoveV,
				[&](int VertEID)
				{
					if (HasBadEdge || VertEID == SourceEID)
					{
						return;
					}

					DynamicMesh::Edge e = Mesh.GetEdge(VertEID);
					if (PreserveTriangleGroups && Mesh.HasTriangleGroups())
					{
						int GroupA = Mesh.GetTriangleGroup(e.Tri.a);
						int GroupB = e.Tri.b == -1 ? SourceGroupID : Mesh.GetTriangleGroup(e.Tri.b);

						if (
							(GroupA != SourceGroupID && (!AllowTwoGroups || GroupA != OtherGroupID)) ||
							(GroupB != SourceGroupID && (!AllowTwoGroups || GroupB != OtherGroupID))
							)
						{
							HasBadEdge = true;
							return;
						}
					}

					if (PathEdgeSet.count(VertEID) > 0)
					{
						if (OpposedEdge != -1)
						{
							HasBadEdge = true;
							return;
						}
						Index2 OtherEdgeV = e.Vert;
						int OtherV = FindEdgeOtherVertex(OtherEdgeV, RemoveV);
						Vector3 OtherVPos = Mesh.GetVertex(OtherV);
						Vector3 OtherEdgeDir = OtherVPos - RemoveVPos;
						if (OtherEdgeDir.SafeNormalize() == 0)
						{
							HasBadEdge = true;
							return;
						}
						if (OtherEdgeDir.Dot(EdgeDir) <= -DotTolerance)
						{
							OpposedEdge = VertEID;
						}
						else
						{
							HasBadEdge = true;
							return;
						}

						float LerpT = float((OtherVPos - RemoveVPos).Dot(OtherEdgeDir) / (OtherVPos - KeepVPos).Dot(OtherEdgeDir));

						if (PreserveVertexNormals && Mesh.HasVertexNormals())
						{
							Vector3 OtherN = Mesh.GetVertexNormal(OtherV);
							Vector3 RemoveN = Mesh.GetVertexNormal(RemoveV);
							Vector3 KeepN = Mesh.GetVertexNormal(KeepV);
							if (Maths::LinearInterp(OtherN, KeepN, LerpT).Unit().Dot(RemoveN.Unit()) < NormalEqualCosThreshold)
							{
								HasBadEdge = true;
								return;
							}
						}

						if (!PreserveUVsForMesh)
						{
							return;
						}

						if (PreserveVertexUVs && Mesh.HasVertexUVs())
						{
							Vector2 OtherUV = Mesh.GetVertexUV(OtherV);
							Vector2 RemoveUV = Mesh.GetVertexUV(RemoveV);
							Vector2 KeepUV = Mesh.GetVertexUV(KeepV);
							if ((Maths::LinearInterp(OtherUV, KeepUV, LerpT) - RemoveUV).SquareLength() > UVEqualThresholdSq)
							{
								HasBadEdge = true;
								return;
							}
						}
						if (PreserveOverlayUVs && Mesh.HasAttributes())
						{
							int NumLayers = Mesh.Attributes()->NumUVLayers();
							Index2 SourceEdgeTris = Mesh.GetEdgeT(SourceEID);
							Index2 OppEdgeTris = e.Tri;

							auto CanCollapseOverlayUVs = [&Mesh, KeepV, RemoveV, OtherV, NumLayers, LerpT, UVEqualThresholdSq](int SourceEdgeTID, int OppEdgeTID) -> bool
							{
								Index3 SourceBaseTri = Mesh.GetTriangle(SourceEdgeTID);
								Index3 OppBaseTri = Mesh.GetTriangle(OppEdgeTID);
								int KeepSourceIdx = FindTriIndex(KeepV, SourceBaseTri);
								int RemoveSourceIdx = FindTriIndex(RemoveV, SourceBaseTri);
								int OtherOppIdx = FindTriIndex(OtherV, OppBaseTri);
								if (!(KeepSourceIdx != -1 && RemoveSourceIdx != -1 && OtherOppIdx != -1))
								{
									return false;
								}

								for (int UVLayerIdx = 0; UVLayerIdx < NumLayers; UVLayerIdx++)
								{
									const DynamicMeshUVOverlay* UVs = Mesh.Attributes()->GetUVLayer(UVLayerIdx);
									if (UVs->ElementCount() < 3)
									{
										continue;
									}
									Index3 SourceT = UVs->GetTriangle(SourceEdgeTID);
									Index3 OppT = UVs->GetTriangle(OppEdgeTID);
									int KeepE = SourceT[KeepSourceIdx];
									int RemoveE = SourceT[RemoveSourceIdx];
									int OtherE = OppT[OtherOppIdx];
									if (KeepE == -1 || RemoveE == -1 || OtherE == -1)
									{
										continue;
									}
									Vector2 OtherUV = UVs->GetElement(OtherE);
									Vector2 RemoveUV = UVs->GetElement(RemoveE);
									Vector2 KeepUV = UVs->GetElement(KeepE);
									if ((Maths::LinearInterp(OtherUV, KeepUV, LerpT) -  RemoveUV).SquareLength() > UVEqualThresholdSq)
									{
										return false;
									}
								}

								return true;
							};

							// Non-boundary seam collapses must preserve both UV sides.
							if (SourceEdgeTris.b != -1 || OppEdgeTris.b != -1)
							{
								if (SourceEdgeTris.b == -1 || OppEdgeTris.b == -1)
								{
									HasBadEdge = true;
									return;
								}
								bool SourceIsSeam = Mesh.Attributes()->IsSeamEdge(SourceEID);
								bool OtherIsSeam = Mesh.Attributes()->IsSeamEdge(VertEID);
								if (SourceIsSeam != OtherIsSeam)
								{
									HasBadEdge = true;
									return;
								}
								if (SourceIsSeam)
								{
									Index3 SourceBaseTri = Mesh.GetTriangle(SourceEdgeTris.a);
									Index3 OppBaseTri = Mesh.GetTriangle(OppEdgeTris.a);
									int SrcTriRmVSubIdx = FindTriIndex(RemoveV, SourceBaseTri);
									int OppTriRmVSubIdx = FindTriIndex(RemoveV, OppBaseTri);
									bool SourceAIsRemoveThenKeep = SourceBaseTri[(SrcTriRmVSubIdx + 1) % 3] == KeepV;
									bool OppAIsRemoveThenOther = OppBaseTri[(OppTriRmVSubIdx + 1) % 3] == OtherV;
									// Keep both seam sides paired before testing the opposite UV side.
									if (SourceAIsRemoveThenKeep == OppAIsRemoveThenOther)
									{
										std::swap(OppEdgeTris.a, OppEdgeTris.b);
									}
									if (!CanCollapseOverlayUVs(SourceEdgeTris.b, OppEdgeTris.b))
									{
										HasBadEdge = true;
										return;
									}
								}
							}

							if (!CanCollapseOverlayUVs(SourceEdgeTris.a, OppEdgeTris.a))
							{
								HasBadEdge = true;
								return;
							}
						}
					}
					else
					{
						if (Mesh.IsBoundaryEdge(VertEID) || (Mesh.HasAttributes() && Mesh.Attributes()->IsSeamEdge(VertEID)))
						{
							HasBadEdge = true;
						}
					}
				});

			return HasBadEdge;
		}
	};

	static int FindNearestEdge(const DynamicMesh& OnMesh, const std::vector<int>& EIDs, const Vector3& Pos)
	{
		int NearEID = -1;
		float NearSqr = 1e-6f;
		Vector3 EdgePts[2];
		for (int EID : EIDs)
		{
			OnMesh.GetEdgeV(EID, EdgePts[0], EdgePts[1]);

			Segment3 Seg(EdgePts[0], EdgePts[1]);
			float DSqr = Seg.SqrDistanceToPoint(Pos);
			if (DSqr < NearSqr)
			{
				NearEID = EID;
				NearSqr = DSqr;
			}
		}
		return NearEID;
	}

	bool MergeEdges(const FMeshIndexMappings& IndexMaps, DynamicMesh* CutMesh[2], const std::vector<int> CutBoundaryEdges[2], const std::map<int, int>& AllVIDMatches, GeometryBoolean* Geom)
	{
		// Map mesh 1 boundary edges into result edge IDs.
		DynamicMesh *Result = Geom->Result;

		DynamicArray<int> OtherMeshEdges;
		for (int OldMeshEID : CutBoundaryEdges[1])
		{
			if (!(CutMesh[1]->IsEdge(OldMeshEID)))
			{
				continue;
			}
			Index2 OtherEV = CutMesh[1]->GetEdgeV(OldMeshEID);
			int MappedEID = Result->FindEdge(IndexMaps.GetNewVertex(OtherEV.a), IndexMaps.GetNewVertex(OtherEV.b));
			if (MappedEID != -1 && Result->IsBoundaryEdge(MappedEID))
			{
				OtherMeshEdges.push_back(MappedEID);
			}
		}

		// Match edges that already have matching endpoint vertices.
		DynamicArray<Index2> CandidateMatches;
		DynamicArray<int> UnmatchedEdges;
		for (int EID : CutBoundaryEdges[0])
		{
			if (!Result->IsEdge(EID) || !Result->IsBoundaryEdge(EID))
			{
				continue;
			}
			Index2 VIDs = Result->GetEdgeV(EID);
			const int* OtherA = MapFind<int, int>(AllVIDMatches, VIDs.a);
			const int* OtherB = MapFind<int, int>(AllVIDMatches, VIDs.b);
			bool AddedCandidate = false;
			if (OtherA && OtherB)
			{
				int MapOtherA = IndexMaps.GetNewVertex(*OtherA);
				int MapOtherB = IndexMaps.GetNewVertex(*OtherB);
				int OtherEID = Result->FindEdge(MapOtherA, MapOtherB);
				if (OtherEID != -1)
				{
					CandidateMatches.push_back(Index2(EID, OtherEID));
					AddedCandidate = true;
				}
			}
			if (!AddedCandidate)
			{
				UnmatchedEdges.push_back(EID);
			}
		}

		for (Index2 Candidate : CandidateMatches)
		{
			if (!Result->IsEdge(Candidate.a) || !Result->IsBoundaryEdge(Candidate.a) ||
				!Result->IsEdge(Candidate.b) || !Result->IsBoundaryEdge(Candidate.b))
			{
				continue;
			}

			MergeEdgesInfo MergeInfo;
			EMeshResult EdgeMergeResult = Result->MergeEdges(Candidate.a, Candidate.b, MergeInfo, true);
			if (EdgeMergeResult != EMeshResult::Ok)
			{
				UnmatchedEdges.push_back(Candidate.a);
			}
			else
			{
				if (Geom->TrackAllNewEdges)
				{
					Geom->AllNewEdges.insert(Candidate.a);
				}
			}
		}

		OtherMeshEdges.remove_if(
			[Result = Geom->Result](int EID)
			{
				return !Result->IsEdge(EID) || !Result->IsBoundaryEdge(EID);
			});

		bool AllMatched = true;
		if (UnmatchedEdges.size() > 0)
		{
			float SnapToleranceSq = Geom->SnapTolerance * Geom->SnapTolerance;
			for (int OtherEID : OtherMeshEdges)
			{
				if (!Result->IsEdge(OtherEID) || !Result->IsBoundaryEdge(OtherEID))
				{
					continue;
				}
				Vector3 OA, OB;
				Result->GetEdgeV(OtherEID, OA, OB);
				for (int UnmatchedIdx = 0; UnmatchedIdx < (int)UnmatchedEdges.size(); UnmatchedIdx++)
				{
					int EID = UnmatchedEdges[UnmatchedIdx];
					if (!Result->IsEdge(EID) || !Result->IsBoundaryEdge(EID))
					{
						UnmatchedEdges.remove_at(UnmatchedIdx, false);
						UnmatchedIdx--;
						continue;
					}
					Vector3 A, B;
					Result->GetEdgeV(EID, A, B);
					if ((OA - A).SquareLength() < SnapToleranceSq && (OB - B).SquareLength() < SnapToleranceSq)
					{
						MergeEdgesInfo MergeInfo;
						EMeshResult EdgeMergeResult = Result->MergeEdges(EID, OtherEID, MergeInfo, true);
						if (EdgeMergeResult == EMeshResult::Ok)
						{
							UnmatchedEdges.remove_at(UnmatchedIdx, false);
							if (Geom->TrackAllNewEdges)
							{
								Geom->AllNewEdges.insert(EID);
							}
							break;
						}
					}
				}
			}

			for (int EID : UnmatchedEdges)
			{
				if (Result->IsEdge(EID) && Result->IsBoundaryEdge(EID))
				{
					Geom->CreatedBoundaryEdges.push_back(EID);
					AllMatched = false;
				}
			}
		}
		for (int OtherEID : OtherMeshEdges)
		{
			if (Result->IsEdge(OtherEID) && Result->IsBoundaryEdge(OtherEID))
			{
				Geom->CreatedBoundaryEdges.push_back(OtherEID);
				AllMatched = false;
			}
		}
		return AllMatched;
	}
	
	void SimplifyCutEdges(int NumMeshesToProcess, DynamicMesh* CutMesh[2], std::vector<int> CutBoundaryEdges[2], std::map<int, int>& AllVIDMatches, GeometryBoolean* Geom)
	{
		float DotTolerance = cosf(Geom->SimplificationAngleTolerance * DEG_TO_RAD);

		std::set<int> CutBoundaryEdgeSets[2];
		for (int MeshIdx = 0; MeshIdx < NumMeshesToProcess; MeshIdx++)
		{
			CutBoundaryEdgeSets[MeshIdx] = std::set<int>(CutBoundaryEdges[MeshIdx].begin(), CutBoundaryEdges[MeshIdx].end());
		}

		int NumCollapses = 0, CollapseIters = 0;
		int MaxCollapseIters = 1;
		while (CollapseIters < MaxCollapseIters)
		{
			int LastNumCollapses = NumCollapses;
			for (int EID : CutBoundaryEdges[0])
			{
				if (!CutMesh[0]->IsEdge(EID))
				{
					continue;
				}
				if (CutMesh[0]->GetTriangleCount() <= 1 || (NumMeshesToProcess == 2 && CutMesh[1]->GetTriangleCount() <= 1))
				{
					break;
				}

				const DynamicMesh::Edge& e = CutMesh[0]->GetEdge(EID);
				int Matches[2]{ -1, -1 };
				bool HasMatches = NumMeshesToProcess == 2;
				if (HasMatches)
				{
					for (int MatchIdx = 0; MatchIdx < 2; MatchIdx++)
					{
						int* Match = MapFind<int, int>(AllVIDMatches, e.Vert[MatchIdx]);
						if (Match)
						{
							Matches[MatchIdx] = *Match;
						}
						else
						{
							HasMatches = false;
							break;
						}
					}
					if (!HasMatches)
					{
						continue;
					}
				}
				int OtherEID = -1;
				if (HasMatches)
				{
					OtherEID = CutMesh[1]->FindEdge(Matches[0], Matches[1]);
					if (OtherEID == -1)
					{
						continue;
					}
				}
				bool Flat[2]{ false, false };
				Vector3 FlatNormals[2][2]{ {Vector3::Zero(), Vector3::Zero()}, {Vector3::Zero(), Vector3::Zero()} };
				int NumFlat = 0;
				for (int VIdx = 0; VIdx < 2; VIdx++)
				{
					if (LocalPlanarSimplify::IsFlat(*CutMesh[0], e.Vert[VIdx], DotTolerance, FlatNormals[VIdx][0]))
					{
						Flat[VIdx] = (Matches[VIdx] == -1) || LocalPlanarSimplify::IsFlat(*CutMesh[1], Matches[VIdx], DotTolerance, FlatNormals[VIdx][1]);
					}

					if (Flat[VIdx])
					{
						NumFlat++;
					}
				}

				if (NumFlat == 0)
				{
					continue;
				}

				for (int RemoveVIdx = 0; RemoveVIdx < 2; RemoveVIdx++)
				{
					if (!Flat[RemoveVIdx])
					{
						continue;
					}
					int KeepVIdx = 1 - RemoveVIdx;
					Vector3 RemoveVPos = CutMesh[0]->GetVertex(e.Vert[RemoveVIdx]);
					Vector3 KeepVPos = CutMesh[0]->GetVertex(e.Vert[KeepVIdx]);
					Vector3 EdgeDir = KeepVPos - RemoveVPos;
					if (EdgeDir.SafeNormalize() == 0)
					{
						assert(!Geom->CollapseDegenerateEdgesOnCut);
						break;
					}

					bool HasBadEdge = false;
					for (int MeshIdx = 0; !HasBadEdge && MeshIdx < NumMeshesToProcess; MeshIdx++)
					{
						int RemoveV = MeshIdx == 0 ? e.Vert[RemoveVIdx] : Matches[RemoveVIdx];
						int KeepV = MeshIdx == 0 ? e.Vert[KeepVIdx] : Matches[KeepVIdx];
						int SourceEID = MeshIdx == 0 ? EID : OtherEID;

						HasBadEdge = HasBadEdge || LocalPlanarSimplify::CollapseWouldHurtTriangleQuality(*CutMesh[MeshIdx],
							FlatNormals[RemoveVIdx][MeshIdx], RemoveV, KeepV, KeepVPos, Geom->TryToImproveTriQualityThreshold);

						HasBadEdge = HasBadEdge || LocalPlanarSimplify::CollapseWouldChangeShapeOrUVs(
							*CutMesh[MeshIdx], CutBoundaryEdgeSets[MeshIdx], DotTolerance,
							SourceEID, RemoveV, RemoveVPos, KeepV, KeepVPos, EdgeDir, Geom->PreserveTriangleGroups,
							Geom->PreserveUVsOnlyForMesh == -1 || MeshIdx == Geom->PreserveUVsOnlyForMesh,
							Geom->PreserveVertexUVs, Geom->PreserveOverlayUVs, Geom->UVDistortTolerance * Geom->UVDistortTolerance,
							Geom->PreserveVertexNormals, cosf(Geom->NormalDistortTolerance * DEG_TO_RAD));
					};

					if (HasBadEdge)
					{
						continue;
					}

					bool AttemptCollapse = true;
					if (HasMatches)
					{
						int OtherRemoveV = Matches[RemoveVIdx];
						int OtherKeepV = Matches[KeepVIdx];

						int a = OtherRemoveV, b = OtherKeepV;
						int eab = CutMesh[1]->FindEdge(OtherRemoveV, OtherKeepV);

						const DynamicMesh::Edge& EdgeAB = CutMesh[1]->GetEdge(eab);
						int t0 = EdgeAB.Tri[0];
						if (t0 == -1)
						{
							AttemptCollapse = false;
						}
						else
						{
							Index3 T0tv = CutMesh[1]->GetTriangle(t0);
							int c = FindTriOtherVtx(a, b, T0tv);
							assert(EdgeAB.Tri[1] == -1);
							// Avoid collapsing into a duplicate triangle.
							CutMesh[1]->EnumerateVertexVertices(a, [&](int VID)
								{
									if (!AttemptCollapse || VID == c || VID == b)
									{
										return;
									}
									CutMesh[1]->EnumerateVertexVertices(b, [&](int VID2)
										{
											AttemptCollapse &= (VID != VID2);
										});
								});
						}
					}
					if (!AttemptCollapse)
					{
						break;
					}

					EdgeCollapseInfo CollapseInfo;
					int RemoveV = e.Vert[RemoveVIdx];
					int KeepV = e.Vert[KeepVIdx];
					// Preserve the boundary when a collapse removes a triangle with two boundary edges.
					auto WouldRemoveTwoBoundaryEdges = [](const DynamicMesh& Mesh, int EID, int RemoveV)
					{
						assert(Mesh.IsEdge(EID));
						int OppV = Mesh.GetEdgeOpposingV(EID).a;
						int NextOnTri = Mesh.FindEdge(RemoveV, OppV);
						return Mesh.IsBoundaryEdge(NextOnTri);
					};
					bool WouldRemoveNext = WouldRemoveTwoBoundaryEdges(*CutMesh[0], EID, RemoveV);
					EMeshResult CollapseResult = CutMesh[0]->CollapseEdge(KeepV, RemoveV, 0, CollapseInfo);
					if (CollapseResult == EMeshResult::Ok)
					{
						if (WouldRemoveNext && CutMesh[0]->IsBoundaryEdge(CollapseInfo.KeptEdges.a))
						{
							CutBoundaryEdgeSets[0].insert(CollapseInfo.KeptEdges.a);
						}

						if (HasMatches)
						{
							int OtherRemoveV = Matches[RemoveVIdx];
							int OtherKeepV = Matches[KeepVIdx];
							bool OtherWouldRemoveNext = WouldRemoveTwoBoundaryEdges(*CutMesh[1], OtherEID, OtherRemoveV);
							EdgeCollapseInfo OtherCollapseInfo;
							EMeshResult OtherCollapseResult = CutMesh[1]->CollapseEdge(OtherKeepV, OtherRemoveV, 0, OtherCollapseInfo);
							if (OtherCollapseResult != EMeshResult::Ok)
							{
								// The matched edge should fail only on already invalid topology.
								assert(OtherCollapseResult == EMeshResult::Failed_CollapseTriangle);
							}
							else
							{
								if (OtherWouldRemoveNext && CutMesh[1]->IsBoundaryEdge(OtherCollapseInfo.KeptEdges.a))
								{
									CutBoundaryEdgeSets[1].insert(OtherCollapseInfo.KeptEdges.a);
								}
								AllVIDMatches.erase(RemoveV);
								CutBoundaryEdgeSets[1].erase(OtherCollapseInfo.CollapsedEdge);
								CutBoundaryEdgeSets[1].erase(OtherCollapseInfo.RemovedEdges[0]);
								if (OtherCollapseInfo.RemovedEdges[1] != -1)
								{
									CutBoundaryEdgeSets[1].erase(OtherCollapseInfo.RemovedEdges[1]);
								}
							}
						}

						NumCollapses++;
						CutBoundaryEdgeSets[0].erase(CollapseInfo.CollapsedEdge);
						CutBoundaryEdgeSets[0].erase(CollapseInfo.RemovedEdges[0]);
						if (CollapseInfo.RemovedEdges[1] != -1)
						{
							CutBoundaryEdgeSets[0].erase(CollapseInfo.RemovedEdges[1]);
						}
					}
					break;
				}
			}

			CutBoundaryEdges[0] = ToVector<int>(CutBoundaryEdgeSets[0]);
			CutBoundaryEdges[1] = ToVector<int>(CutBoundaryEdgeSets[1]);

			if (NumCollapses == LastNumCollapses)
			{
				break;
			}

			CollapseIters++;
		}
	}

	}

	bool GeometryBoolean::Compute()
	{
		DynamicMesh CutMeshB(*Meshes[1]);

		Result = new DynamicMesh;
		*Result = *Meshes[0];

		DynamicMesh* CutMesh[2]{ Result, &CutMeshB };

		Box3 CombinedAABB = Box3::Transform(CutMesh[0]->GetBounds(), Transforms[0].pos, Transforms[0].quat);
		Box3 MeshB_AABB = Box3::Transform(CutMesh[1]->GetBounds(), Transforms[1].pos, Transforms[1].quat);
		CombinedAABB.Encapsulate(MeshB_AABB);
		for (int MeshIdx = 0; MeshIdx < 2; MeshIdx++)
		{
			Transform CenteredTransform = Transforms[MeshIdx];
			CenteredTransform.pos = CenteredTransform.pos - CombinedAABB.GetCenter();
			CutMesh[MeshIdx]->ApplyTransform(CenteredTransform, true);
			CutMesh[MeshIdx]->BuildBounds();
		}
		ResultTransform = Transform(CombinedAABB.GetCenter());

		DynamicMeshAABBTree Spatial[2]{ CutMesh[0], CutMesh[1] };
		IntersectionsQueryResult Intersections = Spatial[0].FindAllIntersections(Spatial[1], nullptr);

		const bool OpOnSingleMesh = false;

		GeometryCut Cut(CutMesh[0], CutMesh[1]);
		Cut.SnapTolerance = SnapTolerance;
		Cut.MutuallyCut = !OpOnSingleMesh;
		bool Success = Cut.Compute(Intersections);

		int NumMeshesToProcess = OpOnSingleMesh ? 1 : 2;
		float DegenerateEdgeTolFactor = 1.5f;

		if (CollapseDegenerateEdgesOnCut)
		{
			float DegenerateEdgeTolSq = DegenerateEdgeTolFactor * DegenerateEdgeTolFactor * SnapTolerance * SnapTolerance;
			for (int MeshIdx = 0; MeshIdx < NumMeshesToProcess; MeshIdx++)
			{
				std::vector<int> EIDs;
				for (int ChainIdx = 0; ChainIdx < Cut.VertexChains[MeshIdx].size();)
				{
					int ChainLen = Cut.VertexChains[MeshIdx][ChainIdx];
					int ChainEnd = ChainIdx + 1 + ChainLen;
					for (int ChainSubIdx = ChainIdx + 1; ChainSubIdx + 1 < ChainEnd; ChainSubIdx++)
					{
						int VID[2]{ Cut.VertexChains[MeshIdx][ChainSubIdx], Cut.VertexChains[MeshIdx][ChainSubIdx + 1] };
						if ((CutMesh[MeshIdx]->GetVertex(VID[0]) - CutMesh[MeshIdx]->GetVertex(VID[1])).SquareLength() < DegenerateEdgeTolSq)
						{
							EIDs.push_back(CutMesh[MeshIdx]->FindEdge(VID[0], VID[1]));
						}
					}
					ChainIdx = ChainEnd;
				}
				std::set<int> AllEIDs(EIDs.begin(), EIDs.end());
				for (int Idx = 0; Idx < EIDs.size(); Idx++)
				{
					int EID = EIDs[Idx];
					if (!CutMesh[MeshIdx]->IsEdge(EID))
					{
						continue;
					}
					Vector3 A, B;
					CutMesh[MeshIdx]->GetEdgeV(EID, A, B);
					if ((A - B).SquareLength() > DegenerateEdgeTolSq)
					{
						continue;
					}
					Index2 EV = CutMesh[MeshIdx]->GetEdgeV(EID);

					if (CutMesh[MeshIdx]->HasAttributes() && CutMesh[MeshIdx]->Attributes()->IsSeamVertex(EV.b, false))
					{
						std::swap(EV.a, EV.b);
						if (CutMesh[MeshIdx]->HasAttributes() && CutMesh[MeshIdx]->Attributes()->IsSeamVertex(EV.b, false))
						{
							continue;
						}
					}

					EdgeCollapseInfo CollapseInfo;
					EMeshResult CollapseResult = CutMesh[MeshIdx]->CollapseEdge(EV.a, EV.b, .5, CollapseInfo);
					if (CollapseResult != EMeshResult::Ok)
					{
						for (int i = 0; i < 2; i++)
						{
							if (AllEIDs.count(CollapseInfo.RemovedEdges[i]) > 0)
							{
								int ToAdd = CollapseInfo.KeptEdges[i];
								bool WasPresent = AllEIDs.count(ToAdd) > 0;
								if (!WasPresent)
								{
									AllEIDs.insert(ToAdd);
									EIDs.push_back(ToAdd);
								}
							}
						}
					}
				}
			}
		}

		// Cutting and degenerate cleanup change triangle IDs and bounds.
		for (int MeshIdx = 0; MeshIdx < NumMeshesToProcess; MeshIdx++)
		{
			Spatial[MeshIdx].Build();
		}

		std::vector<int> CutBoundaryEdges[2];
		// Boundary vertices that may need a matching vertex on the other mesh.
		std::set<int> PossUnmatchedBdryVerts[2];

		const float WindingThreshold = 0.5f;
		{
			// Classify triangles before deleting any geometry.
			std::vector<bool> KeepTri[2];
			// Mesh 0 keeps some coplanar faces; this tracks the paired mesh 1 face to delete.
			std::vector<int> DeleteIfOtherKept;
			if (NumMeshesToProcess > 1)
			{
				DeleteIfOtherKept.resize(CutMesh[0]->GetTriangleCount(), -1);
			}
			for (int MeshIdx = 0; MeshIdx < NumMeshesToProcess; MeshIdx++)
			{
				TFastWindingTree Winding(&Spatial[1 - MeshIdx]);
				DynamicMeshAABBTree& OtherSpatial = Spatial[1 - MeshIdx];
				DynamicMesh& ProcessMesh = *CutMesh[MeshIdx];
				int MaxTriID = ProcessMesh.GetTriangleCount();
				KeepTri[MeshIdx].resize(MaxTriID);
				bool CoplanarKeepSameDir = (Operation != BooleanOp::Difference);
				bool RemoveInside = true;
				if (Operation == BooleanOp::Intersect || (Operation == BooleanOp::Difference && MeshIdx == 1))
				{
					RemoveInside = false;
				}

				std::vector<Vector3> OtherNormals(OtherSpatial.Mesh->GetTriangleCount());
				for (size_t ii = 0; ii < OtherNormals.size(); ++ii)
				{
					OtherNormals[ii] = OtherSpatial.Mesh->GetTriNormal((int)ii);
				}

				const float OnPlaneTolerance = SnapTolerance;
				FQueryOptions NonDegenCoplanarCandidateFilter(OnPlaneTolerance,
					[&OtherNormals](int TID) -> bool
					{
						return !OtherNormals[TID].IsZero();
					});

				for (int TID = 0; TID < MaxTriID; ++TID)
				{
					if (!ProcessMesh.IsTriangle(TID))
					{
						continue;
					}

					Triangle3 Tri;
					ProcessMesh.GetTriVertices(TID, Tri.v0, Tri.v1, Tri.v2);
					Vector3 Centroid = Tri.GetCenter();

					// Prefer exact coplanar matches before falling back to winding.
					{
						float DSq;
						int OtherTID = OtherSpatial.FindNearestTriangle(Centroid, DSq, NonDegenCoplanarCandidateFilter);
						if (OtherTID > -1)
						{

							Vector3 OtherNormal = OtherNormals[OtherTID];
							Vector3 Normal = ProcessMesh.GetTriNormal(TID);
							float DotNormals = OtherNormal.Dot(Normal);

							{
								bool AllTrisOnOtherMesh = true;
								for (int Idx = 0; Idx < 3; Idx++)
								{
									if (OtherSpatial.FindNearestTriangle(Tri[Idx], DSq, OnPlaneTolerance * 2) == -1)
									{
										AllTrisOnOtherMesh = false;
										break;
									}
								}
								if (AllTrisOnOtherMesh)
								{
									// Mesh 0 owns coplanar faces; degenerate faces are removed.
									if (MeshIdx != 0 || Normal.IsZero())
									{
										KeepTri[MeshIdx][TID] = false;
										continue;
									}
									else
									{
										bool Keep = DotNormals > 0 == CoplanarKeepSameDir;
										KeepTri[MeshIdx][TID] = Keep;
										if (NumMeshesToProcess > 1 && Keep)
										{
											DeleteIfOtherKept[TID] = OtherTID;
										}
										continue;
									}
								}
							}
						}
					}

					float WindingNum = Winding.FastWindingNumber(Centroid);
					KeepTri[MeshIdx][TID] = (WindingNum > WindingThreshold) != RemoveInside;
				};
			}

			if (NumMeshesToProcess > 1)
			{
				for (int TID = 0; TID < CutMesh[0]->GetTriangleCount(); ++TID)
				{
					if (!CutMesh[0]->IsTriangleFast(TID))
					{
						continue;
					}

					int DeleteIfOtherKeptTID = DeleteIfOtherKept[TID];
					if (DeleteIfOtherKeptTID > -1 && KeepTri[1][DeleteIfOtherKeptTID])
					{
						KeepTri[0][TID] = false;
					}
				}
			}

			for (int MeshIdx = 0; MeshIdx < NumMeshesToProcess; MeshIdx++)
			{
				DynamicMesh& ProcessMesh = *CutMesh[MeshIdx];
				for (int EID = 0; EID < ProcessMesh.GetEdgeCount(); ++EID)
				{
					if (!ProcessMesh.IsEdgeFast(EID))
					{
						continue;
					}

					const DynamicMesh::Edge& e = ProcessMesh.GetEdge(EID);
					if (e.Tri[1] == -1 || KeepTri[MeshIdx][e.Tri[0]] == KeepTri[MeshIdx][e.Tri[1]])
					{
						continue;
					}

					CutBoundaryEdges[MeshIdx].push_back(EID);
					PossUnmatchedBdryVerts[MeshIdx].insert(e.Vert[0]);
					PossUnmatchedBdryVerts[MeshIdx].insert(e.Vert[1]);
				}
			}

			for (int MeshIdx = 0; MeshIdx < NumMeshesToProcess; MeshIdx++)
			{
				DynamicMesh& ProcessMesh = *CutMesh[MeshIdx];

				for (int TID = 0; TID < KeepTri[MeshIdx].size(); TID++)
				{
					if (ProcessMesh.IsTriangle(TID) && !KeepTri[MeshIdx][TID])
					{
						ProcessMesh.RemoveTriangle(TID, true, false);
					}
				}
			}
		}

		std::map<int, int> AllVIDMatches;
		if (NumMeshesToProcess == 2)
		{
			std::map<int, int> FoundMatchesMaps[2];
			float SnapToleranceSq = SnapTolerance * SnapTolerance;

			// Keep boundary loops from both meshes in one-to-one correspondence.
			for (int MeshIdx = 0; MeshIdx < 2; MeshIdx++)
			{
				int OtherMeshIdx = 1 - MeshIdx;
				DynamicMesh& OtherMesh = *CutMesh[OtherMeshIdx];

				SparseSpatialHash3<int> OtherMeshPointHash(OtherMesh.GetBounds().MaxDim() / 64, 1024);
				for (int BoundaryVID : PossUnmatchedBdryVerts[OtherMeshIdx])
				{
					OtherMeshPointHash.Insert(OtherMesh.GetVertex(BoundaryVID), BoundaryVID);
				}

				SparseOctree EdgeOctree(OtherMesh.GetBounds().Min, OtherMesh.GetBounds().Max, 7);
				auto EdgeBounds = [&OtherMesh](int EID)
				{
					const DynamicMesh::Edge& e = OtherMesh.GetEdge(EID);
					Vector3 A = OtherMesh.GetVertex(e.Vert[0]);
					Vector3 B = OtherMesh.GetVertex(e.Vert[1]);
					if (A.x > B.x)
					{
						std::swap(A.x, B.x);
					}
					if (A.y > B.y)
					{
						std::swap(A.y, B.y);
					}
					if (A.z > B.z)
					{
						std::swap(A.z, B.z);
					}
					return Box3(A, B);
				};
				auto AddEdge = [&EdgeOctree, &OtherMesh, EdgeBounds](int EID)
				{
					EdgeOctree.InsertObject(EID, EdgeBounds(EID));
				};
				auto UpdateEdge = [&EdgeOctree, &OtherMesh, EdgeBounds](int EID)
				{
					EdgeOctree.ReinsertObject(EID, EdgeBounds(EID));
				};
				for (int EID : CutBoundaryEdges[OtherMeshIdx])
				{
					AddEdge(EID);
				}
				std::vector<int> EdgesInRange;

				std::map<int, int>& FoundMatches = FoundMatchesMaps[MeshIdx];

				for (int BoundaryVID : PossUnmatchedBdryVerts[MeshIdx])
				{
					if (MeshIdx == 1 && FoundMatchesMaps[0].count(BoundaryVID) > 0)
					{
						continue;
					}

					Vector3 Pos = CutMesh[MeshIdx]->GetVertex(BoundaryVID);
					int NearestVID = -1;
					if (OtherMeshPointHash.FindNearest(Pos, SnapTolerance, NearestVID))
					{
						float DSq = (OtherMesh.GetVertex(NearestVID) - Pos).SquareLength();
						auto it = FoundMatches.find(NearestVID);
						if (it != FoundMatches.end())
						{
							int Match = it->second;
							float OldDSq = (CutMesh[MeshIdx]->GetVertex(Match) - OtherMesh.GetVertex(NearestVID)).SquareLength();
							if (DSq < OldDSq)
							{
								int OldVID = Match;
								FoundMatches.emplace(NearestVID, BoundaryVID);
								BoundaryVID = OldVID;
								Pos = CutMesh[MeshIdx]->GetVertex(BoundaryVID);
								DSq = OldDSq;
							}
							NearestVID = -1;
						}
						else
						{
							FoundMatches.emplace(NearestVID, BoundaryVID);
						}
					}

					if (NearestVID == -1)
					{
						Box3 QueryBox(Pos, SnapTolerance);
						EdgesInRange.clear();
						EdgeOctree.RangeQuery(QueryBox, EdgesInRange);

						int OtherEID = FindNearestEdge(OtherMesh, EdgesInRange, Pos);
						if (OtherEID != -1)
						{
							Vector3 EdgePts[2];
							OtherMesh.GetEdgeV(OtherEID, EdgePts[0], EdgePts[1]);
							if ((EdgePts[0] - Pos).SquareLength() > SnapToleranceSq && (EdgePts[1] - Pos).SquareLength() > SnapToleranceSq)
							{
								Segment3 Seg(EdgePts[0], EdgePts[1]);
								float Along = Seg.ProjectUnitRange(Pos);
								EdgeSplitInfo SplitInfo;
								if (EMeshResult::Ok == OtherMesh.SplitEdge(OtherEID, SplitInfo, Along))
								{
									FoundMatches.emplace(SplitInfo.NewVertex, BoundaryVID);
									OtherMesh.SetVertex(SplitInfo.NewVertex, Pos);
									CutBoundaryEdges[OtherMeshIdx].push_back(SplitInfo.NewEdges.a);
									UpdateEdge(OtherEID);
									AddEdge(SplitInfo.NewEdges.a);
								}
							}
						}
					}
				}

				for (auto& Match : FoundMatches)
				{
					CutMesh[MeshIdx]->SetVertex(Match.second, OtherMesh.GetVertex(Match.first));

					int VIDs[2]{ Match.first, Match.second };
					AllVIDMatches.emplace(VIDs[1 - MeshIdx], VIDs[MeshIdx]);
				}
			}
		}

		if (SimplifyAlongNewEdges)
		{
			SimplifyCutEdges(NumMeshesToProcess, CutMesh, CutBoundaryEdges, AllVIDMatches, this);
		}

		if (Operation == BooleanOp::Difference)
		{
			std::vector<int> AllTID;
			for (int TID  = 0; TID < CutMesh[1]->GetTriangleCount(); ++TID)
			{
				if (!CutMesh[1]->IsTriangleFast(TID))
				{
					continue;
				}

				AllTID.push_back(TID);
			}
			FDynamicMeshEditor FlipEditor(CutMesh[1]);
			FlipEditor.ReverseTriangleOrientations(AllTID, true);
		}

		if (NumMeshesToProcess > 1)
		{
			FDynamicMeshEditor Editor(Result);
			FMeshIndexMappings IndexMaps;
			Editor.AppendMesh(CutMesh[1], IndexMaps);

			if (PopulateSecondMeshGroupMap)
			{
				SecondMeshGroupMap = IndexMaps.GetGroupMap();
			}

			if (WeldSharedEdges)
			{
				bool WeldSuccess = MergeEdges(IndexMaps, CutMesh, CutBoundaryEdges, AllVIDMatches, this);
				Success = Success && WeldSuccess;
			}
			else
			{
				CreatedBoundaryEdges = CutBoundaryEdges[0];
				for (int OldMeshEID : CutBoundaryEdges[1])
				{
					if (!CutMesh[1]->IsEdge(OldMeshEID))
					{
						assert(false);
						continue;
					}
					Index2 OtherEV = CutMesh[1]->GetEdgeV(OldMeshEID);
					int MappedEID = Result->FindEdge(IndexMaps.GetNewVertex(OtherEV.a), IndexMaps.GetNewVertex(OtherEV.b));
					assert(Result->IsBoundaryEdge(MappedEID));
					CreatedBoundaryEdges.push_back(MappedEID);
				}
			}
		}
		else
		{
			CreatedBoundaryEdges = CutBoundaryEdges[0];
		}

		if (TrackAllNewEdges)
		{
			for (int eid : CreatedBoundaryEdges)
			{
				AllNewEdges.insert(eid);
			}
		}

		if (PutResultInInputSpace)
		{
			Result->ApplyTransform(ResultTransform, false);
			ResultTransform = Transform::Identity();
		}

		return Success;
	}

}
