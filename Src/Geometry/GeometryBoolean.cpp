#include <assert.h>
#include <vector>
#include <functional>
#include <map>
#include <set>
#include "../CollisionPrimitive/Segment3.h"
#include "../CollisionPrimitive/Triangle3.h"
#include "../Core/Base.h"
#include "../Core/DynamicArray.h"
#include "../Core/PriorityQueue.h"
#include "../Maths/Maths.h"
#include "GeometryBoolean.h"
#include "DynamicMesh.h"
#include "SparseOctree.h"
#include "SparseSpatialHash.h"
#include "WindingNumber.h"

namespace Riemann
{

	enum class EVertexType
	{
		Unknown = -1,
		Vertex = 0,
		Edge = 1,
		Face = 2
	};

	struct FPtOnMesh
	{
		Vector3 Pos;
		EVertexType Type = EVertexType::Unknown;
		int ElemID = -1;
	};

	struct FSegmentToElements
	{
		int BaseTID;
		int PtOnMeshIdx[2];
	};

	enum class ESurfacePoint
	{
		Vertex = 0,
		Edge = 1,
		Triangle = 2
	};

	struct FMeshSurfacePoint
	{
		int ElementID;
		Vector3 BaryCoord;
		ESurfacePoint Point;

		FMeshSurfacePoint() : ElementID(-1)
		{
		}
		FMeshSurfacePoint(int TriangleID, const Vector3& BaryCoord) : ElementID(TriangleID), BaryCoord(BaryCoord), Point(ESurfacePoint::Triangle)
		{
		}
		FMeshSurfacePoint(int EdgeID, float FirstCoordWt) : ElementID(EdgeID), BaryCoord(FirstCoordWt, 1 - FirstCoordWt, 0), Point(ESurfacePoint::Edge)
		{
		}
		FMeshSurfacePoint(int VertexID) : ElementID(VertexID), BaryCoord(1, 0, 0), Point(ESurfacePoint::Vertex)
		{
		}

		Vector3 Pos(const DynamicMesh* Mesh) const
		{
			if (Point == ESurfacePoint::Vertex)
			{
				return Mesh->GetVertex(ElementID);
			}
			else if (Point == ESurfacePoint::Edge)
			{
				Vector3 EA, EB;
				Mesh->GetEdgeV(ElementID, EA, EB);
				return BaryCoord[0] * EA + BaryCoord[1] * EB;
			}
			else // Point == ESurfacePoint::Triangle
			{
				Vector3 TA, TB, TC;
				Mesh->GetTriVertices(ElementID, TA, TB, TC);
				return BaryCoord[0] * TA + BaryCoord[1] * TB + BaryCoord[2] * TC;
			}
		}
	};

	class FMeshSurfacePath
	{
	public:
		DynamicMesh* Mesh;
		std::vector<std::pair<FMeshSurfacePoint, int>> Path;
		bool bIsClosed;

	public:
		FMeshSurfacePath(DynamicMesh* Mesh) : Mesh(Mesh), bIsClosed(false)
		{
		}
		virtual ~FMeshSurfacePath() {}

		bool IsConnected() const
		{
			int Idx = 1, LastIdx = 0;
			if (bIsClosed)
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
					const FMeshSurfacePoint& P = Path[Inds[IndIdx]].first;
					switch (P.Point)
					{
					case ESurfacePoint::Triangle:
						if (P.ElementID != WalkingOnTri)
						{
							return false;
						}
						break;
					case ESurfacePoint::Edge:
						if (!Mesh->GetEdgeT(P.ElementID).Contains(WalkingOnTri))
						{
							return false;
						}
						break;
					case ESurfacePoint::Vertex:
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
			return bIsClosed;
		}

		void Reset()
		{
			Path.clear();
			bIsClosed = false;
		}

		bool AddViaPlanarWalk(int StartTri, int StartVID, Vector3 StartPt, int EndTri, int EndVertID, Vector3 EndPt,
			Vector3 WalkPlaneNormal, std::function<Vector3(const DynamicMesh*, int)> VertexToPosnFn = nullptr,
			bool bAllowBackwardsSearch = true, float AcceptEndPtOutsideDist = 1e-6f,
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
				bAllowBackwardsSearch, AcceptEndPtOutsideDist, PtOnPlaneThresholdSq, Path, BackwardsTolerance);
		}

		bool EmbedSimplePath(bool bUpdatePath, std::vector<int>& PathVertices, bool bDoNotDuplicateFirstVertexID = true, float SnapElementThresholdSq = 1e-4f)
		{
			int InitialPathIdx = (int)PathVertices.size();
			if (Path.empty())
			{
				return true;
			}

			int PathNum = (int)Path.size();
			const FMeshSurfacePoint& OrigEndPt = Path[PathNum - 1].first;

			int StartProcessIdx = 0, EndSimpleProcessIdx = PathNum - 1;
			bool bEndPointSpecialProcess = false;
			if (PathNum > 1 && OrigEndPt.Point == ESurfacePoint::Triangle)
			{
				EndSimpleProcessIdx = PathNum - 2;
				bEndPointSpecialProcess = true;
			}
			bool bNeedFinalRelocate = false;
            (void)bNeedFinalRelocate;
			FMeshSurfacePoint EndPtUpdated = Path.back().first;
			Vector3 EndPtPos = OrigEndPt.Pos(Mesh);

			if (Path[0].first.Point == ESurfacePoint::Triangle)
			{
				FPokeTriangleInfo PokeInfo;
				Mesh->PokeTriangle(Path[0].first.ElementID, Path[0].first.BaryCoord, PokeInfo);
				if (EndPtUpdated.Point == ESurfacePoint::Triangle && Path[0].first.ElementID == EndPtUpdated.ElementID)
				{
					EndPtUpdated = RelocateTrianglePointAfterRefinement(Mesh, EndPtPos, { PokeInfo.NewTriangles.a, PokeInfo.NewTriangles.b, PokeInfo.OriginalTriangle }, SnapElementThresholdSq);
				}
				PathVertices.push_back(PokeInfo.NewVertex);
				StartProcessIdx = 1;
			}
			for (int PathIdx = StartProcessIdx; PathIdx <= EndSimpleProcessIdx; PathIdx++)
			{
				if (!(Path[PathIdx].first.Point != ESurfacePoint::Triangle))
				{
					return false;
				}
				const FMeshSurfacePoint& Pt = Path[PathIdx].first;
				if (Pt.Point == ESurfacePoint::Edge)
				{
					assert(Mesh->IsEdge(Pt.ElementID));
					FEdgeSplitInfo SplitInfo;
					Mesh->SplitEdge(Pt.ElementID, SplitInfo, Pt.BaryCoord[0]);
					PathVertices.push_back(SplitInfo.NewVertex);
					if (EndPtUpdated.Point == ESurfacePoint::Triangle && SplitInfo.OriginalTriangles.Contains(EndPtUpdated.ElementID))
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
					else if (PathIdx != PathNum - 1 && EndPtUpdated.Point == ESurfacePoint::Edge && Pt.ElementID == EndPtUpdated.ElementID)
					{
						assert(false);
					}
				}
				else
				{
					assert(Pt.Point == ESurfacePoint::Vertex);
					assert(Mesh->IsVertex(Pt.ElementID));
					if (!bDoNotDuplicateFirstVertexID || PathVertices.size() != InitialPathIdx || 0 == PathVertices.size() || PathVertices.back() != Pt.ElementID)
					{
						PathVertices.push_back(Pt.ElementID);
					}
				}
			}

			if (bEndPointSpecialProcess)
			{
				if (EndPtUpdated.Point == ESurfacePoint::Triangle)
				{
					FPokeTriangleInfo PokeInfo;
					Mesh->PokeTriangle(EndPtUpdated.ElementID, EndPtUpdated.BaryCoord, PokeInfo);
					PathVertices.push_back(PokeInfo.NewVertex);
				}
				else if (EndPtUpdated.Point == ESurfacePoint::Edge)
				{
					FEdgeSplitInfo SplitInfo;
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

			if (bUpdatePath)
			{
				assert(false);
			}

			return true;
		}

	private:
		struct FIndexDistance
		{
			int Index;
			float PathLength;
			float DistanceToEnd;
			bool operator<(const FIndexDistance& Other) const
			{
				return PathLength + DistanceToEnd < Other.PathLength + Other.DistanceToEnd;
			}
		};

		static bool WalkMeshPlanar(const DynamicMesh* Mesh, int StartTri, int StartVID, Vector3 StartPt, int EndTri, int EndVertID, Vector3 EndPt, Vector3 WalkPlaneNormal, std::function<Vector3(const DynamicMesh*, int)> VertexToPosnFn,
			bool bAllowBackwardsSearch, float AcceptEndPtOutsideDist, float PtOnPlaneThresholdSq, std::vector<std::pair<FMeshSurfacePoint, int>>& WalkedPath, float BackwardsTolerance)
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

			struct FWalkIndices
			{
				Vector3 Position;
				int WalkedFromPt;
				int	WalkingOnTri;

				FWalkIndices() : WalkedFromPt(-1), WalkingOnTri(-1)
				{}

				FWalkIndices(Vector3 Position, int FromPt, int OnTri) : Position(Position), WalkedFromPt(FromPt), WalkingOnTri(OnTri)
				{}
			};

			std::vector<std::pair<FMeshSurfacePoint, FWalkIndices>> ComputedPointsAndSources;
			PriorityQueue<FIndexDistance> UnexploredEnds;

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
				ComputedPointsAndSources.emplace_back(FMeshSurfacePoint(StartTri, CurrentTriDist.BaryCoords), FWalkIndices(StartPt, -1, StartTri));
			}
			else
			{
				CurrentTriDist.BaryCoords = Vector3::Zero();
				CurrentTriDist.BaryCoords[StartVIDIndex] = 1.0;
				CurrentTriDist.ClosestPoint = StartPt;
				ComputedPointsAndSources.emplace_back(FMeshSurfacePoint(StartTri, CurrentTriDist.BaryCoords), FWalkIndices(StartPt, -1, StartTri));
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
					FIndexDistance TopEndWithDistance = UnexploredEnds.pop();

					CurrentEnd = TopEndWithDistance.Index;
					CurrentPathLength = TopEndWithDistance.PathLength;
					CurrentDistanceToEnd = TopEndWithDistance.DistanceToEnd;
				}
				else
				{
					return false; // failed to find path
				}

				FMeshSurfacePoint FromPt = ComputedPointsAndSources[CurrentEnd].first;
				FWalkIndices CurrentWalk = ComputedPointsAndSources[CurrentEnd].second;
				int TriID = CurrentWalk.WalkingOnTri;
				assert(Mesh->IsTriangle(TriID));
				Index3 TriVertIDs = Mesh->GetTriangle(TriID);
				SetTriVertPositions(TriVertIDs, CurrentTri);

				if (EndVertID >= 0 && TriVertIDs.Contains(EndVertID))
				{
					CurrentEnd = (int)ComputedPointsAndSources.size();
					ComputedPointsAndSources.emplace_back(FMeshSurfacePoint(EndVertID), FWalkIndices(EndPt, CurrentEnd, TriID));
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
					CurrentEnd = (int)ComputedPointsAndSources.size();
					ComputedPointsAndSources.emplace_back(FMeshSurfacePoint(TriID, CurrentTriDist.BaryCoords), FWalkIndices(EndPt, CurrentEnd, TriID));

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
						// Vertex crossing
						Side[TriSubIdx] = 0;
						int CandidateVertID = TriVertIDs[TriSubIdx];
						if (FromPt.Point != ESurfacePoint::Vertex || CandidateVertID != FromPt.ElementID)
						{
							FMeshSurfacePoint SurfPt(CandidateVertID);
							FWalkIndices WalkInds(CurrentTri[TriSubIdx], CurrentEnd, -1);
							bool bIsForward = ForwardsDirection.Dot(CurrentTri[TriSubIdx] - StartPt) >= -BackwardsTolerance;
							if ((bAllowBackwardsSearch || bIsForward) && !CrossedVertices.count(CandidateVertID))
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
						// edge crossing
						int CandidateEdgeID = TriEdgeIDs[TriSubIdx];
						if (FromPt.Point != ESurfacePoint::Edge || CandidateEdgeID != FromPt.ElementID)
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
							bool bIsForward = ForwardsDirection.Dot(CrossingP - StartPt) >= -BackwardsTolerance;
							if (!bAllowBackwardsSearch && !bIsForward)
							{
								continue;
							}
							ComputedPointsAndSources.emplace_back(FMeshSurfacePoint(CandidateEdgeID, CrossingT), FWalkIndices(CrossingP, CurrentEnd, CrossToTriID));
						}
					}
				}

				const Vector3& PreviousPathPoint = ComputedPointsAndSources[CurrentEnd].second.Position;
				for (int NewComputedPtIdx = InitialComputedPointsNum; NewComputedPtIdx < ComputedPointsAndSources.size(); NewComputedPtIdx++)
				{
					const Vector3& CurrentPathPoint = ComputedPointsAndSources[NewComputedPtIdx].second.Position;
					float PathLength = CurrentPathLength + (PreviousPathPoint - CurrentPathPoint).Length();
					float DistanceToEnd = (EndPt - CurrentPathPoint).Length();

					bool bIsForward = ForwardsDirection.Dot(CurrentPathPoint - StartPt) >= -BackwardsTolerance;
					if (bAllowBackwardsSearch || bIsForward)
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

			if (WalkedPath.size() && WalkedPath[0].first.Point == ESurfacePoint::Triangle)
			{
				FMeshSurfacePoint& SurfacePt = WalkedPath[0].first;

				if (StartVIDIndex > -1 && SurfacePt.BaryCoord[StartVIDIndex] == 1.0)
				{
					Index3 TriVertIDs = Mesh->GetTriangle(SurfacePt.ElementID);
					SurfacePt.ElementID = TriVertIDs[StartVIDIndex];
					SurfacePt.Point = ESurfacePoint::Vertex;
				}
				else
				{
					RefineSurfacePtFromTriangleToSubElement(Mesh, SurfacePt.Pos(Mesh), SurfacePt, PtOnPlaneThresholdSq);
				}
				if (WalkedPath.size() > 1 &&
					SurfacePt.Point != ESurfacePoint::Triangle &&
					SurfacePt.Point == WalkedPath[1].first.Point &&
					SurfacePt.ElementID == WalkedPath[1].first.ElementID)
				{
					if (SurfacePt.Point == ESurfacePoint::Edge)
					{
						WalkedPath[1].first.BaryCoord = SurfacePt.BaryCoord;
					}
					WalkedPath.erase(WalkedPath.begin());
				}
			}
			if (WalkedPath.size() && WalkedPath.back().first.Point == ESurfacePoint::Triangle)
			{
				FMeshSurfacePoint& SurfacePt = WalkedPath.back().first;
				RefineSurfacePtFromTriangleToSubElement(Mesh, SurfacePt.Pos(Mesh), SurfacePt, PtOnPlaneThresholdSq);
				if (WalkedPath.size() > 1 &&
					SurfacePt.Point != ESurfacePoint::Triangle &&
					SurfacePt.Point == WalkedPath[WalkedPath.size() - 2].first.Point &&
					SurfacePt.ElementID == WalkedPath[WalkedPath.size() - 2].first.ElementID)
				{
					if (SurfacePt.Point == ESurfacePoint::Edge) // copy closer barycoord
					{
						WalkedPath[WalkedPath.size() - 2].first.BaryCoord = SurfacePt.BaryCoord;
					}
					WalkedPath.pop_back();
				}
			}

			return true;
		}

		static void RefineSurfacePtFromTriangleToSubElement(const DynamicMesh* Mesh, Vector3 Pos, FMeshSurfacePoint& SurfacePt, float SnapElementThresholdSq)
		{
			if (!(SurfacePt.Point == ESurfacePoint::Triangle))
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
				SurfacePt.Point = ESurfacePoint::Vertex;
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
				SurfacePt.Point = ESurfacePoint::Edge;
				SurfacePt.BaryCoord = Vector3(BestEdgeParam, 1 - BestEdgeParam, 0);
				return;
			}
		}

		static FMeshSurfacePoint RelocateTrianglePointAfterRefinement(const DynamicMesh* Mesh, const Vector3& PosInVertexCoordSpace, const std::vector<int>& TriIDs, float SnapElementThresholdSq)
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
			FMeshSurfacePoint SurfacePt(BestTriID, BestBaryCoords);
			RefineSurfacePtFromTriangleToSubElement(Mesh, PosInVertexCoordSpace, SurfacePt, SnapElementThresholdSq);
			return SurfacePt;
		}


	};

	struct FCutWorkingInfo
	{
		FCutWorkingInfo(DynamicMesh* WorkingMesh, float SnapTolerance) : Mesh(WorkingMesh), SnapToleranceSq(SnapTolerance* SnapTolerance)
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
		std::vector<FPtOnMesh> IntersectionVerts;
		std::vector<FSegmentToElements> Segments;

		void AddSegments(const IntersectionsQueryResult& Intersections, int WhichSide)
		{
			size_t SegStart = Segments.size();
			Segments.resize(SegStart + Intersections.Segments.size());

			// classify the points of each intersection segment as on-vertex, on-edge, or on-face
			for (size_t SegIdx = 0, SegCount = Intersections.Segments.size(); SegIdx < SegCount; SegIdx++)
			{
				const IntersectionsQueryResult::SegmentIntersection& Seg = Intersections.Segments[SegIdx];
				FSegmentToElements& SegToEls = Segments[SegStart + SegIdx];
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
					FPtOnMesh& PtOnMesh = IntersectionVerts.back();
					PtOnMesh.Pos = Seg.Point[SegPtIdx];
					SegToEls.PtOnMeshIdx[SegPtIdx] = NewPtIdx;

					// decide whether the point is on a vertex, edge or triangle

					int OnVertexIdx = OnVertex(Tri, PtOnMesh.Pos);
					if (OnVertexIdx > -1)
					{
						PtOnMesh.Type = EVertexType::Vertex;
						PtOnMesh.ElemID = TriVIDs[OnVertexIdx];
						continue;
					}

					// assert for an edge match
					int OnEdgeIdx = OnEdge(Tri, PtOnMesh.Pos);
					if (OnEdgeIdx > -1)
					{
						// if segment is degenerate and stuck to one edge, see if it could cross
						if (PrevOnEdgeIdx == OnEdgeIdx && (PrevOnEdgePos - PtOnMesh.Pos).SquareLength() < SnapToleranceSq)
						{
							int OnEdgeReplaceIdx = OnEdgeWithSkip(Tri, PtOnMesh.Pos, OnEdgeIdx);
							if (OnEdgeReplaceIdx > -1)
							{
								OnEdgeIdx = OnEdgeReplaceIdx;
							}
						}
						PtOnMesh.Type = EVertexType::Edge;
						PtOnMesh.ElemID = Mesh->GetTriEdges(SegToEls.BaseTID, OnEdgeIdx);

						assert(PtOnMesh.ElemID > -1);
						EdgeVertices.emplace(PtOnMesh.ElemID, NewPtIdx);

						PrevOnEdgeIdx = OnEdgeIdx;
						PrevOnEdgePos = PtOnMesh.Pos;

						continue;
					}

					// wasn't vertex or edge, so it's a face vertex
					PtOnMesh.Type = EVertexType::Face;
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

				FPtOnMesh& Pt = IntersectionVerts[PtIdx];

				Mesh->GetTriVertices(TID, Tri.v0, Tri.v1, Tri.v2);

				Vector3 BaryCoords = Tri.BarycentricCoods(Pt.Pos);
				FPokeTriangleInfo PokeInfo;
				EMeshResult Result = Mesh->PokeTriangle(TID, BaryCoords, PokeInfo);
				assert(Result == EMeshResult::Ok);
				int PokeVID = PokeInfo.NewVertex;
				Mesh->SetVertex(PokeVID, Pt.Pos);
				Pt.ElemID = PokeVID;
				Pt.Type = EVertexType::Vertex;

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

						FPtOnMesh& RelocatePt = IntersectionVerts[RelocatePtIdx];
						UpdateFromPoke(RelocatePt, PokeInfo.NewVertex, PokeInfo.NewEdges, PokeTriangles);
						if (RelocatePt.Type == EVertexType::Edge)
						{
							assert(RelocatePt.ElemID > -1);
							EdgeVertices.emplace(RelocatePt.ElemID, RelocatePtIdx);
						}
						else if (RelocatePt.Type == EVertexType::Face)
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

				int nums = (int)FaceVertices.count(EID);
				auto it = EdgeVertices.find(EID);
				while (nums--)
				{
					PtIndices.push_back(it->second);
					++it;
				}

				FPtOnMesh& Pt = IntersectionVerts[PtIdx];

				Vector3 EA, EB;
				Mesh->GetEdgeV(EID, EA, EB);
				Segment3 Seg(EA, EB);
				float SplitParam = Seg.ProjectUnitRange(Pt.Pos);

				Index2 SplitTris = Mesh->GetEdgeT(EID);
                (void)SplitTris;
				FEdgeSplitInfo SplitInfo;
				EMeshResult Result = Mesh->SplitEdge(EID, SplitInfo, SplitParam);
				assert(Result == EMeshResult::Ok);

				Mesh->SetVertex(SplitInfo.NewVertex, Pt.Pos);
				Pt.ElemID = SplitInfo.NewVertex;
				Pt.Type = EVertexType::Vertex;

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

						FPtOnMesh& RelocatePt = IntersectionVerts[RelocatePtIdx];
						UpdateFromSplit(RelocatePt, SplitInfo.NewVertex, SplitEdges);
						if (RelocatePt.Type == EVertexType::Edge)
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

			bool bSuccess = true; // remains true if we successfully connect all edges

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
				FSegmentToElements& Seg = Segments[SegIdx];
				if (Seg.PtOnMeshIdx[0] == Seg.PtOnMeshIdx[1])
				{
					continue; // degenerate case, but OK
				}
				FPtOnMesh& PtA = IntersectionVerts[Seg.PtOnMeshIdx[0]];
				FPtOnMesh& PtB = IntersectionVerts[Seg.PtOnMeshIdx[1]];
				if (!(PtA.Type == EVertexType::Vertex && PtB.Type == EVertexType::Vertex && PtA.ElemID != -1 && PtB.ElemID != -1))
				{
					bSuccess = false;
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
					continue; // degenerate case, but OK
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
					continue; // already connected
				}

				FMeshSurfacePath SurfacePath(Mesh);
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

				bool bWalkSuccess = SurfacePath.AddViaPlanarWalk(StartTID, PtA.ElemID,
					Mesh->GetVertex(PtA.ElemID), -1, PtB.ElemID,
					Mesh->GetVertex(PtB.ElemID), WalkPlaneNormal, nullptr, false, 1e-6f, SnapToleranceSq, 0.001f);
				if (!bWalkSuccess)
				{
					bSuccess = false;
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
							VertexChains->insert(VertexChains->begin(), EmbeddedPath.begin(), EmbeddedPath.end());
						}
					}
					else
					{
						bSuccess = false;
					}
				}
			}

			return bSuccess;
		}

		void UpdateFromSplit(FPtOnMesh& Pt, int SplitVertex, const Index2& SplitEdges)
		{
			if ((Pt.Pos - Mesh->GetVertex(SplitVertex)).SquareLength() < SnapToleranceSq)
			{
				Pt.Type = EVertexType::Vertex;
				Pt.ElemID = SplitVertex;
				return;
			}

			int EdgeIdx = ClosestEdge(SplitEdges, Pt.Pos);
			assert(EdgeIdx > -1 && EdgeIdx < 2 && SplitEdges[EdgeIdx]>-1);
			Pt.Type = EVertexType::Edge;
			Pt.ElemID = SplitEdges[EdgeIdx];
		}

		void UpdateFromPoke(FPtOnMesh& Pt, int PokeVertex, const Index3& PokeEdges, const Index3& PokeTris)
		{
			if ((Pt.Pos - Mesh->GetVertex(PokeVertex)).SquareLength() < SnapToleranceSq)
			{
				Pt.Type = EVertexType::Vertex;
				Pt.ElemID = PokeVertex;
				return;
			}

			int EdgeIdx = OnEdge(PokeEdges, Pt.Pos, SnapToleranceSq);
			if (EdgeIdx > -1)
			{
				Pt.Type = EVertexType::Edge;
				Pt.ElemID = PokeEdges[EdgeIdx];
				return;
			}

			for (int j = 0; j < 3; ++j)
			{
				if (IsInTriangle(PokeTris[j], Pt.Pos))
				{
					assert(Pt.Type == EVertexType::Face);
					Pt.ElemID = PokeTris[j];
					return;
				}
			}

			EdgeIdx = OnEdge(PokeEdges, Pt.Pos, FLT_MAX);
			Pt.Type = EVertexType::Edge;
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

		int ClosestEdge(Index2 EIDs, const Vector3& Pos)
		{
			int BestIdx = -1;
			float BestDSq = SnapToleranceSq;
			for (int Idx = 0; Idx < 2; Idx++)
			{
				int EID = EIDs[Idx];
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
			bool bSuccess = true;

			int MeshesToProcess = bMutuallyCut ? 2 : 1;
			for (int MeshIdx = 0; MeshIdx < MeshesToProcess; MeshIdx++)
			{
				FCutWorkingInfo WorkingInfo(Mesh[MeshIdx], SnapTolerance);
				WorkingInfo.AddSegments(Intersections, MeshIdx);
				WorkingInfo.InsertFaceVertices();
				WorkingInfo.InsertEdgeVertices();

				bool bConnected = WorkingInfo.ConnectEdges(&VertexChains[MeshIdx], &SegmentToChain[MeshIdx]);
				if (!bConnected)
				{
					bSuccess = false;
				}
			}

			return bSuccess;
		}

		float SnapTolerance = 1e-4f;
		bool bMutuallyCut = true;
		bool bCutCoplanar = false;

		std::vector<int> VertexChains[2];
		std::vector<int> SegmentToChain[2];
	};

	class FLocalPlanarSimplify
	{
	public:
		float SimplificationAngleTolerance = 0.1f;
		float TryToImproveTriQualityThreshold = 0.25f;
		bool bPreserveTriangleGroups = true;
		bool bPreserveVertexUVs = true;
		bool bPreserveOverlayUVs = true;
		float UVDistortTolerance = 1e-6f;
		bool bPreserveVertexNormals = true;
		float NormalDistortTolerance = .01f;

		void SimplifyAlongEdges(DynamicMesh& Mesh, std::set<int>& InOutEdges, std::function<void(const FEdgeCollapseInfo&)> ProcessCollapse = nullptr) const
		{
			float DotTolerance = cosf(SimplificationAngleTolerance * DEG_TO_RAD);

			// For some simplification passes we can safely only consider locally-flat vertices for simplification, ignoring "straight crease" cases because
			// the new edges that we consider simplifying were generated by cutting across flat triangles. This makes the simplification logic a bit simpler.
			// You can use this variable to toggle that path.
			// TODO: We could consider exposing this variable as a parameter (but it does not seem likely to affect performance enough to really be worth doing so)
			const bool bOnlySimplifyWhereFlat = false;

			// Save the input list of edges to iterate over, so we can edit the set w/ new edges while safely iterating over the old ones
			std::vector<int> CutBoundaryEdgesArray = ToVector<int>(InOutEdges);

			int NumCollapses = 0, CollapseIters = 0;
			int MaxCollapseIters = 1; // TODO: is there a case where we need more iterations?  Perhaps if we add some triangle quality criteria?
			while (CollapseIters < MaxCollapseIters)
			{
				int LastNumCollapses = NumCollapses;
				for (int EID : CutBoundaryEdgesArray)
				{
					// this can happen if a collapse removes another cut boundary edge
					// (which can happen e.g. if you have a degenerate (colinear) tri flat on the cut boundary)
					if (!Mesh.IsEdge(EID))
					{
						continue;
					}

					DynamicMesh::Edge& e = Mesh.GetEdge(EID);

					// track whether the neighborhood of the vertex is flat
					bool Developable[2]{ false, false };
					// normals for each flat vertex
					Vector3 FlatNormals[2]{ Vector3::Zero(), Vector3::Zero() };
					bool Flat[2]{ false, false };
					int NumDevelopable = 0;
					for (int VIdx = 0; VIdx < 2; VIdx++)
					{
						if (bOnlySimplifyWhereFlat)
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

					// see if we can collapse to remove either vertex
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
						if (EdgeDir.SafeNormalize() == 0) // 0 is returned as a special case when the edge was too short to normalize
						{
							// This case is often avoided by collapsing degenerate edges in a separate pre-pass, so we just skip over it here for now
							// TODO: Consider adding degenerate edge-collapse logic here
							break; // break instead of continue to skip the whole edge
						}

						bool bHasBadEdge = false; // will be set if either mesh can't collapse the edge
						int RemoveV = e.Vert[RemoveVIdx];
						int KeepV = e.Vert[KeepVIdx];
						int SourceEID = EID;

						bHasBadEdge = bHasBadEdge || CollapseWouldHurtTriangleQuality(
							Mesh, FlatNormals[RemoveVIdx], RemoveV, RemoveVPos, KeepV, KeepVPos, TryToImproveTriQualityThreshold, !Flat[RemoveVIdx]);

						bHasBadEdge = bHasBadEdge || CollapseWouldChangeShapeOrUVs(
							Mesh, InOutEdges, DotTolerance,
							SourceEID, RemoveV, RemoveVPos, KeepV, KeepVPos, EdgeDir, bPreserveTriangleGroups,
							true, bPreserveVertexUVs, bPreserveOverlayUVs, UVDistortTolerance * UVDistortTolerance,
							bPreserveVertexNormals, cosf(NormalDistortTolerance * DEG_TO_RAD));

						if (bHasBadEdge)
						{
							continue;
						}

						FEdgeCollapseInfo CollapseInfo;
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
						break; // if we got through to trying to collapse the edge, don't try to collapse from the other vertex.
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
			bool bHasFirst = false;
			bool bIsFlat = true;
			Mesh.EnumerateVertexTriangles(VID, [&Mesh, DotTolerance, &OutFirstNormal, &bHasFirst, &bIsFlat](int TID)
				{
					if (!bIsFlat)
					{
						return;
					}
					Vector3 Normal = Mesh.GetTriNormal(TID);
					if (!bHasFirst)
					{
						OutFirstNormal = Normal;
						bHasFirst = true;
					}
					else
					{
						bIsFlat = bIsFlat && Normal.Dot(OutFirstNormal) >= DotTolerance;
					}
				});
			return bIsFlat;
		}

		static bool IsDevelopableAlongEdge(const DynamicMesh& Mesh, int EID, int VID, float DotTolerance, Vector3& NormalA, bool& bIsFlat)
		{
			Index2 EdgeT = Mesh.GetEdgeT(EID);
			NormalA = Mesh.GetTriNormal(EdgeT.a);
			bIsFlat = true;
			Vector3 NormalB;
			if (EdgeT.b != -1)
			{
				NormalB = Mesh.GetTriNormal(EdgeT.b);
				// Only consider a second normal if the across-edge triangle normal doesn't match the first normal
				if (NormalA.Dot(NormalB) < DotTolerance)
				{
					bIsFlat = false;
				}
			}
			bool bIsDevelopable = true;
			Mesh.EnumerateVertexTriangles(VID, [&bIsDevelopable, EdgeT, &Mesh, NormalA, NormalB, DotTolerance, bIsFlat](int TID)
				{
					if (!bIsDevelopable || EdgeT.Contains(TID))
					{
						return;
					}
					Vector3 Normal = Mesh.GetTriNormal(TID);
					bIsDevelopable = Normal.Dot(NormalA) >= DotTolerance;
					if (!bIsDevelopable && !bIsFlat) // if we didn't match first normal, test the second normal (if we have one)
					{
						bIsDevelopable = Normal.Dot(NormalB) >= DotTolerance;
					}
				});
			bIsFlat = bIsFlat && bIsDevelopable;

			return bIsDevelopable;
		}

		static bool CollapseWouldHurtTriangleQuality(
			const DynamicMesh& Mesh, const Vector3& RemoveVNormal,
			int RemoveV, const Vector3& RemoveVPos, int KeepV, const Vector3& KeepVPos,
			float TryToImproveTriQualityThreshold, bool bHasMultipleNormals = false)
		{
			float WorstQualityNewTriangle = FLT_MAX;

			bool bIsHurt = false;
			Mesh.EnumerateVertexTriangles(RemoveV,
				[&Mesh, &bIsHurt, &KeepVPos, RemoveV, KeepV, &RemoveVNormal,
				TryToImproveTriQualityThreshold, &WorstQualityNewTriangle, bHasMultipleNormals](int TID)
				{
					if (bIsHurt)
					{
						return;
					}
					Index3 Tri = Mesh.GetTriangle(TID);
					// Skip re-computing the triangle normal for flat surfaces
					Vector3 TriNormal = bHasMultipleNormals ? Mesh.GetTriNormal(TID) : RemoveVNormal;
					Vector3 Verts[3];
					for (int Idx = 0; Idx < 3; Idx++)
					{
						int VID = Tri[Idx];
						if (VID == KeepV)
						{
							// this tri has both RemoveV and KeepV, so it'll be removed and we don't need to consider it
							return;
						}
						else if (VID == RemoveV)
						{
							// anything at RemoveV is reconnected to KeepV's position
							Verts[Idx] = KeepVPos;
						}
						else
						{
							// it's not on the collapsed edge so it stays still
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

					bIsHurt = VCross.Dot(TriNormal) <= EdgeFlipTolerance;
				}
			);

			// note tri quality was computed as 2*Area / MaxEdgeLenSquared
			//  so need to multiply it by 2/sqrt(3) to get actual aspect ratio
			if (!bIsHurt && WorstQualityNewTriangle * 2.0f * INV_SQRT_3 < TryToImproveTriQualityThreshold)
			{
				// we found a bad tri; tentatively switch to rejecting this edge collapse
				bIsHurt = true;
				// but if there was an even worse tri in the original neighborhood, accept the collapse after all
				Mesh.EnumerateVertexTriangles(RemoveV, [&Mesh, &bIsHurt, WorstQualityNewTriangle](int TID)
					{
						if (!bIsHurt) // early out if we already found an originally-worse tri
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
							bIsHurt = false;
						}
					}
				);
			}
			return bIsHurt;
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
			bool bPreserveTriangleGroups, bool bPreserveUVsForMesh, bool bPreserveVertexUVs, bool bPreserveOverlayUVs,
			float UVEqualThresholdSq, bool bPreserveVertexNormals, float NormalEqualCosThreshold)
		{
			// Search the edges connected to the vertex to find one in the boundary set that points in the opposite direction
			// If we don't find that edge, or if there are other boundary/seam edges attached, we can't remove this vertex
			// We also can't remove the vertex if doing so would distort the UVs
			bool bHasBadEdge = false;

			int OpposedEdge = -1;
			Index2 EdgeT = Mesh.GetEdgeT(SourceEID);
			int SourceGroupID = Mesh.GetTriangleGroup(EdgeT.a);
			int OtherGroupID = -1;
			// Note: In many cases (e.g. plane cut, new edge insertions) we know there will be only one group,
			// so we could use a slightly faster path with bAllowTwoGroups==false. But this is probably not
			// a significant performance difference, so for now we just expose the allows-two-group path.
			constexpr bool bAllowTwoGroups = true;
			if (bAllowTwoGroups && EdgeT.b != -1)
			{
				OtherGroupID = Mesh.GetTriangleGroup(EdgeT.b);
			}

			Mesh.EnumerateVertexEdges(RemoveV,
				[&](int VertEID)
				{
					if (bHasBadEdge || VertEID == SourceEID)
					{
						return;
					}

					DynamicMesh::Edge e = Mesh.GetEdge(VertEID);
					if (bPreserveTriangleGroups && Mesh.HasTriangleGroups())
					{
						int GroupA = Mesh.GetTriangleGroup(e.Tri.a);
						int GroupB = e.Tri.b == -1 ? SourceGroupID : Mesh.GetTriangleGroup(e.Tri.b);

						if (
							(GroupA != SourceGroupID && (!bAllowTwoGroups || GroupA != OtherGroupID)) ||
							(GroupB != SourceGroupID && (!bAllowTwoGroups || GroupB != OtherGroupID))
							)
						{
							// RemoveV is bordering too many groups, so the edge collapse would change the shape of the groups
							bHasBadEdge = true;
							return;
						}
					}

					// it's a path edge; check if it's the opposite-facing one we need
					if (PathEdgeSet.count(VertEID) > 0)
					{
						if (OpposedEdge != -1)
						{
							bHasBadEdge = true;
							return;
						}
						Index2 OtherEdgeV = e.Vert;
						int OtherV = FindEdgeOtherVertex(OtherEdgeV, RemoveV);
						Vector3 OtherVPos = Mesh.GetVertex(OtherV);
						Vector3 OtherEdgeDir = OtherVPos - RemoveVPos;
						if (OtherEdgeDir.SafeNormalize() == 0)
						{
							// collapsing degenerate edges above should prevent this
							bHasBadEdge = true;
							return; // break instead of continue to skip the whole edge
						}
						if (OtherEdgeDir.Dot(EdgeDir) <= -DotTolerance)
						{
							OpposedEdge = VertEID;
						}
						else
						{
							bHasBadEdge = true;
							return;
						}

						// LerpT used for vertex normal and UV interpolation
						float LerpT = float((OtherVPos - RemoveVPos).Dot(OtherEdgeDir) / (OtherVPos - KeepVPos).Dot(OtherEdgeDir));

						if (bPreserveVertexNormals && Mesh.HasVertexNormals())
						{
							Vector3 OtherN = Mesh.GetVertexNormal(OtherV);
							Vector3 RemoveN = Mesh.GetVertexNormal(RemoveV);
							Vector3 KeepN = Mesh.GetVertexNormal(KeepV);
							if (Maths::LinearInterp(OtherN, KeepN, LerpT).Unit().Dot(RemoveN.Unit()) < NormalEqualCosThreshold)
							{
								bHasBadEdge = true;
								return;
							}
						}

						// Controls whether any UV distortion is tested
						if (!bPreserveUVsForMesh)
						{
							return;
						}

						if (bPreserveVertexUVs && Mesh.HasVertexUVs())
						{
							Vector2 OtherUV = Mesh.GetVertexUV(OtherV);
							Vector2 RemoveUV = Mesh.GetVertexUV(RemoveV);
							Vector2 KeepUV = Mesh.GetVertexUV(KeepV);
							if ((Maths::LinearInterp(OtherUV, KeepUV, LerpT) - RemoveUV).SquareLength() > UVEqualThresholdSq)
							{
								bHasBadEdge = true;
								return;
							}
						}
						if (bPreserveOverlayUVs && Mesh.HasAttributes())
						{
							int NumLayers = Mesh.Attributes()->NumUVLayers();
							Index2 SourceEdgeTris = Mesh.GetEdgeT(SourceEID);
							Index2 OppEdgeTris = e.Tri;

							// returns true if collapse will result in acceptable overlay UVs
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

								// get the UVs per overlay off the triangle(s) attached the two edges
								for (int UVLayerIdx = 0; UVLayerIdx < NumLayers; UVLayerIdx++)
								{
									const FDynamicMeshUVOverlay* UVs = Mesh.Attributes()->GetUVLayer(UVLayerIdx);
									if (UVs->ElementCount() < 3)
									{
										// overlay is not actually in use; skip it
										continue;
									}
									Index3 SourceT = UVs->GetTriangle(SourceEdgeTID);
									Index3 OppT = UVs->GetTriangle(OppEdgeTID);
									int KeepE = SourceT[KeepSourceIdx];
									int RemoveE = SourceT[RemoveSourceIdx];
									int OtherE = OppT[OtherOppIdx];
									if (KeepE == -1 || RemoveE == -1 || OtherE == -1)
									{
										// overlay is not set on relevant triangles; skip it
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

							// If we're not on a boundary, check if we're on a seam -- and if so, test that
							// the collapse won't change the UVs on the other side of the seam
							if (SourceEdgeTris.b != -1 || OppEdgeTris.b != -1)
							{
								// the edges must be both boundary or neither
								if (SourceEdgeTris.b == -1 || OppEdgeTris.b == -1)
								{
									bHasBadEdge = true;
									return;
								}
								// likewise must be both seam or neither
								bool bSourceIsSeam = Mesh.Attributes()->IsSeamEdge(SourceEID);
								bool bOtherIsSeam = Mesh.Attributes()->IsSeamEdge(VertEID);
								if (bSourceIsSeam != bOtherIsSeam)
								{
									bHasBadEdge = true;
									return;
								}
								// if both are seam, make sure that the A and B sides are consistent,
								// then test the UVs on the other side of the seam
								if (bSourceIsSeam)
								{
									// test whether the A/B sides are consistent for SourceEdgeTris and OppEdgeTris,
									// based on the ordering of the remove and keep/other vertices in each triangle
									Index3 SourceBaseTri = Mesh.GetTriangle(SourceEdgeTris.a);
									Index3 OppBaseTri = Mesh.GetTriangle(OppEdgeTris.a);
									int SrcTriRmVSubIdx = FindTriIndex(RemoveV, SourceBaseTri);
									int OppTriRmVSubIdx = FindTriIndex(RemoveV, OppBaseTri);
									// Test whether the Triangle "A" for the Source Edge is wound such that the 'Remove' Vertex is before the 'Keep' Vertex
									bool bSrcAOrderIsRmThenKeep = SourceBaseTri[(SrcTriRmVSubIdx + 1) % 3] == KeepV;
									// Test whether the Triangle "A" for the Opposite Edge is wound such that the 'Remove' Vertex is before the 'Other' Vertex
									bool bOppAOrderIsRmThenOther = OppBaseTri[(OppTriRmVSubIdx + 1) % 3] == OtherV;
									// If the two "A" sides are on the same side of the seam, we expect these two vertex ordering bools *not* to match --
									// i.e., the oriented edges should be "Keep Remove, then Remove Other" or "Other Remove, then Remove Keep"
									// If the bools match, we swap the A and B of the Opposite Edge Triangles, to move the A side back to the same side of the seam
									// as its Source Edge Triangle counterpart.
									if (bSrcAOrderIsRmThenKeep == bOppAOrderIsRmThenOther)
									{
										std::swap(OppEdgeTris.a, OppEdgeTris.b);
									}
									if (!CanCollapseOverlayUVs(SourceEdgeTris.b, OppEdgeTris.b))
									{
										bHasBadEdge = true;
										return;
									}
								}
							}

							if (!CanCollapseOverlayUVs(SourceEdgeTris.a, OppEdgeTris.a))
							{
								bHasBadEdge = true;
								return;
							}
						}
					}
					else // it wasn't in the boundary edge set; check if it's one that would prevent us from safely removing the vertex
					{
						// bool b = false;
						if (Mesh.IsBoundaryEdge(VertEID) || (Mesh.HasAttributes() && Mesh.Attributes()->IsSeamEdge(VertEID)))
						{
							bHasBadEdge = true;
						}
					}
				});

			return bHasBadEdge;
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
		// translate the edge IDs from CutMesh[1] over to Result mesh edge IDs
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
			if (Result->IsBoundaryEdge(MappedEID))
			{
				OtherMeshEdges.push_back(MappedEID);
			}
		}

		// find "easy" match candidates using the already-made vertex correspondence
		DynamicArray<Index2> CandidateMatches;
		DynamicArray<int> UnmatchedEdges;
		for (int EID : CutBoundaryEdges[0])
		{
			if (!Result->IsBoundaryEdge(EID))
			{
				continue;
			}
			Index2 VIDs = Result->GetEdgeV(EID);
			const int* OtherA = MapFind<int, int>(AllVIDMatches, VIDs.a);
			const int* OtherB = MapFind<int, int>(AllVIDMatches, VIDs.b);
			bool bAddedCandidate = false;
			if (OtherA && OtherB)
			{
				int MapOtherA = IndexMaps.GetNewVertex(*OtherA);
				int MapOtherB = IndexMaps.GetNewVertex(*OtherB);
				int OtherEID = Result->FindEdge(MapOtherA, MapOtherB);
				if (OtherEID != -1)
				{
					CandidateMatches.push_back(Index2(EID, OtherEID));
					bAddedCandidate = true;
				}
			}
			if (!bAddedCandidate)
			{
				UnmatchedEdges.push_back(EID);
			}
		}

		// merge the easy matches
		for (Index2 Candidate : CandidateMatches)
		{
			if (!Result->IsEdge(Candidate.a) || !Result->IsBoundaryEdge(Candidate.a))
			{
				continue;
			}

			FMergeEdgesInfo MergeInfo;
			EMeshResult EdgeMergeResult = Result->MergeEdges(Candidate.a, Candidate.b, MergeInfo, true);
			if (EdgeMergeResult != EMeshResult::Ok)
			{
				UnmatchedEdges.push_back(Candidate.a);
			}
			else
			{
				if (Geom->bTrackAllNewEdges)
				{
					Geom->AllNewEdges.insert(Candidate.a);
				}
			}
		}

		// filter matched edges from the edge array for the other mesh
		OtherMeshEdges.remove_if(
			[Result = Geom->Result](int EID)
			{
				return !Result->IsEdge(EID) || !Result->IsBoundaryEdge(EID);
			});

		// see if we can match anything else
		bool bAllMatched = true;
		if (UnmatchedEdges.size() > 0)
		{
			// greedily match within snap tolerance
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
						FMergeEdgesInfo MergeInfo;
						EMeshResult EdgeMergeResult = Result->MergeEdges(EID, OtherEID, MergeInfo, true);
						if (EdgeMergeResult == EMeshResult::Ok)
						{
							UnmatchedEdges.remove_at(UnmatchedIdx, false);
							if (Geom->bTrackAllNewEdges)
							{
								Geom->AllNewEdges.insert(EID);
							}
							break;
						}
					}
				}
			}

			// store the failure cases from the first mesh's array
			for (int EID : UnmatchedEdges)
			{
				if (Result->IsEdge(EID) && Result->IsBoundaryEdge(EID))
				{
					Geom->CreatedBoundaryEdges.push_back(EID);
					bAllMatched = false;
				}
			}
		}
		// store the failure cases from the second mesh's array
		for (int OtherEID : OtherMeshEdges)
		{
			if (Result->IsEdge(OtherEID) && Result->IsBoundaryEdge(OtherEID))
			{
				Geom->CreatedBoundaryEdges.push_back(OtherEID);
				bAllMatched = false;
			}
		}
		return bAllMatched;
	}
	
	void SimplifyAlongNewEdges(int NumMeshesToProcess, DynamicMesh* CutMesh[2], std::vector<int> CutBoundaryEdges[2], std::map<int, int>& AllVIDMatches, GeometryBoolean* Geom)
	{
		float DotTolerance = cosf(Geom->SimplificationAngleTolerance * DEG_TO_RAD);

		std::set<int> CutBoundaryEdgeSets[2]; // set versions of CutBoundaryEdges, for faster membership tests
		for (int MeshIdx = 0; MeshIdx < NumMeshesToProcess; MeshIdx++)
		{
			CutBoundaryEdgeSets[MeshIdx] = std::set<int>(CutBoundaryEdges[MeshIdx].begin(), CutBoundaryEdges[MeshIdx].end());
		}

		int NumCollapses = 0, CollapseIters = 0;
		int MaxCollapseIters = 1; // TODO: is there a case where we need more iterations?  Perhaps if we add some triangle quality criteria?
		while (CollapseIters < MaxCollapseIters)
		{
			int LastNumCollapses = NumCollapses;
			for (int EID : CutBoundaryEdges[0])
			{
				// this can happen if a collapse removes another cut boundary edge
				// (which can happen e.g. if you have a degenerate (colinear) tri flat on the cut boundary)
				if (!CutMesh[0]->IsEdge(EID))
				{
					continue;
				}
				// don't allow collapses if we somehow get down to our last triangle on either mesh
				if (CutMesh[0]->GetTriangleCount() <= 1 || (NumMeshesToProcess == 2 && CutMesh[1]->GetTriangleCount() <= 1))
				{
					break;
				}

				const DynamicMesh::Edge& e = CutMesh[0]->GetEdge(EID);
				int Matches[2]{ -1, -1 };
				bool bHasMatches = NumMeshesToProcess == 2;
				if (bHasMatches)
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
							bHasMatches = false;
							// TODO: if we switch to allow collapse on unmatched edges, we shouldn't break here
							//        b/c we may be partially matched, and need to track which is matched.
							break;
						}
					}
					if (!bHasMatches)
					{
						continue; // edge wasn't matched up on the other mesh; can't collapse it?
						// TODO: consider supporting collapses in this case?
					}
				}
				// if we have matched vertices, we also need a matched edge to collapse
				int OtherEID = -1;
				if (bHasMatches)
				{
					OtherEID = CutMesh[1]->FindEdge(Matches[0], Matches[1]);
					if (OtherEID == -1)
					{
						continue;
					}
				}
				// track whether the neighborhood of the vertex is flat (and likewise its matched pair's neighborhood, if present)
				bool Flat[2]{ false, false };
				// normals for each flat vertex, and each "side" (mesh 0 and mesh 1, if mesh 1 is present)
				Vector3 FlatNormals[2][2]{ {Vector3::Zero(), Vector3::Zero()}, {Vector3::Zero(), Vector3::Zero()} };
				int NumFlat = 0;
				for (int VIdx = 0; VIdx < 2; VIdx++)
				{
					if (FLocalPlanarSimplify::IsFlat(*CutMesh[0], e.Vert[VIdx], DotTolerance, FlatNormals[VIdx][0]))
					{
						Flat[VIdx] = (Matches[VIdx] == -1) || FLocalPlanarSimplify::IsFlat(*CutMesh[1], Matches[VIdx], DotTolerance, FlatNormals[VIdx][1]);
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

				// see if we can collapse to remove either vertex
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
					if (EdgeDir.SafeNormalize() == 0) // 0 is returned as a special case when the edge was too short to normalize
					{
						// collapsing degenerate edges above should prevent this
						assert(!Geom->bCollapseDegenerateEdgesOnCut);
						// Just skip these edges, because in practice we generally have bCollapseDegenerateEdgesOnCut enabled
						break; // break instead of continue to skip the whole edge
					}

					bool bHasBadEdge = false; // will be set if either mesh can't collapse the edge
					for (int MeshIdx = 0; !bHasBadEdge && MeshIdx < NumMeshesToProcess; MeshIdx++)
					{
						int RemoveV = MeshIdx == 0 ? e.Vert[RemoveVIdx] : Matches[RemoveVIdx];
						int KeepV = MeshIdx == 0 ? e.Vert[KeepVIdx] : Matches[KeepVIdx];
						int SourceEID = MeshIdx == 0 ? EID : OtherEID;

						bHasBadEdge = bHasBadEdge || FLocalPlanarSimplify::CollapseWouldHurtTriangleQuality(*CutMesh[MeshIdx],
							FlatNormals[RemoveVIdx][MeshIdx], RemoveV, RemoveVPos, KeepV, KeepVPos, Geom->TryToImproveTriQualityThreshold);

						bHasBadEdge = bHasBadEdge || FLocalPlanarSimplify::CollapseWouldChangeShapeOrUVs(
							*CutMesh[MeshIdx], CutBoundaryEdgeSets[MeshIdx], DotTolerance,
							SourceEID, RemoveV, RemoveVPos, KeepV, KeepVPos, EdgeDir, Geom->bPreserveTriangleGroups,
							Geom->PreserveUVsOnlyForMesh == -1 || MeshIdx == Geom->PreserveUVsOnlyForMesh,
							Geom->bPreserveVertexUVs, Geom->bPreserveOverlayUVs, Geom->UVDistortTolerance * Geom->UVDistortTolerance,
							Geom->bPreserveVertexNormals, cosf(Geom->NormalDistortTolerance * DEG_TO_RAD));
					};

					if (bHasBadEdge)
					{
						continue;
					}

					// do some pre-collapse sanity asserts on the matched edge (if present) to see if it will fail to collapse
					bool bAttemptCollapse = true;
					if (bHasMatches)
					{
						int OtherRemoveV = Matches[RemoveVIdx];
						int OtherKeepV = Matches[KeepVIdx];

						int a = OtherRemoveV, b = OtherKeepV;
						int eab = CutMesh[1]->FindEdge(OtherRemoveV, OtherKeepV);

						const DynamicMesh::Edge& EdgeAB = CutMesh[1]->GetEdge(eab);
						int t0 = EdgeAB.Tri[0];
						if (t0 == -1)
						{
							bAttemptCollapse = false;
						}
						else
						{
							Index3 T0tv = CutMesh[1]->GetTriangle(t0);
							int c = FindTriOtherVtx(a, b, T0tv);
							assert(EdgeAB.Tri[1] == -1);
							// We cannot collapse if edge lists of a and b share vertices other
							//  than c and d  (because then we will make a triangle [x b b].
							//  Brute-force search logic adapted from DynamicMesh::CollapseEdge implementation.
							//  (simplified because we know this is a boundary edge)
							CutMesh[1]->EnumerateVertexVertices(a, [&](int VID)
								{
									if (!bAttemptCollapse || VID == c || VID == b)
									{
										return;
									}
									CutMesh[1]->EnumerateVertexVertices(b, [&](int VID2)
										{
											bAttemptCollapse &= (VID != VID2);
										});
								});
						}
					}
					if (!bAttemptCollapse)
					{
						break; // don't try starting from other vertex if the match edge couldn't be collapsed
					}

					FEdgeCollapseInfo CollapseInfo;
					int RemoveV = e.Vert[RemoveVIdx];
					int KeepV = e.Vert[KeepVIdx];
					// Detect the case of a triangle with two boundary edges, where collapsing 
					// the target boundary edge would keep the non-boundary edge.
					// This collapse will remove the triangle, so we add the
					// (formerly) non-boundary edge as our new boundary edge.
					auto WouldRemoveTwoBoundaryEdges = [](const DynamicMesh& Mesh, int EID, int RemoveV)
					{
						assert(Mesh.IsEdge(EID));
						int OppV = Mesh.GetEdgeOpposingV(EID).a;
						int NextOnTri = Mesh.FindEdge(RemoveV, OppV);
						return Mesh.IsBoundaryEdge(NextOnTri);
					};
					bool bWouldRemoveNext = WouldRemoveTwoBoundaryEdges(*CutMesh[0], EID, RemoveV);
					EMeshResult CollapseResult = CutMesh[0]->CollapseEdge(KeepV, RemoveV, 0, CollapseInfo);
					if (CollapseResult == EMeshResult::Ok)
					{
						if (bWouldRemoveNext && CutMesh[0]->IsBoundaryEdge(CollapseInfo.KeptEdges.a))
						{
							CutBoundaryEdgeSets[0].insert(CollapseInfo.KeptEdges.a);
						}

						if (bHasMatches)
						{
							int OtherRemoveV = Matches[RemoveVIdx];
							int OtherKeepV = Matches[KeepVIdx];
							bool bOtherWouldRemoveNext = WouldRemoveTwoBoundaryEdges(*CutMesh[1], OtherEID, OtherRemoveV);
							FEdgeCollapseInfo OtherCollapseInfo;
							EMeshResult OtherCollapseResult = CutMesh[1]->CollapseEdge(OtherKeepV, OtherRemoveV, 0, OtherCollapseInfo);
							if (OtherCollapseResult != EMeshResult::Ok)
							{
								// if we get here, we've somehow managed to collapse the first edge but failed on the second (matched) edge
								// which will leave a crack in the result unless we can somehow undo the first collapse, which would require a bunch of extra work
								// but the only case where I could see this happen is if the second edge is on an isolated triangle, which means there is a hole anyway
								// or if the mesh topology is somehow invalid
								assert(OtherCollapseResult == EMeshResult::Failed_CollapseTriangle);
							}
							else
							{
								if (bOtherWouldRemoveNext && CutMesh[1]->IsBoundaryEdge(OtherCollapseInfo.KeptEdges.a))
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
					break; // if we got through to trying to collapse the edge, don't try to collapse from the other vertex.
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

	bool GeometryBoolean::Compute()
	{
		// copy meshes
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
			CenteredTransform.SetTranslation(CenteredTransform.GetTranslation() - CombinedAABB.GetCenter());
			CutMesh[MeshIdx]->ApplyTransform(CenteredTransform, true);
		}
		TransformNew = Transform(CombinedAABB.GetCenter());

		// build spatial data and use it to find intersections
		DynamicMeshAABBTree Spatial[2]{ CutMesh[0], CutMesh[1] };
		IntersectionsQueryResult Intersections = Spatial[0].FindAllIntersections(Spatial[1], nullptr);

		const bool bOpOnSingleMesh = false;

		GeometryCut Cut(CutMesh[0], CutMesh[1]);
		Cut.SnapTolerance = SnapTolerance;
		Cut.bMutuallyCut = !bOpOnSingleMesh;
		Cut.Compute(Intersections);

		int NumMeshesToProcess = bOpOnSingleMesh ? 1 : 2;
		float DegenerateEdgeTolFactor = 1.5f;

		if (bCollapseDegenerateEdgesOnCut)
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

					// if the vertex we'd remove is on a seam, try removing the other one instead
					if (CutMesh[MeshIdx]->HasAttributes() && CutMesh[MeshIdx]->Attributes()->IsSeamVertex(EV.b, false))
					{
						std::swap(EV.a, EV.b);
						// if they were both on seams, then collapse should not happen?  (& would break OnCollapseEdge assumptions in overlay)
						if (CutMesh[MeshIdx]->HasAttributes() && CutMesh[MeshIdx]->Attributes()->IsSeamVertex(EV.b, false))
						{
							continue;
						}
					}

					FEdgeCollapseInfo CollapseInfo;
					EMeshResult Result = CutMesh[MeshIdx]->CollapseEdge(EV.a, EV.b, .5, CollapseInfo);
					if (Result != EMeshResult::Ok)
					{
						for (int i = 0; i < 2; i++)
						{
							if (AllEIDs.count(CollapseInfo.RemovedEdges[i]) > 0)
							{
								int ToAdd = CollapseInfo.KeptEdges[i];
								bool bWasPresent = AllEIDs.count(ToAdd) > 0;
								if (!bWasPresent)
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

		// edges that will become new boundary edges after the boolean op removes triangles on each mesh
		std::vector<int> CutBoundaryEdges[2];
		// Vertices on the cut boundary that *may* not have a corresonding vertex on the other mesh
		std::set<int> PossUnmatchedBdryVerts[2];

		const float WindingThreshold = 0.5f;
		// delete geometry according to boolean rules, tracking the boundary edges
		{ // (just for scope)
			// first decide what triangles to delete for both meshes (*before* deleting anything so winding doesn't get messed up!)
			std::vector<bool> KeepTri[2];
			// This array is used to float-assert the assumption that we will delete the other surface when we keep a coplanar tri
			// Note we only need it for mesh 0 (i.e., the mesh we try to keep triangles from when we preserve coplanar surfaces)
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
				bool bCoplanarKeepSameDir = (Operation != BooleanOp::Difference);
				bool bRemoveInside = true;
				if (Operation == BooleanOp::Intersect || (Operation == BooleanOp::Difference && MeshIdx == 1))
				{
					bRemoveInside = false;
				}

				std::vector<Vector3> OtherNormals(OtherSpatial.Mesh->GetTriangleCount());			// <-- OtherSpatial
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

					// first assert for the coplanar case
					{
						float DSq;
						int OtherTID = OtherSpatial.FindNearestTriangle(Centroid, DSq, NonDegenCoplanarCandidateFilter);
						if (OtherTID > -1)
						{

							Vector3 OtherNormal = OtherNormals[OtherTID];
							Vector3 Normal = ProcessMesh.GetTriNormal(TID);
							float DotNormals = OtherNormal.Dot(Normal);

							//if (FMath::Abs(DotNormals) > .9) // TODO: do we actually want to assert for a normal match? coplanar vertex assert below is more robust?
							{
								// To be extra sure it's a coplanar match, assert the vertices are *also* on the other mesh (w/in SnapTolerance)

								bool bAllTrisOnOtherMesh = true;
								for (int Idx = 0; Idx < 3; Idx++)
								{
									// use a slightly more forgiving tolerance to account for the likelihood that these vertices were mesh-cut right to the boundary of the coplanar region and have some additional error
									if (OtherSpatial.FindNearestTriangle(Tri[Idx], DSq, OnPlaneTolerance * 2) == -1)
									{
										bAllTrisOnOtherMesh = false;
										break;
									}
								}
								if (bAllTrisOnOtherMesh)
								{
									// for coplanar tris favor the first mesh; just delete from the other mesh
									// for fully degenerate tris, favor deletion also
									//  (Note: For degenerate tris we have no orientation info, so we are choosing between
									//         potentially leaving 'cracks' in solid regions or 'spikes' in empty regions)
									if (MeshIdx != 0 || Normal.IsZero())
									{
										KeepTri[MeshIdx][TID] = false;
										continue;
									}
									else // for the first mesh, & with a valid normal, logic depends on orientation of matching tri
									{
										bool bKeep = DotNormals > 0 == bCoplanarKeepSameDir;
										KeepTri[MeshIdx][TID] = bKeep;
										if (NumMeshesToProcess > 1 && bKeep)
										{
											// If we kept this tri, remember the coplanar pair we expect to be deleted, in case
											// it isn't deleted (e.g. because it wasn't coplanar); to then delete this one instead.
											// This can help clean up sliver triangles near a cut boundary that look locally coplanar
											DeleteIfOtherKept[TID] = OtherTID;
										}
										continue;
									}
								}
							}
						}
					}

					// didn't already return a coplanar result; use the winding number
					float WindingNum = Winding.FastWindingNumber(Centroid);
					KeepTri[MeshIdx][TID] = (WindingNum > WindingThreshold) != bRemoveInside;
				};
			}

			// Don't keep coplanar tris if the matched, second-mesh tri that we expected to delete was actually kept
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

		// correspond vertices across both meshes (in cases where both meshes were processed)
		std::map<int, int> AllVIDMatches; // mapping of matched vertex IDs from cutmesh 0 to cutmesh 1
		if (NumMeshesToProcess == 2)
		{
			std::map<int, int> FoundMatchesMaps[2]; // mappings of matched vertex IDs from mesh 1->0 and mesh 0->1
			float SnapToleranceSq = SnapTolerance * SnapTolerance;

			// ensure segments that are now on boundaries have 1:1 vertex correspondence across meshes
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


				// mapping from OtherMesh VIDs to ProcessMesh VIDs
				// used to ensure we only keep the best match, in cases where multiple boundary vertices map to a given vertex on the other mesh boundary
				std::map<int, int>& FoundMatches = FoundMatchesMaps[MeshIdx];

				for (int BoundaryVID : PossUnmatchedBdryVerts[MeshIdx])
				{
					if (MeshIdx == 1 && FoundMatchesMaps[0].count(BoundaryVID) > 0)
					{
						continue; // was already snapped to a vertex
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
							if (DSq < OldDSq) // new vertex is a better match than the old one
							{
								int OldVID = Match; // copy old VID out of match before updating the std::map
								FoundMatches.emplace(NearestVID, BoundaryVID); // new VID is recorded as best match

								// old VID is swapped in as the one to consider as unmatched
								// it will now be matched below
								BoundaryVID = OldVID;
								Pos = CutMesh[MeshIdx]->GetVertex(BoundaryVID);
								DSq = OldDSq;
							}
							NearestVID = -1; // one of these vertices will be unmatched
						}
						else
						{
							FoundMatches.emplace(NearestVID, BoundaryVID);
						}
					}

					// if we didn't find a valid match, try to split the nearest edge to create a match
					if (NearestVID == -1)
					{
						// vertex had no match -- try to split edge to match it
						Box3 QueryBox(Pos, SnapTolerance);
						EdgesInRange.clear();
						EdgeOctree.RangeQuery(QueryBox, EdgesInRange);

						int OtherEID = FindNearestEdge(OtherMesh, EdgesInRange, Pos);
						if (OtherEID != -1)
						{
							Vector3 EdgePts[2];
							OtherMesh.GetEdgeV(OtherEID, EdgePts[0], EdgePts[1]);
							// only accept the match if it's not going to create a degenerate edge -- TODO: filter already-matched edges from the FindNearestEdge query!
							if ((EdgePts[0] - Pos).SquareLength() > SnapToleranceSq && (EdgePts[1] - Pos).SquareLength() > SnapToleranceSq)
							{
								Segment3 Seg(EdgePts[0], EdgePts[1]);
								float Along = Seg.ProjectUnitRange(Pos);
								FEdgeSplitInfo SplitInfo;
								if (EMeshResult::Ok == OtherMesh.SplitEdge(OtherEID, SplitInfo, Along))
								{
									FoundMatches.emplace(SplitInfo.NewVertex, BoundaryVID);
									OtherMesh.SetVertex(SplitInfo.NewVertex, Pos);
									CutBoundaryEdges[OtherMeshIdx].push_back(SplitInfo.NewEdges.a);
									UpdateEdge(OtherEID);
									AddEdge(SplitInfo.NewEdges.a);
									// Note: Do not update PossUnmatchedBdryVerts with the new vertex, because it is already matched by construction
									// Likewise do not update the pointhash -- we don't want it to find vertices that were already perfectly matched
								}
							}
						}
					}
				}

				// actually snap the positions together for final matches
				for (auto& Match : FoundMatches)
				{
					CutMesh[MeshIdx]->SetVertex(Match.second, OtherMesh.GetVertex(Match.first));

					// Copy match to AllVIDMatches; note this is always mapping from CutMesh 0 to 1
					int VIDs[2]{ Match.first, Match.second }; // just so we can access by index
					AllVIDMatches.emplace(VIDs[1 - MeshIdx], VIDs[MeshIdx]);
				}
			}
		}

		bool bSuccess = true;

		if (bSimplifyAlongNewEdges)
		{
			SimplifyAlongNewEdges(NumMeshesToProcess, CutMesh, CutBoundaryEdges, AllVIDMatches, this);
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

			if (bPopulateSecondMeshGroupMap)
			{
				SecondMeshGroupMap = IndexMaps.GetGroupMap();
			}

			if (bWeldSharedEdges)
			{
				bool bWeldSuccess = MergeEdges(IndexMaps, CutMesh, CutBoundaryEdges, AllVIDMatches, this);
				bSuccess = bSuccess && bWeldSuccess;
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

		if (bTrackAllNewEdges)
		{
			for (int eid : CreatedBoundaryEdges)
			{
				AllNewEdges.insert(eid);
			}
		}

		if (bPutResultInInputSpace)
		{
			Result->ApplyTransform(TransformNew, false);
			TransformNew = Transform::Identity();
		}

		return bSuccess;
	}

}
