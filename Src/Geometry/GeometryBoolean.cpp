#include <assert.h>
#include <vector>
#include <functional>
#include <map>
#include <set>
#include "../CollisionPrimitive/Segment3.h"
#include "../CollisionPrimitive/Triangle3.h"
#include "../Core/PriorityQueue.h"
#include "../Maths/Maths.h"
#include "GeometryBoolean.h"
#include "DynamicMesh.h"

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

	enum class ESurfacePointType
	{
		Vertex = 0,
		Edge = 1,
		Triangle = 2
	};

	struct FMeshSurfacePoint
	{
		int ElementID;
		Vector3 BaryCoord;
		ESurfacePointType PointType;

		FMeshSurfacePoint() : ElementID(-1)
		{
		}
		FMeshSurfacePoint(int TriangleID, const Vector3& BaryCoord) : ElementID(TriangleID), BaryCoord(BaryCoord), PointType(ESurfacePointType::Triangle)
		{
		}
		FMeshSurfacePoint(int EdgeID, float FirstCoordWt) : ElementID(EdgeID), BaryCoord(FirstCoordWt, 1 - FirstCoordWt, 0), PointType(ESurfacePointType::Edge)
		{
		}
		FMeshSurfacePoint(int VertexID) : ElementID(VertexID), BaryCoord(1, 0, 0), PointType(ESurfacePointType::Vertex)
		{
		}

		Vector3 Pos(const DynamicMesh* Mesh) const
		{
			if (PointType == ESurfacePointType::Vertex)
			{
				return Mesh->GetVertex(ElementID);
			}
			else if (PointType == ESurfacePointType::Edge)
			{
				Vector3 EA, EB;
				Mesh->GetEdgeV(ElementID, EA, EB);
				return BaryCoord[0] * EA + BaryCoord[1] * EB;
			}
			else // PointType == ESurfacePointType::Triangle
			{
				Vector3 TA, TB, TC;
				Mesh->GetTriangleVertices(ElementID, TA, TB, TC);
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
					switch (P.PointType)
					{
					case ESurfacePointType::Triangle:
						if (P.ElementID != WalkingOnTri)
						{
							return false;
						}
						break;
					case ESurfacePointType::Edge:
						if (!Mesh->GetEdgeT(P.ElementID).Contains(WalkingOnTri))
						{
							return false;
						}
						break;
					case ESurfacePointType::Vertex:
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
			if (PathNum > 1 && OrigEndPt.PointType == ESurfacePointType::Triangle)
			{
				EndSimpleProcessIdx = PathNum - 2;
				bEndPointSpecialProcess = true;
			}
			bool bNeedFinalRelocate = false;
			FMeshSurfacePoint EndPtUpdated = Path.back().first;
			Vector3 EndPtPos = OrigEndPt.Pos(Mesh);

			if (Path[0].first.PointType == ESurfacePointType::Triangle)
			{
				PokeTriangleInfo PokeInfo;
				Mesh->PokeTriangle(Path[0].first.ElementID, Path[0].first.BaryCoord, PokeInfo);
				if (EndPtUpdated.PointType == ESurfacePointType::Triangle && Path[0].first.ElementID == EndPtUpdated.ElementID)
				{
					EndPtUpdated = RelocateTrianglePointAfterRefinement(Mesh, EndPtPos, { PokeInfo.NewTriangles.a, PokeInfo.NewTriangles.b, PokeInfo.OriginalTriangle }, SnapElementThresholdSq);
				}
				PathVertices.push_back(PokeInfo.NewVertex);
				StartProcessIdx = 1;
			}
			for (int PathIdx = StartProcessIdx; PathIdx <= EndSimpleProcessIdx; PathIdx++)
			{
				if (!(Path[PathIdx].first.PointType != ESurfacePointType::Triangle))
				{
					return false;
				}
				const FMeshSurfacePoint& Pt = Path[PathIdx].first;
				if (Pt.PointType == ESurfacePointType::Edge)
				{
					assert(Mesh->IsEdge(Pt.ElementID));
					EdgeSplitInfo SplitInfo;
					Mesh->SplitEdge(Pt.ElementID, SplitInfo, Pt.BaryCoord[0]);
					PathVertices.push_back(SplitInfo.NewVertex);
					if (EndPtUpdated.PointType == ESurfacePointType::Triangle && SplitInfo.OriginalTriangles.Contains(EndPtUpdated.ElementID))
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
					else if (PathIdx != PathNum - 1 && EndPtUpdated.PointType == ESurfacePointType::Edge && Pt.ElementID == EndPtUpdated.ElementID)
					{
						assert(false);
					}
				}
				else
				{
					assert(Pt.PointType == ESurfacePointType::Vertex);
					assert(Mesh->IsVertex(Pt.ElementID));
					if (!bDoNotDuplicateFirstVertexID || PathVertices.size() != InitialPathIdx || 0 == PathVertices.size() || PathVertices.back() != Pt.ElementID)
					{
						PathVertices.push_back(Pt.ElementID);
					}
				}
			}

			if (bEndPointSpecialProcess)
			{
				if (EndPtUpdated.PointType == ESurfacePointType::Triangle)
				{
					PokeTriangleInfo PokeInfo;
					Mesh->PokeTriangle(EndPtUpdated.ElementID, EndPtUpdated.BaryCoord, PokeInfo);
					PathVertices.push_back(PokeInfo.NewVertex);
				}
				else if (EndPtUpdated.PointType == ESurfacePointType::Edge)
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

			auto PtInsideTri = [](const Vector3& BaryCoord, double BaryThreshold = 1e-6f)
			{
				return BaryCoord[0] >= -BaryThreshold && BaryCoord[1] >= -BaryThreshold && BaryCoord[2] >= -BaryThreshold;
			};

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
			Triangle3::BaryCentric3DDecomposionResult CurrentTriDist;
			int StartVIDIndex = -1;
			if (StartVID != -1)
			{
				StartVIDIndex = StartTriVertIDs.IndexOf(StartVID);
			}
			if (StartVIDIndex == -1)
			{
				Triangle3::BaryCentric3DDecomposionResult info = CurrentTri.BaryCentric3DDecomposion(StartPt);
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
					CurrentTriDist = CurrentTri.BaryCentric3DDecomposion(EndPt);
					ComputedEndPtOnTri = true;
					double DistSq = CurrentTriDist.SqrDistance;
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
						CurrentTriDist = CurrentTri.BaryCentric3DDecomposion(EndPt);
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
						if (FromPt.PointType != ESurfacePointType::Vertex || CandidateVertID != FromPt.ElementID)
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
				Index3 TriEdgeIDs = Mesh->GetTriangleEdge(TriID);
				for (int TriSubIdx = 0; TriSubIdx < 3; TriSubIdx++)
				{
					int NextSubIdx = (TriSubIdx + 1) % 3;
					if (Side[TriSubIdx] * Side[NextSubIdx] < 0)
					{
						// edge crossing
						int CandidateEdgeID = TriEdgeIDs[TriSubIdx];
						if (FromPt.PointType != ESurfacePointType::Edge || CandidateEdgeID != FromPt.ElementID)
						{
							float CrossingT = SignDist[TriSubIdx] / (SignDist[TriSubIdx] - SignDist[NextSubIdx]);
							Vector3 CrossingP = (1.0f - CrossingT) * CurrentTri[TriSubIdx] + CrossingT * CurrentTri[NextSubIdx];
							const DynamicMesh::Edge edge = Mesh->GetEdge(CandidateEdgeID);
							if (edge.v0 != TriVertIDs[TriSubIdx])
							{
								CrossingT = 1 - CrossingT;
							}
							int CrossToTriID = edge.t0;
							if (CrossToTriID == TriID)
							{
								CrossToTriID = edge.t1;
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

			if (WalkedPath.size() && WalkedPath[0].first.PointType == ESurfacePointType::Triangle)
			{
				FMeshSurfacePoint& SurfacePt = WalkedPath[0].first;

				if (StartVIDIndex > -1 && SurfacePt.BaryCoord[StartVIDIndex] == 1.0)
				{
					Index3 TriVertIDs = Mesh->GetTriangle(SurfacePt.ElementID);
					SurfacePt.ElementID = TriVertIDs[StartVIDIndex];
					SurfacePt.PointType = ESurfacePointType::Vertex;
				}
				else
				{
					RefineSurfacePtFromTriangleToSubElement(Mesh, SurfacePt.Pos(Mesh), SurfacePt, PtOnPlaneThresholdSq);
				}
				if (WalkedPath.size() > 1 &&
					SurfacePt.PointType != ESurfacePointType::Triangle &&
					SurfacePt.PointType == WalkedPath[1].first.PointType &&
					SurfacePt.ElementID == WalkedPath[1].first.ElementID)
				{
					if (SurfacePt.PointType == ESurfacePointType::Edge)
					{
						WalkedPath[1].first.BaryCoord = SurfacePt.BaryCoord;
					}
					WalkedPath.erase(WalkedPath.begin());
				}
			}
			if (WalkedPath.size() && WalkedPath.back().first.PointType == ESurfacePointType::Triangle)
			{
				FMeshSurfacePoint& SurfacePt = WalkedPath.back().first;
				RefineSurfacePtFromTriangleToSubElement(Mesh, SurfacePt.Pos(Mesh), SurfacePt, PtOnPlaneThresholdSq);
				if (WalkedPath.size() > 1 &&
					SurfacePt.PointType != ESurfacePointType::Triangle &&
					SurfacePt.PointType == WalkedPath[WalkedPath.size() - 2].first.PointType &&
					SurfacePt.ElementID == WalkedPath[WalkedPath.size() - 2].first.ElementID)
				{
					if (SurfacePt.PointType == ESurfacePointType::Edge) // copy closer barycoord
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
			if (!(SurfacePt.PointType == ESurfacePointType::Triangle))
			{
				return;
			}
			int TriID = SurfacePt.ElementID;

			Index3 TriVertIDs = Mesh->GetTriangle(TriID);
			int BestSubIdx = -1;
			double BestElementDistSq = 0;
			for (int VertSubIdx = 0; VertSubIdx < 3; VertSubIdx++)
			{
				double DistSq = (Pos - Mesh->GetVertex(TriVertIDs[VertSubIdx])).SquareLength();
				if (DistSq <= SnapElementThresholdSq && (BestSubIdx == -1 || DistSq < BestElementDistSq))
				{
					BestSubIdx = VertSubIdx;
					BestElementDistSq = DistSq;
				}
			}

			if (BestSubIdx > -1)
			{
				SurfacePt.ElementID = TriVertIDs[BestSubIdx];
				SurfacePt.PointType = ESurfacePointType::Vertex;
				return;
			}

			Index3 TriEdgeIDs = Mesh->GetTriangleEdge(TriID);

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
				SurfacePt.PointType = ESurfacePointType::Edge;
				SurfacePt.BaryCoord = Vector3(BestEdgeParam, 1 - BestEdgeParam, 0);
				return;
			}
		}

		static FMeshSurfacePoint RelocateTrianglePointAfterRefinement(const DynamicMesh* Mesh, const Vector3& PosInVertexCoordSpace, const std::vector<int>& TriIDs, float SnapElementThresholdSq)
		{
			double BestTriDistSq = 0;
			Vector3 BestBaryCoords;
			int BestTriID = -1;
			for (int TriID : TriIDs)
			{
				assert(Mesh->IsTriangle(TriID));
				Index3 TriVertIDs = Mesh->GetTriangle(TriID);
				Triangle3 Tri(Mesh->GetVertex(TriVertIDs.a), Mesh->GetVertex(TriVertIDs.b), Mesh->GetVertex(TriVertIDs.c));
				Triangle3::BaryCentric3DDecomposionResult info = Tri.BaryCentric3DDecomposion(PosInVertexCoordSpace);
				double DistSq = info.SqrDistance;
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
			Mesh->GetTriangleVertices(TID, V[0], V[1], V[2]);
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
				BaseFaceNormals.push_back(WorkingMesh->GetTriangleNormal(i));
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
				Mesh->GetTriangleVertices(SegToEls.BaseTID, Tri.v0, Tri.v1, Tri.v2);
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

					// check for an edge match
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
						PtOnMesh.ElemID = Mesh->GetTriangleEdge(SegToEls.BaseTID, OnEdgeIdx);

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

				auto it = FaceVertices.find(TID);
				while (it != FaceVertices.end())
				{
					PtIndices.push_back(it->second);
					++it;
				}

				FPtOnMesh& Pt = IntersectionVerts[PtIdx];

				Mesh->GetTriangleVertices(TID, Tri.v0, Tri.v1, Tri.v2);

				Vector3 BaryCoords = Tri.BaryCentric3D(Pt.Pos);
				PokeTriangleInfo PokeInfo;
				bool success = Mesh->PokeTriangle(TID, BaryCoords, PokeInfo);
				assert(success);
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

				auto it = EdgeVertices.find(EID);
				while (it != EdgeVertices.end())
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
				EdgeSplitInfo SplitInfo;
				bool success = Mesh->SplitEdge(EID, SplitInfo, SplitParam);
				assert(success);

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
			double BestDSq = SnapToleranceSq;
			int BestIdx = -1;
			for (int SubIdx = 0; SubIdx < 3; SubIdx++)
			{
				double DSq = (Tri[SubIdx] - V).SquareLength();
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
				float DSq = Seg.SqrDistanceToPoint(V);
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
				float DSq = Seg.SqrDistanceToPoint(Pos);
				if (DSq < BestDSq)
				{
					BestDSq = DSq;
					BestIdx = Idx;
				}
			}
			return BestIdx;
		}

		int OnEdge(Index3 EIDs, const Vector3& Pos, double BestDSq)
		{
			int BestIdx = -1;
			for (int Idx = 0; Idx < 3; Idx++)
			{
				int EID = EIDs[Idx];
				Index2 EVIDs = Mesh->GetEdgeV(EID);
				Segment3 Seg(Mesh->GetVertex(EVIDs.a), Mesh->GetVertex(EVIDs.b));
				double DSq = Seg.SqrDistanceToPoint(Pos);
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
			Mesh->GetTriangleVertices(TID, Tri.v0, Tri.v1, Tri.v2);
			Vector3 bary = Tri.BaryCentric3D(Pos);
			return (bary.x >= 0 && bary.y >= 0 && bary.z >= 0
				&& bary.x < 1 && bary.y <= 1 && bary.z <= 1);

		}
	};

	bool GeometryCut::Compute(const IntersectionsQueryResult& Intersections)
	{
		bool bSuccess = true;

		for (int MeshIdx = 0; MeshIdx < 2; MeshIdx++)
		{
			FCutWorkingInfo WorkingInfo(Meshe[MeshIdx], SnapTolerance);
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

	bool GeometryBoolean::Compute()
	{
		// copy meshes
		DynamicMesh CutMeshB(*Meshes[1]);

		MeshNew = new DynamicMesh;
		*MeshNew = *Meshes[0];

		DynamicMesh* CutMesh[2]{ MeshNew, &CutMeshB };

		Box3 CombinedAABB = Box3::Transform(CutMesh[0]->Bounds, Transforms[0].pos, Transforms[0].quat);
		Box3 MeshB_AABB = Box3::Transform(CutMesh[1]->Bounds, Transforms[1].pos, Transforms[1].quat);
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

		bool bOpOnSingleMesh = Operation == BooleanOp::TrimInside || Operation == BooleanOp::TrimOutside || Operation == BooleanOp::NewGroupInside || Operation == BooleanOp::NewGroupOutside;

		GeometryCut Cut(CutMesh[0], CutMesh[1]);
		Cut.SnapTolerance = SnapTolerance;
		Cut.Compute(Intersections);

		int NumMeshesToProcess = bOpOnSingleMesh ? 1 : 2;
		float DegenerateEdgeTolFactor = 1.5f;

		if (bCollapseDegenerateEdgesOnCut)
		{
			double DegenerateEdgeTolSq = DegenerateEdgeTolFactor * DegenerateEdgeTolFactor * SnapTolerance * SnapTolerance;
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
					if ((A, B).SquareLength() > DegenerateEdgeTolSq)
					{
						continue;
					}
					Index2 EV = CutMesh[MeshIdx]->GetEdgeV(EID);

					// if the vertex we'd remove is on a seam, try removing the other one instead
					// TODO
					//if (CutMesh[MeshIdx]->HasAttributes() && CutMesh[MeshIdx]->Attributes()->IsSeamVertex(EV.B, false))
					//{
					//	Swap(EV.a, EV.b);
					//	// if they were both on seams, then collapse should not happen?  (& would break OnCollapseEdge assumptions in overlay)
					//	if (CutMesh[MeshIdx]->HasAttributes() && CutMesh[MeshIdx]->Attributes()->IsSeamVertex(EV.B, false))
					//	{
					//		continue;
					//	}
					//}

					EdgeCollapseInfo CollapseInfo;
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

		/*
		// edges that will become new boundary edges after the boolean op removes triangles on each mesh
		std::vector<int> CutBoundaryEdges[2];
		// Vertices on the cut boundary that *may* not have a corresonding vertex on the other mesh
		std::set<int> PossUnmatchedBdryVerts[2];

		// delete geometry according to boolean rules, tracking the boundary edges
		{ // (just for scope)
			// first decide what triangles to delete for both meshes (*before* deleting anything so winding doesn't get messed up!)
			std::vector<bool> KeepTri[2];
			// This array is used to double-check the assumption that we will delete the other surface when we keep a coplanar tri
			// Note we only need it for mesh 0 (i.e., the mesh we try to keep triangles from when we preserve coplanar surfaces)
			std::vector<int> DeleteIfOtherKept;
			if (NumMeshesToProcess > 1)
			{
				DeleteIfOtherKept.Init(-1, CutMesh[0]->MaxTriangleID());
			}
			for (int MeshIdx = 0; MeshIdx < NumMeshesToProcess; MeshIdx++)
			{
				TFastWindingTree<GeometryData> Winding(&Spatial[1 - MeshIdx]);
				FDynamicMeshAABBTree3& OtherSpatial = Spatial[1 - MeshIdx];
				GeometryData& ProcessMesh = *CutMesh[MeshIdx];
				int MaxTriID = ProcessMesh.MaxTriangleID();
				KeepTri[MeshIdx].SetNumUninitialized(MaxTriID);
				bool bCoplanarKeepSameDir = (Operation != BooleanOp::Difference && Operation != BooleanOp::TrimInside && Operation != BooleanOp::NewGroupInside);
				bool bRemoveInside = 1; // whether to remove the inside triangles (e.g. for union) or the outside ones (e.g. for intersection)
				if (Operation == BooleanOp::NewGroupOutside || Operation == BooleanOp::TrimOutside || Operation == BooleanOp::Intersect || (Operation == BooleanOp::Difference && MeshIdx == 1))
				{
					bRemoveInside = 0;
				}
				FMeshNormals OtherNormals(OtherSpatial.GetMesh());
				OtherNormals.ComputeTriangleNormals();
				const double OnPlaneTolerance = SnapTolerance;
				IMeshSpatial::FQueryOptions NonDegenCoplanarCandidateFilter(OnPlaneTolerance,
					[&OtherNormals](int TID) -> bool // filter degenerate triangles from matching
					{
						// By convention, the normal for degenerate triangles is the zero vector
						return !OtherNormals[TID].IsZero();
					});
				ParallelFor(MaxTriID, [&](int TID)
					{
						if (!ProcessMesh.IsTriangle(TID))
						{
							return;
						}

						FTriangle3d Tri;
						ProcessMesh.GetTriVertices(TID, Tri.V[0], Tri.V[1], Tri.V[2]);
						Vector3 Centroid = Tri.Centroid();

						// first check for the coplanar case
						{
							double DSq;
							int OtherTID = OtherSpatial.FindNearestTriangle(Centroid, DSq, NonDegenCoplanarCandidateFilter);
							if (OtherTID > -1) // only consider it coplanar if there is a matching tri
							{

								Vector3 OtherNormal = OtherNormals[OtherTID];
								Vector3 Normal = ProcessMesh.GetTriNormal(TID);
								double DotNormals = OtherNormal.Dot(Normal);

								//if (FMath::Abs(DotNormals) > .9) // TODO: do we actually want to check for a normal match? coplanar vertex check below is more robust?
								{
									// To be extra sure it's a coplanar match, check the vertices are *also* on the other mesh (w/in SnapTolerance)

									bool bAllTrisOnOtherMesh = true;
									for (int Idx = 0; Idx < 3; Idx++)
									{
										// use a slightly more forgiving tolerance to account for the likelihood that these vertices were mesh-cut right to the boundary of the coplanar region and have some additional error
										if (OtherSpatial.FindNearestTriangle(Tri.V[Idx], DSq, OnPlaneTolerance * 2) == GeometryData::InvalidID)
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
											return;
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
											return;
										}
									}
								}
							}
						}

						// didn't already return a coplanar result; use the winding number
						double WindingNum = Winding.FastWindingNumber(Centroid);
						KeepTri[MeshIdx][TID] = (WindingNum > WindingThreshold) != bRemoveInside;
					});
			}

			// Don't keep coplanar tris if the matched, second-mesh tri that we expected to delete was actually kept
			if (NumMeshesToProcess > 1)
			{
				for (int TID : CutMesh[0]->TriangleIndicesItr())
				{
					int DeleteIfOtherKeptTID = DeleteIfOtherKept[TID];
					if (DeleteIfOtherKeptTID > -1 && KeepTri[1][DeleteIfOtherKeptTID])
					{
						KeepTri[0][TID] = false;
					}
				}
			}

			for (int MeshIdx = 0; MeshIdx < NumMeshesToProcess; MeshIdx++)
			{
				GeometryMesh& ProcessMesh = *CutMesh[MeshIdx];
				for (int EID : ProcessMesh.EdgeIndicesItr())
				{
					GeometryMesh::FEdge Edge = ProcessMesh.GetEdge(EID);
					if (Edge.Tri.B == IndexConstants::InvalidID || KeepTri[MeshIdx][Edge.Tri.A] == KeepTri[MeshIdx][Edge.Tri.B])
					{
						continue;
					}

					CutBoundaryEdges[MeshIdx].Add(EID);
					PossUnmatchedBdryVerts[MeshIdx].Add(Edge.Vert.A);
					PossUnmatchedBdryVerts[MeshIdx].Add(Edge.Vert.B);
				}
			}
			// now go ahead and delete from both meshes
			bool bRegroupInsteadOfDelete = Operation == BooleanOp::NewGroupInside || Operation == BooleanOp::NewGroupOutside;
			int NewGroupID = -1;
			std::vector<int> NewGroupTris;
			if (bRegroupInsteadOfDelete)
			{
				assert(NumMeshesToProcess == 1);
				NewGroupID = CutMesh[0]->AllocateTriangleGroup();
			}
			for (int MeshIdx = 0; MeshIdx < NumMeshesToProcess; MeshIdx++)
			{
				GeometryMesh& ProcessMesh = *CutMesh[MeshIdx];

				for (int TID = 0; TID < KeepTri[MeshIdx].size(); TID++)
				{
					if (ProcessMesh.IsTriangle(TID) && !KeepTri[MeshIdx][TID])
					{
						if (bRegroupInsteadOfDelete)
						{
							ProcessMesh.SetTriangleGroup(TID, NewGroupID);
							NewGroupTris.Add(TID);
						}
						else
						{
							ProcessMesh.RemoveTriangle(TID, true, false);
						}
					}
				}
			}
			if (bRegroupInsteadOfDelete)
			{
				// the new triangle group could include disconnected components; best to give them separate triangle groups
				FMeshConnectedComponents Components(CutMesh[0]);
				Components.FindConnectedTriangles(NewGroupTris);
				for (int ComponentIdx = 1; ComponentIdx < Components.size(); ComponentIdx++)
				{
					int SplitGroupID = CutMesh[0]->AllocateTriangleGroup();
					for (int TID : Components.GetComponent(ComponentIdx).Indices)
					{
						CutMesh[0]->SetTriangleGroup(TID, SplitGroupID);
					}
				}
			}
		}

		// correspond vertices across both meshes (in cases where both meshes were processed)
		std::map<int, int> AllVIDMatches; // mapping of matched vertex IDs from cutmesh 0 to cutmesh 1
		if (NumMeshesToProcess == 2)
		{
			std::map<int, int> FoundMatchesMaps[2]; // mappings of matched vertex IDs from mesh 1->0 and mesh 0->1
			double SnapToleranceSq = SnapTolerance * SnapTolerance;

			// ensure segments that are now on boundaries have 1:1 vertex correspondence across meshes
			for (int MeshIdx = 0; MeshIdx < 2; MeshIdx++)
			{
				int OtherMeshIdx = 1 - MeshIdx;
				GeometryMesh& OtherMesh = *CutMesh[OtherMeshIdx];

				TPointHashGrid3d<int> OtherMeshPointHash(OtherMesh.GetBounds(true).MaxDim() / 64, -1);
				for (int BoundaryVID : PossUnmatchedBdryVerts[OtherMeshIdx])
				{
					OtherMeshPointHash.InsertPointUnsafe(BoundaryVID, OtherMesh.GetVertex(BoundaryVID));
				}

				FSparseDynamicOctree3 EdgeOctree;
				EdgeOctree.RootDimension = .25;
				EdgeOctree.SetMaxTreeDepth(7);
				auto EdgeBounds = [&OtherMesh](int EID)
				{
					GeometryMesh::FEdge Edge = OtherMesh.GetEdge(EID);
					Vector3 A = OtherMesh.GetVertex(Edge.Vert.A);
					Vector3 B = OtherMesh.GetVertex(Edge.Vert.B);
					if (A.X > B.X)
					{
						Swap(A.X, B.X);
					}
					if (A.Y > B.Y)
					{
						Swap(A.Y, B.Y);
					}
					if (A.Z > B.Z)
					{
						Swap(A.Z, B.Z);
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
					if (MeshIdx == 1 && FoundMatchesMaps[0].Contains(BoundaryVID))
					{
						continue; // was already snapped to a vertex
					}

					Vector3 Pos = CutMesh[MeshIdx]->GetVertex(BoundaryVID);
					TPair<int, double> VIDDist = OtherMeshPointHash.FindNearestInRadius(Pos, SnapTolerance, [&Pos, &OtherMesh](int VID)
						{
							return DistanceSquared(Pos, OtherMesh.GetVertex(VID));
						});
					int NearestVID = VIDDist.Key; // ID of nearest vertex on other mesh
					double DSq = VIDDist.Value;   // square distance to that vertex

					if (NearestVID != GeometryData::InvalidID)
					{

						int* Match = FoundMatches.Find(NearestVID);
						if (Match)
						{
							double OldDSq = DistanceSquared(CutMesh[MeshIdx]->GetVertex(*Match), OtherMesh.GetVertex(NearestVID));
							if (DSq < OldDSq) // new vertex is a better match than the old one
							{
								int OldVID = *Match; // copy old VID out of match before updating the std::map
								FoundMatches.Add(NearestVID, BoundaryVID); // new VID is recorded as best match

								// old VID is swapped in as the one to consider as unmatched
								// it will now be matched below
								BoundaryVID = OldVID;
								Pos = CutMesh[MeshIdx]->GetVertex(BoundaryVID);
								DSq = OldDSq;
							}
							NearestVID = GeometryData::InvalidID; // one of these vertices will be unmatched
						}
						else
						{
							FoundMatches.Add(NearestVID, BoundaryVID);
						}
					}

					// if we didn't find a valid match, try to split the nearest edge to create a match
					if (NearestVID == GeometryData::InvalidID)
					{
						// vertex had no match -- try to split edge to match it
						Box3 QueryBox(Pos, SnapTolerance);
						EdgesInRange.Reset();
						EdgeOctree.RangeQuery(QueryBox, EdgesInRange);

						int OtherEID = FindNearestEdge(OtherMesh, EdgesInRange, Pos);
						if (OtherEID != GeometryData::InvalidID)
						{
							Vector3 EdgePts[2];
							OtherMesh.GetEdgeV(OtherEID, EdgePts[0], EdgePts[1]);
							// only accept the match if it's not going to create a degenerate edge -- TODO: filter already-matched edges from the FindNearestEdge query!
							if (DistanceSquared(EdgePts[0], Pos) > SnapToleranceSq && DistanceSquared(EdgePts[1], Pos) > SnapToleranceSq)
							{
								FSegment3d Seg(EdgePts[0], EdgePts[1]);
								double Along = Seg.ProjectUnitRange(Pos);
								GeometryData::FEdgeSplitInfo SplitInfo;
								if (ensure(EMeshResult::Ok == OtherMesh.SplitEdge(OtherEID, SplitInfo, Along)))
								{
									FoundMatches.Add(SplitInfo.NewVertex, BoundaryVID);
									OtherMesh.SetVertex(SplitInfo.NewVertex, Pos);
									CutBoundaryEdges[OtherMeshIdx].Add(SplitInfo.NewEdges.A);
									UpdateEdge(OtherEID);
									AddEdge(SplitInfo.NewEdges.A);
									// Note: Do not update PossUnmatchedBdryVerts with the new vertex, because it is already matched by construction
									// Likewise do not update the pointhash -- we don't want it to find vertices that were already perfectly matched
								}
							}
						}
					}
				}

				// actually snap the positions together for final matches
				for (TPair<int, int>& Match : FoundMatches)
				{
					CutMesh[MeshIdx]->SetVertex(Match.Value, OtherMesh.GetVertex(Match.Key));

					// Copy match to AllVIDMatches; note this is always mapping from CutMesh 0 to 1
					int VIDs[2]{ Match.Key, Match.Value }; // just so we can access by index
					AllVIDMatches.Add(VIDs[1 - MeshIdx], VIDs[MeshIdx]);
				}
			}
		}

		if (bSimplifyAlongNewEdges)
		{
			SimplifyAlongNewEdges(NumMeshesToProcess, CutMesh, CutBoundaryEdges, AllVIDMatches);
		}

		if (Operation == BooleanOp::Difference)
		{
			// TODO: implement a way to flip all the triangles in the mesh without building this AllTID array
			std::vector<int> AllTID;
			for (int TID : CutMesh[1]->TriangleIndicesItr())
			{
				AllTID.Add(TID);
			}
			FDynamicMeshEditor FlipEditor(CutMesh[1]);
			FlipEditor.ReverseTriangleOrientations(AllTID, true);
		}

		if (Cancelled())
		{
			return false;
		}

		bool bSuccess = true;

		if (NumMeshesToProcess > 1)
		{
			FDynamicMeshEditor Editor(MeshNew);
			FMeshIndexMappings IndexMaps;
			Editor.AppendMesh(CutMesh[1], IndexMaps);

			if (bPopulateSecondMeshGroupMap)
			{
				SecondMeshGroupMap = IndexMaps.GetGroupMap();
			}

			if (bWeldSharedEdges)
			{
				bool bWeldSuccess = MergeEdges(IndexMaps, CutMesh, CutBoundaryEdges, AllVIDMatches);
				bSuccess = bSuccess && bWeldSuccess;
			}
			else
			{
				CreatedBoundaryEdges = CutBoundaryEdges[0];
				for (int OldMeshEID : CutBoundaryEdges[1])
				{
					if (!CutMesh[1]->IsEdge(OldMeshEID))
					{
						ensure(false);
						continue;
					}
					FIndex2i OtherEV = CutMesh[1]->GetEdgeV(OldMeshEID);
					int MappedEID = MeshNew->FindEdge(IndexMaps.GetNewVertex(OtherEV.A), IndexMaps.GetNewVertex(OtherEV.B));
					checkSlow(MeshNew->IsBoundaryEdge(MappedEID));
					CreatedBoundaryEdges.Add(MappedEID);
				}
			}
		}
		// For NewGroupInside and NewGroupOutside, the cut doesn't create boundary edges.
		else if (Operation != BooleanOp::NewGroupInside && Operation != BooleanOp::NewGroupOutside)
		{
			CreatedBoundaryEdges = CutBoundaryEdges[0];
		}

		if (bTrackAllNewEdges)
		{
			for (int eid : CreatedBoundaryEdges)
			{
				AllNewEdges.Add(eid);
			}
		}

		if (bPutResultInInputSpace)
		{
			MeshTransforms::ApplyTransform(*MeshNew, ResultTransform);
			ResultTransform = Transform::Identity();
		}

		return bSuccess;
		*/

		return true;
	}

}