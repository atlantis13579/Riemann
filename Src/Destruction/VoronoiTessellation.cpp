
#include "VoronoiTessellation.h"

#include "../CollisionPrimitive/Triangle3.h"
#include "../Maths/Box1.h"
#include "../Maths/Box2.h"
#include "../Maths/Maths.h"
#include "../Maths/Frame3.h"
#include "../Geometry/Delaunay.h"
#include "../Geometry/DynamicMesh.h"
#include "../Geometry/GeometryAttributes.h"

namespace Riemann
{
	static int EncodePlaneToMaterial(int PlaneIdx)
	{
		return -(PlaneIdx + 1);
	}

	VoronoiMesh::VoronoiMesh(const std::vector<Vector3>& points, const Box3& bounds, const float eps)
	{
		Voronoi3 v(points, bounds, eps);
		v.Build();

		int NumCells = v.GetNumPoints();
		mMeshs.resize(NumCells);
		for (size_t i = 0; i < NumCells; ++i)
		{
			mMeshs[i] = new DynamicMesh();
		}

		if (v.IsInfinitePlane())
		{
			BuildMesh_SinglePlane(v);
		}
		else
		{
			const bool bNoise = true;
			if (bNoise)
			{
				BuildMesh_WithNoise(v);
			}
			else
			{
				BuildMesh_WithoutNoise(v);
			}
		}
	}

	VoronoiMesh::~VoronoiMesh()
	{
	}

	void VoronoiMesh::BuildMesh_SinglePlane(const Voronoi3& v)
	{
		/*
		bool bHasGrout = Grout > 0;

		int MID = EncodePlaneToMaterial(0);
		Voronoi3::Plane Plane = v.mPlanes[0];

		Maths::Frame3 PlaneFrame(Plane.GetOrigin(), Plane.GetOrigin());
		Box1 ZRange;
		Box2 XYRange;
		for (int CornerIdx = 0; CornerIdx < 8; CornerIdx++)
		{
			Vector3 Corner = DomainBounds.GetCorner(CornerIdx);
			XYRange.Encapsulate(PlaneFrame.ProjectXZ(Corner));
			ZRange.Encapsulate(Plane.PlaneDot(Vector3(Corner)));
		}
		//if (FMathd::SignNonZero(ZRange.Min) == FMathd::SignNonZero(ZRange.Max))
		//{
		//	// TODO: early out for plane that doesn't even intersect the domain bounding box?
		//}

		DynamicMesh PlaneMesh(true, true, false, false);
		AugmentedDynamicMesh::EnableUVChannels(PlaneMesh, NumUVLayers);
		VertexInfo PlaneVertInfo;
		PlaneVertInfo.bHasColor = true;
		PlaneVertInfo.bHasNormal = true;
		PlaneVertInfo.Normal = -Vector3(Plane.GetNormal());

		for (int CornerIdx = 0; CornerIdx < 4; CornerIdx++)
		{
			PlaneVertInfo.Position = PlaneFrame.FromPlaneUV(XYRange.GetCorner(CornerIdx));
			Vector2 UV = Vector2(XYRange.GetCorner(CornerIdx) - XYRange.Min);
			int VID = PlaneMesh.AppendVertex(PlaneVertInfo);
			AugmentedDynamicMesh::SetAllUV(PlaneMesh, VID, UV, NumUVLayers);
		}
		PlaneMesh.AppendTriangle(0, 1, 2);
		PlaneMesh.AppendTriangle(0, 2, 3);

		if (bNoise)
		{
			float Spacing = GetSafeNoiseSpacing(static_cast<float>(XYRange.Area()), v.InternalSurfaceMaterials.NoiseSettings->PointSpacing);
			RemeshForNoise(PlaneMesh, EEdgeRefineFlags::SplitsOnly, Spacing);
			ApplyNoise(PlaneMesh, PlaneFrame.GetAxis(2), v.InternalSurfaceMaterials.NoiseSettings.GetValue(), true);
			FMeshNormals::QuickComputeVertexNormals(PlaneMesh);
		}
		std::vector<int> PlaneBoundary,  // loop of vertex IDs on the boundary of PlaneMesh (starting with vertex 0)
			PlaneBoundaryCornerIndices; // indices of the corner vertices in the PlaneBoundary array
		{
			float Offset = ZRange.Max;
			FMeshBoundaryLoops Boundary(&PlaneMesh);
			assert(Boundary.GetLoopCount() == 1);
			int FirstIdx;
			bool bFound = Boundary[0].Vertices.Find(0, FirstIdx);
			assert(bFound);
			PlaneBoundary = Boundary[0].Vertices;
			if (FirstIdx != 0)
			{
				Algo::Rotate(PlaneBoundary, FirstIdx);
			}
			assert(PlaneBoundary[0] == 0);

			PlaneBoundaryCornerIndices.push_back(0);
			int FoundIndices = 1;
			for (int VIDIdx = 0; VIDIdx < PlaneBoundary.size(); VIDIdx++)
			{
				int VID = PlaneBoundary[VIDIdx];
				if (VID == FoundIndices)
				{
					FoundIndices++;
					PlaneBoundaryCornerIndices.push_back(VIDIdx);
				}
			}
		}
		DynamicMesh* Meshes[2];
		if (!bOnlyGrout)
		{
			for (int Side = 0; Side < 2; Side++)
			{
				Meshes[Side] = mMeshs[Side];
				*Meshes[Side] = PlaneMesh;

				float Offset = ZRange.Max;
				std::vector<int> CapBoundary, CapBoundaryCornerIndices;

				if (Side == 0)
				{
					Meshes[Side]->ReverseOrientation(true);
					Offset = ZRange.Min;
				}
				PlaneVertInfo.Normal = Vector3(Plane.GetNormal()) * (-1.0f + (float)Side * 2.0f);
				Vector3 OffsetVec = Vector3(Plane.GetNormal()) * Offset;

				for (int CornerIdx = 0; CornerIdx < 4; CornerIdx++)
				{
					PlaneVertInfo.Position = Meshes[Side]->GetVertex(CornerIdx) + OffsetVec;
					// UVs shouldn't matter for outer box vertices because they're outside of the domain by construction ...
					CapBoundary.push_back(Meshes[Side]->AppendVertex(PlaneVertInfo));
					CapBoundaryCornerIndices.push_back(CornerIdx);
				}
				int NewTris[2]{
					Meshes[Side]->AppendTriangle(CapBoundary[0], CapBoundary[1], CapBoundary[2]),
					Meshes[Side]->AppendTriangle(CapBoundary[0], CapBoundary[2], CapBoundary[3])
				};
				if (Side == 1)
				{
					Meshes[Side]->ReverseTriOrientation(NewTris[0]);
					Meshes[Side]->ReverseTriOrientation(NewTris[1]);
				}
				FDynamicMeshEditor Editor(Meshes[Side]);
				FDynamicMeshEditResult ResultOut;
				Editor.StitchSparselyCorrespondedVertexLoops(PlaneBoundary, PlaneBoundaryCornerIndices, CapBoundary, CapBoundaryCornerIndices, ResultOut, Side == 0);
			}
		}
		if (bHasGrout)
		{
			int GroutIdx = bOnlyGrout ? 0 : 2;
			DynamicMesh* GroutMesh = mMeshs[GroutIdx];
			Vector3 GroutOffset = (Vector3)Plane.GetNormal() * (Grout * .5);
			if (!bOnlyGrout)
			{
				for (int Side = 0; Side < 2; Side++)
				{
					// shift both sides out by Grout/2
					MeshTransforms::Translate(*Meshes[Side], GroutOffset * (-1.0 + (float)Side * 2.0));
				}
			}

			// make the center (grout) by stitching together two offset copies of PlaneMesh
			*GroutMesh = PlaneMesh;
			GroutMesh->ReverseOrientation(true);
			MeshTransforms::Translate(*GroutMesh, GroutOffset);
			FMeshIndexMappings IndexMaps;
			FDynamicMeshEditor Editor(GroutMesh);
			Editor.AppendMesh(&PlaneMesh, IndexMaps, [GroutOffset](int VID, const Vector3& PosIn) {return PosIn - GroutOffset; });
			std::vector<int> AppendPlaneBoundary; AppendPlaneBoundary.Reserve(PlaneBoundary.size());
			std::vector<int> RevBoundary = PlaneBoundary;
			Algo::Reverse(RevBoundary);
			for (int VID : RevBoundary)
			{
				AppendPlaneBoundary.push_back(IndexMaps.GetNewVertex(VID));
			}
			FDynamicMeshEditResult ResultOut;
			Editor.StitchVertexLoopsMinimal(RevBoundary, AppendPlaneBoundary, ResultOut);
		}

		// fix up custom attributes and material IDs for all meshes
		for (int CellIdx = 0; CellIdx < CellMeshes.size(); CellIdx++)
		{
			DynamicMesh& Mesh = CellMeshes[CellIdx].AugMesh;

			// re-enable tangents and visibility attributes, since these are lost when we set the mesh to a copy of the plane mesh
			AugmentedDynamicMesh::Augment(Mesh, NumUVLayers);

			// Set all material IDs to the one plane's corresponding material ID
			for (int TID : Mesh.TriangleIndicesItr())
			{
				Mesh.Attributes()->GetMaterialID()->SetNewValue(TID, MID);
			}
		}
		*/
	}

	void VoronoiMesh::BuildMesh_WithoutNoise(const Voronoi3& v)
	{
		for (size_t i = 0; i < v.mCells.size(); ++i)
		{
			const std::pair<int, int>& p = v.mCells[i];
			DynamicMesh* Meshes[2] = { 0 };
			Meshes[0] = mMeshs[p.first];
			Meshes[1] = nullptr;

			int OtherCell = p.second < 0 ? OutsideCellIndex : p.second;
			int NumMeshes = OtherCell < 0 ? 1 : 2;
			if (NumMeshes == 2)
			{
				Meshes[1] = mMeshs[OtherCell];
			}

			const std::vector<int>& PlaneBoundary = v.mBoundaries[i];
			Vector3 Origin = v.mPlanes[i].GetOrigin();
			Vector3 Normal = v.mPlanes[i].GetNormal();
			Maths::Frame3 PlaneFrame(Origin, Normal);

			VertexInfo info;
			info.bHasColor = true;
			info.bHasNormal = true;
			info.bHasUV = true;
			int VertStart[2]{ -1, -1 };
			for (int j = 0; j < NumMeshes; ++j)
			{
				info.Normal = Normal;
				if (j == 1 && OtherCell != OutsideCellIndex)
				{
					info.Normal *= -1.0f;
				}
				VertStart[j] = Meshes[j]->GetVertexCount();
				Vector2 MinUV(FLT_MAX, FLT_MAX);
				for (int BoundaryVertex : PlaneBoundary)
				{
					Vector3 Position = v.mBoundaryVertices[BoundaryVertex];
					Vector2 UV = PlaneFrame.ProjectXZ(Position);
					MinUV.x = std::min(UV.x, MinUV.x);
					MinUV.y = std::min(UV.y, MinUV.y);
				}
				for (int BoundaryVertex : PlaneBoundary)
				{
					info.Position = v.mBoundaryVertices[BoundaryVertex];
					info.Color = Vector3::InfMax();
					info.UVs = PlaneFrame.ProjectXZ(info.Position) - MinUV;
					int VID = Meshes[j]->AppendVertex(info);
					DynamicMeshAttributeSet::SetVertexColor(Meshes[j], VID, Vector4::Zero());
					DynamicMeshAttributeSet::SetAllUV(Meshes[j], VID, info.UVs, NumUVLayers);
				}
			}

			int MID = EncodePlaneToMaterial((int)i);
			if (AssumeConvexCells)
			{
				// put a fan
				for (int V0 = 0, V1 = 1, V2 = 2; V2 < (int)PlaneBoundary.size(); V1 = V2++)
				{
					for (int MeshIdx = 0; MeshIdx < NumMeshes; MeshIdx++)
					{
						int Offset = VertStart[MeshIdx];
						Index3 Tri(V0 + Offset, V1 + Offset, V2 + Offset);
						if (MeshIdx == 1 && OtherCell != OutsideCellIndex)
						{
							std::swap(Tri.b, Tri.c);
						}
						int TID = Meshes[MeshIdx]->AppendTriangle(Tri);
						if (TID > -1)
						{
							Meshes[MeshIdx]->Attributes()->GetMaterialID()->SetNewValue(TID, MID);
						}
					}
				}
			}
			else // cells may not be convex; cannot triangulate w/ fan
			{
				// Delaunay triangulate
				std::vector<Vector2> Polygon;
				for (int V = 0; V < PlaneBoundary.size(); ++V)
				{
					Vector2 UV = Meshes[0]->GetVertexUV(VertStart[0] + V);
					Polygon.push_back(UV);
				}

				Delaunay Triangulation;
				Triangulation.Triangulate(Polygon);

				for (int MeshIdx = 0; MeshIdx < NumMeshes; MeshIdx++)
				{
					int Offset = VertStart[MeshIdx];
					for (DelaunayTriangle& Triangle : Triangulation.Triangles)
					{
						Index3 vt(Triangle.v1 + Offset, Triangle.v2 + Offset, Triangle.v3 + Offset);
						if (MeshIdx == 1 && OtherCell != OutsideCellIndex)
						{
							std::swap(vt.b, vt.c);
						}
						int TID = Meshes[MeshIdx]->AppendTriangle(vt);
						if (TID > -1)
						{
							Meshes[MeshIdx]->Attributes()->GetMaterialID()->SetNewValue(TID, MID);
						}
					}
				}
			}
		}
	}

	void VoronoiMesh::BuildMesh_WithNoise(const Voronoi3& v)
	{
		/*
		std::vector<DynamicMesh> PlaneMeshes;
		PlaneMeshes.resize(v.mPlanes.size());
		std::string OriginalPositionAttribute = "OriginalPosition";
		for (DynamicMesh& PlaneMesh : PlaneMeshes)
		{
			AugmentedDynamicMesh::EnableUVChannels(PlaneMesh, NumUVLayers);
			PlaneMesh.EnableVertexNormals(Vector3::UnitZ());
			PlaneMesh.EnableAttributes();
			PlaneMesh.Attributes()->EnableMaterialID();
			PlaneMesh.Attributes()->AttachAttribute(OriginalPositionAttribute, new TDynamicMeshVertexAttribute<float, 3>(&PlaneMesh));
		}

		struct FPlaneIdxAndFlip
		{
			int PlaneIdx;
			bool bIsFlipped;
		};
		std::vector<std::vector<FPlaneIdxAndFlip>> CellPlanes; // per cell, the planes that border that cell
		CellPlanes.resize(NumCells);

		for (size_t PlaneIdx = 0; PlaneIdx < v.mCells.size(); PlaneIdx++)
		{
			const std::pair<int, int>& CellPair = v.mCells[PlaneIdx];
			int OtherCell = CellPair.second < 0 ? OutsideCellIndex : CellPair.second;
			if (CellPlanes.IsValidIndex(CellPair.first))
			{
				CellPlanes[CellPair.first].push_back({ PlaneIdx, false });
			}
			if (CellPlanes.IsValidIndex(OtherCell))
			{
				CellPlanes[OtherCell].push_back({ PlaneIdx, true });
			}
		}

		// heuristic to protect against creating too many vertices on remeshing
		float TotalArea = 0;
		for (size_t PlaneIdx = 0; PlaneIdx < v.mPlanes.size(); PlaneIdx++)
		{
			const std::vector<int>& PlaneBoundary = v.mBoundaries[PlaneIdx];
			const Vector3& V0 = v.mBoundaryVertices[PlaneBoundary[0]];
			Vector3 AreaVec = Vector3::Zero();
			for (int V1Idx = 1, V2Idx = 2; V2Idx < PlaneBoundary.size(); V1Idx = V2Idx++)
			{
				const Vector3& V1 = v.mBoundaryVertices[PlaneBoundary[V1Idx]];
				const Vector3& V2 = v.mBoundaryVertices[PlaneBoundary[V2Idx]];
				AreaVec += (V1 - V0) ^ (V2 - V1);
			}
			TotalArea += static_cast<float>(AreaVec.Size());
		}
		float Spacing = GetSafeNoiseSpacing(TotalArea, v.InternalSurfaceMaterials.NoiseSettings->PointSpacing);

		for (size_t PlaneIdx = 0; PlaneIdx < v.mPlanes.size(); ++PlaneIdx)
		{
			DynamicMesh& Mesh = PlaneMeshes[PlaneIdx];
			const std::vector<int>& PlaneBoundary = v.mBoundaries[PlaneIdx];
			Vector3 Normal(v.mPlanes[PlaneIdx].GetNormal());
			Maths::Frame3 PlaneFrame(v.mPlanes[PlaneIdx]);
			VertexInfo PlaneVertInfo;
			PlaneVertInfo.bHasColor = true;
			PlaneVertInfo.bHasUV = false;
			PlaneVertInfo.bHasNormal = true;
			PlaneVertInfo.Normal = Normal;
			// UVs will be set below, after noise is added
			// UnsetVertexColor will be set below

			std::vector<Vector2> Polygon;
			for (int BoundaryVertex : PlaneBoundary)
			{
				PlaneVertInfo.Position = Vector3(v.mBoundaryVertices[BoundaryVertex]);
				Polygon.push_back(PlaneFrame.ProjectXZ(PlaneVertInfo.Position));
				Mesh.AppendVertex(PlaneVertInfo);
			}

			// we do a CDT here to give a slightly better start to remeshing; we could try simple ear clipping instead
			Delaunay Triangulation;
			Triangulation.Triangulate(Polygon);
			if (Triangulation.Triangles.size() == 0) // fall back to ear clipping if the triangulation came back empty
			{
				PolygonTriangulation::TriangulateSimplePolygon(Polygon.GetVertices(), Triangulation.Triangles, false);
			}
			if (Triangulation.Triangles.size() > 0)
			{
				int MID = EncodePlaneToMaterial(PlaneIdx);
				for (Index3 Triangle : Triangulation.Triangles)
				{
					int TID = Mesh.AppendTriangle(Triangle);
					if (TID > -1)
					{
						Mesh.Attributes()->GetMaterialID()->SetNewValue(TID, MID);
					}
				}

				RemeshForNoise(Mesh, EEdgeRefineFlags::SplitsOnly, Spacing);
				TDynamicMeshVertexAttribute<float, 3>* OriginalPosns =
					static_cast<TDynamicMeshVertexAttribute<float, 3>*>(Mesh.Attributes()->GetAttachedAttribute(OriginalPositionAttribute));
				for (int VID : Mesh.VertexIndicesItr())
				{
					OriginalPosns->SetValue(VID, Mesh.GetVertex(VID));
				}
				ApplyNoise(Mesh, Vector3(Normal), v.InternalSurfaceMaterials.NoiseSettings.GetValue());

				FMeshNormals::QuickComputeVertexNormals(Mesh);
			}
		}

		for (size_t CellIdx = 0; CellIdx < mMeshs.size(); CellIdx++)
		{
			DynamicMesh& Mesh = *mMeshs[CellIdx];
			Mesh.Attributes()->AttachAttribute(OriginalPositionAttribute, new TDynamicMeshVertexAttribute<float, 3>(&Mesh));
			bool bFlipForOutsideCell = CellIdx == OutsideCellIndex; // outside cell will be subtracted, and needs all planes flipped vs normal
			for (FPlaneIdxAndFlip PlaneInfo : CellPlanes[CellIdx])
			{
				AppendMesh(Mesh, PlaneMeshes[PlaneInfo.PlaneIdx], PlaneInfo.bIsFlipped ^ bFlipForOutsideCell);
			}
		}

		// resolve self-intersections

		// build hash grid of mesh vertices so we correspond all same-pos vertices across touching meshes
		TPointHashGrid3d<Index2> MeshesVertices(1e-3f, Index2(-1, -1));
		for (size_t CellIdx = 0; CellIdx < mMeshs.size(); CellIdx++)
		{
			DynamicMesh& Mesh = *mMeshs[CellIdx];
			for (int VID : Mesh.VertexIndicesItr())
			{
				MeshesVertices.InsertPointUnsafe(Index2(CellIdx, VID), Mesh.GetVertex(VID));
			}
		}

		// repeatedly detect and resolve collisions until there are no more (or give up after too many iterations)
		std::vector<bool> CellUnmoved; CellUnmoved.Init(false, NumCells);
		const int MaxIters = 10;
		for (int Iters = 0; Iters < MaxIters; Iters++)
		{
			struct FUpdate
			{
				Index2 Tris;
				std::vector<Index2> IDs;
				FUpdate(int TriA = -1, int TriB = -1) : Tris(TriA, TriB)
				{}
			};

			// todo: can parallelize?
			std::vector<std::vector<FUpdate>> Updates; Updates.resize(NumCells);
			bool bAnyUpdatesNeeded = false;
			for (int CellIdx = 0; CellIdx < NumCells; CellIdx++)
			{
				if (CellUnmoved[CellIdx])
				{
					// if nothing moved since last time we resolved self intersections on this cell, don't need to process again
					continue;
				}
				DynamicMesh& Mesh = *mMeshs[CellIdx];
				FDynamicMeshAABBTree3 CellTree(&Mesh, true);
				MeshIntersection::FIntersectionsQueryResult Intersections = CellTree.FindAllSelfIntersections(true);
				for (MeshIntersection::FSegmentIntersection& Seg : Intersections.Segments)
				{
					// manually check for shared edges by vertex position because they might not be topologically connected
					Index3 Tri[2]{ Mesh.GetTriangle(Seg.TriangleID[0]), Mesh.GetTriangle(Seg.TriangleID[1]) };
					int MatchedVertices = 0;
					for (int T0SubIdx = 0; T0SubIdx < 3; T0SubIdx++)
					{
						Vector3 V0 = Mesh.GetVertex(Tri[0][T0SubIdx]);
						for (int T1SubIdx = 0; T1SubIdx < 3; T1SubIdx++)
						{
							Vector3 V1 = Mesh.GetVertex(Tri[1][T1SubIdx]);
							if (DistanceSquared(V0, V1) < FMathd::ZeroTolerance)
							{
								MatchedVertices++;
								break;
							}
						}
					}
					// no shared vertices: treat as a real collision
					// (TODO: only skip shared edges? will need to do something to avoid shared vertices becoming collisions)
					if (MatchedVertices < 1)
					{
						bAnyUpdatesNeeded = true;
						FUpdate& Update = Updates[CellIdx].Emplace_GetRef(Seg.TriangleID[0], Seg.TriangleID[1]);
						for (int TriIdx = 0; TriIdx < 2; TriIdx++)
						{
							for (int VSubIdx = 0; VSubIdx < 3; VSubIdx++)
							{
								int VIdx = Tri[TriIdx][VSubIdx];
								Vector3 P = Mesh.GetVertex(VIdx);
								Index2 IDs(CellIdx, VIdx);
								MeshesVertices.FindPointsInBall(P, 1e-6f, [this, P](Index2 IDs)
									{
										Vector3 Pos = mMeshs[IDs.a]->GetVertex(IDs.b);
										return DistanceSquared(P, Pos);
									}, Update.IDs);
							}
						}
					}
				}
			}
			if (!bAnyUpdatesNeeded)
			{
				break;
			}
			for (int CellIdx = 0; CellIdx < NumCells; CellIdx++)
			{
				CellUnmoved[CellIdx] = true;
			}

			// todo: maybe can parallelize if movements are not applied until after?
			for (size_t CellIdx = 0; CellIdx < mMeshs.size(); CellIdx++)
			{
				DynamicMesh& Mesh = *mMeshs[CellIdx];
				TDynamicMeshVertexAttribute<float, 3>* OriginalPosns =
					static_cast<TDynamicMeshVertexAttribute<float, 3>*>(Mesh.Attributes()->GetAttachedAttribute(OriginalPositionAttribute));
				auto InterpVert = [&Mesh, &OriginalPosns](int VID, float t)
				{
					Vector3 OrigPos, NoisePos;
					OriginalPosns->GetValue(VID, OrigPos);
					NoisePos = Mesh.GetVertex(VID);
					return Maths::LinearInterp(OrigPos, NoisePos, t);
				};
				auto InterpTri = [&Mesh, &InterpVert](int TID, float t)
				{
					Index3 TriVIDs = Mesh.GetTriangle(TID);
					Triangle3 Tri;
					for (int i = 0; i < 3; i++)
					{
						Tri.V[i] = InterpVert(TriVIDs[i], t);
					}
					return Tri;
				};
				auto TestIntersection = [&InterpTri](int TIDA, int TIDB, float t)
				{
					FIntrTriangle3Triangle3d TriTri(InterpTri(TIDA, t), InterpTri(TIDB, t));
					return TriTri.Find();
				};
				// resolve tri-tri intersections on this cell's mesh (moving associated verts on other meshes as needed also)
				for (FUpdate& Update : Updates[CellIdx])
				{
					float tsafe = 0;
					float tbad = 1;
					if (!TestIntersection(Update.Tris.a, Update.Tris.b, tbad))
					{
						continue;
					}
					for (int SearchSteps = 0; SearchSteps < 4; SearchSteps++)
					{
						float tmid = (tsafe + tbad) * 0.5f;
						if (TestIntersection(Update.Tris.a, Update.Tris.b, tmid))
						{
							tbad = tmid;
						}
						else
						{
							tsafe = tmid;
						}
					}
					CellUnmoved[CellIdx] = false;
					for (Index2 IDs : Update.IDs)
					{
						Vector3 OldPos = mMeshs[IDs.a]->GetVertex(IDs.b);
						Vector3 NewPos;
						if (IDs.a == CellIdx)
						{
							NewPos = InterpVert(IDs.b, tsafe);
							Mesh.SetVertex(IDs.b, NewPos);
						}
						else
						{
							CellUnmoved[IDs.a] = false;
							DynamicMesh& OtherMesh = *mMeshs[IDs.a];
							TDynamicMeshVertexAttribute<float, 3>* OtherOriginalPosns =
								static_cast<TDynamicMeshVertexAttribute<float, 3>*>(OtherMesh.Attributes()->GetAttachedAttribute(OriginalPositionAttribute));
							Vector3 OrigPos;
							OtherOriginalPosns->GetValue(IDs.b, OrigPos);
							NewPos = Maths::LinearInterp(OrigPos, OldPos, tsafe);
							OtherMesh.SetVertex(IDs.b, NewPos);
						}
						MeshesVertices.UpdatePoint(IDs, OldPos, NewPos);
					}
				}
			}
		}

		// clear "original position" attribute now that we have removed self-intersections
		// and set unset vertex colors
		for (size_t CellIdx = 0; CellIdx < mMeshs.size(); CellIdx++)
		{
			DynamicMesh& Mesh = *mMeshs[CellIdx];
			Mesh.Attributes()->RemoveAttribute(OriginalPositionAttribute);
			for (int VID : Mesh.VertexIndicesItr())
			{
				AugmentedDynamicMesh::SetVertexColor(Mesh, VID, AugmentedDynamicMesh::UnsetVertexColor);
			}
		}

		// recompute UVs using new positions after noise was applied + fixed
		std::vector<Vector2> PlaneMinUVs;
		PlaneMinUVs.resize(v.mPlanes.size(), Vector2(FLT_MAX, FLT_MAX));
		std::vector<Maths::Frame3> PlaneFrames;
		PlaneFrames.reserve(v.mPlanes.size());
		for (int PlaneIdx = 0; PlaneIdx < v.mPlanes.size(); PlaneIdx++)
		{
			PlaneFrames.push_back(AxisAlignedFrame(v.mPlanes[PlaneIdx]));
		}
		// first pass to compute min UV for each plane
		for (size_t CellIdx = 0; CellIdx < mMeshs.size(); CellIdx++)
		{
			DynamicMesh& Mesh = *mMeshs[CellIdx];
			FDynamicMeshMaterialAttribute* MaterialIDs = Mesh.Attributes()->GetMaterialID();

			for (int TID : Mesh.TriangleIndicesItr())
			{
				int PlaneIdx = EncodePlaneToMaterial(MaterialIDs->GetValue(TID));
				if (PlaneIdx > -1)
				{
					Index3 Tri = Mesh.GetTriangle(TID);
					for (int Idx = 0; Idx < 3; Idx++)
					{
						Vector2 UV = PlaneFrames[PlaneIdx].ProjectXZ(Mesh.GetVertex(Tri[Idx]));
						Vector2& MinUV = PlaneMinUVs[PlaneIdx];
						MinUV.x = std::min(UV.x, MinUV.x);
						MinUV.y = std::min(UV.y, MinUV.y);
					}
				}
			}
		}
		// second pass to actually set UVs
		for (size_t CellIdx = 0; CellIdx < mMeshs.size(); CellIdx++)
		{
			DynamicMesh& Mesh = *mMeshs[CellIdx];
			FDynamicMeshMaterialAttribute* MaterialIDs = Mesh.Attributes()->GetMaterialID();

			for (int TID : Mesh.TriangleIndicesItr())
			{
				int PlaneIdx = EncodePlaneToMaterial(MaterialIDs->GetValue(TID));
				if (PlaneIdx > -1)
				{
					Index3 Tri = Mesh.GetTriangle(TID);
					for (int Idx = 0; Idx < 3; Idx++)
					{
						Vector2 UV = PlaneFrames[PlaneIdx].ProjectXZ(Mesh.GetVertex(Tri[Idx])) - PlaneMinUVs[PlaneIdx];
						for (int UVLayerIdx = 0; UVLayerIdx < NumUVLayers; UVLayerIdx++)
						{
							AugmentedDynamicMesh::SetUV(Mesh, Tri[Idx], UV, UVLayerIdx);
						}
					}
				}
			}
		}
		*/
	}
}	// namespace Riemann
