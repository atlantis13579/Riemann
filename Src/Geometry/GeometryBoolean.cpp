
#include <vector>
#include <map>
#include "../Maths/Maths.h"
#include "GeometryBoolean.h"
#include "GeometrySet.h"

namespace Geometry
{

	void GeometryBoolean::Compute()
	{
		// copy meshes
		GeometryData CutMeshB(*Meshes[1]);

		MeshNew = new GeometryData;
		*MeshNew = *Meshes[0];

		GeometryData* CutMesh[2]{ MeshNew, &CutMeshB };

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
		GeometryAABBTree Spatial[2]{ CutMesh[0], CutMesh[1] };
		GeometryAABBTree::IntersectionsQueryResult Intersections = Spatial[0].FindAllIntersections(Spatial[1], nullptr);

		bool bOpOnSingleMesh = Operation == BooleanOp::TrimInside || Operation == BooleanOp::TrimOutside || Operation == BooleanOp::NewGroupInside || Operation == BooleanOp::NewGroupOutside;

		/*
		// cut the meshes
		FMeshMeshCut Cut(CutMesh[0], CutMesh[1]);
		Cut.bTrackInsertedVertices = bCollapseDegenerateEdgesOnCut; // to collect candidates to collapse
		Cut.bMutuallyCut = !bOpOnSingleMesh;
		Cut.SnapTolerance = SnapTolerance;
		Cut.Cut(Intersections);

		int NumMeshesToProcess = bOpOnSingleMesh ? 1 : 2;

		// collapse tiny edges along cut boundary
		if (bCollapseDegenerateEdgesOnCut)
		{
			double DegenerateEdgeTolSq = DegenerateEdgeTolFactor * DegenerateEdgeTolFactor * SnapTolerance * SnapTolerance;
			for (int MeshIdx = 0; MeshIdx < NumMeshesToProcess; MeshIdx++)
			{
				// convert vertex chains to edge IDs to simplify logic of finding remaining candidate edges after collapses
				std::vector<int> EIDs;
				for (int ChainIdx = 0; ChainIdx < Cut.VertexChains[MeshIdx].Num();)
				{
					int ChainLen = Cut.VertexChains[MeshIdx][ChainIdx];
					int ChainEnd = ChainIdx + 1 + ChainLen;
					for (int ChainSubIdx = ChainIdx + 1; ChainSubIdx + 1 < ChainEnd; ChainSubIdx++)
					{
						int VID[2]{ Cut.VertexChains[MeshIdx][ChainSubIdx], Cut.VertexChains[MeshIdx][ChainSubIdx + 1] };
						if (DistanceSquared(CutMesh[MeshIdx]->GetVertex(VID[0]), CutMesh[MeshIdx]->GetVertex(VID[1])) < DegenerateEdgeTolSq)
						{
							EIDs.Add(CutMesh[MeshIdx]->FindEdge(VID[0], VID[1]));
						}
					}
					ChainIdx = ChainEnd;
				}
				TSet<int> AllEIDs(EIDs);
				for (int Idx = 0; Idx < EIDs.Num(); Idx++)
				{
					int EID = EIDs[Idx];
					if (!CutMesh[MeshIdx]->IsEdge(EID))
					{
						continue;
					}
					Vector3 A, B;
					CutMesh[MeshIdx]->GetEdgeV(EID, A, B);
					if (DistanceSquared(A, B) > DegenerateEdgeTolSq)
					{
						continue;
					}
					FIndex2i EV = CutMesh[MeshIdx]->GetEdgeV(EID);

					// if the vertex we'd remove is on a seam, try removing the other one instead
					if (CutMesh[MeshIdx]->HasAttributes() && CutMesh[MeshIdx]->Attributes()->IsSeamVertex(EV.B, false))
					{
						Swap(EV.A, EV.B);
						// if they were both on seams, then collapse should not happen?  (& would break OnCollapseEdge assumptions in overlay)
						if (CutMesh[MeshIdx]->HasAttributes() && CutMesh[MeshIdx]->Attributes()->IsSeamVertex(EV.B, false))
						{
							continue;
						}
					}
					GeometryData::FEdgeCollapseInfo CollapseInfo;
					EMeshResult CollapseResult = CutMesh[MeshIdx]->CollapseEdge(EV.A, EV.B, .5, CollapseInfo);
					if (CollapseResult == EMeshResult::Ok)
					{
						for (int i = 0; i < 2; i++)
						{
							if (AllEIDs.Contains(CollapseInfo.RemovedEdges[i]))
							{
								int ToAdd = CollapseInfo.KeptEdges[i];
								bool bWasPresent;
								AllEIDs.Add(ToAdd, &bWasPresent);
								if (!bWasPresent)
								{
									EIDs.Add(ToAdd);
								}
							}
						}
					}
				}
			}
		}

		if (Cancelled())
		{
			return false;
		}

		// edges that will become new boundary edges after the boolean op removes triangles on each mesh
		std::vector<int> CutBoundaryEdges[2];
		// Vertices on the cut boundary that *may* not have a corresonding vertex on the other mesh
		TSet<int> PossUnmatchedBdryVerts[2];

		// delete geometry according to boolean rules, tracking the boundary edges
		{ // (just for scope)
			// first decide what triangles to delete for both meshes (*before* deleting anything so winding doesn't get messed up!)
			TArray<bool> KeepTri[2];
			// This array is used to double-check the assumption that we will delete the other surface when we keep a coplanar tri
			// Note we only need it for mesh 0 (i.e., the mesh we try to keep triangles from when we preserve coplanar surfaces)
			TArray<int> DeleteIfOtherKept;
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
				GeometryData& ProcessMesh = *CutMesh[MeshIdx];
				for (int EID : ProcessMesh.EdgeIndicesItr())
				{
					GeometryData::FEdge Edge = ProcessMesh.GetEdge(EID);
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
				ensure(NumMeshesToProcess == 1);
				NewGroupID = CutMesh[0]->AllocateTriangleGroup();
			}
			for (int MeshIdx = 0; MeshIdx < NumMeshesToProcess; MeshIdx++)
			{
				GeometryData& ProcessMesh = *CutMesh[MeshIdx];

				for (int TID = 0; TID < KeepTri[MeshIdx].Num(); TID++)
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
				for (int ComponentIdx = 1; ComponentIdx < Components.Num(); ComponentIdx++)
				{
					int SplitGroupID = CutMesh[0]->AllocateTriangleGroup();
					for (int TID : Components.GetComponent(ComponentIdx).Indices)
					{
						CutMesh[0]->SetTriangleGroup(TID, SplitGroupID);
					}
				}
			}
		}

		if (Cancelled())
		{
			return false;
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
				GeometryData& OtherMesh = *CutMesh[OtherMeshIdx];

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
					GeometryData::FEdge Edge = OtherMesh.GetEdge(EID);
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
	}

}