#pragma once

#include <vector>
#include "../Maths/Index2.h"
#include "../Maths/Index3.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	enum class EMeshResult
	{
		Ok = 0,
		Failed_NotAVertex = 1,
		Failed_NotATriangle = 2,
		Failed_NotAnEdge = 3,

		Failed_BrokenTopology = 10,
		Failed_HitValenceLimit = 11,

		Failed_IsBoundaryEdge = 20,
		Failed_FlippedEdgeExists = 21,
		Failed_IsBowtieVertex = 22,
		Failed_InvalidNeighbourhood = 23,       // these are all failures for CollapseEdge
		Failed_FoundDuplicateTriangle = 24,
		Failed_CollapseTetrahedron = 25,
		Failed_CollapseTriangle = 26,
		Failed_NotABoundaryEdge = 27,
		Failed_SameOrientation = 28,

		Failed_WouldCreateBowtie = 30,
		Failed_VertexAlreadyExists = 31,
		Failed_CannotAllocateVertex = 32,
		Failed_VertexStillReferenced = 33,

		Failed_WouldCreateNonmanifoldEdge = 50,
		Failed_TriangleAlreadyExists = 51,
		Failed_CannotAllocateTriangle = 52,

		Failed_UnrecoverableError = 1000,
		Failed_Unsupported = 1001
	};

	struct FEdgeSplitInfo
	{
		int OriginalEdge;
		Index2 OriginalVertices;
		Index2 OtherVertices;
		Index2 OriginalTriangles;
		bool bIsBoundary;
		int NewVertex;
		Index2 NewTriangles;
		Index3 NewEdges;
		float SplitT;
	};

	struct FEdgeFlipInfo
	{
		int EdgeID;
		Index2 OriginalVerts;
		Index2 OpposingVerts;
		Index2 Triangles;
	};

	struct FMergeEdgesInfo
	{
		int KeptEdge;
		int RemovedEdge;
		Index2 KeptVerts;
		Index2 RemovedVerts;
		Index2 ExtraRemovedEdges;
		Index2 ExtraKeptEdges;
		std::vector<int> BowtiesRemovedEdges, BowtiesKeptEdges;
	};

	struct FPokeTriangleInfo
	{
		int OriginalTriangle;
		Index3 TriVertices;
		int NewVertex;
		Index2 NewTriangles;
		Index3 NewEdges;
		Vector3 BaryCoords;
	};

	struct FEdgeCollapseInfo
	{
		int KeptVertex;
		int RemovedVertex;
		Index2 OpposingVerts;
		bool bIsBoundary;
		int CollapsedEdge;
		Index2 RemovedTris;
		Index2 RemovedEdges;
		Index2 KeptEdges;
		float CollapseT;
	};

	struct FVertexSplitInfo
	{
		int OriginalVertex;
		int NewVertex;
	};


}	// namespace Riemann
