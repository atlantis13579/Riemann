
#include "Fracture.h"

#include <algorithm>

#include "../Geometry/GeometryBoolean.h"
#include "PlanarCut.h"

namespace Riemann
{
	namespace
	{
		static int CountValidTriangles(const DynamicMesh& Mesh)
		{
			int Count = 0;
			for (int TID = 0; TID < Mesh.GetTriangleCount(); ++TID)
			{
				if (Mesh.IsTriangleFast(TID))
				{
					++Count;
				}
			}
			return Count;
		}

		static float BoundsMaxDim(const Box3& Bounds)
		{
			const Vector3 Size = Bounds.Max - Bounds.Min;
			return std::max(Size.x, std::max(Size.y, Size.z));
		}

		static bool AppendOutwardTriangle(
			DynamicMesh& Mesh,
			const std::vector<Vector3>& Vertices,
			int A,
			int B,
			int C,
			const Vector3& Site,
			const Vector3& FaceCenter)
		{
			if (A == B || A == C || B == C)
			{
				return false;
			}
			if (A < 0 || B < 0 || C < 0 ||
				A >= (int)Vertices.size() || B >= (int)Vertices.size() || C >= (int)Vertices.size())
			{
				return false;
			}

			Vector3 Normal = (Vertices[C] - Vertices[A]).Cross(Vertices[B] - Vertices[A]);
			if (Normal.SafeNormalize() == 0.0f)
			{
				return false;
			}

			Vector3 Outward = FaceCenter - Site;
			if (Outward.SafeNormalize() > 0.0f && Normal.Dot(Outward) < 0.0f)
			{
				std::swap(B, C);
			}

			return Mesh.AppendTriangle(A, B, C) >= 0;
		}
	}

	bool Fracture::BuildVoronoiCellMesh(
		const Voronoi3::Cell& Cell,
		const Vector3& Site,
		DynamicMesh& CellMesh)
	{
		CellMesh.Clear();
		if (Cell.Vertices.empty() || Cell.Faces.empty())
		{
			return false;
		}

		for (const Vector3& Vertex : Cell.Vertices)
		{
			CellMesh.AppendVertex(Vertex);
		}

		for (int FaceOffset = 0; FaceOffset < (int)Cell.Faces.size();)
		{
			const int FaceVertexCount = Cell.Faces[FaceOffset];
			if (FaceVertexCount < 3 || FaceOffset + FaceVertexCount >= (int)Cell.Faces.size())
			{
				break;
			}

			Vector3 FaceCenter = Vector3::Zero();
			for (int FaceVertexIdx = 0; FaceVertexIdx < FaceVertexCount; ++FaceVertexIdx)
			{
				const int VertexID = Cell.Faces[FaceOffset + 1 + FaceVertexIdx];
				if (VertexID >= 0 && VertexID < (int)Cell.Vertices.size())
				{
					FaceCenter += Cell.Vertices[VertexID];
				}
			}
			FaceCenter /= (float)FaceVertexCount;

			const int FirstVertexID = Cell.Faces[FaceOffset + 1];
			for (int FaceVertexIdx = 2; FaceVertexIdx < FaceVertexCount; ++FaceVertexIdx)
			{
				const int PrevVertexID = Cell.Faces[FaceOffset + FaceVertexIdx];
				const int NextVertexID = Cell.Faces[FaceOffset + FaceVertexIdx + 1];
				AppendOutwardTriangle(CellMesh, Cell.Vertices, FirstVertexID, PrevVertexID, NextVertexID, Site, FaceCenter);
			}

			FaceOffset += FaceVertexCount + 1;
		}

		if (CountValidTriangles(CellMesh) == 0)
		{
			CellMesh.Clear();
			return false;
		}

		CellMesh.BuildBounds();
		CellMesh.CalculateWeightAverageNormals();
		return true;
	}

	bool Fracture::VoronoiFracture(
		const DynamicMesh& SourceMesh,
		const std::vector<Vector3>& Sites,
		std::vector<FracturePiece>& Pieces,
		const FractureOptions& Options)
	{
		Pieces.clear();
		if (Sites.empty() || CountValidTriangles(SourceMesh) == 0)
		{
			return false;
		}

		Box3 VoronoiBounds = SourceMesh.GetBounds();
		float MaxDim = BoundsMaxDim(VoronoiBounds);
		if (MaxDim <= 0.0f)
		{
			MaxDim = 1.0f;
		}
		VoronoiBounds.Thicken(std::max(MaxDim * Options.BoundsPaddingScale, Options.SnapTolerance * 16.0f));
		VoronoiBounds = Voronoi3::GetVoronoiBounds(VoronoiBounds, Sites);

		PlanarCells Cells(Sites, VoronoiBounds, Options.SnapTolerance);
		PlanarCutOptions CutOptions;
		CutOptions.SnapTolerance = Options.SnapTolerance;
		CutOptions.BoundsPaddingScale = Options.BoundsPaddingScale;
		CutOptions.Grout = Options.Grout;
		CutOptions.WeldSharedEdges = Options.WeldSharedEdges;
		CutOptions.SimplifyAlongCut = Options.SimplifyAlongCut;
		CutOptions.MinTriangleCount = Options.MinTriangleCount;

		std::vector<PlanarCutPiece> CutPieces;
		if (!PlanarCut::CutWithPlanarCells(SourceMesh, Cells, CutPieces, CutOptions))
		{
			return false;
		}

		for (const PlanarCutPiece& CutPiece : CutPieces)
		{
			if (CutPiece.CellIndex >= 0 && CutPiece.CellIndex < (int)Sites.size())
			{
				FracturePiece Piece;
				Piece.Mesh = CutPiece.Mesh;
				Piece.Site = Sites[CutPiece.CellIndex];
				Piece.SiteIndex = CutPiece.CellIndex;
				Piece.Center = CutPiece.Center;
				Pieces.push_back(Piece);
			}
		}

		return !Pieces.empty();
	}

	bool Fracture::VoronoiFracture(
		const DynamicMesh& SourceMesh,
		const std::vector<Vector3>& Sites,
		std::vector<DynamicMesh>& Pieces,
		const FractureOptions& Options)
	{
		Pieces.clear();

		std::vector<FracturePiece> FracturePieces;
		const bool bSuccess = VoronoiFracture(SourceMesh, Sites, FracturePieces, Options);
		for (const FracturePiece& Piece : FracturePieces)
		{
			Pieces.push_back(Piece.Mesh);
		}
		return bSuccess;
	}

}	// namespace Riemann
