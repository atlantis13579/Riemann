
#include "Fracture.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <random>

#include "../Geometry/GeometryBoolean.h"
#include "../Geometry/MeshCut.h"

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

		static int ClampInt(int Value, int MinValue, int MaxValue)
		{
			return std::max(MinValue, std::min(Value, MaxValue));
		}

		static float ClampFloat(float Value, float MinValue, float MaxValue)
		{
			return std::max(MinValue, std::min(Value, MaxValue));
		}

		static float AxisValue(const Vector3& Value, int Axis)
		{
			return Axis == 0 ? Value.x : (Axis == 1 ? Value.y : Value.z);
		}

		static void SetAxisValue(Vector3& Value, int Axis, float AxisValueIn)
		{
			if (Axis == 0)
			{
				Value.x = AxisValueIn;
			}
			else if (Axis == 1)
			{
				Value.y = AxisValueIn;
			}
			else
			{
				Value.z = AxisValueIn;
			}
		}

		static Vector3 AxisVector(int Axis)
		{
			return Axis == 0 ? Vector3::UnitX() : (Axis == 1 ? Vector3::UnitY() : Vector3::UnitZ());
		}

		static int FindSmallestBoundsAxis(const Box3& Bounds)
		{
			const Vector3 Size = Bounds.Max - Bounds.Min;
			if (Size.x <= Size.y && Size.x <= Size.z)
			{
				return 0;
			}
			if (Size.y <= Size.z)
			{
				return 1;
			}
			return 2;
		}

		static int FindDominantAxis(const Vector3& Direction, float MinAbsDot)
		{
			const Vector3 Unit = Direction.SafeUnit();
			const float AbsX = std::fabs(Unit.x);
			const float AbsY = std::fabs(Unit.y);
			const float AbsZ = std::fabs(Unit.z);
			if (AbsX >= AbsY && AbsX >= AbsZ && AbsX >= MinAbsDot)
			{
				return 0;
			}
			if (AbsY >= AbsZ && AbsY >= MinAbsDot)
			{
				return 1;
			}
			if (AbsZ >= MinAbsDot)
			{
				return 2;
			}
			return -1;
		}

		static Vector3 ResolveCutNormal(const DynamicMesh& SourceMesh, const Vector3& RequestedNormal)
		{
			Vector3 Normal = RequestedNormal.SafeUnit();
			if (!Normal.IsZero())
			{
				return Normal;
			}
			return AxisVector(FindSmallestBoundsAxis(SourceMesh.GetBounds()));
		}

		static bool BuildPlaneBasis(const Vector3& PlaneNormal, Vector3& Tangent, Vector3& Normal, Vector3& Binormal)
		{
			Normal = PlaneNormal.SafeUnit();
			if (Normal.IsZero())
			{
				return false;
			}

			Tangent = Normal.Cross(Vector3::UnitX());
			if (Tangent.SquareLength() < 1e-8f)
			{
				Tangent = Normal.Cross(Vector3::UnitY());
			}
			if (Tangent.SafeNormalize() == 0.0f)
			{
				return false;
			}

			Binormal = Tangent.Cross(Normal);
			return Binormal.SafeNormalize() > 0.0f;
		}

		static PlanarCutOptions MakePlanarOptions(
			const DynamicMesh& SourceMesh,
			const PlanarCutOptions& Options,
			float DefaultBoundsPaddingScale)
		{
			const float MaxSize = std::max(BoundsMaxDim(SourceMesh.GetBounds()), 1e-3f);

			PlanarCutOptions CutOptions = Options;
			CutOptions.SnapTolerance = Options.SnapTolerance > 0.0f ? Options.SnapTolerance : std::max(MaxSize * 1e-5f, 1e-6f);
			CutOptions.BoundsPaddingScale = Options.BoundsPaddingScale > 0.0f ? Options.BoundsPaddingScale : DefaultBoundsPaddingScale;
			CutOptions.Grout = std::max(0.0f, Options.Grout);
			CutOptions.MinTriangleCount = std::max(1, Options.MinTriangleCount);
			return CutOptions;
		}

		static Vector3 FallbackPieceDirection(size_t PieceIndex)
		{
			const float Angle = static_cast<float>(PieceIndex) * 2.39996323f;
			return Vector3(cosf(Angle), 0.25f * ((PieceIndex & 1) ? 1.0f : -1.0f), sinf(Angle)).SafeUnit();
		}

		static void ComputePlaneRanges(
			const Box3& Bounds,
			const Vector3& Origin,
			const Vector3& Tangent,
			const Vector3& Normal,
			const Vector3& Binormal,
			float& MinU,
			float& MaxU,
			float& MinV,
			float& MaxV,
			float& MinN,
			float& MaxN)
		{
			Vector3 Corners[8];
			Box3::GetVertices(Bounds.Min, Bounds.Max, Corners);

			MinU = MinV = MinN = FLT_MAX;
			MaxU = MaxV = MaxN = -FLT_MAX;
			for (int CornerIdx = 0; CornerIdx < 8; ++CornerIdx)
			{
				const Vector3 Rel = Corners[CornerIdx] - Origin;
				const float U = Rel.Dot(Tangent);
				const float V = Rel.Dot(Binormal);
				const float N = Rel.Dot(Normal);
				MinU = std::min(MinU, U);
				MaxU = std::max(MaxU, U);
				MinV = std::min(MinV, V);
				MaxV = std::max(MaxV, V);
				MinN = std::min(MinN, N);
				MaxN = std::max(MaxN, N);
			}
		}

		static void CopyCutPieceConnectivity(
			const std::vector<PlanarCutPiece>& CutPieces,
			const std::vector<int>& CutPieceToPiece,
			std::vector<FracturePiece>& Pieces)
		{
			for (size_t CutPieceIdx = 0; CutPieceIdx < CutPieces.size(); ++CutPieceIdx)
			{
				if (CutPieceIdx >= CutPieceToPiece.size())
				{
					continue;
				}

				const int PieceIdx = CutPieceToPiece[CutPieceIdx];
				if (PieceIdx < 0 || PieceIdx >= (int)Pieces.size())
				{
					continue;
				}

				std::vector<int>& Neighbors = Pieces[(size_t)PieceIdx].NeighborPieceIndices;
				Neighbors.clear();
				for (int NeighborCutPieceIdx : CutPieces[CutPieceIdx].NeighborPieceIndices)
				{
					if (NeighborCutPieceIdx >= 0 && NeighborCutPieceIdx < (int)CutPieceToPiece.size())
					{
						const int NeighborPieceIdx = CutPieceToPiece[(size_t)NeighborCutPieceIdx];
						if (NeighborPieceIdx >= 0 && NeighborPieceIdx != PieceIdx &&
							std::find(Neighbors.begin(), Neighbors.end(), NeighborPieceIdx) == Neighbors.end())
						{
							Neighbors.push_back(NeighborPieceIdx);
						}
					}
				}
			}
		}

		static void AppendCutPiecesRadial(
			const std::vector<PlanarCutPiece>& CutPieces,
			const Vector3& SourceCenter,
			std::vector<FracturePiece>& Pieces)
		{
			std::vector<int> CutPieceToPiece(CutPieces.size(), -1);
			for (size_t CutPieceIdx = 0; CutPieceIdx < CutPieces.size(); ++CutPieceIdx)
			{
				const PlanarCutPiece& CutPiece = CutPieces[CutPieceIdx];
				if (CountValidTriangles(CutPiece.Mesh) == 0)
				{
					continue;
				}

				FracturePiece Piece;
				Piece.Mesh = CutPiece.Mesh;
				Piece.Center = CutPiece.Center;
				Piece.CellIndex = CutPiece.CellIndex;
				Piece.SiteIndex = CutPiece.CellIndex;
				Piece.Direction = Piece.Center - SourceCenter;
				if (Piece.Direction.SafeNormalize() == 0.0f)
				{
					Piece.Direction = FallbackPieceDirection(Pieces.size());
				}
				CutPieceToPiece[CutPieceIdx] = (int)Pieces.size();
				Pieces.push_back(Piece);
			}

			CopyCutPieceConnectivity(CutPieces, CutPieceToPiece, Pieces);
		}

		static void ApplyPlanarAxisSeparationDirections(
			const Box3& SourceBounds,
			const Vector3& Normal,
			std::vector<FracturePiece>& Pieces)
		{
			Vector3 Tangent, PlaneNormal, Binormal;
			if (!BuildPlaneBasis(Normal, Tangent, PlaneNormal, Binormal))
			{
				for (size_t PieceIdx = 0; PieceIdx < Pieces.size(); ++PieceIdx)
				{
					Pieces[PieceIdx].Direction = FallbackPieceDirection(PieceIdx);
				}
				return;
			}

			const Vector3 SourceCenter = SourceBounds.GetCenter();
			float MinU, MaxU, MinV, MaxV, MinN, MaxN;
			ComputePlaneRanges(SourceBounds, SourceCenter, Tangent, PlaneNormal, Binormal, MinU, MaxU, MinV, MaxV, MinN, MaxN);
			const float HalfU = std::max((MaxU - MinU) * 0.5f, 1e-3f);
			const float HalfV = std::max((MaxV - MinV) * 0.5f, 1e-3f);

			for (FracturePiece& Piece : Pieces)
			{
				const Vector3 Delta = Piece.Center - SourceCenter;
				const float U = ClampFloat(Delta.Dot(Tangent) / HalfU, -1.0f, 1.0f);
				const float V = ClampFloat(Delta.Dot(Binormal) / HalfV, -1.0f, 1.0f);
				Piece.Direction = Tangent * U + Binormal * V;
			}
		}

		static void AppendCutPiecesPlanar(
			const std::vector<PlanarCutPiece>& CutPieces,
			const Box3& SourceBounds,
			const Vector3& Normal,
			std::vector<FracturePiece>& Pieces)
		{
			std::vector<int> CutPieceToPiece(CutPieces.size(), -1);
			for (size_t CutPieceIdx = 0; CutPieceIdx < CutPieces.size(); ++CutPieceIdx)
			{
				const PlanarCutPiece& CutPiece = CutPieces[CutPieceIdx];
				if (CountValidTriangles(CutPiece.Mesh) == 0)
				{
					continue;
				}

				FracturePiece Piece;
				Piece.Mesh = CutPiece.Mesh;
				Piece.Center = CutPiece.Center;
				Piece.CellIndex = CutPiece.CellIndex;
				Piece.SiteIndex = CutPiece.CellIndex;
				CutPieceToPiece[CutPieceIdx] = (int)Pieces.size();
				Pieces.push_back(Piece);
			}

			CopyCutPieceConnectivity(CutPieces, CutPieceToPiece, Pieces);
			ApplyPlanarAxisSeparationDirections(SourceBounds, Normal, Pieces);
		}

		static void GenerateUniformVoronoiSites3D(
			const DynamicMesh& SourceMesh,
			int PieceCount,
			int Seed,
			std::vector<Vector3>& Sites)
		{
			Sites.clear();
			const int Count = ClampInt(PieceCount, 2, 512);
			const Box3 Bounds = SourceMesh.GetBounds();
			const Vector3 Center = Bounds.GetCenter();
			const Vector3 Extent = Bounds.GetExtent();

			std::mt19937 Rng(static_cast<unsigned int>(std::max(0, Seed)));
			std::uniform_real_distribution<float> Random01(0.0f, 1.0f);

			Sites.reserve(Count);
			for (int SiteIdx = 0; SiteIdx < Count; ++SiteIdx)
			{
				const float X = Random01(Rng) * 2.0f - 1.0f;
				const float Y = Random01(Rng) * 2.0f - 1.0f;
				const float Z = Random01(Rng) * 2.0f - 1.0f;
				Sites.push_back(Center + Vector3(X * Extent.x, Y * Extent.y, Z * Extent.z) * 0.78f);
			}
		}

		static bool GenerateUniformVoronoiSites2D(
			const DynamicMesh& SourceMesh,
			const Vector3& NormalIn,
			int PieceCount,
			int Seed,
			std::vector<Vector3>& Sites)
		{
			Sites.clear();
			Vector3 Tangent, Normal, Binormal;
			if (!BuildPlaneBasis(NormalIn, Tangent, Normal, Binormal))
			{
				return false;
			}

			const int Count = ClampInt(PieceCount, 2, 512);
			const Box3 Bounds = SourceMesh.GetBounds();
			const Vector3 Origin = Bounds.GetCenter();
			float MinU, MaxU, MinV, MaxV, MinN, MaxN;
			ComputePlaneRanges(Bounds, Origin, Tangent, Normal, Binormal, MinU, MaxU, MinV, MaxV, MinN, MaxN);

			std::mt19937 Rng(static_cast<unsigned int>(std::max(0, Seed)));
			std::uniform_real_distribution<float> Random01(0.0f, 1.0f);

			Sites.reserve(Count);
			for (int SiteIdx = 0; SiteIdx < Count; ++SiteIdx)
			{
				const float U = MinU + (MaxU - MinU) * Random01(Rng);
				const float V = MinV + (MaxV - MinV) * Random01(Rng);
				Sites.push_back(Origin + Tangent * U + Binormal * V);
			}
			return true;
		}

		static void GenerateClusterVoronoiSites(
			const DynamicMesh& SourceMesh,
			int PieceCount,
			int Seed,
			std::vector<Vector3>& Sites)
		{
			Sites.clear();
			const int TotalSites = ClampInt(PieceCount, 2, 512);
			const int ClusterCount = ClampInt((int)(std::sqrt((float)TotalSites) + 0.5f), 1, TotalSites);

			const Box3 Bounds = SourceMesh.GetBounds();
			const Vector3 Extent = Bounds.GetExtent();
			const float MaxExtent = std::max(Extent.x, std::max(Extent.y, Extent.z));
			const float MinRadius = MaxExtent * 0.10f;
			const float MaxRadius = MaxExtent * 0.22f;

			std::mt19937 Rng(static_cast<unsigned int>(std::max(0, Seed)));
			std::uniform_real_distribution<float> Random01(0.0f, 1.0f);

			std::vector<Vector3> Centers;
			Centers.reserve(ClusterCount);
			for (int ClusterIdx = 0; ClusterIdx < ClusterCount; ++ClusterIdx)
			{
				Centers.push_back(Bounds.Min + Vector3(Random01(Rng), Random01(Rng), Random01(Rng)) * (Bounds.Max - Bounds.Min));
			}

			Sites.reserve(TotalSites);
			for (int ClusterIdx = 0; ClusterIdx < ClusterCount && (int)Sites.size() < TotalSites; ++ClusterIdx)
			{
				const int RemainingClusters = ClusterCount - ClusterIdx;
				const int RemainingSites = TotalSites - (int)Sites.size();
				const int SitesInCluster = std::max(1, (RemainingSites + RemainingClusters - 1) / RemainingClusters);
				for (int SiteIdx = 0; SiteIdx < SitesInCluster && (int)Sites.size() < TotalSites; ++SiteIdx)
				{
					Vector3 Direction(Random01(Rng) * 2.0f - 1.0f, Random01(Rng) * 2.0f - 1.0f, Random01(Rng) * 2.0f - 1.0f);
					if (Direction.SafeNormalize() == 0.0f)
					{
						Direction = Vector3::UnitX();
					}
					const float Radius = MinRadius + (MaxRadius - MinRadius) * Random01(Rng);
					Sites.push_back(Centers[ClusterIdx] + Direction * Radius);
				}
			}
		}

		static void ComputeGridCounts2D(float SizeU, float SizeV, int TargetCount, int& CountU, int& CountV)
		{
			TargetCount = ClampInt(TargetCount, 2, 512);
			CountU = 1;
			CountV = 1;
			SizeU = std::max(SizeU, 1e-3f);
			SizeV = std::max(SizeV, 1e-3f);
			while (CountU * CountV < TargetCount)
			{
				const float CellU = SizeU / (float)CountU;
				const float CellV = SizeV / (float)CountV;
				if (CellU >= CellV)
				{
					++CountU;
				}
				else
				{
					++CountV;
				}
			}
		}

		static void ResolveGridCounts2D(
			int RequestedPiecesX,
			int RequestedPiecesY,
			int TargetPieceCount,
			float SizeU,
			float SizeV,
			int& CountU,
			int& CountV)
		{
			if (RequestedPiecesX > 0 || RequestedPiecesY > 0)
			{
				CountU = ClampInt(RequestedPiecesX > 0 ? RequestedPiecesX : 1, 1, 64);
				CountV = ClampInt(RequestedPiecesY > 0 ? RequestedPiecesY : 1, 1, 64);
				while (CountU * CountV > 512)
				{
					if (CountU >= CountV && CountU > 1)
					{
						--CountU;
					}
					else if (CountV > 1)
					{
						--CountV;
					}
					else
					{
						break;
					}
				}
				return;
			}

			ComputeGridCounts2D(SizeU, SizeV, TargetPieceCount, CountU, CountV);
		}

		static void ComputeGridCounts3D(const Box3& Bounds, int TargetCount, int& PiecesX, int& PiecesY, int& PiecesZ)
		{
			TargetCount = ClampInt(TargetCount, 2, 512);
			const Vector3 Size = Bounds.Max - Bounds.Min;
			PiecesX = PiecesY = PiecesZ = 1;
			while (PiecesX * PiecesY * PiecesZ < TargetCount)
			{
				const float CellX = std::max(Size.x, 1e-3f) / (float)PiecesX;
				const float CellY = std::max(Size.y, 1e-3f) / (float)PiecesY;
				const float CellZ = std::max(Size.z, 1e-3f) / (float)PiecesZ;
				if (CellX >= CellY && CellX >= CellZ)
				{
					++PiecesX;
				}
				else if (CellY >= CellZ)
				{
					++PiecesY;
				}
				else
				{
					++PiecesZ;
				}
			}
		}

		static void ResolveGridCounts3D(
			const Box3& Bounds,
			int RequestedPiecesX,
			int RequestedPiecesY,
			int RequestedPiecesZ,
			int TargetPieceCount,
			int& PiecesX,
			int& PiecesY,
			int& PiecesZ)
		{
			if (RequestedPiecesX > 0 || RequestedPiecesY > 0 || RequestedPiecesZ > 0)
			{
				PiecesX = ClampInt(RequestedPiecesX > 0 ? RequestedPiecesX : 1, 1, 32);
				PiecesY = ClampInt(RequestedPiecesY > 0 ? RequestedPiecesY : 1, 1, 32);
				PiecesZ = ClampInt(RequestedPiecesZ > 0 ? RequestedPiecesZ : 1, 1, 32);
				while (PiecesX * PiecesY * PiecesZ > 512)
				{
					if (PiecesX >= PiecesY && PiecesX >= PiecesZ && PiecesX > 1)
					{
						--PiecesX;
					}
					else if (PiecesY >= PiecesZ && PiecesY > 1)
					{
						--PiecesY;
					}
					else if (PiecesZ > 1)
					{
						--PiecesZ;
					}
					else
					{
						break;
					}
				}
				return;
			}

			ComputeGridCounts3D(Bounds, TargetPieceCount, PiecesX, PiecesY, PiecesZ);
		}

		static void AppendOrientedBoxCell(
			PlanarCells& Cells,
			int CellIdx,
			int UIdx,
			int VIdx,
			int CountU,
			int CountV,
			const Vector3& Center,
			const Vector3& AxisU,
			const Vector3& AxisV,
			const Vector3& AxisN,
			float HalfU,
			float HalfV,
			float HalfN)
		{
			Vector3 BoxAxisV = AxisV;
			if (AxisU.Cross(BoxAxisV).Dot(AxisN) < 0.0f)
			{
				BoxAxisV = -BoxAxisV;
			}

			const int VertexStart = (int)Cells.PlaneBoundaryVertices.size();
			Cells.PlaneBoundaryVertices.push_back(Center - AxisU * HalfU - BoxAxisV * HalfV - AxisN * HalfN);
			Cells.PlaneBoundaryVertices.push_back(Center + AxisU * HalfU - BoxAxisV * HalfV - AxisN * HalfN);
			Cells.PlaneBoundaryVertices.push_back(Center + AxisU * HalfU + BoxAxisV * HalfV - AxisN * HalfN);
			Cells.PlaneBoundaryVertices.push_back(Center - AxisU * HalfU + BoxAxisV * HalfV - AxisN * HalfN);
			Cells.PlaneBoundaryVertices.push_back(Center - AxisU * HalfU - BoxAxisV * HalfV + AxisN * HalfN);
			Cells.PlaneBoundaryVertices.push_back(Center + AxisU * HalfU - BoxAxisV * HalfV + AxisN * HalfN);
			Cells.PlaneBoundaryVertices.push_back(Center + AxisU * HalfU + BoxAxisV * HalfV + AxisN * HalfN);
			Cells.PlaneBoundaryVertices.push_back(Center - AxisU * HalfU + BoxAxisV * HalfV + AxisN * HalfN);

			const std::vector<Vector3>& V = Cells.PlaneBoundaryVertices;
			Cells.AddPlane(PlanarCutPlane(-AxisN, V[VertexStart + 0]), CellIdx, -1, { VertexStart + 0, VertexStart + 1, VertexStart + 2, VertexStart + 3 });
			Cells.AddPlane(PlanarCutPlane(AxisN, V[VertexStart + 4]), CellIdx, -1, { VertexStart + 4, VertexStart + 7, VertexStart + 6, VertexStart + 5 });

			if (VIdx == 0)
			{
				Cells.AddPlane(PlanarCutPlane(-BoxAxisV, V[VertexStart + 0]), CellIdx, -1, { VertexStart + 0, VertexStart + 4, VertexStart + 5, VertexStart + 1 });
			}
			Cells.AddPlane(PlanarCutPlane(BoxAxisV, V[VertexStart + 3]), CellIdx, VIdx + 1 < CountV ? CellIdx + CountU : -1, { VertexStart + 3, VertexStart + 2, VertexStart + 6, VertexStart + 7 });

			if (UIdx == 0)
			{
				Cells.AddPlane(PlanarCutPlane(-AxisU, V[VertexStart + 0]), CellIdx, -1, { VertexStart + 0, VertexStart + 3, VertexStart + 7, VertexStart + 4 });
			}
			Cells.AddPlane(PlanarCutPlane(AxisU, V[VertexStart + 1]), CellIdx, UIdx + 1 < CountU ? CellIdx + 1 : -1, { VertexStart + 1, VertexStart + 5, VertexStart + 6, VertexStart + 2 });
		}

		static bool BuildVoronoiPiecesFromSites(
			const DynamicMesh& SourceMesh,
			const std::vector<Vector3>& Sites,
			std::vector<FracturePiece>& Pieces,
			const PlanarCutOptions& Options,
			float DefaultBoundsPaddingScale)
		{
			Pieces.clear();
			if (Sites.empty() || CountValidTriangles(SourceMesh) == 0)
			{
				return false;
			}

			PlanarCutOptions CutOptions = MakePlanarOptions(SourceMesh, Options, DefaultBoundsPaddingScale);
			Box3 VoronoiBounds = SourceMesh.GetBounds();
			const float MaxSize = std::max(BoundsMaxDim(VoronoiBounds), 1e-3f);
			VoronoiBounds.Thicken(std::max(MaxSize * CutOptions.BoundsPaddingScale, CutOptions.SnapTolerance * 16.0f));
			VoronoiBounds = Voronoi3::GetVoronoiBounds(VoronoiBounds, Sites);

			PlanarCells Cells(Sites, VoronoiBounds, CutOptions.SnapTolerance);

			std::vector<PlanarCutPiece> CutPieces;
			if (!MeshCut::CutWithPlanarCells(SourceMesh, Cells, CutPieces, CutOptions))
			{
				return false;
			}

			AppendCutPiecesRadial(CutPieces, SourceMesh.GetBounds().GetCenter(), Pieces);
			for (FracturePiece& Piece : Pieces)
			{
				if (Piece.CellIndex >= 0 && Piece.CellIndex < (int)Sites.size())
				{
					Piece.Site = Sites[Piece.CellIndex];
				}
			}
			return !Pieces.empty();
		}

		static bool BuildParallelCutPieces(
			const DynamicMesh& SourceMesh,
			int Axis,
			std::vector<FracturePiece>& Pieces,
			int PieceCount,
			const PlanarCutOptions& Options)
		{
			Pieces.clear();
			Axis = ClampInt(Axis, 0, 2);

			const Box3 Bounds = SourceMesh.GetBounds();
			const Vector3 Center = Bounds.GetCenter();
			const float MaxSize = std::max(BoundsMaxDim(Bounds), 1e-3f);
			const float MinAxis = AxisValue(Bounds.Min, Axis);
			const float MaxAxis = AxisValue(Bounds.Max, Axis);
			const float AxisSize = std::max(MaxAxis - MinAxis, 1e-3f);
			const int Count = ClampInt(PieceCount, 2, 512);
			const float Padding = std::max(MaxSize * 0.01f, 1e-4f);

			std::vector<Box3> Cells;
			Cells.reserve(Count);
			for (int CellIdx = 0; CellIdx < Count; ++CellIdx)
			{
				Vector3 CellMin = Bounds.Min - Vector3(Padding, Padding, Padding);
				Vector3 CellMax = Bounds.Max + Vector3(Padding, Padding, Padding);
				const float T0 = static_cast<float>(CellIdx) / static_cast<float>(Count);
				const float T1 = static_cast<float>(CellIdx + 1) / static_cast<float>(Count);
				SetAxisValue(CellMin, Axis, MinAxis + AxisSize * T0 - (CellIdx == 0 ? Padding : 0.0f));
				SetAxisValue(CellMax, Axis, MinAxis + AxisSize * T1 + (CellIdx == Count - 1 ? Padding : 0.0f));
				Cells.emplace_back(CellMin, CellMax);
			}

			PlanarCells PlanarCellsIn(Cells, true);
			PlanarCutOptions CutOptions = MakePlanarOptions(SourceMesh, Options, 0.08f);

			std::vector<PlanarCutPiece> CutPieces;
			if (!MeshCut::CutWithPlanarCells(SourceMesh, PlanarCellsIn, CutPieces, CutOptions))
			{
				return false;
			}

			const Vector3 AxisDir = AxisVector(Axis);
			std::vector<int> CutPieceToPiece(CutPieces.size(), -1);
			for (size_t CutPieceIdx = 0; CutPieceIdx < CutPieces.size(); ++CutPieceIdx)
			{
				const PlanarCutPiece& CutPiece = CutPieces[CutPieceIdx];
				if (CountValidTriangles(CutPiece.Mesh) == 0)
				{
					continue;
				}

				const float Relative = (AxisValue(CutPiece.Center, Axis) - AxisValue(Center, Axis)) / std::max(AxisSize * 0.5f, 1e-3f);
				FracturePiece Piece;
				Piece.Mesh = CutPiece.Mesh;
				Piece.Center = CutPiece.Center;
				Piece.CellIndex = CutPiece.CellIndex;
				Piece.SiteIndex = CutPiece.CellIndex;
				Piece.Direction = AxisDir * ClampFloat(Relative, -1.0f, 1.0f);
				if (Piece.Direction.SafeNormalize() == 0.0f)
				{
					Piece.Direction = AxisDir;
				}
				CutPieceToPiece[CutPieceIdx] = (int)Pieces.size();
				Pieces.push_back(Piece);
			}

			CopyCutPieceConnectivity(CutPieces, CutPieceToPiece, Pieces);
			return !Pieces.empty();
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

	bool Fracture::ParallelCutX(
		const DynamicMesh& SourceMesh,
		std::vector<FracturePiece>& Pieces,
		int PieceCount,
		const PlanarCutOptions& CutOptions)
	{
		return BuildParallelCutPieces(SourceMesh, 0, Pieces, PieceCount, CutOptions);
	}

	bool Fracture::ParallelCutY(
		const DynamicMesh& SourceMesh,
		std::vector<FracturePiece>& Pieces,
		int PieceCount,
		const PlanarCutOptions& CutOptions)
	{
		return BuildParallelCutPieces(SourceMesh, 1, Pieces, PieceCount, CutOptions);
	}

	bool Fracture::ParallelCutZ(
		const DynamicMesh& SourceMesh,
		std::vector<FracturePiece>& Pieces,
		int PieceCount,
		const PlanarCutOptions& CutOptions)
	{
		return BuildParallelCutPieces(SourceMesh, 2, Pieces, PieceCount, CutOptions);
	}

	bool Fracture::VoronoiFracture3D(
		const DynamicMesh& SourceMesh,
		std::vector<FracturePiece>& Pieces,
		int PieceCount,
		int Seed,
		const PlanarCutOptions& CutOptions)
	{
		PlanarCutOptions UseCutOptions = CutOptions;
		UseCutOptions.RandomSeed = Seed;

		std::vector<Vector3> Sites;
		GenerateUniformVoronoiSites3D(SourceMesh, PieceCount, Seed, Sites);
		return BuildVoronoiPiecesFromSites(SourceMesh, Sites, Pieces, UseCutOptions, 0.20f);
	}

	bool Fracture::VoronoiFracture2D(
		const DynamicMesh& SourceMesh,
		const Vector3& Normal,
		std::vector<FracturePiece>& Pieces,
		int PieceCount,
		int Seed,
		const PlanarCutOptions& CutOptions)
	{
		const Vector3 UseNormal = ResolveCutNormal(SourceMesh, Normal);
		PlanarCutOptions UseCutOptions = CutOptions;
		UseCutOptions.RandomSeed = Seed;

		std::vector<Vector3> Sites;
		if (!GenerateUniformVoronoiSites2D(SourceMesh, UseNormal, PieceCount, Seed, Sites))
		{
			Pieces.clear();
			return false;
		}

		const bool bBuilt = BuildVoronoiPiecesFromSites(SourceMesh, Sites, Pieces, UseCutOptions, 0.20f);
		if (bBuilt)
		{
			ApplyPlanarAxisSeparationDirections(SourceMesh.GetBounds(), UseNormal, Pieces);
		}
		return bBuilt;
	}

	bool Fracture::ClusterVoronoiFracture(
		const DynamicMesh& SourceMesh,
		std::vector<FracturePiece>& Pieces,
		int PieceCount,
		int Seed,
		const PlanarCutOptions& CutOptions)
	{
		PlanarCutOptions UseCutOptions = CutOptions;
		UseCutOptions.RandomSeed = Seed;

		std::vector<Vector3> Sites;
		GenerateClusterVoronoiSites(SourceMesh, PieceCount, Seed, Sites);
		return BuildVoronoiPiecesFromSites(SourceMesh, Sites, Pieces, UseCutOptions, 0.20f);
	}

	bool Fracture::Voxel2D(
		const DynamicMesh& SourceMesh,
		const Vector3& Normal,
		std::vector<FracturePiece>& Pieces,
		int RequestedPiecesX,
		int RequestedPiecesY,
		int PieceCount,
		const PlanarCutOptions& CutOptions)
	{
		Pieces.clear();

		const Vector3 UseNormal = ResolveCutNormal(SourceMesh, Normal);
		Vector3 Tangent, PlaneNormal, Binormal;
		if (!BuildPlaneBasis(UseNormal, Tangent, PlaneNormal, Binormal))
		{
			return false;
		}

		const Box3 Bounds = SourceMesh.GetBounds();
		const Vector3 Origin = Bounds.GetCenter();
		const float MaxSize = std::max(BoundsMaxDim(Bounds), 1e-3f);
		const float Padding = std::max(MaxSize * 0.01f, 1e-4f);

		const int NormalAxis = FindDominantAxis(UseNormal, 0.999f);
		if (NormalAxis >= 0)
		{
			const int AxisU = NormalAxis == 0 ? 1 : 0;
			const int AxisV = NormalAxis == 2 ? 1 : 2;
			const float MinU = AxisValue(Bounds.Min, AxisU);
			const float MaxU = AxisValue(Bounds.Max, AxisU);
			const float MinV = AxisValue(Bounds.Min, AxisV);
			const float MaxV = AxisValue(Bounds.Max, AxisV);

			int CountU = 1;
			int CountV = 1;
			ResolveGridCounts2D(RequestedPiecesX, RequestedPiecesY, PieceCount, MaxU - MinU, MaxV - MinV, CountU, CountV);

			std::vector<Box3> Cells;
			Cells.reserve(CountU * CountV);
			for (int VIdx = 0; VIdx < CountV; ++VIdx)
			{
				const float V0 = MinV + (MaxV - MinV) * ((float)VIdx / (float)CountV);
				const float V1 = MinV + (MaxV - MinV) * ((float)(VIdx + 1) / (float)CountV);
				for (int UIdx = 0; UIdx < CountU; ++UIdx)
				{
					const float U0 = MinU + (MaxU - MinU) * ((float)UIdx / (float)CountU);
					const float U1 = MinU + (MaxU - MinU) * ((float)(UIdx + 1) / (float)CountU);

					Vector3 CellMin = Bounds.Min - Vector3(Padding, Padding, Padding);
					Vector3 CellMax = Bounds.Max + Vector3(Padding, Padding, Padding);
					SetAxisValue(CellMin, AxisU, U0 - (UIdx == 0 ? Padding : 0.0f));
					SetAxisValue(CellMax, AxisU, U1 + (UIdx == CountU - 1 ? Padding : 0.0f));
					SetAxisValue(CellMin, AxisV, V0 - (VIdx == 0 ? Padding : 0.0f));
					SetAxisValue(CellMax, AxisV, V1 + (VIdx == CountV - 1 ? Padding : 0.0f));
					Cells.emplace_back(CellMin, CellMax);
				}
			}

			PlanarCells PlanarCellsIn(Cells, true);
			PlanarCutOptions UseCutOptions = MakePlanarOptions(SourceMesh, CutOptions, 0.08f);
			std::vector<PlanarCutPiece> CutPieces;
			if (!MeshCut::CutWithPlanarCells(SourceMesh, PlanarCellsIn, CutPieces, UseCutOptions))
			{
				return false;
			}

			AppendCutPiecesPlanar(CutPieces, Bounds, UseNormal, Pieces);
			return !Pieces.empty();
		}

		float MinU, MaxU, MinV, MaxV, MinN, MaxN;
		ComputePlaneRanges(Bounds, Origin, Tangent, PlaneNormal, Binormal, MinU, MaxU, MinV, MaxV, MinN, MaxN);
		MinU -= Padding;
		MaxU += Padding;
		MinV -= Padding;
		MaxV += Padding;
		MinN -= Padding;
		MaxN += Padding;

		int CountU = 1;
		int CountV = 1;
		ResolveGridCounts2D(RequestedPiecesX, RequestedPiecesY, PieceCount, MaxU - MinU, MaxV - MinV, CountU, CountV);

		PlanarCells Cells;
		Cells.NumCells = CountU * CountV;
		Cells.AssumeConvexCells = true;

		int CellIdx = 0;
		for (int VIdx = 0; VIdx < CountV; ++VIdx)
		{
			const float V0 = MinV + (MaxV - MinV) * ((float)VIdx / (float)CountV);
			const float V1 = MinV + (MaxV - MinV) * ((float)(VIdx + 1) / (float)CountV);
			for (int UIdx = 0; UIdx < CountU; ++UIdx)
			{
				const float U0 = MinU + (MaxU - MinU) * ((float)UIdx / (float)CountU);
				const float U1 = MinU + (MaxU - MinU) * ((float)(UIdx + 1) / (float)CountU);
				const float N0 = MinN;
				const float N1 = MaxN;
				const Vector3 CellCenter = Origin
					+ Tangent * ((U0 + U1) * 0.5f)
					+ Binormal * ((V0 + V1) * 0.5f)
					+ PlaneNormal * ((N0 + N1) * 0.5f);
				AppendOrientedBoxCell(Cells, CellIdx++, UIdx, VIdx, CountU, CountV, CellCenter, Tangent, Binormal, PlaneNormal,
					std::max((U1 - U0) * 0.5f, 1e-4f),
					std::max((V1 - V0) * 0.5f, 1e-4f),
					std::max((N1 - N0) * 0.5f, 1e-4f));
			}
		}

		PlanarCutOptions UseCutOptions = MakePlanarOptions(SourceMesh, CutOptions, 0.08f);
		std::vector<PlanarCutPiece> CutPieces;
		if (!MeshCut::CutWithPlanarCells(SourceMesh, Cells, CutPieces, UseCutOptions))
		{
			return false;
		}

		AppendCutPiecesPlanar(CutPieces, Bounds, PlaneNormal, Pieces);
		return !Pieces.empty();
	}

	bool Fracture::Voxel3D(
		const DynamicMesh& SourceMesh,
		std::vector<FracturePiece>& Pieces,
		int RequestedPiecesX,
		int RequestedPiecesY,
		int RequestedPiecesZ,
		int PieceCount,
		const PlanarCutOptions& CutOptions)
	{
		Pieces.clear();

		const Box3 Bounds = SourceMesh.GetBounds();
		const float MaxSize = std::max(BoundsMaxDim(Bounds), 1e-3f);
		const float Padding = std::max(MaxSize * 0.01f, 1e-4f);
		const Vector3 Size = Bounds.Max - Bounds.Min;

		int CountX = 1;
		int CountY = 1;
		int CountZ = 1;
		ResolveGridCounts3D(Bounds, RequestedPiecesX, RequestedPiecesY, RequestedPiecesZ, PieceCount, CountX, CountY, CountZ);

		std::vector<Box3> Cells;
		Cells.reserve(CountX * CountY * CountZ);
		for (int ZIdx = 0; ZIdx < CountZ; ++ZIdx)
		{
			for (int YIdx = 0; YIdx < CountY; ++YIdx)
			{
				for (int XIdx = 0; XIdx < CountX; ++XIdx)
				{
					const float X0 = Bounds.Min.x + Size.x * ((float)XIdx / (float)CountX);
					const float X1 = Bounds.Min.x + Size.x * ((float)(XIdx + 1) / (float)CountX);
					const float Y0 = Bounds.Min.y + Size.y * ((float)YIdx / (float)CountY);
					const float Y1 = Bounds.Min.y + Size.y * ((float)(YIdx + 1) / (float)CountY);
					const float Z0 = Bounds.Min.z + Size.z * ((float)ZIdx / (float)CountZ);
					const float Z1 = Bounds.Min.z + Size.z * ((float)(ZIdx + 1) / (float)CountZ);
					Vector3 CellMin(X0, Y0, Z0);
					Vector3 CellMax(X1, Y1, Z1);
					if (XIdx == 0) { CellMin.x -= Padding; }
					if (YIdx == 0) { CellMin.y -= Padding; }
					if (ZIdx == 0) { CellMin.z -= Padding; }
					if (XIdx == CountX - 1) { CellMax.x += Padding; }
					if (YIdx == CountY - 1) { CellMax.y += Padding; }
					if (ZIdx == CountZ - 1) { CellMax.z += Padding; }
					Cells.emplace_back(CellMin, CellMax);
				}
			}
		}

		PlanarCells PlanarCellsIn(Cells, true);
		PlanarCutOptions UseCutOptions = MakePlanarOptions(SourceMesh, CutOptions, 0.08f);
		std::vector<PlanarCutPiece> CutPieces;
		if (!MeshCut::CutWithPlanarCells(SourceMesh, PlanarCellsIn, CutPieces, UseCutOptions))
		{
			return false;
		}

		AppendCutPiecesRadial(CutPieces, Bounds.GetCenter(), Pieces);
		return !Pieces.empty();
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
		const PlanarCutOptions& CutOptions)
	{
		Pieces.clear();
		if (Sites.empty() || CountValidTriangles(SourceMesh) == 0)
		{
			return false;
		}

		const PlanarCutOptions UseCutOptions = MakePlanarOptions(SourceMesh, CutOptions, 0.20f);
		Box3 VoronoiBounds = SourceMesh.GetBounds();
		float MaxDim = BoundsMaxDim(VoronoiBounds);
		if (MaxDim <= 0.0f)
		{
			MaxDim = 1.0f;
		}
		VoronoiBounds.Thicken(std::max(MaxDim * UseCutOptions.BoundsPaddingScale, UseCutOptions.SnapTolerance * 16.0f));
		VoronoiBounds = Voronoi3::GetVoronoiBounds(VoronoiBounds, Sites);

		PlanarCells Cells(Sites, VoronoiBounds, UseCutOptions.SnapTolerance);

		std::vector<PlanarCutPiece> CutPieces;
		if (!MeshCut::CutWithPlanarCells(SourceMesh, Cells, CutPieces, UseCutOptions))
		{
			return false;
		}

		std::vector<int> CutPieceToPiece(CutPieces.size(), -1);
		for (size_t CutPieceIdx = 0; CutPieceIdx < CutPieces.size(); ++CutPieceIdx)
		{
			const PlanarCutPiece& CutPiece = CutPieces[CutPieceIdx];
			if (CutPiece.CellIndex >= 0 && CutPiece.CellIndex < (int)Sites.size())
			{
				FracturePiece Piece;
				Piece.Mesh = CutPiece.Mesh;
				Piece.Site = Sites[CutPiece.CellIndex];
				Piece.CellIndex = CutPiece.CellIndex;
				Piece.SiteIndex = CutPiece.CellIndex;
				Piece.Center = CutPiece.Center;
				CutPieceToPiece[CutPieceIdx] = (int)Pieces.size();
				Pieces.push_back(Piece);
			}
		}

		CopyCutPieceConnectivity(CutPieces, CutPieceToPiece, Pieces);
		return !Pieces.empty();
	}

	bool Fracture::VoronoiFracture(
		const DynamicMesh& SourceMesh,
		const std::vector<Vector3>& Sites,
		std::vector<DynamicMesh>& Pieces,
		const PlanarCutOptions& CutOptions)
	{
		Pieces.clear();

		std::vector<FracturePiece> FracturePieces;
		const bool bSuccess = VoronoiFracture(SourceMesh, Sites, FracturePieces, CutOptions);
		for (const FracturePiece& Piece : FracturePieces)
		{
			Pieces.push_back(Piece.Mesh);
		}
		return bSuccess;
	}

}	// namespace Riemann
