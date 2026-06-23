
#include "MeshCut.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <initializer_list>
#include <map>
#include <random>

#include "DynamicMesh.h"
#include "GeometryBoolean.h"
#include "../Maths/Box3.h"
#include "Voronoi3.h"

namespace Riemann
{
	namespace
	{
		struct QuantizedPointKey
		{
			long long X = 0;
			long long Y = 0;
			long long Z = 0;

			bool operator<(const QuantizedPointKey& Rhs) const
			{
				if (X != Rhs.X)
				{
					return X < Rhs.X;
				}
				if (Y != Rhs.Y)
				{
					return Y < Rhs.Y;
				}
				return Z < Rhs.Z;
			}
		};

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

		static int CountValidVertices(const DynamicMesh& Mesh)
		{
			int Count = 0;
			for (int VID = 0; VID < Mesh.GetVertexCount(); ++VID)
			{
				if (Mesh.IsVertexFast(VID))
				{
					++Count;
				}
			}
			return Count;
		}

		static void AddPlanarCutPieceNeighbor(std::vector<PlanarCutPiece>& Pieces, int PieceA, int PieceB)
		{
			if (PieceA < 0 || PieceB < 0 || PieceA == PieceB ||
				PieceA >= (int)Pieces.size() || PieceB >= (int)Pieces.size())
			{
				return;
			}

			std::vector<int>& NeighborsA = Pieces[(size_t)PieceA].NeighborPieceIndices;
			if (std::find(NeighborsA.begin(), NeighborsA.end(), PieceB) == NeighborsA.end())
			{
				NeighborsA.push_back(PieceB);
			}

			std::vector<int>& NeighborsB = Pieces[(size_t)PieceB].NeighborPieceIndices;
			if (std::find(NeighborsB.begin(), NeighborsB.end(), PieceA) == NeighborsB.end())
			{
				NeighborsB.push_back(PieceA);
			}
		}

		static void BuildPlanarCutPieceConnectivity(const PlanarCells& Cells, std::vector<PlanarCutPiece>& Pieces)
		{
			for (PlanarCutPiece& Piece : Pieces)
			{
				Piece.NeighborPieceIndices.clear();
			}

			if (Cells.NumCells <= 0)
			{
				return;
			}

			std::vector<int> CellToPiece((size_t)Cells.NumCells, -1);
			for (size_t PieceIndex = 0; PieceIndex < Pieces.size(); ++PieceIndex)
			{
				const int CellIndex = Pieces[PieceIndex].CellIndex;
				if (CellIndex >= 0 && CellIndex < Cells.NumCells)
				{
					CellToPiece[(size_t)CellIndex] = (int)PieceIndex;
				}
			}

			for (const Index2& CellPair : Cells.PlaneCells)
			{
				if (CellPair.a < 0 || CellPair.a >= Cells.NumCells ||
					CellPair.b < 0 || CellPair.b >= Cells.NumCells)
				{
					continue;
				}

				AddPlanarCutPieceNeighbor(Pieces, CellToPiece[(size_t)CellPair.a], CellToPiece[(size_t)CellPair.b]);
			}
		}

		static float BoundsMaxDim(const Box3& Bounds)
		{
			const Vector3 Size = Bounds.Max - Bounds.Min;
			return std::max(Size.x, std::max(Size.y, Size.z));
		}

		static float BoxAxisMin(const Box3& Bounds, int Axis)
		{
			switch (Axis)
			{
			case 0:
				return Bounds.Min.x;
			case 1:
				return Bounds.Min.y;
			default:
				return Bounds.Min.z;
			}
		}

		static float BoxAxisMax(const Box3& Bounds, int Axis)
		{
			switch (Axis)
			{
			case 0:
				return Bounds.Max.x;
			case 1:
				return Bounds.Max.y;
			default:
				return Bounds.Max.z;
			}
		}

		static bool BoxFacesMatchOnOtherAxes(const Box3& BoxA, const Box3& BoxB, int FaceAxis, float Tolerance)
		{
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				if (Axis == FaceAxis)
				{
					continue;
				}

				if (std::fabs(BoxAxisMin(BoxA, Axis) - BoxAxisMin(BoxB, Axis)) > Tolerance ||
					std::fabs(BoxAxisMax(BoxA, Axis) - BoxAxisMax(BoxB, Axis)) > Tolerance)
				{
					return false;
				}
			}
			return true;
		}

		static int FindAdjacentBoxFace(
			const std::vector<Box3>& Boxes,
			int BoxIdx,
			int FaceAxis,
			bool bPositiveFace,
			float Tolerance)
		{
			if (BoxIdx < 0 || BoxIdx >= (int)Boxes.size())
			{
				return -1;
			}

			const Box3& Box = Boxes[(size_t)BoxIdx];
			const float FaceValue = bPositiveFace ? BoxAxisMax(Box, FaceAxis) : BoxAxisMin(Box, FaceAxis);
			for (int OtherIdx = 0; OtherIdx < (int)Boxes.size(); ++OtherIdx)
			{
				if (OtherIdx == BoxIdx)
				{
					continue;
				}

				const Box3& Other = Boxes[(size_t)OtherIdx];
				const float OtherFaceValue = bPositiveFace ? BoxAxisMin(Other, FaceAxis) : BoxAxisMax(Other, FaceAxis);
				if (std::fabs(OtherFaceValue - FaceValue) <= Tolerance &&
					BoxFacesMatchOnOtherAxes(Box, Other, FaceAxis, Tolerance))
				{
					return OtherIdx;
				}
			}

			return -1;
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

		static Vector3 FromPlaneBoxCoordinates(
			const Vector3& Origin,
			const Vector3& Tangent,
			const Vector3& Normal,
			const Vector3& Binormal,
			float X,
			float Y,
			float Z)
		{
			return Origin + Tangent * X + Normal * Y + Binormal * Z;
		}

		static QuantizedPointKey MakePointKey(const Vector3& Position, float Tolerance)
		{
			const float InvTolerance = 1.0f / std::max(Tolerance, 1e-6f);
			QuantizedPointKey Key;
			Key.X = (long long)std::llround(Position.x * InvTolerance);
			Key.Y = (long long)std::llround(Position.y * InvTolerance);
			Key.Z = (long long)std::llround(Position.z * InvTolerance);
			return Key;
		}

		static int FindOrAppendCellVertex(
			DynamicMesh& Mesh,
			std::map<QuantizedPointKey, int>& VertexMap,
			const Vector3& Position,
			float Tolerance)
		{
			const QuantizedPointKey Key = MakePointKey(Position, Tolerance);
			auto Found = VertexMap.find(Key);
			if (Found != VertexMap.end())
			{
				return Found->second;
			}

			const int VertexID = Mesh.AppendVertex(Position);
			VertexMap.emplace(Key, VertexID);
			return VertexID;
		}

		static bool AppendPlanarFace(
			DynamicMesh& Mesh,
			std::map<QuantizedPointKey, int>& VertexMap,
			const std::vector<Vector3>& BoundaryVertices,
			const std::vector<int>& Boundary,
			bool bFlipOrientation,
			float Tolerance)
		{
			const int NumFaceVertices = (int)Boundary.size();
			if (NumFaceVertices < 3)
			{
				return false;
			}

			std::vector<int> FaceVertexIDs;
			FaceVertexIDs.reserve(NumFaceVertices);
			for (int BoundaryVertexID : Boundary)
			{
				if (BoundaryVertexID < 0 || BoundaryVertexID >= (int)BoundaryVertices.size())
				{
					return false;
				}
				FaceVertexIDs.push_back(FindOrAppendCellVertex(Mesh, VertexMap, BoundaryVertices[BoundaryVertexID], Tolerance));
			}

			bool bAdded = false;
			for (int V1 = 1, V2 = 2; V2 < NumFaceVertices; V1 = V2++)
			{
				Index3 Tri(FaceVertexIDs[0], FaceVertexIDs[V1], FaceVertexIDs[V2]);
				if (bFlipOrientation)
				{
					std::swap(Tri.b, Tri.c);
				}
				if (Tri.a != Tri.b && Tri.a != Tri.c && Tri.b != Tri.c)
				{
					bAdded = Mesh.AppendTriangle(Tri) >= 0 || bAdded;
				}
			}
			return bAdded;
		}

		static DynamicMesh MakeAABBMesh(const Box3& Bounds)
		{
			std::vector<Vector3> Vertices(8);
			Box3::GetVertices(Bounds.Min, Bounds.Max, Vertices.data());

			std::vector<uint16_t> Indices =
			{
				0, 1, 2,
				1, 3, 2,
				4, 6, 5,
				5, 6, 7,
				0, 4, 1,
				5, 1, 4,
				1, 5, 3,
				7, 3, 5,
				2, 4, 0,
				6, 4, 2,
				3, 6, 2,
				6, 3, 7
			};

			const Vector3 Center = Bounds.GetCenter();
			std::vector<Vector3> Normals(8);
			for (int VertexIdx = 0; VertexIdx < 8; ++VertexIdx)
			{
				Normals[VertexIdx] = (Vertices[VertexIdx] - Center).SafeUnit();
			}

			DynamicMesh Mesh;
			Mesh.SetData(Vertices, Indices, Normals);
			return Mesh;
		}

		static Vector3 CalculateVertexCentroid(const DynamicMesh& Mesh)
		{
			Vector3 Sum = Vector3::Zero();
			int NumVertices = 0;
			for (int VID = 0; VID < Mesh.GetVertexCount(); ++VID)
			{
				if (Mesh.IsVertexFast(VID))
				{
					Sum += Mesh.GetVertex(VID);
					++NumVertices;
				}
			}
			return NumVertices > 0 ? Sum / (float)NumVertices : Vector3::Zero();
		}

		static void ScaleMeshAround(DynamicMesh& Mesh, const Vector3& Center, float Scale)
		{
			for (int VID = 0; VID < Mesh.GetVertexCount(); ++VID)
			{
				if (Mesh.IsVertexFast(VID))
				{
					Mesh.SetVertex(VID, Center + (Mesh.GetVertex(VID) - Center) * Scale);
				}
			}
			Mesh.BuildBounds();
		}

		static float HashNoise01(float X, float Y, float Z, int Seed)
		{
			const float Value = std::sin(X * 12.9898f + Y * 78.233f + Z * 37.719f + (float)Seed * 19.19f) * 43758.5453f;
			return Value - std::floor(Value);
		}

		static float FractalNoise(const Vector3& Position, const PlanarCellNoiseSettings& Settings, int Seed)
		{
			float Sum = 0.0f;
			float FrequencyScale = 1.0f;
			float AmplitudeScale = 1.0f;
			for (int Octave = 0; Octave < std::max(1, Settings.Octaves); ++Octave)
			{
				const Vector3 P = Position * (Settings.Frequency * FrequencyScale);
				Sum += (HashNoise01(P.x, P.y, P.z, Seed + Octave * 17) * 2.0f - 1.0f) * AmplitudeScale;
				FrequencyScale *= Settings.Lacunarity;
				AmplitudeScale *= Settings.Persistence;
			}
			return Sum;
		}

		static void ApplyNoise(DynamicMesh& Mesh, const PlanarCellNoiseSettings& Settings, int Seed)
		{
			if (Settings.Amplitude == 0.0f || CountValidVertices(Mesh) == 0)
			{
				return;
			}

			const Vector3 Center = CalculateVertexCentroid(Mesh);
			for (int VID = 0; VID < Mesh.GetVertexCount(); ++VID)
			{
				if (!Mesh.IsVertexFast(VID))
				{
					continue;
				}

				const Vector3 Position = Mesh.GetVertex(VID);
				Vector3 Direction = Position - Center;
				if (Direction.SafeNormalize() == 0.0f)
				{
					continue;
				}
				const float Offset = FractalNoise(Position, Settings, Seed) * Settings.Amplitude;
				Mesh.SetVertex(VID, Position + Direction * Offset);
			}
			Mesh.BuildBounds();
			Mesh.CalculateWeightAverageNormals();
		}

		static void AppendMesh(DynamicMesh& Destination, const DynamicMesh& Source)
		{
			FDynamicMeshEditor Editor(&Destination);
			FMeshIndexMappings Mappings;
			Editor.AppendMesh(&Source, Mappings);
		}

		static bool IntersectWithHalfSpace(
			const DynamicMesh& SourceMesh,
			const DynamicMesh& HalfSpaceMesh,
			const PlanarCutOptions& Options,
			DynamicMesh* ResultOut)
		{
			if (ResultOut == nullptr)
			{
				return true;
			}

			ResultOut->Clear();

			GeometryBoolean Boolean(&SourceMesh, &HalfSpaceMesh, GeometryBoolean::BooleanOp::Intersect);
			Boolean.SnapTolerance = Options.SnapTolerance;
			Boolean.WeldSharedEdges = Options.WeldSharedEdges;
			Boolean.SimplifyAlongNewEdges = Options.SimplifyAlongCut;
			Boolean.CollapseDegenerateEdgesOnCut = true;

			const bool bComputed = Boolean.Compute();
			bool bHasResult = false;
			if (Boolean.Result != nullptr)
			{
				if (CountValidTriangles(*Boolean.Result) > 0)
				{
					Boolean.Result->FixTriangleOrientation(false);
					Boolean.Result->CalculateWeightAverageNormals();
					Boolean.Result->BuildBounds();
					*ResultOut = *Boolean.Result;
					bHasResult = true;
				}
				delete Boolean.Result;
				Boolean.Result = nullptr;
			}

			return bComputed || bHasResult;
		}
	}

	PlanarCells::PlanarCells(const PlanarCutPlane& Plane)
	{
		NumCells = 2;
		AddPlane(Plane, 0, 1);
	}

	PlanarCells::PlanarCells(const std::vector<Vector3>& Sites, const Box3& Bounds, float Eps)
	{
		NumCells = (int)Sites.size();
		AssumeConvexCells = true;
		if (Sites.empty())
		{
			return;
		}

		Voronoi3 Voronoi(Sites, Bounds, Eps);
		Voronoi.Build();

		PlaneBoundaryVertices = Voronoi.mBoundaryVertices;
		Planes.reserve(Voronoi.mPlanes.size());
		PlaneCells.reserve(Voronoi.mCells.size());
		PlaneBoundaries.reserve(Voronoi.mBoundaries.size());

		for (int PlaneIdx = 0; PlaneIdx < (int)Voronoi.mPlanes.size(); ++PlaneIdx)
		{
			const Voronoi3::Plane& Plane = Voronoi.mPlanes[PlaneIdx];
			AddPlane(
				PlanarCutPlane(Plane.GetNormal(), Plane.w),
				Voronoi.mCells[PlaneIdx].first,
				Voronoi.mCells[PlaneIdx].second,
				Voronoi.mBoundaries[PlaneIdx]);
		}
	}

	PlanarCells::PlanarCells(const std::vector<Box3>& Boxes, bool ResolveAdjacencies)
	{
		NumCells = (int)Boxes.size();
		AssumeConvexCells = true;

		float MaxDim = 1.0f;
		for (const Box3& Box : Boxes)
		{
			MaxDim = std::max(MaxDim, BoundsMaxDim(Box));
		}
		const float AdjacencyTolerance = std::max(MaxDim * 1e-5f, 1e-6f);

		for (int BoxIdx = 0; BoxIdx < (int)Boxes.size(); ++BoxIdx)
		{
			const Box3& Box = Boxes[BoxIdx];
			const Vector3& Min = Box.Min;
			const Vector3& Max = Box.Max;

			const int VertexStart = (int)PlaneBoundaryVertices.size();
			PlaneBoundaryVertices.push_back(Min);
			PlaneBoundaryVertices.push_back(Vector3(Max.x, Min.y, Min.z));
			PlaneBoundaryVertices.push_back(Vector3(Max.x, Max.y, Min.z));
			PlaneBoundaryVertices.push_back(Vector3(Min.x, Max.y, Min.z));
			PlaneBoundaryVertices.push_back(Vector3(Min.x, Min.y, Max.z));
			PlaneBoundaryVertices.push_back(Vector3(Max.x, Min.y, Max.z));
			PlaneBoundaryVertices.push_back(Max);
			PlaneBoundaryVertices.push_back(Vector3(Min.x, Max.y, Max.z));

			const auto AddBoxFace = [&](const PlanarCutPlane& Plane, int FaceAxis, bool bPositiveFace, std::initializer_list<int> Boundary)
			{
				const int NeighborCell = ResolveAdjacencies ? FindAdjacentBoxFace(Boxes, BoxIdx, FaceAxis, bPositiveFace, AdjacencyTolerance) : -1;
				if (NeighborCell >= 0 && BoxIdx > NeighborCell)
				{
					return;
				}

				AddPlane(Plane, BoxIdx, NeighborCell, std::vector<int>(Boundary));
			};

			AddBoxFace(PlanarCutPlane(Vector3(0.0f, 0.0f, -1.0f), -Min.z), 2, false, { VertexStart + 0, VertexStart + 1, VertexStart + 2, VertexStart + 3 });
			AddBoxFace(PlanarCutPlane(Vector3(0.0f, 0.0f, 1.0f), Max.z), 2, true, { VertexStart + 4, VertexStart + 7, VertexStart + 6, VertexStart + 5 });
			AddBoxFace(PlanarCutPlane(Vector3(0.0f, -1.0f, 0.0f), -Min.y), 1, false, { VertexStart + 0, VertexStart + 4, VertexStart + 5, VertexStart + 1 });
			AddBoxFace(PlanarCutPlane(Vector3(0.0f, 1.0f, 0.0f), Max.y), 1, true, { VertexStart + 3, VertexStart + 2, VertexStart + 6, VertexStart + 7 });
			AddBoxFace(PlanarCutPlane(Vector3(-1.0f, 0.0f, 0.0f), -Min.x), 0, false, { VertexStart + 0, VertexStart + 3, VertexStart + 7, VertexStart + 4 });
			AddBoxFace(PlanarCutPlane(Vector3(1.0f, 0.0f, 0.0f), Max.x), 0, true, { VertexStart + 1, VertexStart + 5, VertexStart + 6, VertexStart + 2 });
		}
	}

	bool PlanarCells::IsInfinitePlane() const
	{
		return NumCells == 2 && Planes.size() == 1 && (PlaneBoundaries.empty() || PlaneBoundaries[0].empty());
	}

	bool PlanarCells::HasValidPlaneBoundaryOrientations(float Tolerance) const
	{
		for (int PlaneIdx = 0; PlaneIdx < (int)PlaneBoundaries.size(); ++PlaneIdx)
		{
			const std::vector<int>& Boundary = PlaneBoundaries[PlaneIdx];
			if (Boundary.size() < 3)
			{
				continue;
			}

			const PlanarCutPlane& Plane = Planes[PlaneIdx];
			for (int BoundaryVertexID : Boundary)
			{
				if (BoundaryVertexID < 0 || BoundaryVertexID >= (int)PlaneBoundaryVertices.size())
				{
					return false;
				}
				if (std::fabs(Plane.SignedDistance(PlaneBoundaryVertices[BoundaryVertexID])) > Tolerance)
				{
					return false;
				}
			}

			const Vector3& A = PlaneBoundaryVertices[Boundary[0]];
			const Vector3& B = PlaneBoundaryVertices[Boundary[1]];
			const Vector3& C = PlaneBoundaryVertices[Boundary[2]];
			Vector3 Normal = (C - B).Cross(B - A);
			if (Normal.SafeNormalize() == 0.0f)
			{
				continue;
			}
			if (AssumeConvexCells && Normal.Dot(Plane.Normal) < -Tolerance)
			{
				return false;
			}
		}
		return true;
	}

	void PlanarCells::AddPlane(const PlanarCutPlane& Plane, int NegativeCell, int PositiveCell)
	{
		Planes.push_back(Plane);
		PlaneCells.emplace_back(NegativeCell, PositiveCell);
		PlaneBoundaries.emplace_back();
	}

	int PlanarCells::AddPlane(const PlanarCutPlane& Plane, int NegativeCell, int PositiveCell, const std::vector<int>& PlaneBoundary)
	{
		const int PlaneIdx = (int)Planes.size();
		Planes.push_back(Plane);
		PlaneCells.emplace_back(NegativeCell, PositiveCell);
		PlaneBoundaries.push_back(PlaneBoundary);
		return PlaneIdx;
	}

	void PlanarCells::DiscardCells(const std::vector<bool>& KeepCell, bool KeepNeighbors)
	{
		std::vector<int> CellRemap(NumCells, -1);
		int NewNumCells = 0;
		for (int CellIdx = 0; CellIdx < NumCells && CellIdx < (int)KeepCell.size(); ++CellIdx)
		{
			if (KeepCell[CellIdx])
			{
				CellRemap[CellIdx] = NewNumCells++;
			}
		}

		std::vector<PlanarCutPlane> NewPlanes;
		std::vector<Index2> NewPlaneCells;
		std::vector<std::vector<int>> NewPlaneBoundaries;
		for (int PlaneIdx = 0; PlaneIdx < (int)Planes.size(); ++PlaneIdx)
		{
			const int CellA = PlaneCells[PlaneIdx].a;
			const int CellB = PlaneCells[PlaneIdx].b;
			const bool bKeepA = CellA >= 0 && CellA < (int)CellRemap.size() && CellRemap[CellA] >= 0;
			const bool bKeepB = CellB >= 0 && CellB < (int)CellRemap.size() && CellRemap[CellB] >= 0;
			if (KeepNeighbors ? !(bKeepA || bKeepB) : !(bKeepA && (CellB < 0 || bKeepB)))
			{
				continue;
			}

			NewPlanes.push_back(Planes[PlaneIdx]);
			NewPlaneCells.emplace_back(bKeepA ? CellRemap[CellA] : -1, bKeepB ? CellRemap[CellB] : -1);
			NewPlaneBoundaries.push_back(PlaneBoundaries[PlaneIdx]);
		}

		NumCells = NewNumCells;
		Planes.swap(NewPlanes);
		PlaneCells.swap(NewPlaneCells);
		PlaneBoundaries.swap(NewPlaneBoundaries);
	}

	void PlanarCellMeshes::Clear()
	{
		Meshes.clear();
		OutsideCellIndex = -1;
	}

	bool PlanarCellMeshes::Build(const PlanarCells& Cells, const Box3& DomainBounds, const PlanarCutOptions& Options)
	{
		Clear();
		if (Cells.NumCells <= 0)
		{
			return false;
		}

		if (Cells.IsInfinitePlane())
		{
			DynamicMesh DomainMesh = MakeAABBMesh(DomainBounds);
			Meshes.resize(2);
			const PlanarCutPlane& Plane = Cells.Planes[0];
			if (!MeshCut::BuildHalfSpaceMesh(DomainMesh, Plane.GetOrigin(), Plane.Normal, false, Options, Meshes[0]) ||
				!MeshCut::BuildHalfSpaceMesh(DomainMesh, Plane.GetOrigin(), Plane.Normal, true, Options, Meshes[1]))
			{
				Clear();
				return false;
			}
			return true;
		}

		const bool bIncludeOutsideCell = Options.IncludeOutsideCell;
		OutsideCellIndex = bIncludeOutsideCell ? Cells.NumCells : -1;
		const int NumMeshes = Cells.NumCells + (bIncludeOutsideCell ? 1 : 0);
		Meshes.resize(NumMeshes);

		std::vector<std::map<QuantizedPointKey, int>> VertexMaps(NumMeshes);
		const float VertexTolerance = std::max(Options.SnapTolerance, 1e-6f);

		for (int PlaneIdx = 0; PlaneIdx < (int)Cells.PlaneCells.size(); ++PlaneIdx)
		{
			if (PlaneIdx >= (int)Cells.PlaneBoundaries.size())
			{
				continue;
			}

			const Index2 CellPair = Cells.PlaneCells[PlaneIdx];
			if (CellPair.a >= 0 && CellPair.a < NumMeshes)
			{
				AppendPlanarFace(Meshes[CellPair.a], VertexMaps[CellPair.a],
					Cells.PlaneBoundaryVertices, Cells.PlaneBoundaries[PlaneIdx], false, VertexTolerance);
			}

			int OtherCell = CellPair.b < 0 ? OutsideCellIndex : CellPair.b;
			if (OtherCell >= 0 && OtherCell < NumMeshes)
			{
				const bool bFlip = OtherCell != OutsideCellIndex;
				AppendPlanarFace(Meshes[OtherCell], VertexMaps[OtherCell],
					Cells.PlaneBoundaryVertices, Cells.PlaneBoundaries[PlaneIdx], bFlip, VertexTolerance);
			}
		}

		for (int MeshIdx = 0; MeshIdx < (int)Meshes.size(); ++MeshIdx)
		{
			DynamicMesh& Mesh = Meshes[MeshIdx];
			if (CountValidTriangles(Mesh) == 0)
			{
				continue;
			}

			Mesh.BuildBounds();
			if (Options.Grout > 0.0f && MeshIdx != OutsideCellIndex)
			{
				const float BoundsSize = BoundsMaxDim(Mesh.GetBounds());
				const float ScaleFactor = BoundsSize > 0.0f ? (BoundsSize - Options.Grout * 0.5f) / BoundsSize : 0.0f;
				if (ScaleFactor <= 1e-4f)
				{
					Mesh.Clear();
					continue;
				}
				ScaleMeshAround(Mesh, CalculateVertexCentroid(Mesh), ScaleFactor);
			}

			if (Cells.SurfaceSettings.EnableNoise && MeshIdx != OutsideCellIndex)
			{
				ApplyNoise(Mesh, Cells.SurfaceSettings.Noise, Options.RandomSeed + MeshIdx * 31);
			}

			Mesh.CalculateWeightAverageNormals();
		}

		return true;
	}

	bool MeshCut::BuildHalfSpaceMesh(
		const DynamicMesh& SourceMesh,
		const Vector3& PlaneOrigin,
		const Vector3& PlaneNormal,
		bool bPositiveSide,
		const PlanarCutOptions& Options,
		DynamicMesh& HalfSpaceMesh)
	{
		HalfSpaceMesh.Clear();

		Vector3 Tangent, Normal, Binormal;
		if (!BuildPlaneBasis(PlaneNormal, Tangent, Normal, Binormal))
		{
			return false;
		}

		Box3 Bounds = SourceMesh.GetBounds();
		float MaxDim = BoundsMaxDim(Bounds);
		if (MaxDim <= 0.0f)
		{
			MaxDim = 1.0f;
		}

		const float Padding = std::max(MaxDim * Options.BoundsPaddingScale, Options.SnapTolerance * 16.0f);

		Vector3 Corners[8];
		Box3::GetVertices(Bounds.Min, Bounds.Max, Corners);

		float XExtent = Padding;
		float ZExtent = Padding;
		float MinY = FLT_MAX;
		float MaxY = -FLT_MAX;

		for (int CornerIdx = 0; CornerIdx < 8; ++CornerIdx)
		{
			const Vector3 Rel = Corners[CornerIdx] - PlaneOrigin;
			XExtent = std::max(XExtent, std::fabs(Rel.Dot(Tangent)) + Padding);
			ZExtent = std::max(ZExtent, std::fabs(Rel.Dot(Binormal)) + Padding);
			const float Y = Rel.Dot(Normal);
			MinY = std::min(MinY, Y);
			MaxY = std::max(MaxY, Y);
		}

		const float YMin = bPositiveSide ? 0.0f : std::min(MinY - Padding, -Padding);
		const float YMax = bPositiveSide ? std::max(MaxY + Padding, Padding) : 0.0f;

		std::vector<Vector3> Vertices(8);
		Vertices[0] = FromPlaneBoxCoordinates(PlaneOrigin, Tangent, Normal, Binormal, -XExtent, YMin, -ZExtent);
		Vertices[1] = FromPlaneBoxCoordinates(PlaneOrigin, Tangent, Normal, Binormal,  XExtent, YMin, -ZExtent);
		Vertices[2] = FromPlaneBoxCoordinates(PlaneOrigin, Tangent, Normal, Binormal, -XExtent, YMax, -ZExtent);
		Vertices[3] = FromPlaneBoxCoordinates(PlaneOrigin, Tangent, Normal, Binormal,  XExtent, YMax, -ZExtent);
		Vertices[4] = FromPlaneBoxCoordinates(PlaneOrigin, Tangent, Normal, Binormal, -XExtent, YMin,  ZExtent);
		Vertices[5] = FromPlaneBoxCoordinates(PlaneOrigin, Tangent, Normal, Binormal,  XExtent, YMin,  ZExtent);
		Vertices[6] = FromPlaneBoxCoordinates(PlaneOrigin, Tangent, Normal, Binormal, -XExtent, YMax,  ZExtent);
		Vertices[7] = FromPlaneBoxCoordinates(PlaneOrigin, Tangent, Normal, Binormal,  XExtent, YMax,  ZExtent);

		std::vector<uint16_t> Indices =
		{
			0, 1, 2,
			1, 3, 2,
			4, 6, 5,
			5, 6, 7,
			0, 4, 1,
			5, 1, 4,
			1, 5, 3,
			7, 3, 5,
			2, 4, 0,
			6, 4, 2,
			3, 6, 2,
			6, 3, 7
		};

		const Vector3 Center = (Vertices[0] + Vertices[7]) * 0.5f;
		std::vector<Vector3> Normals(8);
		for (int VertexIdx = 0; VertexIdx < 8; ++VertexIdx)
		{
			Normals[VertexIdx] = (Vertices[VertexIdx] - Center).SafeUnit();
		}

		HalfSpaceMesh.SetData(Vertices, Indices, Normals);
		return CountValidTriangles(HalfSpaceMesh) > 0;
	}

	bool MeshCut::Cut(
		const DynamicMesh& SourceMesh,
		const Vector3& PlaneOrigin,
		const Vector3& PlaneNormal,
		DynamicMesh* NegativeSide,
		DynamicMesh* PositiveSide,
		const PlanarCutOptions& Options)
	{
		if (CountValidTriangles(SourceMesh) == 0)
		{
			if (NegativeSide)
			{
				NegativeSide->Clear();
			}
			if (PositiveSide)
			{
				PositiveSide->Clear();
			}
			return false;
		}

		DynamicMesh PositiveHalfSpace;
		DynamicMesh NegativeHalfSpace;
		if (!BuildHalfSpaceMesh(SourceMesh, PlaneOrigin, PlaneNormal, true, Options, PositiveHalfSpace) ||
			!BuildHalfSpaceMesh(SourceMesh, PlaneOrigin, PlaneNormal, false, Options, NegativeHalfSpace))
		{
			return false;
		}

		bool bSuccess = true;
		bSuccess = IntersectWithHalfSpace(SourceMesh, NegativeHalfSpace, Options, NegativeSide) && bSuccess;
		bSuccess = IntersectWithHalfSpace(SourceMesh, PositiveHalfSpace, Options, PositiveSide) && bSuccess;
		return bSuccess;
	}

	bool MeshCut::Cut(
		const DynamicMesh& SourceMesh,
		const Vector3& PlaneOrigin,
		const Vector3& PlaneNormal,
		std::vector<DynamicMesh>& Pieces,
		const PlanarCutOptions& Options)
	{
		Pieces.clear();

		DynamicMesh NegativeSide;
		DynamicMesh PositiveSide;
		const bool bSuccess = Cut(SourceMesh, PlaneOrigin, PlaneNormal, &NegativeSide, &PositiveSide, Options);

		if (CountValidTriangles(NegativeSide) > 0)
		{
			Pieces.push_back(NegativeSide);
		}
		if (CountValidTriangles(PositiveSide) > 0)
		{
			Pieces.push_back(PositiveSide);
		}

		return bSuccess && !Pieces.empty();
	}

	bool MeshCut::BuildCellMeshes(
		const PlanarCells& Cells,
		const Box3& DomainBounds,
		std::vector<DynamicMesh>& CellMeshes,
		const PlanarCutOptions& Options)
	{
		PlanarCellMeshes MeshBuilder;
		const bool bBuilt = MeshBuilder.Build(Cells, DomainBounds, Options);
		CellMeshes = MeshBuilder.Meshes;
		return bBuilt;
	}

	bool MeshCut::CutWithPlanarCells(
		const DynamicMesh& SourceMesh,
		const PlanarCells& Cells,
		std::vector<PlanarCutPiece>& Pieces,
		const PlanarCutOptions& Options)
	{
		Pieces.clear();
		if (CountValidTriangles(SourceMesh) == 0 || Cells.NumCells <= 0)
		{
			return false;
		}

		Box3 DomainBounds = SourceMesh.GetBounds();
		const float MaxDim = std::max(BoundsMaxDim(DomainBounds), 1.0f);
		DomainBounds.Thicken(std::max(MaxDim * Options.BoundsPaddingScale, Options.SnapTolerance * 16.0f));

		PlanarCellMeshes CellMeshes;
		if (!CellMeshes.Build(Cells, DomainBounds, Options))
		{
			return false;
		}

		for (int CellMeshIdx = 0; CellMeshIdx < (int)CellMeshes.Meshes.size(); ++CellMeshIdx)
		{
			const DynamicMesh& CellMesh = CellMeshes.Meshes[CellMeshIdx];
			if (CountValidTriangles(CellMesh) == 0)
			{
				continue;
			}

			GeometryBoolean Boolean(&SourceMesh, &CellMesh, GeometryBoolean::BooleanOp::Intersect);
			Boolean.SnapTolerance = Options.SnapTolerance;
			Boolean.WeldSharedEdges = Options.WeldSharedEdges;
			Boolean.SimplifyAlongNewEdges = Options.SimplifyAlongCut;
			Boolean.CollapseDegenerateEdgesOnCut = true;

			Boolean.Compute();
			if (Boolean.Result != nullptr)
			{
				if (CountValidTriangles(*Boolean.Result) >= Options.MinTriangleCount)
				{
					PlanarCutPiece Piece;
					Piece.Mesh = *Boolean.Result;
					Piece.Mesh.FixTriangleOrientation(false);
					Piece.Mesh.CalculateWeightAverageNormals();
					Piece.Mesh.BuildBounds();
					Piece.Center = Piece.Mesh.GetBounds().GetCenter();
					Piece.CellIndex = CellMeshIdx == CellMeshes.OutsideCellIndex ? -1 : CellMeshIdx;
					Pieces.push_back(Piece);
				}

				delete Boolean.Result;
				Boolean.Result = nullptr;
			}
		}

		BuildPlanarCutPieceConnectivity(Cells, Pieces);
		return !Pieces.empty();
	}

	bool MeshCut::CutWithPlanarCells(
		const DynamicMesh& SourceMesh,
		const PlanarCells& Cells,
		std::vector<DynamicMesh>& Pieces,
		const PlanarCutOptions& Options)
	{
		Pieces.clear();

		std::vector<PlanarCutPiece> CutPieces;
		const bool bSuccess = CutWithPlanarCells(SourceMesh, Cells, CutPieces, Options);
		for (const PlanarCutPiece& Piece : CutPieces)
		{
			Pieces.push_back(Piece.Mesh);
		}
		return bSuccess;
	}

	bool MeshCut::CreateCuttingSurfacePreview(
		const PlanarCells& Cells,
		const Box3& DomainBounds,
		DynamicMesh& PreviewMesh,
		const PlanarCutOptions& Options)
	{
		PreviewMesh.Clear();

		PlanarCellMeshes CellMeshes;
		if (!CellMeshes.Build(Cells, DomainBounds, Options))
		{
			return false;
		}

		for (const DynamicMesh& CellMesh : CellMeshes.Meshes)
		{
			if (CountValidTriangles(CellMesh) > 0)
			{
				AppendMesh(PreviewMesh, CellMesh);
			}
		}

		PreviewMesh.BuildBounds();
		PreviewMesh.CalculateWeightAverageNormals();
		return CountValidTriangles(PreviewMesh) > 0;
	}

}	// namespace Riemann
