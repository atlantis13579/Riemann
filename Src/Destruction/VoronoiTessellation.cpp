
#include "VoronoiTessellation.h"

#include "../CollisionPrimitive/Triangle3.h"
#include "../Maths/Box1.h"
#include "../Maths/Box2.h"
#include "../Maths/Maths.h"
#include "../Maths/Frame3.h"
#include "../Geometry/Delaunay.h"
#include "../Geometry/DynamicMesh.h"
#include "../Geometry/GeometryAttributes.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>

namespace Riemann
{
	static int EncodePlaneToMaterial(int PlaneIdx)
	{
		return -(PlaneIdx + 1);
	}

	namespace
	{
		static int CountValidTriangles(const DynamicMesh* Mesh)
		{
			if (!Mesh)
			{
				return 0;
			}

			int Count = 0;
			for (int TID = 0; TID < Mesh->GetTriangleCount(); ++TID)
			{
				if (Mesh->IsTriangleFast(TID))
				{
					++Count;
				}
			}
			return Count;
		}

		static void EnsureVoronoiMeshAttributes(DynamicMesh* Mesh, int NumUVLayers)
		{
			if (!Mesh)
			{
				return;
			}
			if (!DynamicMeshAttributeSet::IsAugmented(Mesh))
			{
				DynamicMeshAttributeSet::Augment(Mesh, NumUVLayers);
			}
			else
			{
				DynamicMeshAttributeSet::EnableUVChannels(Mesh, NumUVLayers, false, false);
				if (!Mesh->Attributes()->HasMaterialID())
				{
					Mesh->Attributes()->EnableMaterialID();
				}
			}
		}

		static void MakePlaneBasis(const Vector3& InNormal, Vector3& AxisX, Vector3& AxisY)
		{
			Vector3 Normal = InNormal.SafeUnit();
			if (Normal.IsZero())
			{
				Normal = Vector3::UnitZ();
			}

			const Vector3 Reference = std::fabs(Normal.z) < 0.9f ? Vector3::UnitZ() : Vector3::UnitY();
			AxisX = Reference.Cross(Normal).SafeUnit();
			if (AxisX.IsZero())
			{
				AxisX = Vector3::UnitX();
			}
			AxisY = Normal.Cross(AxisX).SafeUnit();
			if (AxisY.IsZero())
			{
				AxisY = Vector3::UnitY();
			}
		}

		static Vector2 ProjectToPlaneUV(const Vector3& Position, const Vector3& Origin, const Vector3& AxisX, const Vector3& AxisY)
		{
			const Vector3 Delta = Position - Origin;
			return Vector2(Delta.Dot(AxisX), Delta.Dot(AxisY));
		}

		static Vector3 ComputePolygonNormal(const std::vector<Vector3>& Polygon)
		{
			if (Polygon.size() < 3)
			{
				return Vector3::Zero();
			}

			Vector3 Center = Vector3::Zero();
			for (const Vector3& Point : Polygon)
			{
				Center += Point;
			}
			Center /= (float)Polygon.size();

			Vector3 AreaNormal = Vector3::Zero();
			for (size_t Idx = 0; Idx < Polygon.size(); ++Idx)
			{
				const Vector3& A = Polygon[Idx];
				const Vector3& B = Polygon[(Idx + 1) % Polygon.size()];
				AreaNormal += (A - Center).Cross(B - Center);
			}
			return AreaNormal.SafeUnit();
		}

		static void AddUniquePoint(std::vector<Vector3>& Points, const Vector3& Point, float Epsilon)
		{
			const float EpsilonSqr = Epsilon * Epsilon;
			for (const Vector3& Existing : Points)
			{
				if ((Existing - Point).SquareLength() <= EpsilonSqr)
				{
					return;
				}
			}
			Points.push_back(Point);
		}

		static std::vector<Vector3> CleanPolygon(const std::vector<Vector3>& Polygon, float Epsilon)
		{
			std::vector<Vector3> Cleaned;
			const float EpsilonSqr = Epsilon * Epsilon;
			for (const Vector3& Point : Polygon)
			{
				if (Cleaned.empty() || (Cleaned.back() - Point).SquareLength() > EpsilonSqr)
				{
					Cleaned.push_back(Point);
				}
			}
			if (Cleaned.size() > 1 && (Cleaned.front() - Cleaned.back()).SquareLength() <= EpsilonSqr)
			{
				Cleaned.pop_back();
			}
			return Cleaned;
		}

		static int AppendConvexPolygon(
			DynamicMesh* Mesh,
			const std::vector<Vector3>& InPolygon,
			const Vector3& InNormal,
			int MaterialID,
			int NumUVLayers)
		{
			if (!Mesh)
			{
				return 0;
			}

			std::vector<Vector3> Polygon = CleanPolygon(InPolygon, 1e-6f);
			if (Polygon.size() < 3)
			{
				return 0;
			}

			Vector3 Normal = InNormal.SafeUnit();
			if (Normal.IsZero())
			{
				Normal = ComputePolygonNormal(Polygon);
			}
			if (Normal.IsZero())
			{
				return 0;
			}

			const Vector3 PolygonNormal = ComputePolygonNormal(Polygon);
			if (!PolygonNormal.IsZero() && PolygonNormal.Dot(Normal) < 0.0f)
			{
				std::reverse(Polygon.begin(), Polygon.end());
			}

			EnsureVoronoiMeshAttributes(Mesh, NumUVLayers);

			Vector3 AxisX;
			Vector3 AxisY;
			MakePlaneBasis(Normal, AxisX, AxisY);
			const Vector3 Origin = Polygon[0];

			Vector2 MinUV(FLT_MAX, FLT_MAX);
			for (const Vector3& Position : Polygon)
			{
				const Vector2 UV = ProjectToPlaneUV(Position, Origin, AxisX, AxisY);
				MinUV.x = std::min(MinUV.x, UV.x);
				MinUV.y = std::min(MinUV.y, UV.y);
			}

			VertexInfo Info;
			Info.bHasColor = true;
			Info.bHasNormal = true;
			Info.bHasUV = true;
			Info.Color = Vector3::Zero();
			Info.Normal = Normal;

			const int VertexStart = Mesh->GetVertexCount();
			for (const Vector3& Position : Polygon)
			{
				Info.Position = Position;
				Info.UVs = ProjectToPlaneUV(Position, Origin, AxisX, AxisY) - MinUV;
				const int VID = Mesh->AppendVertex(Info);
				DynamicMeshAttributeSet::SetVertexColor(Mesh, VID, Vector4::Zero());
				DynamicMeshAttributeSet::SetAllUV(Mesh, VID, Info.UVs, NumUVLayers);
			}

			int AddedTriangles = 0;
			for (int V1 = 1, V2 = 2; V2 < (int)Polygon.size(); V1 = V2++)
			{
				const int TID = Mesh->AppendTriangle(Index3(VertexStart, VertexStart + V1, VertexStart + V2));
				if (TID > -1)
				{
					Mesh->Attributes()->GetMaterialID()->SetNewValue(TID, MaterialID);
					++AddedTriangles;
				}
			}
			return AddedTriangles;
		}

		static void SortPolygonOnPlane(std::vector<Vector3>& Polygon, const Vector3& Normal)
		{
			if (Polygon.size() < 3)
			{
				return;
			}

			Vector3 Center = Vector3::Zero();
			for (const Vector3& Point : Polygon)
			{
				Center += Point;
			}
			Center /= (float)Polygon.size();

			Vector3 AxisX;
			Vector3 AxisY;
			MakePlaneBasis(Normal, AxisX, AxisY);
			std::sort(Polygon.begin(), Polygon.end(),
				[Center, AxisX, AxisY](const Vector3& A, const Vector3& B)
				{
					const Vector3 DA = A - Center;
					const Vector3 DB = B - Center;
					const float AngleA = std::atan2(DA.Dot(AxisY), DA.Dot(AxisX));
					const float AngleB = std::atan2(DB.Dot(AxisY), DB.Dot(AxisX));
					return AngleA < AngleB;
				});
		}

		static std::vector<Vector3> ClipPolygonToHalfspace(
			const std::vector<Vector3>& Polygon,
			const Vector3& Normal,
			float PlaneW,
			bool bKeepPositive,
			float Epsilon,
			std::vector<Vector3>& CutPoints)
		{
			std::vector<Vector3> Result;
			if (Polygon.empty())
			{
				return Result;
			}

			auto SignedDistance = [Normal, PlaneW](const Vector3& Point)
			{
				return Normal.Dot(Point) - PlaneW;
			};
			auto IsInside = [bKeepPositive, Epsilon](float Distance)
			{
				return bKeepPositive ? Distance >= -Epsilon : Distance <= Epsilon;
			};

			Vector3 Previous = Polygon.back();
			float PreviousDistance = SignedDistance(Previous);
			bool bPreviousInside = IsInside(PreviousDistance);

			for (const Vector3& Current : Polygon)
			{
				const float CurrentDistance = SignedDistance(Current);
				const bool bCurrentInside = IsInside(CurrentDistance);

				if (bPreviousInside != bCurrentInside)
				{
					const float Denominator = PreviousDistance - CurrentDistance;
					if (std::fabs(Denominator) > 1e-8f)
					{
						float T = PreviousDistance / Denominator;
						T = std::max(0.0f, std::min(1.0f, T));
						const Vector3 Intersection = Previous + (Current - Previous) * T;
						Result.push_back(Intersection);
						AddUniquePoint(CutPoints, Intersection, Epsilon * 4.0f);
					}
				}

				if (bCurrentInside)
				{
					Result.push_back(Current);
					if (std::fabs(CurrentDistance) <= Epsilon)
					{
						AddUniquePoint(CutPoints, Current, Epsilon * 4.0f);
					}
				}

				Previous = Current;
				PreviousDistance = CurrentDistance;
				bPreviousInside = bCurrentInside;
			}

			return CleanPolygon(Result, Epsilon);
		}

		static void FinalizeVoronoiMesh(DynamicMesh* Mesh)
		{
			if (!Mesh)
			{
				return;
			}
			Mesh->BuildBounds();
			if (CountValidTriangles(Mesh) > 0)
			{
				Mesh->CalculateWeightAverageNormals();
			}
		}

		static void BuildClippedBoxHalfspaceMesh(
			DynamicMesh* Mesh,
			const Box3& Bounds,
			const Vector3& Normal,
			float PlaneW,
			bool bKeepPositive,
			int MaterialID,
			int NumUVLayers)
		{
			if (!Mesh)
			{
				return;
			}

			Mesh->Clear();
			EnsureVoronoiMeshAttributes(Mesh, NumUVLayers);

			Vector3 Corners[8];
			Box3::GetVertices(Bounds.Min, Bounds.Max, Corners);

			const std::array<std::array<int, 4>, 6> FaceIndices =
			{ {
				{ 0, 2, 3, 1 },
				{ 4, 5, 7, 6 },
				{ 0, 1, 5, 4 },
				{ 2, 6, 7, 3 },
				{ 0, 4, 6, 2 },
				{ 1, 3, 7, 5 }
			} };
			const std::array<Vector3, 6> FaceNormals =
			{ {
				-Vector3::UnitZ(),
				Vector3::UnitZ(),
				-Vector3::UnitY(),
				Vector3::UnitY(),
				-Vector3::UnitX(),
				Vector3::UnitX()
			} };

			const float Epsilon = std::max(Bounds.MaxDim() * 1e-5f, 1e-6f);
			std::vector<Vector3> CutPoints;
			for (size_t FaceIdx = 0; FaceIdx < FaceIndices.size(); ++FaceIdx)
			{
				std::vector<Vector3> FacePolygon;
				for (int CornerIdx : FaceIndices[FaceIdx])
				{
					FacePolygon.push_back(Corners[CornerIdx]);
				}

				const std::vector<Vector3> Clipped = ClipPolygonToHalfspace(
					FacePolygon, Normal, PlaneW, bKeepPositive, Epsilon, CutPoints);
				AppendConvexPolygon(Mesh, Clipped, FaceNormals[FaceIdx], MaterialID, NumUVLayers);
			}

			CutPoints = CleanPolygon(CutPoints, Epsilon);
			if (CutPoints.size() >= 3)
			{
				const Vector3 CapNormal = bKeepPositive ? -Normal : Normal;
				SortPolygonOnPlane(CutPoints, CapNormal);
				AppendConvexPolygon(Mesh, CutPoints, CapNormal, MaterialID, NumUVLayers);
			}

			FinalizeVoronoiMesh(Mesh);
		}

		static float HashNoise(const Vector3& Position)
		{
			const float N = std::sin(Position.x * 12.9898f + Position.y * 78.233f + Position.z * 37.719f) * 43758.5453f;
			return N - std::floor(N);
		}

		static void ApplyDeterministicSurfaceNoise(DynamicMesh* Mesh)
		{
			if (!Mesh || CountValidTriangles(Mesh) == 0)
			{
				return;
			}

			Mesh->BuildBounds();
			const Box3& Bounds = Mesh->GetBounds();
			const float Amplitude = std::max(Bounds.MaxDim() * 0.008f, 1e-5f);
			const Vector3 Center = Bounds.GetCenter();

			for (int VID = 0; VID < Mesh->GetVertexCount(); ++VID)
			{
				if (!Mesh->IsVertexFast(VID))
				{
					continue;
				}

				const Vector3 Position = Mesh->GetVertex(VID);
				Vector3 Direction = Mesh->GetVertexNormal(VID).SafeUnit();
				if (Direction.IsZero())
				{
					Direction = (Position - Center).SafeUnit();
				}
				if (Direction.IsZero())
				{
					Direction = Vector3::UnitZ();
				}

				const float RandomValue = HashNoise(Position) * 2.0f - 1.0f;
				const float WaveValue = std::sin(Position.x * 9.17f + Position.y * 5.31f + Position.z * 7.43f);
				const float Offset = Amplitude * (RandomValue * 0.65f + WaveValue * 0.35f);
				Mesh->SetVertex(VID, Position + Direction * Offset);
			}

			FinalizeVoronoiMesh(Mesh);
		}
	}

	VoronoiMesh::VoronoiMesh(const std::vector<Vector3>& points, const Box3& bounds, const float eps)
	{
		mDomainBounds = bounds;

		Voronoi3 v(points, bounds, eps);
		v.Build();

		int NumCells = v.GetNumPoints();
		mMeshs.resize(NumCells);
		for (size_t i = 0; i < NumCells; ++i)
		{
			mMeshs[i] = new DynamicMesh();
			EnsureVoronoiMeshAttributes(mMeshs[i], NumUVLayers);
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
		for (DynamicMesh* Mesh : mMeshs)
		{
			delete Mesh;
		}
		mMeshs.clear();
	}

	const DynamicMesh* VoronoiMesh::GetMesh(int index) const
	{
		if (index < 0 || index >= (int)mMeshs.size())
		{
			return nullptr;
		}
		return mMeshs[index];
	}

	int VoronoiMesh::GetTotalTriangleCount() const
	{
		int Count = 0;
		for (const DynamicMesh* Mesh : mMeshs)
		{
			Count += CountValidTriangles(Mesh);
		}
		return Count;
	}

	void VoronoiMesh::BuildMesh_SinglePlane(const Voronoi3& v)
	{
		if (v.mPlanes.empty() || mMeshs.size() < 2)
		{
			return;
		}

		const Voronoi3::Plane& Plane = v.mPlanes[0];
		Vector3 Normal = Plane.GetNormal();
		const float NormalLength = Normal.SafeNormalize();
		if (NormalLength <= 0.0f)
		{
			return;
		}

		const float PlaneW = Plane.w / NormalLength;
		const int MaterialID = EncodePlaneToMaterial(0);
		BuildClippedBoxHalfspaceMesh(mMeshs[0], mDomainBounds, Normal, PlaneW, false, MaterialID, NumUVLayers);
		BuildClippedBoxHalfspaceMesh(mMeshs[1], mDomainBounds, Normal, PlaneW, true, MaterialID, NumUVLayers);
	}

	void VoronoiMesh::BuildMesh_WithoutNoise(const Voronoi3& v)
	{
		for (DynamicMesh* Mesh : mMeshs)
		{
			EnsureVoronoiMeshAttributes(Mesh, NumUVLayers);
		}

		for (size_t i = 0; i < v.mCells.size(); ++i)
		{
			if (i >= v.mPlanes.size() || i >= v.mBoundaries.size())
			{
				continue;
			}

			const std::pair<int, int>& p = v.mCells[i];
			if (p.first < 0 || p.first >= (int)mMeshs.size())
			{
				continue;
			}

			const std::vector<int>& PlaneBoundary = v.mBoundaries[i];
			if (PlaneBoundary.size() < 3)
			{
				continue;
			}
			std::vector<Vector3> BoundaryPositions;
			BoundaryPositions.reserve(PlaneBoundary.size());
			for (int BoundaryVertex : PlaneBoundary)
			{
				if (BoundaryVertex >= 0 && BoundaryVertex < (int)v.mBoundaryVertices.size())
				{
					BoundaryPositions.push_back(v.mBoundaryVertices[BoundaryVertex]);
				}
			}
			if (BoundaryPositions.size() < 3)
			{
				continue;
			}

			DynamicMesh* Meshes[2] = { 0 };
			Meshes[0] = mMeshs[p.first];
			Meshes[1] = nullptr;

			int OtherCell = p.second < 0 ? OutsideCellIndex : p.second;
			if (OtherCell >= (int)mMeshs.size())
			{
				OtherCell = OutsideCellIndex;
			}
			int NumMeshes = OtherCell < 0 ? 1 : 2;
			if (NumMeshes == 2)
			{
				Meshes[1] = mMeshs[OtherCell];
			}

			Vector3 Normal = v.mPlanes[i].GetNormal();
			const float NormalLength = Normal.SafeNormalize();
			if (NormalLength <= 0.0f)
			{
				continue;
			}
			const Vector3 Origin = Normal * (v.mPlanes[i].w / NormalLength);

			Vector3 AxisX;
			Vector3 AxisY;
			MakePlaneBasis(Normal, AxisX, AxisY);

			Vector2 MinUV(FLT_MAX, FLT_MAX);
			for (const Vector3& Position : BoundaryPositions)
			{
				const Vector2 UV = ProjectToPlaneUV(Position, Origin, AxisX, AxisY);
				MinUV.x = std::min(UV.x, MinUV.x);
				MinUV.y = std::min(UV.y, MinUV.y);
			}
			if (MinUV.x == FLT_MAX || MinUV.y == FLT_MAX)
			{
				continue;
			}

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
				for (const Vector3& Position : BoundaryPositions)
				{
					info.Position = Position;
					info.Color = Vector3::InfMax();
					info.UVs = ProjectToPlaneUV(info.Position, Origin, AxisX, AxisY) - MinUV;
					int VID = Meshes[j]->AppendVertex(info);
					DynamicMeshAttributeSet::SetVertexColor(Meshes[j], VID, Vector4::Zero());
					DynamicMeshAttributeSet::SetAllUV(Meshes[j], VID, info.UVs, NumUVLayers);
				}
			}

			int MID = EncodePlaneToMaterial((int)i);
			if (AssumeConvexCells)
			{
				// put a fan
				for (int V0 = 0, V1 = 1, V2 = 2; V2 < (int)BoundaryPositions.size(); V1 = V2++)
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
				for (int V = 0; V < (int)BoundaryPositions.size(); ++V)
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

		for (DynamicMesh* Mesh : mMeshs)
		{
			FinalizeVoronoiMesh(Mesh);
		}
	}

	void VoronoiMesh::BuildMesh_WithNoise(const Voronoi3& v)
	{
		BuildMesh_WithoutNoise(v);
		for (DynamicMesh* Mesh : mMeshs)
		{
			ApplyDeterministicSurfaceNoise(Mesh);
		}
	}
}	// namespace Riemann
