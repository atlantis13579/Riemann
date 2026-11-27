
#include "MeshCut.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <initializer_list>
#include <map>
#include <random>
#include <set>

#include "DynamicMesh.h"
#include "Delaunay.h"
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

		// Cap a single convex contour without invoking the constrained triangulator. Recursive
		// fracture pieces are convex, so this avoids platform-sensitive predicates for the hot path.
		static bool CapConvexPlaneBoundary(
			DynamicMesh& Mesh,
			const std::vector<int>& Vertices,
			const std::vector<std::vector<int>>& Adjacency,
			const PlanarCutPlane& Plane)
		{
			if (Vertices.size() < 3 || Vertices.size() != Adjacency.size())
			{
				return false;
			}

			std::vector<int> Loop;
			Loop.reserve(Vertices.size());
			int Previous = -1;
			int Current = 0;
			do
			{
				if (Current < 0 || Current >= (int)Vertices.size() || Loop.size() >= Vertices.size())
				{
					return false;
				}
				Loop.push_back(Current);
				const std::vector<int>& Neighbors = Adjacency[(size_t)Current];
				if (Neighbors.size() != 2)
				{
					return false;
				}
				const int Next = Neighbors[0] == Previous ? Neighbors[1] : Neighbors[0];
				Previous = Current;
				Current = Next;
			} while (Current != Loop[0]);

			// Multiple contours use the constrained fallback below. This path is selected
			// only for convex source/cell clipping, so its single contour is star-shaped.
			if (Loop.size() != Vertices.size())
			{
				return false;
			}

			Vector3 Center = Vector3::Zero();
			for (int LocalVertex : Loop)
			{
				Center += Mesh.GetVertex(Vertices[(size_t)LocalVertex]);
			}
			Center /= (float)Loop.size();
			Center -= Plane.Normal * Plane.SignedDistance(Center);
			if (!std::isfinite(Center.x) || !std::isfinite(Center.y) || !std::isfinite(Center.z))
			{
				return false;
			}

			bool Flip = false;
			bool FoundRim = false;
			for (size_t Index = 0; Index < Loop.size() && !FoundRim; ++Index)
			{
				const int A = Vertices[(size_t)Loop[Index]];
				const int B = Vertices[(size_t)Loop[(Index + 1) % Loop.size()]];
				const int Edge = Mesh.FindEdge(A, B);
				if (Edge < 0 || !Mesh.IsBoundaryEdge(Edge))
				{
					continue;
				}
				const int TriangleID = Mesh.GetEdgeT(Edge).a;
				if (!Mesh.IsTriangle(TriangleID))
				{
					return false;
				}
				const Index3 Triangle = Mesh.GetTriangle(TriangleID);
				for (int Corner = 0; Corner < 3; ++Corner)
				{
					if (Triangle[Corner] == A && Triangle[(Corner + 1) % 3] == B)
					{
						Flip = true;
					}
				}
				FoundRim = true;
			}
			if (!FoundRim)
			{
				return false;
			}

			const int CenterVertex = Mesh.AppendVertex(Center);
			for (size_t Index = 0; Index < Loop.size(); ++Index)
			{
				int A = Vertices[(size_t)Loop[Index]];
				int B = Vertices[(size_t)Loop[(Index + 1) % Loop.size()]];
				if (Flip)
				{
					std::swap(A, B);
				}
				if (!Mesh.IsVertex(A) || !Mesh.IsVertex(B) || !Mesh.IsVertex(CenterVertex))
				{
					return false;
				}
				if (Mesh.AppendTriangle(A, B, CenterVertex) < 0)
				{
					return false;
				}
			}
			return true;
		}

		// Triangulate non-convex or multiple planar contours together so holes survive.
		static bool CapPlaneBoundary(
			DynamicMesh& Mesh,
			const std::vector<bool>& OnPlane,
			const PlanarCutPlane& Plane,
			float Tolerance,
			bool UseConvexCap)
		{
			std::vector<int> vertices;
			std::vector<int> index(Mesh.GetVertexCount(), -1);
			std::vector<Vector2> points;
			std::vector<DelaunayEdge> edges;
			const Vector3 normal = Plane.Normal;
			int drop = 0;
			for (int d = 1; d < 3; ++d)
				if (std::abs(normal[d]) > std::abs(normal[drop])) drop = d;
			for (int e = 0; e < Mesh.GetEdgeCount(); ++e)
			{
				if (!Mesh.IsEdgeFast(e) || !Mesh.IsBoundaryEdge(e)) continue;
				const Index2 v = Mesh.GetEdgeV(e);
				if (!Mesh.IsVertex(v.a) || !Mesh.IsVertex(v.b) ||
					(size_t)v.a >= OnPlane.size() || (size_t)v.b >= OnPlane.size())
				{
					return false;
				}
				if (!OnPlane[v.a] || !OnPlane[v.b]) continue;
				for (int id : {v.a, v.b})
				{
					if (index[id] >= 0) continue;
					index[id] = (int)vertices.size();
					vertices.push_back(id);
					const Vector3 p = Mesh.GetVertex(id);
					points.emplace_back(p[(drop + 1) % 3], p[(drop + 2) % 3]);
				}
				edges.emplace_back(index[v.a], index[v.b]);
			}
			if (edges.empty()) return true;
			if (edges.size() < 3) return false;
			std::vector<std::vector<int>> adjacency(vertices.size());
			for (const DelaunayEdge& edge : edges)
			{
				if (edge.v1 < 0 || edge.v2 < 0 || edge.v1 == edge.v2 ||
					(size_t)edge.v1 >= vertices.size() || (size_t)edge.v2 >= vertices.size())
				{
					return false;
				}
				adjacency[(size_t)edge.v1].push_back(edge.v2);
				adjacency[(size_t)edge.v2].push_back(edge.v1);
			}
			for (const std::vector<int>& neighbors : adjacency)
			{
				if (neighbors.size() != 2)
				{
					return false;
				}
			}

			std::vector<bool> connected(vertices.size(), false);
			std::vector<int> stack(1, 0);
			connected[0] = true;
			size_t connectedCount = 0;
			while (!stack.empty())
			{
				const int current = stack.back();
				stack.pop_back();
				++connectedCount;
				for (int neighbor : adjacency[(size_t)current])
				{
					if (!connected[(size_t)neighbor])
					{
						connected[(size_t)neighbor] = true;
						stack.push_back(neighbor);
					}
				}
			}
			if (UseConvexCap && connectedCount == vertices.size())
			{
				DynamicMesh convexCapped = Mesh;
				if (!CapConvexPlaneBoundary(convexCapped, vertices, adjacency, Plane))
				{
					return false;
				}
				Mesh = std::move(convexCapped);
				return true;
			}

			// A planar constraint graph cannot contain distinct vertices at the same point.
			std::set<std::pair<float, float>> unique;
			for (const auto& p : points)
				if (!std::isfinite(p.x) || !std::isfinite(p.y) || !unique.emplace(p.x, p.y).second) return false;
			ConstrainedDelaunay triangulation;
			if (!triangulation.Triangulate(points, edges)) return false;
			for (const DelaunayTriangle& triangle : triangulation.Triangles)
			{
				for (int Corner = 0; Corner < 3; ++Corner)
				{
					if (triangle[Corner] < 0 || (size_t)triangle[Corner] >= vertices.size())
					{
						return false;
					}
				}
				if (triangle.v1 == triangle.v2 || triangle.v2 == triangle.v3 || triangle.v3 == triangle.v1)
				{
					return false;
				}
			}
			// Match the source winding along the rim. The triangulator gives every
			// cap triangle the same planar winding, including contours around holes.
			bool flip = false, foundRim = false;
			for (const auto& t : triangulation.Triangles)
			{
				for (int k = 0; k < 3; ++k)
				{
					const int a = vertices[t[k]], b = vertices[t[(k + 1) % 3]];
					const int edge = Mesh.FindEdge(a, b);
					if (edge < 0 || !Mesh.IsBoundaryEdge(edge)) continue;
					const Index3 face = Mesh.GetTriangle(Mesh.GetEdgeT(edge).a);
					for (int j = 0; j < 3; ++j)
						if (face[j] == a && face[(j + 1) % 3] == b) flip = true;
					foundRim = true;
					break;
				}
				if (foundRim) break;
			}
			for (const auto& t : triangulation.Triangles)
			{
				int a = vertices[t.v1], b = vertices[t.v2], c = vertices[t.v3];
				if (flip) std::swap(b, c);
				if (!Mesh.IsVertex(a) || !Mesh.IsVertex(b) || !Mesh.IsVertex(c)) return false;
				if (Mesh.AppendTriangle(a, b, c) < 0) return false;
			}
			return true;
		}

		static bool ClipToPlane(
			DynamicMesh& Mesh,
			const PlanarCutPlane& Plane,
			float Tolerance,
			bool Cap,
			bool UseConvexCap)
		{
			std::vector<double> distances(Mesh.GetVertexCount());
			bool inside = false, outside = false;
			for (int v = 0; v < Mesh.GetVertexCount(); ++v)
			{
				if (!Mesh.IsReferencedVertex(v)) continue;
				const Vector3 p = Mesh.GetVertex(v);
				const double d = (double)Plane.Normal.x * p.x + (double)Plane.Normal.y * p.y +
					(double)Plane.Normal.z * p.z - Plane.W;
				if (!std::isfinite(d)) return false;
				distances[v] = std::abs(d) <= Tolerance ? 0.0f : d;
				inside |= distances[v] < 0;
				outside |= distances[v] > 0;
			}
			if (!outside) return true;
			if (!inside) { Mesh.Clear(); return true; }
			DynamicMesh clipped;
			std::vector<int> kept(Mesh.GetVertexCount(), -1), split(Mesh.GetEdgeCount(), -1);
			std::vector<bool> onPlane;
			for (int tid = 0; tid < Mesh.GetTriangleCount(); ++tid)
			{
				if (!Mesh.IsTriangleFast(tid)) continue;
				const Index3 tri = Mesh.GetTriangle(tid);
				for (int Corner = 0; Corner < 3; ++Corner)
				{
					if (!Mesh.IsVertex(tri[Corner])) return false;
				}
				int polygon[4], n = 0;
				for (int i = 0; i < 3; ++i)
				{
					const int a = tri[i], b = tri[(i + 1) % 3];
					if (distances[a] <= 0)
					{
						if (kept[a] < 0)
						{
							Vector3 position = Mesh.GetVertex(a);
							if (distances[a] == 0.0)
							{
								position -= Plane.Normal * Plane.SignedDistance(position);
							}
							kept[a] = clipped.AppendVertex(position);
							onPlane.push_back(distances[a] == 0);
						}
						polygon[n++] = kept[a];
					}
					if (distances[a] == 0 || distances[b] == 0 || (distances[a] < 0) == (distances[b] < 0)) continue;
					const int edge = Mesh.GetTriEdges(tid, i);
					if (edge < 0 || (size_t)edge >= split.size()) return false;
					int& intersection = split[(size_t)edge];
					if (intersection < 0)
					{
						const double fraction = distances[a] / (distances[a] - distances[b]);
						const Vector3 start = Mesh.GetVertex(a);
						const Vector3 end = Mesh.GetVertex(b);
						Vector3 point;
						for (int Axis = 0; Axis < 3; ++Axis)
						{
							point[Axis] = (float)((double)start[Axis] + ((double)end[Axis] - start[Axis]) * fraction);
						}
						point -= Plane.Normal * Plane.SignedDistance(point);
						intersection = clipped.AppendVertex(point);
						onPlane.push_back(true);
					}
					polygon[n++] = intersection;
				}
				for (int i = 2; i < n; ++i)
				{
					if (!clipped.IsVertex(polygon[0]) || !clipped.IsVertex(polygon[i - 1]) || !clipped.IsVertex(polygon[i])) return false;
					if (clipped.AppendTriangle(polygon[0], polygon[i - 1], polygon[i]) < 0) return false;
				}
			}
			if (Cap && !CapPlaneBoundary(clipped, onPlane, Plane, Tolerance, UseConvexCap)) return false;
			Mesh = std::move(clipped);
			return true;
		}

		static bool CutConvexCells(const DynamicMesh& Source, const PlanarCells& Cells,
			const PlanarCutOptions& Options, std::vector<PlanarCutPiece>& Pieces)
		{
			const bool useConvexCap = Options.AssumeConvexSource;
			std::vector<std::vector<PlanarCutPlane>> boundaries(Cells.NumCells);
			std::vector<std::set<int>> boundaryVertices(Cells.NumCells);
			for (size_t i = 0; i < Cells.Planes.size(); ++i)
			{
				const Index2 pair = Cells.PlaneCells[i];
				const PlanarCutPlane& p = Cells.Planes[i];
				if (pair.a >= 0 && pair.a < Cells.NumCells) boundaries[(size_t)pair.a].push_back(p);
				if (pair.b >= 0 && pair.b < Cells.NumCells) boundaries[(size_t)pair.b].emplace_back(-p.Normal, -p.W);
				if (i < Cells.PlaneBoundaries.size())
				{
					for (int vertexID : Cells.PlaneBoundaries[i])
					{
						if (pair.a >= 0 && pair.a < Cells.NumCells) boundaryVertices[(size_t)pair.a].insert(vertexID);
						if (pair.b >= 0 && pair.b < Cells.NumCells) boundaryVertices[(size_t)pair.b].insert(vertexID);
					}
				}
			}

			std::vector<bool> discardCell((size_t)Cells.NumCells, false);
			if (Options.Grout > 0.0f && !Cells.IsInfinitePlane())
			{
				// PlanarCellMeshes applies grout by uniformly scaling each cell around its
				// vertex centroid. Transforming the supporting planes is equivalent and
				// keeps the convex clipping path independent of mesh booleans.
				for (int cellIndex = 0; cellIndex < Cells.NumCells; ++cellIndex)
				{
					const std::set<int>& vertexIDs = boundaryVertices[(size_t)cellIndex];
					if (boundaries[(size_t)cellIndex].empty()) continue;
					if (vertexIDs.empty()) return false;

					std::map<QuantizedPointKey, Vector3> uniquePositions;
					const float vertexTolerance = std::max(Options.SnapTolerance, 1e-6f);
					for (int vertexID : vertexIDs)
					{
						if (vertexID < 0 || vertexID >= (int)Cells.PlaneBoundaryVertices.size()) return false;
						const Vector3& position = Cells.PlaneBoundaryVertices[(size_t)vertexID];
						if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) return false;
						uniquePositions.emplace(MakePointKey(position, vertexTolerance), position);
					}
					if (uniquePositions.empty()) return false;

					Box3 bounds = Box3::Empty();
					Vector3 center = Vector3::Zero();
					for (const auto& entry : uniquePositions)
					{
						const Vector3& position = entry.second;
						bounds.Encapsulate(position);
						center += position;
					}
					center /= (float)uniquePositions.size();
					const float boundsSize = BoundsMaxDim(bounds);
					const float scale = boundsSize > 0.0f ? (boundsSize - Options.Grout * 0.5f) / boundsSize : 0.0f;
					if (scale <= 1e-4f)
					{
						discardCell[(size_t)cellIndex] = true;
						continue;
					}
					for (PlanarCutPlane& plane : boundaries[(size_t)cellIndex])
					{
						plane.W = scale * plane.W + (1.0f - scale) * plane.Normal.Dot(center);
					}
				}
			}
			for (int i = 0; i < Cells.NumCells; ++i)
			{
				if (boundaries[(size_t)i].empty() || discardCell[(size_t)i]) continue;
				PlanarCutPiece piece;
				piece.Mesh = Source;
				for (const PlanarCutPlane& plane : boundaries[(size_t)i])
				{
					if (!ClipToPlane(piece.Mesh, plane, Options.SnapTolerance, Options.CapCutFaces, useConvexCap))
					{
						Pieces.clear();
						return false;
					}
					if (piece.Mesh.GetTriangleCount() == 0) break;
				}
				if (CountValidTriangles(piece.Mesh) < std::max(1, Options.MinTriangleCount)) continue;
				piece.Mesh.CalculateWeightAverageNormals();
				piece.Mesh.BuildBounds();
				piece.Center = piece.Mesh.GetBounds().GetCenter();
				piece.CellIndex = i;
				Pieces.push_back(std::move(piece));
			}
			BuildPlanarCutPieceConnectivity(Cells, Pieces);
			return !Pieces.empty();
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

			GeometryBoolean Boolean(&SourceMesh, &HalfSpaceMesh, Options.CapCutFaces ?
				GeometryBoolean::BooleanOp::Intersect : GeometryBoolean::BooleanOp::TrimToInterior);
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

		if (Options.ClipConvexCells && !PlaneNormal.SafeUnit().IsZero())
		{
			std::vector<PlanarCutPiece> pieces;
			const bool cut = CutWithPlanarCells(SourceMesh, PlanarCells(PlanarCutPlane(PlaneNormal, PlaneOrigin)), pieces, Options);
			if (NegativeSide) NegativeSide->Clear();
			if (PositiveSide) PositiveSide->Clear();
			for (const PlanarCutPiece& piece : pieces)
			{
				DynamicMesh* side = piece.CellIndex == 0 ? NegativeSide : PositiveSide;
				if (side) *side = piece.Mesh;
			}
			return cut;
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

		// Boolean winding classification requires both closed inputs to use the same orientation.
		DynamicMesh OrientedSourceMesh = SourceMesh;
		OrientedSourceMesh.FixTriangleOrientation(true);

		if (Options.ClipConvexCells && (Cells.AssumeConvexCells || Cells.IsInfinitePlane()) && !Cells.SurfaceSettings.EnableNoise &&
			!Options.IncludeOutsideCell && Options.WeldSharedEdges && !Options.SimplifyAlongCut &&
			!SourceMesh.HasVertexUVs() && !SourceMesh.HasVertexColors() && !SourceMesh.HasTriangleGroups())
			return CutConvexCells(OrientedSourceMesh, Cells, Options, Pieces);

		Box3 DomainBounds = OrientedSourceMesh.GetBounds();
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

			DynamicMesh OrientedCellMesh = CellMesh;
			OrientedCellMesh.FixTriangleOrientation(true);

			GeometryBoolean Boolean(&OrientedSourceMesh, &OrientedCellMesh, Options.CapCutFaces ?
				GeometryBoolean::BooleanOp::Intersect : GeometryBoolean::BooleanOp::TrimToInterior);
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
