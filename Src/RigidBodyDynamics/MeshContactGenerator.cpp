#include "MeshContactGenerator.h"
#include "RigidBody.h"
#include "../Collision/GeometryObject.h"
#include "../Collision/EPAPenetration.h"
#include "../CollisionPrimitive/AxisAlignedBox3.h"
#include "../CollisionPrimitive/Capsule3.h"
#include "../CollisionPrimitive/ConvexMesh.h"
#include "../CollisionPrimitive/DistanceFieldShape.h"
#include "../CollisionPrimitive/GJK.h"
#include "../CollisionPrimitive/HeightField3.h"
#include "../CollisionPrimitive/Segment3.h"
#include "../CollisionPrimitive/Sphere3.h"
#include "../CollisionPrimitive/Triangle3.h"
#include "../CollisionPrimitive/TriangleMesh.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <tuple>

namespace Riemann
{
	/* Mesh contacts are surface constraints, not a penetration estimate obtained
	   by sweeping the complete mesh in several arbitrary directions. The pipeline
	   is: BVH/grid candidates -> triangle tests -> feature-aware normal correction
	   -> connected planar patches -> four-point spatial reduction per patch.

	   Polygonal shapes use the separating-axis theorem and reference-face clipping;
	   rounded shapes use closest points of a sphere/segment and a triangle. Shared
	   coplanar edges have no collision ridge. Concave edges must not contribute an
	   outward edge normal either: the adjacent faces already bound the free space.
	   Boundary edges and convex creases retain their geometric contact normals.
	   See Christer Ericson, Real-Time Collision Detection, chapters 5 and 9.

	   Topology is built from exact shared positions, including duplicated vertices
	   in procedural meshes. Non-manifold edges are kept conservative. Rebuilding
	   shape bounds/volume properties invalidates the topology through its revision;
	   heightfields additionally expose their own edit revision. No asset is needed.
	   ClosedComponentsAreSolids treats indexed watertight components as a union,
	   repairs winding, and permits recovery from embedded configurations. Disable
	   it for sheet semantics or nested shells whose orientation defines cavities.

	   SDF contacts use grid-edge zero crossings and triangle-side witnesses. A
	   finite triangle clips the projected implicit footprint; triangle vertices,
	   edge midpoints and closest points also detect small triangles inside a field.
	   Interior triangle witnesses trace to the implicit back surface along the
	   mesh normal. Boundary witnesses project onto the field and retain exposed
	   edge normals, while shared planar/concave edges use the same topology rules
	   as rounded primitives. This never substitutes the field's convex hull or
	   display triangles. Coverage follows the field resolution, as in the other
	   sampled-field contact routes. Deepest-footprint reduction runs before the
	   ordinary four-point reduction so speculative points cannot replace support.
	*/
	bool MeshContactSettings::IsValid() const
	{
		return std::isfinite(RelativeTolerance) && RelativeTolerance > 0.0f
			&& std::isfinite(MinimumTolerance) && MinimumTolerance > 0.0f
			&& std::isfinite(SmoothNormalCosine) && SmoothNormalCosine > 0.0f && SmoothNormalCosine <= 1.0f
			&& std::isfinite(PatchNormalCosine) && PatchNormalCosine > 0.0f && PatchNormalCosine <= 1.0f;
	}

	namespace
	{
		const int TriangleCorners = 3;
		const int CellTriangles = 2;
		const float AxisLengthSquaredTolerance = 1.0e-12f;
		const float ProjectionCosineTolerance = 1.0e-5f;

		struct SurfaceTriangle
		{
			Triangle3 Shape = Triangle3(Vector3::Zero(), Vector3::Zero(), Vector3::Zero());
			Vector3 Normal = Vector3::Zero();
			int Neighbor[TriangleCorners] = { -1, -1, -1 };
			uint32_t Patch = 0;
			int Solid = -1;
			bool Valid = false;
		};
		struct SolidComponent
		{
			Box3 Bounds = Box3::Empty();
			std::vector<uint32_t> Triangles;
		};
		struct SurfaceTopology
		{
			uint64_t Revision = 0, HeightRevision = 0;
			std::vector<SurfaceTriangle> Triangles;
			std::vector<SolidComponent> Solids;
		};

		void OrientClosedComponents(const TriangleMesh& Mesh, SurfaceTopology& Surface)
		{
			// Indexed closed components represent solids, even when their input winding
			// is inconsistent (for example procedural boxes). Propagate winding across
			// shared index edges, then use signed volume to choose the outward side.
			// Open sheets retain their authored/double-sided behavior. Keep original
			// indices here: touching closed boxes must remain distinct solid components.
			struct Use { uint32_t Triangle; bool Forward; };
			std::map<std::pair<uint32_t, uint32_t>, std::vector<Use>> Edges;
			for (uint32_t Index = 0; Index < Mesh.NumTriangles; ++Index)
			{
				if (!Surface.Triangles[Index].Valid) continue;
				uint32_t Ids[TriangleCorners]; Mesh.GetVertIndices(Index, Ids[0], Ids[1], Ids[2]);
				for (int Side = 0; Side < TriangleCorners; ++Side)
				{
					const uint32_t A = Ids[Side], B = Ids[(Side + 1) % TriangleCorners];
					Edges[std::minmax(A, B)].push_back({ Index, A < B });
				}
			}
			std::vector<std::vector<std::pair<uint32_t, int>>> Neighbors(Mesh.NumTriangles);
			std::vector<bool> Closed(Mesh.NumTriangles, true);
			for (const auto& Edge : Edges)
			{
				const auto& Uses = Edge.second;
				if (Uses.size() != 2) { for (const Use& U : Uses) Closed[U.Triangle] = false; continue; }
				const int Sign = Uses[0].Forward == Uses[1].Forward ? -1 : 1;
				Neighbors[Uses[0].Triangle].emplace_back(Uses[1].Triangle, Sign);
				Neighbors[Uses[1].Triangle].emplace_back(Uses[0].Triangle, Sign);
			}
			std::vector<int> Signs(Mesh.NumTriangles, 0);
			for (uint32_t Seed = 0; Seed < Mesh.NumTriangles; ++Seed)
			{
				if (Signs[Seed] || !Surface.Triangles[Seed].Valid) continue;
				SolidComponent Component; Component.Triangles.push_back(Seed); Signs[Seed] = 1;
				bool IsClosed = true;
				for (size_t Next = 0; Next < Component.Triangles.size(); ++Next)
				{
					const uint32_t Index = Component.Triangles[Next];
					IsClosed = IsClosed && Closed[Index];
					const Triangle3& T = Surface.Triangles[Index].Shape;
					Component.Bounds.Encapsulate(T.v0); Component.Bounds.Encapsulate(T.v1); Component.Bounds.Encapsulate(T.v2);
					for (const auto& Neighbor : Neighbors[Index])
					{
						const int Sign = Signs[Index] * Neighbor.second;
						if (!Signs[Neighbor.first]) { Signs[Neighbor.first] = Sign; Component.Triangles.push_back(Neighbor.first); }
						else if (Signs[Neighbor.first] != Sign) IsClosed = false;
					}
				}
				if (!IsClosed) continue;
				double Volume = 0.0;
				const Vector3 Origin = Component.Bounds.GetCenter();
				for (uint32_t Index : Component.Triangles)
				{
					const Triangle3& T = Surface.Triangles[Index].Shape;
					Volume += (double)(T.v0 - Origin).Dot((T.v2 - Origin).Cross(T.v1 - Origin)) * Signs[Index];
				}
				if (Volume == 0.0) continue;
				for (uint32_t Index : Component.Triangles)
				{
					SurfaceTriangle& T = Surface.Triangles[Index];
					T.Normal *= (float)(Signs[Index] * (Volume > 0.0 ? 1 : -1));
					T.Solid = (int)Surface.Solids.size();
				}
				Surface.Solids.push_back(std::move(Component));
			}
		}

		bool InsideOtherSolid(const SurfaceTopology& Surface, int OwnSolid, const Vector3& Point)
		{
			// A contact on a buried face of touching/overlapping authored components
			// must not push a body into their union. Winding-number containment avoids
			// ray/edge ambiguity; component bounds reject the usual disjoint case.
			for (size_t Index = 0; Index < Surface.Solids.size(); ++Index)
			{
				const SolidComponent& Solid = Surface.Solids[Index];
				if ((int)Index == OwnSolid || !Solid.Bounds.IsInside(Point)) continue;
				double Angle = 0.0;
				for (uint32_t Triangle : Solid.Triangles)
				{
					const SurfaceTriangle& T = Surface.Triangles[Triangle];
					const Vector3 A = (T.Shape.v0 - Point).SafeUnit(), B = (T.Shape.v1 - Point).SafeUnit(), C = (T.Shape.v2 - Point).SafeUnit();
					const double Numerator = A.Dot(C.Cross(B));
					const double Denominator = 1.0 + A.Dot(B) + B.Dot(C) + C.Dot(A);
					const double Orientation = (T.Shape.v2 - T.Shape.v0).Cross(T.Shape.v1 - T.Shape.v0).Dot(T.Normal) >= 0.0f ? 1.0 : -1.0;
					Angle += Orientation * 2.0 * std::atan2(Numerator, Denominator);
				}
				if (std::abs(Angle) > 2.0 * M_PI) return true;
			}
			return false;
		}

		Vector3 ShapeSupport(const GeometryWorldState& State, const Vector3& Direction)
		{
			Vector3 Point = State.BodyToWorld.LocalToWorld(State.Geom->GetSupport(State.BodyToWorld.WorldToLocalDirection(Direction)));
			if (const auto* Field = State.Geom->GetShapeObj<DistanceFieldShape>())
			{
				Vector3 Surface, Normal; float Distance;
				if (Field->Project(State.ShapeToWorld.WorldToLocal(Point), Surface, Normal, Distance)) Point = State.ShapeToWorld.LocalToWorld(Surface);
			}
			return Point;
		}
		Vector3 ShapeCenter(const GeometryWorldState& State)
		{
			return State.BodyToWorld.LocalToWorld(State.Geom->GetCenter());
		}
		std::vector<Vector3> ContactFace(const GeometryWorldState& State, const Vector3& Outward)
		{
			// Legacy primitives disagree on the sign of GetSupportFace. Select the
			// nearer of the two candidates without changing their public convention.
			std::vector<Vector3> Result;
			float BestProjection = FLT_MAX;
			for (float Sign : { -1.0f, 1.0f })
			{
				SupportFace Face;
				State.Geom->GetSupportFace(State.BodyToWorld.WorldToLocalDirection(Outward * Sign), Face);
				if (Face.empty()) continue;
				std::vector<Vector3> Candidate;
				float Projection = 0.0f;
				for (int Index = 0; Index < Face.size(); ++Index)
				{
					Candidate.push_back(State.BodyToWorld.LocalToWorld(Face[Index]));
					Projection += Candidate.back().Dot(Outward);
				}
				Projection /= Face.size();
				if (Projection < BestProjection) { BestProjection = Projection; Result.swap(Candidate); }
			}
			return Result;
		}
		Box3 TransformBounds(const Box3& Bounds, const Transform& Pose)
		{
			Vector3 Corners[8]; Box3::GetVertices(Bounds.Min, Bounds.Max, Corners);
			Box3 Result = Box3::Empty();
			for (const Vector3& Corner : Corners) Result.Encapsulate(Pose.LocalToWorld(Corner));
			return Result;
		}
		Triangle3 WorldTriangle(const SurfaceTriangle& Triangle, const Transform& Pose)
		{
			return Triangle3(Pose.LocalToWorld(Triangle.Shape.v0), Pose.LocalToWorld(Triangle.Shape.v1), Pose.LocalToWorld(Triangle.Shape.v2));
		}
		bool InsideTriangle(const Triangle3& Triangle, const Vector3& Point, float Tolerance)
		{
			return (Triangle.PointDistanceQuery(Point).ClosestPoint - Point).SquareLength() <= Tolerance * Tolerance;
		}

		bool CapsuleExit(const Vector3& Start, const Vector3& End, float Radius,
			const Vector3& Origin, const Vector3& Direction, float& Distance)
		{
			// The closest axis point is sufficient for an exterior capsule contact.
			// Once an edge/face lies inside the capsule, trace to the actual back
			// surface instead: axisPoint - radius * normal can lie inside an oblique
			// capsule and substantially underestimate penetration at a stair corner.
			const Vector3 Axis = End - Start;
			const float Length = Axis.Length();
			const Vector3 Unit = Length > 0.0f ? Axis / Length : Vector3::UnitY();
			const Vector3 Offset = Origin - Start;
			const double Along = Offset.Dot(Unit), Rate = Direction.Dot(Unit);
			const Vector3 Radial = Offset - Unit * (float)Along, RadialDirection = Direction - Unit * (float)Rate;
			const double A = RadialDirection.SquareLength(), B = Radial.Dot(RadialDirection), C = Radial.SquareLength() - Radius * Radius;
			Distance = -1.0f;
			if (A > AxisLengthSquaredTolerance && B * B >= A * C)
			{
				const double Exit = (-B + std::sqrt(B * B - A * C)) / A;
				const double Height = Along + Exit * Rate;
				if (Exit >= 0.0 && Height >= 0.0 && Height <= Length) Distance = (float)Exit;
			}
			for (int Cap = 0; Cap < 2; ++Cap)
			{
				const Vector3 Relative = Origin - (Cap == 0 ? Start : End);
				const double Projection = Relative.Dot(Direction);
				const double Discriminant = Projection * Projection - Relative.SquareLength() + Radius * Radius;
				if (Discriminant < 0.0) continue;
				const double Exit = -Projection + std::sqrt(Discriminant);
				const double Height = Along + Exit * Rate;
				if (Exit >= 0.0 && (Length == 0.0f || (Cap == 0 ? Height <= 0.0 : Height >= Length))) Distance = std::max(Distance, (float)Exit);
			}
			return Distance >= 0.0f;
		}

		void ClipPolygon(std::vector<Vector3>& Polygon, const Vector3& Origin, const Vector3& Inward, float Tolerance)
		{
			if (Polygon.empty()) return;
			std::vector<Vector3> Output;
			Vector3 Previous = Polygon.back();
			float PreviousDistance = (Previous - Origin).Dot(Inward);
			for (const Vector3& Current : Polygon)
			{
				const float Distance = (Current - Origin).Dot(Inward);
				const bool Inside = Distance >= -Tolerance, PreviousInside = PreviousDistance >= -Tolerance;
				if (Inside != PreviousInside)
				{
					const float Fraction = std::max(0.0f, std::min(1.0f, PreviousDistance / (PreviousDistance - Distance)));
					Output.push_back(Previous + (Current - Previous) * Fraction);
				}
				if (Inside) Output.push_back(Current);
				Previous = Current; PreviousDistance = Distance;
			}
			Polygon.swap(Output);
		}

		struct PolygonalShape
		{
			std::vector<Vector3> Normals;
			std::vector<std::pair<Vector3, Vector3>> Edges;
		};
		PolygonalShape BuildPolygonalShape(const GeometryWorldState& Shape)
		{
			PolygonalShape Result;
			if (const AxisAlignedBox3* Box = Shape.Geom->GetShapeObj<AxisAlignedBox3>())
			{
				Vector3 Vertices[8];
				Box3::GetVertices(Box->Min, Box->Max, Vertices);
				for (Vector3& Vertex : Vertices) Vertex = Shape.ShapeToWorld.LocalToWorld(Vertex);
				for (int Axis = 0; Axis < 3; ++Axis)
				{
					Vector3 Unit = Vector3::Zero(); Unit[Axis] = 1.0f;
					Result.Normals.push_back(Shape.ShapeToWorld.LocalToWorldDirection(Unit));
					for (int Vertex = 0; Vertex < 8; ++Vertex)
						if (!(Vertex & (1 << Axis))) Result.Edges.emplace_back(Vertices[Vertex], Vertices[Vertex | (1 << Axis)]);
				}
			}
			else if (const ConvexMesh* Mesh = Shape.Geom->GetShapeObj<ConvexMesh>())
			{
				for (uint16_t Face = 0; Face < Mesh->NumFaces; ++Face)
				{
					const ConvexMeshFace& F = Mesh->Faces[Face];
					Result.Normals.push_back(Shape.ShapeToWorld.LocalToWorldDirection(F.plane.Normal));
					for (uint16_t Index = 0; Index < F.numVerties; ++Index)
						Result.Edges.emplace_back(Shape.ShapeToWorld.LocalToWorld(Mesh->Vertices[Mesh->Indices[F.first + Index]]),
							Shape.ShapeToWorld.LocalToWorld(Mesh->Vertices[Mesh->Indices[F.first + (Index + 1) % F.numVerties]]));
				}
			}
			return Result;
		}

		bool PolygonIntersectsTriangle(const GeometryWorldState& Shape, const PolygonalShape& Poly,
			const Triangle3& Triangle, const Vector3& Normal, float Tolerance, Vector3& SeparationNormal)
		{
			float MinimumDepth = FLT_MAX;
			auto OverlapAxis = [&](Vector3 Axis)
			{
				if (Axis.SquareLength() <= AxisLengthSquaredTolerance) return true;
				Axis = Axis.Unit();
				float Min = FLT_MAX, Max = -FLT_MAX;
				for (int Corner = 0; Corner < TriangleCorners; ++Corner)
				{
					const float Projection = Triangle[Corner].Dot(Axis);
					Min = std::min(Min, Projection); Max = std::max(Max, Projection);
				}
				const float PositiveDepth = Max - ShapeSupport(Shape, -Axis).Dot(Axis);
				const float NegativeDepth = ShapeSupport(Shape, Axis).Dot(Axis) - Min;
				const float Depth = std::min(PositiveDepth, NegativeDepth);
				if (Depth < -Tolerance) return false;
				if (Depth < MinimumDepth)
				{
					MinimumDepth = Depth;
					SeparationNormal = PositiveDepth <= NegativeDepth ? Axis : -Axis;
				}
				return true;
			};
			if (!OverlapAxis(Normal)) return false;
			for (const Vector3& Axis : Poly.Normals) if (!OverlapAxis(Axis)) return false;
			for (const auto& Edge : Poly.Edges)
				for (int Side = 0; Side < TriangleCorners; ++Side)
					if (!OverlapAxis((Edge.second - Edge.first).Cross(Triangle[(Side + 1) % TriangleCorners] - Triangle[Side]))) return false;
			return true;
		}

		class TriangleSupportShape : public GjkShape
		{
		public:
			TriangleSupportShape(const GeometryWorldState& InShape, const Triangle3& InTriangle) : Shape(InShape), Triangle(InTriangle) {}
			Vector3 Center() const override { return ShapeCenter(Shape) - Triangle.GetCenter(); }
			Vector3 Support(const Vector3& Direction) const override
			{
				int Index = 0;
				for (int Corner = 1; Corner < TriangleCorners; ++Corner)
					if (Triangle[Corner].Dot(Direction) < Triangle[Index].Dot(Direction)) Index = Corner;
				return ShapeSupport(Shape, Direction) - Triangle[Index];
			}
			const GeometryWorldState& Shape;
			const Triangle3& Triangle;
		};
	}

	struct MeshContactGenerator::Storage
	{
		MeshContactSettings Settings;
		std::map<const Geometry*, SurfaceTopology> Surfaces;

		SurfaceTopology& Topology(const Geometry* Shape)
		{
			SurfaceTopology& Surface = Surfaces[Shape];
			const HeightField3* Field = Shape->GetShapeObj<HeightField3>();
			const uint64_t HeightRevision = Field ? Field->GetRevision() : 0;
			if (Surface.Revision == Shape->GetCollisionRevision() && Surface.HeightRevision == HeightRevision) return Surface;
			Surface = SurfaceTopology();
			Surface.Revision = Shape->GetCollisionRevision(); Surface.HeightRevision = HeightRevision;
			if (const TriangleMesh* Mesh = Shape->GetShapeObj<TriangleMesh>())
			{
				Surface.Triangles.resize(Mesh->NumTriangles);
				for (uint32_t Index = 0; Index < Mesh->NumTriangles; ++Index)
					Mesh->GetTriangle(Index, &Surface.Triangles[Index].Shape.v0, &Surface.Triangles[Index].Shape.v1, &Surface.Triangles[Index].Shape.v2);
			}
			else if (Field && Field->nX > 1 && Field->nZ > 1 && Field->Cells)
			{
				Surface.Triangles.resize((Field->nX - 1) * (Field->nZ - 1) * CellTriangles);
				for (uint32_t Z = 0; Z + 1 < Field->nZ; ++Z)
				for (uint32_t X = 0; X + 1 < Field->nX; ++X)
				{
					Vector3 Points[TriangleCorners * CellTriangles]; Field->GetCellTriangle(X, Z, Points);
					int Offset = 0;
					for (int Side = 0; Side < CellTriangles; ++Side)
					{
						if (Field->IsTriangleHole(X, Z, Side)) continue;
						const uint32_t Index = ((Field->nX - 1) * Z + X) * CellTriangles + Side;
						Surface.Triangles[Index].Shape = Triangle3(Points[Offset], Points[Offset + 1], Points[Offset + 2]);
						Offset += TriangleCorners;
					}
				}
			}
			using VertexKey = std::tuple<float, float, float>;
			std::map<VertexKey, uint32_t> Vertices;
			using EdgeKey = std::pair<uint32_t, uint32_t>;
			std::map<EdgeKey, std::vector<std::pair<uint32_t, int>>> Edges;
			std::vector<uint32_t> Parents(Surface.Triangles.size());
			std::iota(Parents.begin(), Parents.end(), 0u);
			auto Root = [&](uint32_t Id) { while (Parents[Id] != Id) { Parents[Id] = Parents[Parents[Id]]; Id = Parents[Id]; } return Id; };
			for (uint32_t Index = 0; Index < Surface.Triangles.size(); ++Index)
			{
				SurfaceTriangle& T = Surface.Triangles[Index];
				// Triangle area has squared-length units. An absolute vector epsilon
				// would erase valid small triangles before scale-aware contact tests.
				const Maths::TVector3<double> EdgeA((double)T.Shape.v2.x - T.Shape.v0.x, (double)T.Shape.v2.y - T.Shape.v0.y, (double)T.Shape.v2.z - T.Shape.v0.z);
				const Maths::TVector3<double> EdgeB((double)T.Shape.v1.x - T.Shape.v0.x, (double)T.Shape.v1.y - T.Shape.v0.y, (double)T.Shape.v1.z - T.Shape.v0.z);
				const auto Area = EdgeA.Cross(EdgeB); const double Length = Area.Length();
				T.Normal = Length > 0.0 && std::isfinite(Length) ? Vector3((float)(Area.x / Length), (float)(Area.y / Length), (float)(Area.z / Length)) : Vector3::Zero();
				T.Valid = std::isfinite(T.Normal.SquareLength()) && T.Normal.SquareLength() > ProjectionCosineTolerance;
				if (!T.Valid) continue;
				uint32_t Ids[TriangleCorners];
				for (int Corner = 0; Corner < TriangleCorners; ++Corner)
				{
					const Vector3& V = T.Shape[Corner];
					Ids[Corner] = Vertices.emplace(VertexKey(V.x, V.y, V.z), (uint32_t)Vertices.size()).first->second;
				}
				for (int Side = 0; Side < TriangleCorners; ++Side)
					Edges[std::minmax(Ids[Side], Ids[(Side + 1) % TriangleCorners])].emplace_back(Index, Side);
			}
			if (Settings.ClosedComponentsAreSolids)
				if (const TriangleMesh* Mesh = Shape->GetShapeObj<TriangleMesh>()) OrientClosedComponents(*Mesh, Surface);
			for (const auto& Edge : Edges)
			{
				const auto& Uses = Edge.second;
				for (size_t A = 0; A < Uses.size(); ++A)
				for (size_t B = A + 1; B < Uses.size(); ++B)
				{
					SurfaceTriangle& First = Surface.Triangles[Uses[A].first];
					SurfaceTriangle& Second = Surface.Triangles[Uses[B].first];
					const float Alignment = First.Normal.Dot(Second.Normal);
					const bool Planar = (First.Solid >= 0 || Second.Solid >= 0 ? Alignment : std::abs(Alignment)) >= 1.0f - Settings.RelativeTolerance;
					if (Uses.size() == 2)
					{
						First.Neighbor[Uses[A].second] = (int)Uses[B].first;
						Second.Neighbor[Uses[B].second] = (int)Uses[A].first;
					}
					if (Planar)
					{
						const uint32_t RootA = Root(Uses[A].first), RootB = Root(Uses[B].first);
						Parents[std::max(RootA, RootB)] = std::min(RootA, RootB);
					}
				}
			}
			for (uint32_t Index = 0; Index < Surface.Triangles.size(); ++Index)
			{
				SurfaceTriangle& T = Surface.Triangles[Index];
				const uint32_t RootId = Root(Index);
				T.Patch = RootId + 1;
				if (T.Normal.Dot(Surface.Triangles[RootId].Normal) < 0.0f) T.Normal = -T.Normal;
			}
			return Surface;
		}
	};

	MeshContactGenerator::MeshContactGenerator(const MeshContactSettings& Settings) : m_Storage(new Storage)
	{
		if (!Settings.IsValid()) throw std::invalid_argument("Invalid mesh contact settings");
		m_Storage->Settings = Settings;
	}
	MeshContactGenerator::~MeshContactGenerator() = default;
	bool MeshContactGenerator::IsEnabled() const { return m_Storage->Settings.Enabled; }
	void MeshContactGenerator::SetEnabled(bool Enabled) { m_Storage->Settings.Enabled = Enabled; Reset(); }
	void MeshContactGenerator::Reset() { m_Storage->Surfaces.clear(); }
	void MeshContactGenerator::Invalidate(const Geometry* Shape) { m_Storage->Surfaces.erase(Shape); }

	void MeshContactGenerator::Generate(int IndexA, int IndexB, const GeometryWorldState& A, const GeometryWorldState& B,
		std::vector<ContactManifold>& Manifolds, float ContactDistance, float GeometryTolerance)
	{
		Manifolds.clear();
		const bool SurfaceIsA = A.Geom->GetShapeType() == PrimitiveType::TRIANGLE_MESH || A.Geom->GetShapeType() == PrimitiveType::HEIGHTFIELD;
		const GeometryWorldState& Surface = SurfaceIsA ? A : B;
		const GeometryWorldState& Convex = SurfaceIsA ? B : A;
		const PrimitiveType Type = Convex.Geom->GetShapeType();
		if (Type == PrimitiveType::TRIANGLE_MESH || Type == PrimitiveType::HEIGHTFIELD || Type == PrimitiveType::PLANE) return;
		Storage& S = *m_Storage;
		SurfaceTopology& Topology = S.Topology(Surface.Geom);
		if (Topology.Triangles.empty()) return;
		const float Tolerance = GeometryTolerance > 0.0f ? GeometryTolerance
			: std::max(S.Settings.MinimumTolerance, Convex.Geom->GetShapeBounds().GetExtent().Length() * S.Settings.RelativeTolerance);
		const Transform ConvexToSurface = Surface.ShapeToWorld.TransformInv(Convex.ShapeToWorld);
		Box3 Bounds = TransformBounds(Convex.Geom->GetShapeBounds(), ConvexToSurface); Bounds.Thicken(Tolerance + ContactDistance);
		std::vector<uint32_t> Candidates;
		const HeightField3* Field = Surface.Geom->GetShapeObj<HeightField3>();
		if (const TriangleMesh* Mesh = Surface.Geom->GetShapeObj<TriangleMesh>()) Mesh->QueryTriangles(Bounds, Candidates);
		else if (Field && Field->nX > 1 && Field->nZ > 1)
		{
			const int MinX = std::max(0, (int)std::floor((Bounds.Min.x - Field->BV.Min.x) * Field->InvDX));
			const int MinZ = std::max(0, (int)std::floor((Bounds.Min.z - Field->BV.Min.z) * Field->InvDZ));
			const int MaxX = std::min((int)Field->nX - 2, (int)std::floor((Bounds.Max.x - Field->BV.Min.x) * Field->InvDX));
			const int MaxZ = std::min((int)Field->nZ - 2, (int)std::floor((Bounds.Max.z - Field->BV.Min.z) * Field->InvDZ));
			for (int Z = MinZ; Z <= MaxZ; ++Z)
			for (int X = MinX; X <= MaxX; ++X)
			for (int Side = 0; Side < CellTriangles; ++Side) Candidates.push_back(((Field->nX - 1) * Z + X) * CellTriangles + Side);
		}
		const PolygonalShape Poly = BuildPolygonalShape(Convex);
		const auto* DistanceField = Convex.Geom->GetShapeObj<DistanceFieldShape>();
		std::vector<DistanceFieldSurfaceSample> FieldSamples;
		if (DistanceField) DistanceField->GetSurfaceSamples(DistanceField->GetField().GetBounds(), FieldSamples);
		struct FieldRow { Vector3 Point, Surface, Normal; uint32_t Patch, Feature; };
		std::vector<FieldRow> FieldRows;
		const Vector3 Center = ShapeCenter(Convex);
		const bool CenterInsideSolid = ContactDistance > 0.0f && InsideOtherSolid(Topology, -1, Surface.ShapeToWorld.WorldToLocal(Center));
		auto AppendContact = [&](const Vector3& ConvexPoint, const Vector3& SurfacePoint, const Vector3& Outward, uint32_t PatchId, uint32_t Feature)
		{
			Contact Point;
			Point.Normal = SurfaceIsA ? Outward : -Outward;
			Point.Normal.GetPerpVectors(Point.Tangent, Point.Binormal);
			Point.PositionWorldA = SurfaceIsA ? SurfacePoint : ConvexPoint;
			Point.PositionWorldB = SurfaceIsA ? ConvexPoint : SurfacePoint;
			Point.PositionLocalA = Point.PositionWorldA - (A.Body ? A.Body->X : A.ShapeToWorld.pos);
			Point.PositionLocalB = Point.PositionWorldB - (B.Body ? B.Body->X : B.ShapeToWorld.pos);
			Point.PenetrationDepth = (SurfacePoint - ConvexPoint).Dot(Outward);
			if (!DistanceField && ContactDistance == 0.0f) Point.PenetrationDepth = std::max(0.0f, Point.PenetrationDepth);
			if (SurfaceIsA) Point.FeatureA = Feature; else Point.FeatureB = Feature;
			if (DistanceField)
			{
				const uint32_t FieldFeature = DistanceField->GetSurfaceFeature(Convex.ShapeToWorld.WorldToLocal(ConvexPoint));
				if (SurfaceIsA) Point.FeatureB = FieldFeature; else Point.FeatureA = FieldFeature;
			}
			ContactManifold* Patch = nullptr;
			for (ContactManifold& Existing : Manifolds)
				if (Existing.PatchId == PatchId) { Patch = &Existing; break; }
			if (!Patch) { Manifolds.emplace_back(); Patch = &Manifolds.back(); Patch->PatchId = PatchId; }
			Patch->AddNewContact(IndexA, IndexB, Point);
		};
		auto RecordContact = [&](const Vector3& Point, const Vector3& SurfacePoint, const Vector3& Normal, uint32_t Patch, uint32_t Feature)
		{
			if (DistanceField) FieldRows.push_back({ Point, SurfacePoint, Normal, Patch, Feature });
			else AppendContact(Point, SurfacePoint, Normal, Patch, Feature);
		};
		for (uint32_t Index : Candidates)
		{
			const SurfaceTriangle& Local = Topology.Triangles[Index];
			if (!Local.Valid) continue;
			const Triangle3 Triangle = WorldTriangle(Local, Surface.ShapeToWorld);
			Vector3 Normal = Surface.ShapeToWorld.LocalToWorldDirection(Local.Normal);
			const bool Back = (Center - Triangle.v0).Dot(Normal) < 0.0f;
			if (ContactDistance > 0.0f && Local.Solid >= 0 && Back && !CenterInsideSolid) continue;
			const bool TwoSided = !Field && Local.Solid < 0 && S.Settings.TwoSidedMeshes;
			if (Back && TwoSided) Normal = -Normal;
			else if (Back && !Field && Local.Solid < 0) continue;
			int AddedContacts = 0;
			auto Add = [&](const Vector3& ConvexPoint, const Vector3& SurfacePoint, const Vector3& Outward)
			{
				const float Depth = (SurfacePoint - ConvexPoint).Dot(Outward);
				if (!std::isfinite(Depth) || Depth < -Tolerance - ContactDistance || Outward.SquareLength() < ProjectionCosineTolerance) return;
				if (Local.Solid >= 0 && InsideOtherSolid(Topology, Local.Solid, Surface.ShapeToWorld.WorldToLocal(SurfacePoint + Outward * Tolerance))) return;
				// The side bit keeps opposite sides of a two-sided sheet distinct.
				uint32_t PatchId = Local.Patch * 2 + (Back && TwoSided ? 1u : 0u);
				if (Outward.Dot(Normal) < S.Settings.PatchNormalCosine)
				{
					const Vector3 Barycentric = Triangle.PointDistanceQuery(SurfacePoint).BaryCoords;
					uint32_t Feature = 0;
					for (int Corner = 0; Corner < TriangleCorners; ++Corner)
						if (Barycentric[Corner] <= S.Settings.RelativeTolerance) Feature |= 1u << Corner;
					const uint32_t FeaturesPerTriangle = 1u << TriangleCorners;
					PatchId = (((uint32_t)Topology.Triangles.size() + 1) + Index * FeaturesPerTriangle + Feature) * 2 + (Back && TwoSided ? 1u : 0u);
				}
				RecordContact(ConvexPoint, SurfacePoint, Outward, PatchId, Local.Patch);
				++AddedContacts;
			};
			auto RoundedPoint = [&](const Vector3& AxisPoint, const Vector3& Closest, float Radius)
			{
				const Vector3 Delta = AxisPoint - Closest;
				if (Delta.SquareLength() > (Radius + Tolerance + ContactDistance) * (Radius + Tolerance + ContactDistance)) return;
				Vector3 Direction = Delta.SquareLength() > Tolerance * Tolerance ? Delta.Unit() : Normal;
				Vector3 SurfacePoint = Closest;
				const Vector3 Projection = AxisPoint - Normal * (AxisPoint - Triangle.v0).Dot(Normal);
				if (InsideTriangle(Triangle, Projection, Tolerance)) { Direction = Normal; SurfacePoint = Projection; }
				else
				{
					for (int Side = 0; Side < TriangleCorners; ++Side)
					{
						if (Local.Neighbor[Side] < 0 || Segment3::SqrDistancePointToSegment(Closest, Triangle[Side], Triangle[(Side + 1) % TriangleCorners]) > Tolerance * Tolerance) continue;
						const SurfaceTriangle& Neighbor = Topology.Triangles[(size_t)Local.Neighbor[Side]];
						const Triangle3 Adjacent = WorldTriangle(Neighbor, Surface.ShapeToWorld);
						Vector3 AdjacentNormal = Surface.ShapeToWorld.LocalToWorldDirection(Neighbor.Normal);
						if (S.Settings.TwoSidedMeshes && !Field && Neighbor.Solid < 0 && AdjacentNormal.Dot(AxisPoint - Adjacent.v0) < 0.0f) AdjacentNormal = -AdjacentNormal;
						const bool Smooth = Normal.Dot(AdjacentNormal) >= S.Settings.SmoothNormalCosine;
						const bool Concave = Normal.Dot(Adjacent.GetCenter() - Triangle.v0) > Tolerance;
						if (Smooth && InsideTriangle(Adjacent, Projection, Tolerance)) { Direction = Normal; SurfacePoint = Projection; break; }
						if (Concave) return;
					}
				}
				if (Direction.Dot(Normal) < 0.0f) Direction = Normal;
				Vector3 ConvexPoint = AxisPoint - Direction * Radius;
				if (const Capsule3* Capsule = Convex.Geom->GetShapeObj<Capsule3>())
				{
					float Exit;
					if (CapsuleExit(Convex.ShapeToWorld.LocalToWorld(Capsule->X0), Convex.ShapeToWorld.LocalToWorld(Capsule->X1),
						Capsule->Radius, SurfacePoint, -Direction, Exit)) ConvexPoint = SurfacePoint - Direction * Exit;
				}
				Add(ConvexPoint, SurfacePoint, Direction);
			};
			if (const Sphere3* Sphere = Convex.Geom->GetShapeObj<Sphere3>())
			{
				const Vector3 Position = Convex.ShapeToWorld.LocalToWorld(Sphere->Center);
				RoundedPoint(Position, Triangle.PointDistanceQuery(Position).ClosestPoint, Sphere->Radius);
			}
			else if (const Capsule3* Capsule = Convex.Geom->GetShapeObj<Capsule3>())
			{
				const Vector3 Ends[] = { Convex.ShapeToWorld.LocalToWorld(Capsule->X0), Convex.ShapeToWorld.LocalToWorld(Capsule->X1) };
				// Clip the axis against the triangle's extruded footprint. This retains
				// the full face penetration when a capsule axis crosses the surface.
				std::vector<Vector3> Axis = { Ends[0], Ends[1] };
				for (int Side = 0; Side < TriangleCorners; ++Side)
				{
					Vector3 Inward = (Triangle[(Side + 1) % TriangleCorners] - Triangle[Side]).Cross(Normal).SafeUnit();
					if (Inward.Dot(Triangle[(Side + 2) % TriangleCorners] - Triangle[Side]) < 0.0f) Inward = -Inward;
					ClipPolygon(Axis, Triangle[Side], Inward, Tolerance);
				}
				const float Height0 = (Ends[0] - Triangle.v0).Dot(Normal), Height1 = (Ends[1] - Triangle.v0).Dot(Normal);
				if (std::abs(Height1 - Height0) > Tolerance)
				{
					// Only the lower cap supports an oblique capsule in the face normal.
					const Vector3 Lower = Height0 < Height1 ? Ends[0] : Ends[1];
					const Vector3 Projection = Lower - Normal * std::min(Height0, Height1);
					Axis.clear();
					if (InsideTriangle(Triangle, Projection, Tolerance)) Axis.push_back(Lower);
				}
				if (std::max(Height0, Height1) >= -Capsule->Radius - Tolerance)
					for (const Vector3& Point : Axis)
						Add(Point - Normal * Capsule->Radius, Point - Normal * (Point - Triangle.v0).Dot(Normal), Normal);
				for (const Vector3& End : Ends) RoundedPoint(End, Triangle.PointDistanceQuery(End).ClosestPoint, Capsule->Radius);
				for (int Side = 0; Side < TriangleCorners; ++Side)
				{
					Vector3 AxisPoint, EdgePoint;
					Segment3::ClosestPointsBetweenSegments(Ends[0], Ends[1], Triangle[Side], Triangle[(Side + 1) % TriangleCorners], AxisPoint, EdgePoint);
					RoundedPoint(AxisPoint, EdgePoint, Capsule->Radius);
				}
				const float Denominator = (Ends[1] - Ends[0]).Dot(Normal);
				if (std::abs(Denominator) > Tolerance)
				{
					const float Fraction = (Triangle.v0 - Ends[0]).Dot(Normal) / Denominator;
					const Vector3 Point = Ends[0] + (Ends[1] - Ends[0]) * Fraction;
					if (Fraction >= 0.0f && Fraction <= 1.0f && InsideTriangle(Triangle, Point, Tolerance)) RoundedPoint(Point, Point, Capsule->Radius);
				}
			}
			else if (DistanceField)
			{
				const auto& Implicit = DistanceField->GetField();
				const Transform& Pose = Convex.ShapeToWorld;
				const Vector3 LocalNormal = Pose.WorldToLocalDirection(Normal);
				std::set<std::tuple<float, float, float>> Witnesses;
				auto BoundaryWitness = [&](const Vector3& Point, const Vector3& Reference, bool CorrectEdges)
				{
					Vector3 OnField, Gradient; float Distance;
					const Vector3 LocalPoint = Pose.WorldToLocal(Point);
					if (!DistanceField->Project(LocalPoint, OnField, Gradient, Distance))
					{
						// A speculative range can reach an exposed edge outside the cook
						// domain. Find a real zero-surface witness there, then apply the
						// same side and adjacency checks as for local projections.
						DistanceFieldSurfacePoint Surface;
						if (ContactDistance <= 0.0f || !Implicit.ClosestSurfacePoint(LocalPoint, Surface, ContactDistance + Tolerance)) return;
						DistanceFieldSample Sample; const bool Inside = Implicit.Sample(LocalPoint, Sample) && Sample.Value < 0.0f;
						OnField = Surface.Position; Distance = Surface.Distance * (Inside ? -1.0f : 1.0f);
						const Vector3 Delta = Inside ? OnField - LocalPoint : LocalPoint - OnField;
						const double Length = std::sqrt((double)Delta.x * Delta.x + (double)Delta.y * Delta.y + (double)Delta.z * Delta.z);
						Gradient = Length > 0.0 ? Vector3((float)(Delta.x / Length), (float)(Delta.y / Length), (float)(Delta.z / Length)) : Surface.Normal;
					}
					if (Distance > ContactDistance + Tolerance) return;
					const Vector3 Direction = -Pose.LocalToWorldDirection(Gradient);
					if (Direction.Dot(Normal) < 0.0f) return;
					const Vector3 Projection = Reference - Normal * (Reference - Triangle.v0).Dot(Normal);
					for (int Side = 0; CorrectEdges && Side < TriangleCorners; ++Side)
					{
						if (Local.Neighbor[Side] < 0 || Segment3::SqrDistancePointToSegment(Point, Triangle[Side], Triangle[(Side + 1) % TriangleCorners]) > Tolerance * Tolerance) continue;
						const auto& Neighbor = Topology.Triangles[(size_t)Local.Neighbor[Side]];
						const Triangle3 Adjacent = WorldTriangle(Neighbor, Surface.ShapeToWorld);
						Vector3 AdjacentNormal = Surface.ShapeToWorld.LocalToWorldDirection(Neighbor.Normal);
						if (TwoSided && Neighbor.Solid < 0 && AdjacentNormal.Dot(Center - Adjacent.v0) < 0.0f) AdjacentNormal = -AdjacentNormal;
						const bool Smooth = Normal.Dot(AdjacentNormal) >= S.Settings.SmoothNormalCosine;
						const bool Concave = Normal.Dot(Adjacent.GetCenter() - Triangle.v0) > Tolerance;
						if (Concave || (Smooth && InsideTriangle(Adjacent, Projection, Tolerance))) return;
					}
					Add(Pose.LocalToWorld(OnField), Point, Direction);
				};
				// Minimize the sampled field on the finite triangle. Projected descent
				// distinguishes a true face support from a constrained edge minimum;
				// merely clipping surface samples can invent a deep face row at an edge.
				auto TriangleWitness = [&](Vector3 Point)
				{
					if (!Witnesses.emplace(Point.x, Point.y, Point.z).second) return;
					for (int Iteration = 0; Iteration < DistanceField->GetSettings().ProjectionIterations; ++Iteration)
					{
						DistanceFieldSample Sample;
						if (!Implicit.Sample(Pose.WorldToLocal(Point), Sample)) break;
						const Vector3 Gradient = Pose.LocalToWorldDirection(Sample.Gradient);
						const Vector3 Tangent = Gradient - Normal * Gradient.Dot(Normal);
						const double Length = std::sqrt((double)Gradient.x * Gradient.x + (double)Gradient.y * Gradient.y + (double)Gradient.z * Gradient.z);
						if (!(Length > 0.0)) break;
						const Vector3 Direction((float)(Tangent.x / Length), (float)(Tangent.y / Length), (float)(Tangent.z / Length));
						float Step = Implicit.GetCellSize() * DistanceField->GetSettings().ProjectionStepCells;
						bool Advanced = false;
						while (Step > DistanceField->GetSurfaceTolerance())
						{
							const Vector3 Next = Triangle.PointDistanceQuery(Point - Direction * Step).ClosestPoint;
							if ((Next - Point).SquareLength() <= Tolerance * Tolerance) break;
							DistanceFieldSample Trial;
							if (Implicit.Sample(Pose.WorldToLocal(Next), Trial) && Trial.Value < Sample.Value)
							{ Point = Next; Advanced = true; break; }
							Step /= 2;
						}
						if (!Advanced) break;
					}
					bool Exposed = false;
					for (int Side = 0; Side < TriangleCorners; ++Side)
					{
						if (Segment3::SqrDistancePointToSegment(Point, Triangle[Side], Triangle[(Side + 1) % TriangleCorners]) > Tolerance * Tolerance) continue;
						if (Local.Neighbor[Side] < 0) { Exposed = true; break; }
						const auto& Neighbor = Topology.Triangles[(size_t)Local.Neighbor[Side]];
						if (std::abs(Local.Normal.Dot(Neighbor.Normal)) < S.Settings.SmoothNormalCosine) Exposed = true;
					}
					DistanceFieldSample Sample; const Vector3 LocalPoint = Pose.WorldToLocal(Point);
					if (!Exposed)
					{
						const bool Inside = Implicit.Sample(LocalPoint, Sample) && Sample.Value <= 0.0f;
						DistanceFieldRayHit Hit;
						if (Implicit.RayCast(LocalPoint, Inside ? -LocalNormal : LocalNormal,
							Inside ? std::numeric_limits<float>::max() : ContactDistance + Tolerance, Hit, false))
							Add(Pose.LocalToWorld(Hit.Position), Point, Normal);
					}
					else BoundaryWitness(Point, Center, Exposed);
				};
				for (const auto& Sample : FieldSamples)
				{
					const Vector3 Point = Pose.LocalToWorld(Sample.Position);
					const float Gap = (Point - Triangle.v0).Dot(Normal);
					if (Gap > ContactDistance + Tolerance) continue;
					DistanceFieldSample AtSurface;
					if (!Implicit.Sample(Sample.Position, AtSurface) || AtSurface.Gradient.Dot(LocalNormal) >= 0.0f) continue;
					const Vector3& Gradient = AtSurface.Gradient;
					const double GradientLength = std::sqrt((double)Gradient.x * Gradient.x + (double)Gradient.y * Gradient.y + (double)Gradient.z * Gradient.z);
					const Vector3 Projection = Point - Normal * Gap;
					DistanceFieldSample AtTriangle;
					if (-Gradient.Dot(LocalNormal) >= GradientLength * S.Settings.SmoothNormalCosine && InsideTriangle(Triangle, Projection, Tolerance)
						&& (Gap >= 0.0f || (Implicit.Sample(Pose.WorldToLocal(Projection), AtTriangle) && AtTriangle.Value <= 0.0f)))
					{
						// Parallel implicit/triangle faces already provide a stationary
						// witness. Keep their footprint without repeating descent/raycast.
						Add(Point, Projection, Normal); continue;
					}
					TriangleWitness(Triangle.PointDistanceQuery(Point).ClosestPoint);
				}
				for (int Corner = 0; Corner < TriangleCorners; ++Corner)
				{
					TriangleWitness(Triangle[Corner]);
					TriangleWitness((Triangle[Corner] + Triangle[(Corner + 1) % TriangleCorners]) / 2);
				}
				TriangleWitness(Triangle.GetCenter());
				TriangleWitness(Triangle.PointDistanceQuery(Center).ClosestPoint);
			}
			else
			{
				Vector3 SeparationNormal = Normal;
				EPAPenetration EPA;
				if (!Poly.Normals.empty())
				{
					if (!PolygonIntersectsTriangle(Convex, Poly, Triangle, Normal, Tolerance + ContactDistance, SeparationNormal)) continue;
				}
				else
				{
					TriangleSupportShape Support(Convex, Triangle);
					GJKIntersection GJK;
					const GJK_status Status = GJK.Solve(&Support);
					if (Status != GJK_status::Intersect && (Status != GJK_status::NotIntersect || GJK.distance > ContactDistance + Tolerance)) continue;
					if (Status == GJK_status::Intersect) EPA.Solve(GJK.result);
				}
				std::vector<Vector3> Polygon = ContactFace(Convex, Normal);
				for (int Side = 0; Side < TriangleCorners; ++Side)
				{
					Vector3 Inward = (Triangle[(Side + 1) % TriangleCorners] - Triangle[Side]).Cross(Normal).SafeUnit();
					if (Inward.Dot(Triangle[(Side + 2) % TriangleCorners] - Triangle[Side]) < 0.0f) Inward = -Inward;
					ClipPolygon(Polygon, Triangle[Side], Inward, Tolerance);
				}
				for (const Vector3& Point : Polygon) Add(Point, Point + Normal * (Triangle.v0 - Point).Dot(Normal), Normal);
				if (Polygon.empty())
				{
					const Vector3 Point = ShapeSupport(Convex, -Normal);
					const Vector3 Projection = Point + Normal * (Triangle.v0 - Point).Dot(Normal);
					if (InsideTriangle(Triangle, Projection, Tolerance)) Add(Point, Projection, Normal);
				}
				if (AddedContacts == 0 && !Poly.Normals.empty())
				{
					// Near a boundary the minimum translation may follow a convex face or
					// edge axis. Reverse the clipping roles; both anchors stay on a surface.
					const std::vector<Vector3> Face = ContactFace(Convex, SeparationNormal);
					if (Face.size() < TriangleCorners) continue;
					Vector3 FaceNormal = (Face[1] - Face[0]).Cross(Face[2] - Face[0]).SafeUnit();
					const float Denominator = SeparationNormal.Dot(FaceNormal);
					if (std::abs(Denominator) <= ProjectionCosineTolerance) continue;
					Vector3 FaceCenter = Vector3::Zero();
					for (const Vector3& Point : Face) FaceCenter += Point;
					FaceCenter /= (float)Face.size();
					Polygon = { Triangle.v0, Triangle.v1, Triangle.v2 };
					for (size_t Side = 0; Side < Face.size(); ++Side)
					{
						Vector3 Inward = (Face[(Side + 1) % Face.size()] - Face[Side]).Cross(SeparationNormal).SafeUnit();
						if (Inward.Dot(FaceCenter - Face[Side]) < 0.0f) Inward = -Inward;
						ClipPolygon(Polygon, Face[Side], Inward, Tolerance);
					}
					for (const Vector3& Point : Polygon)
						Add(Point - SeparationNormal * ((Point - Face[0]).Dot(FaceNormal) / Denominator), Point, SeparationNormal);
				}
				else if (AddedContacts == 0 && EPA.result.dimension > 0 && EPA.status != EPA_status::Failed && EPA.status != EPA_status::FallBack)
				{
					Vector3 ConvexPoint = Vector3::Zero(), SurfacePoint = Vector3::Zero();
					for (int Vertex = 0; Vertex < EPA.result.dimension; ++Vertex)
					{
						const Vector3 Direction = EPA.result.v[Vertex].dir;
						int Corner = 0;
						for (int Candidate = 1; Candidate < TriangleCorners; ++Candidate)
							if (Triangle[Candidate].Dot(Direction) < Triangle[Corner].Dot(Direction)) Corner = Candidate;
						ConvexPoint += ShapeSupport(Convex, Direction) * EPA.result.w[Vertex];
						SurfacePoint += Triangle[Corner] * EPA.result.w[Vertex];
					}
					Add(ConvexPoint, SurfacePoint, -EPA.penetration_normal);
				}
			}
		}
		// A capsule cap or convex center can be embedded after a discrete impact.
		// Surface intersections alone then underestimate overlap with a closed solid.
		// Recover toward an actual exterior triangle, excluding buried component faces.
		std::vector<Vector3> InteriorSamples = { Center };
		if (DistanceField)
		{
			// A concave field's centroid can lie in an empty cavity. Only actual
			// occupied field points may trigger recovery from a closed mesh.
			InteriorSamples.clear();
			DistanceFieldSample AtCenter;
			if (DistanceField->GetField().Sample(Convex.ShapeToWorld.WorldToLocal(Center), AtCenter) && AtCenter.Value < 0.0f)
				InteriorSamples.push_back(Center);
			if (!Topology.Solids.empty() && (InteriorSamples.empty() || !InsideOtherSolid(Topology, -1, Surface.ShapeToWorld.WorldToLocal(Center))))
				for (const auto& Sample : FieldSamples)
				{
					const Vector3 Point = Convex.ShapeToWorld.LocalToWorld(Sample.Position);
					if (InsideOtherSolid(Topology, -1, Surface.ShapeToWorld.WorldToLocal(Point))) { InteriorSamples.push_back(Point); break; }
				}
		}
		const Capsule3* Capsule = Convex.Geom->GetShapeObj<Capsule3>();
		if (Capsule)
		{
			InteriorSamples.push_back(Convex.ShapeToWorld.LocalToWorld(Capsule->X0));
			InteriorSamples.push_back(Convex.ShapeToWorld.LocalToWorld(Capsule->X1));
		}
		float RecoveryDepth = FLT_MAX;
		Vector3 RecoveryConvex, RecoverySurface, RecoveryNormal;
		uint32_t RecoveryTriangle = UINT32_MAX;
		for (const Vector3& Sample : InteriorSamples)
		{
			const Vector3 LocalSample = Surface.ShapeToWorld.WorldToLocal(Sample);
			if (!InsideOtherSolid(Topology, -1, LocalSample)) continue;
			for (uint32_t Index = 0; Index < Topology.Triangles.size(); ++Index)
			{
				const SurfaceTriangle& Local = Topology.Triangles[Index];
				if (Local.Solid < 0) continue;
				const Vector3 Closest = Local.Shape.PointDistanceQuery(LocalSample).ClosestPoint;
				Vector3 Direction = (Closest - LocalSample).SafeUnit();
				if (Direction.IsZero()) Direction = Local.Normal;
				if (Direction.Dot(Local.Normal) < -ProjectionCosineTolerance
					|| InsideOtherSolid(Topology, Local.Solid, Closest + Direction * Tolerance)) continue;
				const Vector3 WorldPoint = Surface.ShapeToWorld.LocalToWorld(Closest);
				const Vector3 Normal = Surface.ShapeToWorld.LocalToWorldDirection(Direction);
				Vector3 ConvexPoint = ShapeSupport(Convex, -Normal);
				if (Capsule)
				{
					float Exit;
					if (!CapsuleExit(Convex.ShapeToWorld.LocalToWorld(Capsule->X0), Convex.ShapeToWorld.LocalToWorld(Capsule->X1),
						Capsule->Radius, WorldPoint, -Normal, Exit)) continue;
					ConvexPoint = WorldPoint - Normal * Exit;
				}
				const float Depth = (WorldPoint - ConvexPoint).Dot(Normal);
				if (Depth > Tolerance && Depth < RecoveryDepth)
				{
					RecoveryDepth = Depth; RecoveryConvex = ConvexPoint; RecoverySurface = WorldPoint;
					RecoveryNormal = Normal; RecoveryTriangle = Index;
				}
			}
		}
		if (RecoveryTriangle != UINT32_MAX)
		{
			const uint32_t FeaturesPerTriangle = 1u << TriangleCorners;
			const uint32_t RecoveryPatchBase = (uint32_t)Topology.Triangles.size() * (FeaturesPerTriangle + 1) * 2 + 2;
			if (DistanceField) FieldRows.clear();
			RecordContact(RecoveryConvex, RecoverySurface, RecoveryNormal, RecoveryPatchBase + RecoveryTriangle,
				Topology.Triangles[RecoveryTriangle].Patch);
		}
		if (DistanceField)
		{
			struct FieldPatch { uint32_t Source, Id; Vector3 Normal; float Depth; };
			std::vector<FieldPatch> Patches;
			std::vector<size_t> Assignment;
			const uint32_t ExtraPatchBase = ((uint32_t)Topology.Triangles.size() + 1) * ((1u << TriangleCorners) + 2) * 2;
			for (const auto& Row : FieldRows)
			{
				const float Depth = (Row.Surface - Row.Point).Dot(Row.Normal);
				size_t Group = 0;
				for (; Group < Patches.size(); ++Group) if (Patches[Group].Source == Row.Patch
					&& Patches[Group].Normal.Dot(Row.Normal) >= DistanceField->GetSettings().PatchNormalCosine) break;
				if (Group == Patches.size()) Patches.push_back({ Row.Patch, ExtraPatchBase + (uint32_t)Group, Row.Normal, Depth });
				else Patches[Group].Depth = std::max(Patches[Group].Depth, Depth);
				Assignment.push_back(Group);
			}
			const float DepthTolerance = DistanceField->GetField().GetCellSize() * DistanceField->GetSettings().ManifoldDepthToleranceCells;
			for (size_t I = 0; I < FieldRows.size(); ++I)
			{
				const auto& Row = FieldRows[I]; const auto& Patch = Patches[Assignment[I]];
				if ((Row.Surface - Row.Point).Dot(Row.Normal) + DepthTolerance >= Patch.Depth)
					AppendContact(Row.Point, Row.Surface, Row.Normal, Patch.Id, Row.Feature);
			}
		}
	}
}
