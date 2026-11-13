#include "DistanceFieldCollision.h"
#include "GeometryObject.h"
#include "../CollisionPrimitive/DistanceFieldShape.h"
#include "../CollisionPrimitive/AxisAlignedBox3.h"
#include "../CollisionPrimitive/Sphere3.h"
#include "../CollisionPrimitive/Capsule3.h"
#include "../CollisionPrimitive/Cylinder3.h"
#include "../CollisionPrimitive/ConvexMesh.h"
#include "../CollisionPrimitive/Plane3.h"
#include "../CollisionPrimitive/Triangle3.h"
#include "../CollisionPrimitive/TriangleMesh.h"
#include "../CollisionPrimitive/HeightField3.h"
#include "../RigidBodyDynamics/MeshContactGenerator.h"
#include "../RigidBodyDynamics/RigidBody.h"
#include <algorithm>
#include <cmath>

namespace Riemann
{
	/* Field collision uses two complementary sets of witnesses. Convex surface
	   witnesses projected onto the implicit surface cover objects smaller than a
	   grid cell and objects embedded in the solid. Grid-edge zero crossings inside
	   the other shape's expanded bounds cover broad faces and concave components.
	   The second pass also handles a whole SDF contained in another volume. Every
	   field witness is evaluated against the sampled field, not its display mesh.

	   Spheres use the bounded nearest-zero-surface query directly, including starts
	   inside a solid and inside a cavity. Other primitive point distances are analytic;
	   convex polyhedra use supporting planes inside and closest triangles outside.
	   Alternating surface projections improve separated witnesses. Like particle/SDF
	   contact methods, general surface coverage depends on the cooked grid resolution.
	   Offsets affect candidate distance only; witnesses stay on physical surfaces.
	   Persistent feature matching reuses impulses. Field patches currently regenerate
	   geometry: reusing a reduced footprint without reprojecting it can leave a PBD
	   support patch stale even within the ordinary rigid-anchor refresh threshold.

	   Mesh queries share the triangle contact generator used by narrow phase. Its
	   topology cache preserves sheet sides, closed components and internal edges;
	   the query adapter owns a temporary cache, while simulation retains its cache.

	   Bounded set-distance queries drive translation sweeps for primitive, convex,
	   field, triangle and terrain targets. World CCD adds angular travel bounds and
	   uses the actual field witness as the impulse location. Query distances and
	   normals use world coordinates (normal points from B to A); sweep witnesses
	   and normals use A's original local frame, at the reported impact time.
	   Exhausted work/precision budgets report a conservative stop rather than a miss.
	   The Boolean geometry-query adapter preserves that conservative behavior.

	   Positive contact ranges may extend outside the cooked grid. If local surface
	   projections produce no patch, a bounded pair-distance query supplies a physical
	   surface witness inside that range. It does not extrapolate the sampled field
	   or turn an unresolved lower bound into a speculative contact.
	*/
	namespace
	{
		const int Dimensions = 3;
		const float MinimumNormalSquared = 1.0e-16f;
		Vector3 Unit(const Vector3& P, const Vector3& Fallback = Vector3::UnitY())
		{
			const double Size = std::sqrt((double)P.x * P.x + (double)P.y * P.y + (double)P.z * P.z);
			return Size > 0.0 ? Vector3((float)(P.x / Size), (float)(P.y / Size), (float)(P.z / Size)) : Fallback;
		}
		Vector3 Support(const Geometry* G, const Transform& Pose, const Vector3& Direction)
		{
			const Transform& Stored = *G->GetTransform();
			Vector3 Point = Stored.WorldToLocal(G->GetSupport(Stored.LocalToWorldDirection(Pose.WorldToLocalDirection(Direction))));
			if (const auto* Field = G->GetShapeObj<DistanceFieldShape>())
			{
				Vector3 Surface, Normal; float Distance;
				if (Field->Project(Point, Surface, Normal, Distance)) Point = Surface;
			}
			return Pose.LocalToWorld(Point);
		}
		bool Closest(const Geometry* G, const Vector3& P, Vector3& Q, Vector3& Normal, float& Distance)
		{
			if (const auto* S = G->GetShapeObj<Sphere3>())
			{
				Normal = Unit(P - S->Center); Distance = (P - S->Center).Length() - S->Radius; Q = S->Center + Normal * S->Radius; return true;
			}
			if (const auto* B = G->GetShapeObj<AxisAlignedBox3>())
			{
				Q = P.Max(B->Min).Min(B->Max); const Vector3 Delta = P - Q;
				if (Delta.SquareLength() > 0.0f) { Distance = Delta.Length(); Normal = Delta / Distance; return true; }
				Distance = -std::numeric_limits<float>::max(); Normal = Vector3::Zero();
				for (int Axis = 0; Axis < Dimensions; ++Axis) for (int Side : { -1, 1 })
				{
					const float Gap = Side < 0 ? B->Min[Axis] - P[Axis] : P[Axis] - B->Max[Axis];
					if (Gap > Distance) { Distance = Gap; Normal = Vector3::Zero(); Normal[Axis] = (float)Side; }
				}
				Q = P - Normal * Distance; return true;
			}
			if (const auto* C = G->GetShapeObj<Capsule3>())
			{
				const Vector3 Axis = C->X1 - C->X0;
				const float T = Axis.SquareLength() > 0.0f ? std::max(0.0f, std::min(1.0f, (P - C->X0).Dot(Axis) / Axis.SquareLength())) : 0.0f;
				const Vector3 Center = C->X0 + Axis * T;
				Vector3 Tangent, Binormal; Unit(Axis).GetPerpVectors(Tangent, Binormal);
				Normal = Unit(P - Center, Tangent); Distance = (P - Center).Length() - C->Radius; Q = Center + Normal * C->Radius; return true;
			}
			if (const auto* C = G->GetShapeObj<Cylinder3>())
			{
				const Vector3 Axis = Unit(C->X1 - C->X0), Delta = P - C->GetCenter();
				const float Height = (C->X1 - C->X0).Length() / 2, Axial = Delta.Dot(Axis);
				const Vector3 Radial = Delta - Axis * Axial;
				Vector3 Tangent, Binormal; Axis.GetPerpVectors(Tangent, Binormal);
				const Vector3 RadialNormal = Unit(Radial, Tangent), CapNormal = Axis * (Axial < 0.0f ? -1.0f : 1.0f);
				const float SideGap = Radial.Length() - C->Radius, CapGap = std::abs(Axial) - Height;
				if (SideGap > 0.0f && CapGap > 0.0f)
				{
					Q = C->GetCenter() + RadialNormal * C->Radius + CapNormal * Height;
					Distance = (P - Q).Length(); Normal = Unit(P - Q); return true;
				}
				Distance = std::max(SideGap, CapGap); Normal = SideGap > CapGap ? RadialNormal : CapNormal;
				Q = P - Normal * Distance; return true;
			}
			if (const auto* Plane = G->GetShapeObj<Plane3>())
			{
				Normal = Unit(Plane->Normal); Distance = Normal.Dot(P) + Plane->D / Plane->Normal.Length(); Q = P - Normal * Distance; return true;
			}
			if (const auto* Mesh = G->GetShapeObj<ConvexMesh>())
			{
				if (!Mesh->NumFaces) return false;
				Distance = -std::numeric_limits<float>::max();
				for (uint16_t I = 0; I < Mesh->NumFaces; ++I)
				{
					const auto& Plane = Mesh->Faces[I].plane; const float Length = Plane.Normal.Length();
					const float Gap = (Plane.Normal.Dot(P) + Plane.D) / Length;
					if (Gap > Distance) { Distance = Gap; Normal = Plane.Normal / Length; }
				}
				if (Distance <= 0.0f) { Q = P - Normal * Distance; return true; }
				float Best = std::numeric_limits<float>::max();
				for (uint16_t I = 0; I < Mesh->NumFaces; ++I)
				{
					const auto& Face = Mesh->Faces[I]; const Vector3 A = Mesh->Vertices[Mesh->Indices[Face.first]];
					for (int J = 1; J + 1 < Face.numVerties; ++J)
					{
						const Vector3 Candidate = Triangle3::ClosestPointOnTriangleToPoint(P, A,
							Mesh->Vertices[Mesh->Indices[Face.first + J]], Mesh->Vertices[Mesh->Indices[Face.first + J + 1]]);
						const float Squared = (P - Candidate).SquareLength(); if (Squared < Best) { Best = Squared; Q = Candidate; }
					}
				}
				Distance = std::sqrt(Best); Normal = Unit(P - Q); return true;
			}
			if (const auto* Field = G->GetShapeObj<DistanceFieldShape>()) return Field->Project(P, Q, Normal, Distance);
			return false;
		}
		class GeometryDistanceTarget : public DistanceFieldDistanceTarget
		{
		public:
			GeometryDistanceTarget(const Geometry* Shape, const Transform& SourcePose, const Transform& TargetPose, float Tolerance)
				: Other(Shape), ToOther(TargetPose.TransformInv(SourcePose)), FromOther(SourcePose.TransformInv(TargetPose)), DistanceTolerance(Tolerance) {}
			bool Evaluate(const Vector3& Point, DistanceFieldTargetPoint& Result) const override
			{
				if (!CachedWitness) return EvaluateExact(Point, Result);
				const auto* Field = Other->GetShapeObj<DistanceFieldShape>();
				DistanceFieldSample Sample;
				if (Field->GetField().Sample(ToOther.LocalToWorld(Point), Sample) && Sample.Value < 0.0f)
				{
					Result.Point = Point; Result.Normal = FromOther.LocalToWorldDirection(Unit(Sample.Gradient)); Result.Distance = 0.0f; return true;
				}
				// The last region-bound query already found a feasible target point.
				// Reuse it for upper bounds at the region's many zero-surface seeds.
				// Subdivision still refines both endpoints; no nearest-point claim is
				// made here, and only the independent exact query supplies lower bounds.
				Result.Point = Witness; Result.Normal = Unit(Point - Witness, WitnessNormal);
				Result.Distance = (Point - Witness).Length(); Result.ErrorBound = Result.Distance;
				return true;
			}
			double LowerBound(const Box3& Bounds) const override
			{
				if (!Other->GetShapeObj<DistanceFieldShape>()) return DistanceFieldDistanceTarget::LowerBound(Bounds);
				DistanceFieldTargetPoint Sample;
				if (!EvaluateExact(Bounds.GetCenter(), Sample)) return 0.0;
				if (Sample.Distance > 0.0f)
				{
					Witness = Sample.Point; WitnessNormal = Sample.Normal; CachedWitness = true;
				}
				const Vector3 Extent = Bounds.GetExtent();
				const double Radius = std::sqrt((double)Extent.x * Extent.x + (double)Extent.y * Extent.y + (double)Extent.z * Extent.z);
				return std::max(0.0, (double)Sample.Distance - Sample.ErrorBound - Radius);
			}
			bool EvaluateExact(const Vector3& Point, DistanceFieldTargetPoint& Result) const
			{
				const Vector3 Local = ToOther.LocalToWorld(Point);
				Vector3 ClosestPoint, Normal; float Distance;
				if (const auto* Field = Other->GetShapeObj<DistanceFieldShape>())
				{
					DistanceFieldSample Sample;
					if (Field->GetField().Sample(Local, Sample) && Sample.Value < 0.0f)
					{
						Result.Point = Point; Result.Normal = FromOther.LocalToWorldDirection(Unit(Sample.Gradient)); Result.Distance = 0.0f; return true;
					}
					DistanceFieldSurfacePoint Surface; DistanceFieldQueryStats Work;
					const bool Found = Field->GetField().ClosestSurfacePoint(Local, Surface, std::numeric_limits<float>::max(), &Work, DistanceTolerance);
					Stats.NodeVisits += Work.NodeVisits; Stats.CellTests += Work.CellTests; Stats.Subdivisions += Work.Subdivisions;
					if (!Found)
					{
						if (Surface.Status == DistanceFieldSearchStatus::InvalidInput || Surface.Converged) return false;
						// A cooked shape has a zero surface even when its query budget
						// cannot reach a witness. A cached zero supplies an upper bound;
						// zero remains the safe lower bound until a later query refines it.
						if (FallbackSamples.empty()) Field->GetSurfaceSamples(Field->GetField().GetBounds(), FallbackSamples);
						if (FallbackSamples.empty()) return false;
						Surface.Position = FallbackSamples.front().Position;
						DistanceFieldSample AtSurface; Field->GetField().Sample(Surface.Position, AtSurface); Surface.Normal = Unit(AtSurface.Gradient);
						Surface.Distance = (Local - Surface.Position).Length(); Surface.ErrorBound = Surface.Distance;
					}
					if (!std::isfinite(Surface.ErrorBound)) return false;
					if (!Surface.Converged) Status = Surface.Status;
					ClosestPoint = Surface.Position; Normal = Unit(Local - ClosestPoint, Surface.Normal);
					Distance = Surface.Distance; Result.ErrorBound = Surface.ErrorBound; Result.SupportingPlane = false;
				}
				else
				{
					if (!Closest(Other, Local, ClosestPoint, Normal, Distance)) return false;
					Result.SupportingPlane = true;
				}
				Result.Point = FromOther.LocalToWorld(ClosestPoint); Result.Normal = FromOther.LocalToWorldDirection(Normal);
				Result.Distance = std::max(0.0f, Distance); return true;
			}
			const Geometry* Other;
			Transform ToOther, FromOther;
			float DistanceTolerance;
			DistanceFieldSearchStatus GetStatus() const override { return Status; }
			mutable DistanceFieldSearchStatus Status = DistanceFieldSearchStatus::Complete;
			mutable std::vector<DistanceFieldSurfaceSample> FallbackSamples;
			mutable DistanceFieldQueryStats Stats;
			mutable bool CachedWitness = false;
			mutable Vector3 Witness, WitnessNormal;
		};
		class TriangleDistanceTarget : public DistanceFieldDistanceTarget
		{
		public:
			TriangleDistanceTarget(const Triangle3& Shape, const Transform& SourceToTarget)
				: Triangle(Shape), ToTarget(SourceToTarget), FromTarget(SourceToTarget.TransformInv(Transform())) {}
			bool Evaluate(const Vector3& Point, DistanceFieldTargetPoint& Result) const override
			{
				const Vector3 Local = ToTarget.LocalToWorld(Point);
				// Evaluate face/edge candidates in double precision. Subtracting two
				// rounded world witnesses near impact can otherwise rotate the normal.
				double Edge[2][Dimensions], Relative[Dimensions], Closest[Dimensions], Delta[Dimensions];
				double Gram00 = 0.0, Gram01 = 0.0, Gram11 = 0.0, Dot0 = 0.0, Dot1 = 0.0;
				for (int Axis = 0; Axis < Dimensions; ++Axis)
				{
					Edge[0][Axis] = (double)Triangle.v1[Axis] - Triangle.v0[Axis];
					Edge[1][Axis] = (double)Triangle.v2[Axis] - Triangle.v0[Axis];
					Relative[Axis] = (double)Local[Axis] - Triangle.v0[Axis];
					Gram00 += Edge[0][Axis] * Edge[0][Axis]; Gram01 += Edge[0][Axis] * Edge[1][Axis]; Gram11 += Edge[1][Axis] * Edge[1][Axis];
					Dot0 += Relative[Axis] * Edge[0][Axis]; Dot1 += Relative[Axis] * Edge[1][Axis];
				}
				double Best = std::numeric_limits<double>::infinity();
				auto Accept = [&](double U, double V)
				{
					double Candidate[Dimensions], Squared = 0.0;
					for (int Axis = 0; Axis < Dimensions; ++Axis)
					{
						Candidate[Axis] = Relative[Axis] - Edge[0][Axis] * U - Edge[1][Axis] * V;
						Squared += Candidate[Axis] * Candidate[Axis];
					}
					if (Squared >= Best) return;
					Best = Squared;
					for (int Axis = 0; Axis < Dimensions; ++Axis)
					{
						Delta[Axis] = Candidate[Axis]; Closest[Axis] = (double)Local[Axis] - Delta[Axis];
					}
				};
				const double Determinant = Gram00 * Gram11 - Gram01 * Gram01;
				if (Determinant > 0.0)
				{
					const double U = (Dot0 * Gram11 - Dot1 * Gram01) / Determinant;
					const double V = (Dot1 * Gram00 - Dot0 * Gram01) / Determinant;
					if (U >= 0.0 && V >= 0.0 && U + V <= 1.0) Accept(U, V);
				}
				auto Parameter = [](double Dot, double Squared) { return Squared > 0.0 ? std::max(0.0, std::min(1.0, Dot / Squared)) : 0.0; };
				Accept(Parameter(Dot0, Gram00), 0.0); Accept(0.0, Parameter(Dot1, Gram11));
				const double V = Parameter(Dot1 - Dot0 - Gram01 + Gram00, Gram00 + Gram11 - 2 * Gram01);
				Accept(1.0 - V, V);
				Result.Distance = (float)std::sqrt(Best);
				Result.Point = FromTarget.LocalToWorld(Vector3((float)Closest[0], (float)Closest[1], (float)Closest[2]));
				const Vector3 Normal = Unit(Vector3((float)Delta[0], (float)Delta[1], (float)Delta[2]), Unit((Triangle.v2 - Triangle.v0).Cross(Triangle.v1 - Triangle.v0)));
				Result.Normal = FromTarget.LocalToWorldDirection(Normal);
				Result.SupportingPlane = true; return true;
			}
			Triangle3 Triangle;
			Transform ToTarget, FromTarget;
		};
		void MeshTriangles(const Geometry* Shape, const Box3& Bounds, std::vector<Triangle3>& Triangles)
		{
			Triangles.clear();
			if (const auto* Mesh = Shape->GetShapeObj<TriangleMesh>())
			{
				std::vector<uint32_t> Indices; Mesh->QueryTriangles(Bounds, Indices);
				for (uint32_t Index : Indices)
				{
					Triangle3 Triangle; Mesh->GetTriangle(Index, &Triangle.v0, &Triangle.v1, &Triangle.v2); Triangles.push_back(Triangle);
				}
			}
			else if (const auto* Terrain = Shape->GetShapeObj<HeightField3>())
			{
				if (!Terrain->Cells || Terrain->nX < 2 || Terrain->nZ < 2) return;
				const int X0 = (int)std::max(0.0f, std::min((float)Terrain->nX - 1, std::floor((Bounds.Min.x - Terrain->BV.Min.x) * Terrain->InvDX)));
				const int Z0 = (int)std::max(0.0f, std::min((float)Terrain->nZ - 1, std::floor((Bounds.Min.z - Terrain->BV.Min.z) * Terrain->InvDZ)));
				const int X1 = (int)std::min((float)Terrain->nX - 2, std::max(-1.0f, std::floor((Bounds.Max.x - Terrain->BV.Min.x) * Terrain->InvDX)));
				const int Z1 = (int)std::min((float)Terrain->nZ - 2, std::max(-1.0f, std::floor((Bounds.Max.z - Terrain->BV.Min.z) * Terrain->InvDZ)));
				for (int Z = Z0; Z <= Z1; ++Z) for (int X = X0; X <= X1; ++X)
				{
					Vector3 Points[2 * Dimensions]; const int Count = Terrain->GetCellTriangle(X, Z, Points);
					for (int I = 0; I < Count / Dimensions; ++I) Triangles.emplace_back(Points[I * Dimensions], Points[I * Dimensions + 1], Points[I * Dimensions + 2]);
				}
			}
		}
		double BoxDistance(const Box3& A, const Box3& B)
		{
			double Squared = 0.0;
			for (int Axis = 0; Axis < Dimensions; ++Axis)
			{
				const double Gap = std::max(0.0, std::max((double)A.Min[Axis] - B.Max[Axis], (double)B.Min[Axis] - A.Max[Axis]));
				Squared += Gap * Gap;
			}
			return std::sqrt(Squared);
		}
		bool MeshDistance(const DistanceFieldShape& Field, const Geometry* SourceGeometry, const Geometry* Mesh,
			const Transform& FieldPose, const Transform& MeshPose, DistanceFieldTargetDistance& Result, DistanceFieldQueryStats& Stats, float DistanceTolerance)
		{
			const Transform ToMesh = MeshPose.TransformInv(FieldPose);
			const Box3 Source = Box3::Transform(SourceGeometry->GetShapeBounds(), ToMesh.pos, ToMesh.quat);
			const float Tolerance = Field.GetSurfaceTolerance();
			const float Limit = (Source.GetCenter() - Mesh->GetShapeBounds().GetCenter()).Length()
				+ Source.GetExtent().Length() + Mesh->GetShapeBounds().GetExtent().Length();
			float Radius = std::max(Field.GetField().GetCellSize(), (float)BoxDistance(Source, Mesh->GetShapeBounds()) + Tolerance);
			std::vector<Triangle3> Triangles;
			for (;;)
			{
				Box3 Bounds = Source; Bounds.Thicken(Radius); MeshTriangles(Mesh, Bounds, Triangles);
				if (!Triangles.empty() || Radius >= Limit) break;
				Radius = std::min(Limit, Radius * 2);
			}
			if (Triangles.empty()) { Result.Status = DistanceFieldSearchStatus::Complete; Result.Converged = true; return false; }
			std::vector<DistanceFieldSurfaceSample> Samples; Field.GetSurfaceSamples(Field.GetField().GetBounds(), Samples);
			if (Samples.empty()) return false;
			const Vector3 Seed = ToMesh.LocalToWorld(Samples.front().Position);
			float Upper = std::numeric_limits<float>::max();
			for (const auto& Triangle : Triangles) Upper = std::min(Upper, (Seed - Triangle.PointDistanceQuery(Seed).ClosestPoint).Length());
			// Any closer triangle must intersect this expanded source bound. The first
			// search supplies a feasible upper distance, not a nearest-triangle guess.
			Box3 Bounds = Source; Bounds.Thicken(Upper + Tolerance); MeshTriangles(Mesh, Bounds, Triangles);
			struct Candidate { size_t Index; double Bound; };
			std::vector<Candidate> Candidates;
			for (size_t I = 0; I < Triangles.size(); ++I)
			{
				const auto& T = Triangles[I]; const Box3 TriangleBounds(T.v0.Min(T.v1).Min(T.v2), T.v0.Max(T.v1).Max(T.v2));
				Candidates.push_back({ I, BoxDistance(Source, TriangleBounds) });
			}
			std::sort(Candidates.begin(), Candidates.end(), [](const auto& A, const auto& B) { return A.Bound < B.Bound; });
			float Lower = Upper; bool Found = false; DistanceFieldSearchStatus Status = DistanceFieldSearchStatus::Complete;
			for (const auto& Candidate : Candidates)
			{
				if (Candidate.Bound > Upper + Tolerance) break;
				TriangleDistanceTarget Target(Triangles[Candidate.Index], ToMesh);
				DistanceFieldTargetDistance Query; DistanceFieldQueryStats Work;
				const bool Witness = Field.GetField().DistanceToTarget(Target, Query, &Work, DistanceTolerance);
				Stats.NodeVisits += Work.NodeVisits; Stats.CellTests += Work.CellTests; Stats.Subdivisions += Work.Subdivisions;
				if (Query.Status == DistanceFieldSearchStatus::InvalidInput) return false;
				Lower = std::min(Lower, Query.LowerBound);
				if (!Query.Converged) Status = Query.Status;
				if (Witness && (!Found || Query.UpperBound < Result.UpperBound)) { Result = Query; Found = true; Upper = std::min(Upper, Query.UpperBound); }
			}
			Result.LowerBound = Lower; Result.Status = Status;
			Result.Converged = Found && Result.UpperBound - Lower <= DistanceTolerance;
			if (Result.Converged) Result.Status = DistanceFieldSearchStatus::Complete;
			return Found;
		}

		bool HasDistanceTarget(const Geometry* Shape)
		{
			const auto Type = Shape->GetShapeType();
			return Type == PrimitiveType::BOX || Type == PrimitiveType::SPHERE || Type == PrimitiveType::CAPSULE
				|| Type == PrimitiveType::CYLINDER || Type == PrimitiveType::CONVEX_MESH || Type == PrimitiveType::PLANE || Type == PrimitiveType::DISTANCE_FIELD
				|| Type == PrimitiveType::TRIANGLE_MESH || Type == PrimitiveType::HEIGHTFIELD;
		}
		bool SweepInterval(const Geometry* A, const Transform& PoseA, const Geometry* B, const Transform& PoseB,
			const Vector3& Direction, double& Enter, double& Exit)
		{
			if (A->GetShapeType() == PrimitiveType::PLANE || B->GetShapeType() == PrimitiveType::PLANE)
			{
				const bool PlaneMoves = A->GetShapeType() == PrimitiveType::PLANE;
				const auto* Plane = (PlaneMoves ? A : B)->GetShapeObj<Plane3>();
				const Transform& Pose = PlaneMoves ? PoseA : PoseB;
				const Transform& FieldPose = PlaneMoves ? PoseB : PoseA;
				const Geometry* Field = PlaneMoves ? B : A;
				const Vector3 Normal = Pose.LocalToWorldDirection(Unit(Plane->Normal));
				const Vector3 Anchor = Pose.LocalToWorld(-Plane->Normal * (Plane->D / Plane->Normal.SquareLength()));
				const Box3 Bounds = Box3::Transform(Field->GetShapeBounds(), FieldPose.pos, FieldPose.quat);
				const Vector3 Center = Bounds.GetCenter() - Anchor, Extent = Bounds.GetExtent();
				const double Height = Center.Dot(Normal), Radius = Extent.Dot(Normal.Abs());
				const double Rate = Direction.Dot(Normal) * (PlaneMoves ? -1 : 1);
				if (Rate >= 0.0) return false;
				Enter = std::max(Enter, std::max(0.0, Height - Radius) / -Rate);
				Exit = std::min(Exit, (Height + Radius) / -Rate); return Enter <= Exit;
			}
			const Box3 Moving = Box3::Transform(A->GetShapeBounds(), PoseA.pos, PoseA.quat);
			const Box3 Fixed = Box3::Transform(B->GetShapeBounds(), PoseB.pos, PoseB.quat);
			for (int Axis = 0; Axis < Dimensions; ++Axis)
			{
				if (Direction[Axis] == 0.0f)
				{
					if (Moving.Max[Axis] < Fixed.Min[Axis] || Moving.Min[Axis] > Fixed.Max[Axis]) return false;
					continue;
				}
				double Low = ((double)Fixed.Min[Axis] - Moving.Max[Axis]) / Direction[Axis];
				double High = ((double)Fixed.Max[Axis] - Moving.Min[Axis]) / Direction[Axis];
				if (Low > High) std::swap(Low, High);
				Enter = std::max(Enter, Low); Exit = std::min(Exit, High); if (Enter > Exit) return false;
			}
			return true;
		}

		uint32_t FieldFeature(const DistanceFieldShape& Field, const Vector3& Point)
		{
			return Field.GetSurfaceFeature(Point);
		}
		GeometryWorldState QueryState(const Geometry* Shape, const Transform& Pose)
		{
			GeometryWorldState State;
			State.Geom = const_cast<Geometry*>(Shape); State.ShapeToWorld = Pose;
			State.BodyToWorld = Pose * Shape->GetTransform()->TransformInv(Transform());
			State.WorldBounds = Box3::Transform(Shape->GetShapeBounds(), Pose.pos, Pose.quat);
			return State;
		}
		bool GenerateField(const Geometry* FieldGeometry, const Transform& FieldPose, const Geometry* Other, const Transform& OtherPose,
			float Range, bool Reverse, std::vector<DistanceFieldContact>& Contacts)
		{
			const auto& Field = *FieldGeometry->GetShapeObj<DistanceFieldShape>();
			if (!Field.IsValid()) return false;
			if (Other->GetShapeType() == PrimitiveType::TRIANGLE_MESH || Other->GetShapeType() == PrimitiveType::HEIGHTFIELD) return false;
			const float Tolerance = Field.GetSurfaceTolerance();
			auto Append = [&](const Vector3& FieldPoint, const Vector3& OtherPoint, const Vector3& Normal, uint32_t Feature)
			{
				const float Gap = (OtherPoint - FieldPoint).Dot(Normal);
				if (Gap > Range + Tolerance || !std::isfinite(Gap) || Normal.SquareLength() < MinimumNormalSquared) return false;
				DistanceFieldContact C; C.PointA = FieldPoint; C.PointB = OtherPoint; C.Normal = Normal; C.Separation = Gap; C.FeatureA = Feature;
				if (Reverse) { std::swap(C.PointA, C.PointB); std::swap(C.FeatureA, C.FeatureB); C.Normal = -C.Normal; }
				Contacts.push_back(C);
				return true;
			};
			if (const auto* Sphere = Other->GetShapeObj<Sphere3>())
			{
				const Vector3 Center = FieldPose.WorldToLocal(OtherPose.LocalToWorld(Sphere->Center));
				DistanceFieldSample Sample; const bool Inside = Field.GetField().Sample(Center, Sample) && Sample.Value < 0.0f;
				DistanceFieldSurfacePoint Surface;
				if (!Field.GetField().ClosestSurfacePoint(Center, Surface, Inside ? std::numeric_limits<float>::max() : Sphere->Radius + Range + Tolerance)) return false;
				const Vector3 Delta = Inside ? Surface.Position - Center : Center - Surface.Position;
				DistanceFieldSample AtSurface;
				Field.GetField().Sample(Surface.Position, AtSurface);
				if (Delta == Vector3::Zero() && AtSurface.Gradient == Vector3::Zero()) return false;
				const Vector3 Normal = FieldPose.LocalToWorldDirection(Unit(Delta, Unit(AtSurface.Gradient, Surface.Normal)));
				Append(FieldPose.LocalToWorld(Surface.Position), OtherPose.LocalToWorld(Sphere->Center) - Normal * Sphere->Radius,
					Normal, FieldFeature(Field, Surface.Position));
				return false;
			}
			std::vector<DistanceFieldSurfaceSample> Samples;
			Box3 Bounds = Other->GetShapeType() == PrimitiveType::PLANE ? Field.GetField().GetBounds()
				: Box3::Transform(Box3::Transform(Other->GetShapeBounds(), OtherPose.pos, OtherPose.quat), FieldPose.quat.Conjugate() * (-FieldPose.pos), FieldPose.quat.Conjugate());
			Bounds.Thicken(Range + Tolerance); Field.GetSurfaceSamples(Bounds, Samples);
			const size_t FirstContact = Contacts.size();
			bool HasInteriorWitness = false;
			Vector3 InteriorField, InteriorOther, InteriorNormal; float InteriorDepth = 0.0f; uint32_t InteriorFeature = 0;
			const Vector3 OtherCenter = OtherPose.LocalToWorld(Other->GetTransform()->WorldToLocal(Other->GetCenter()));
			DistanceFieldSample CenterSample;
			if (Samples.empty() && Field.GetField().Sample(FieldPose.WorldToLocal(OtherCenter), CenterSample) && CenterSample.Value < 0.0f)
			{
				// No zero cell crosses the intruder's bounds: it is wholly embedded.
				// Select one escape direction, rather than opposing rows on every side.
				DistanceFieldSurfacePoint Exit;
				if (Field.GetField().ClosestSurfacePoint(FieldPose.WorldToLocal(OtherCenter), Exit))
				{
					const Vector3 Direction = Unit(Exit.Position - FieldPose.WorldToLocal(OtherCenter), Exit.Normal);
					const Vector3 OtherPoint = Support(Other, OtherPose, -FieldPose.LocalToWorldDirection(Direction));
					DistanceFieldRayHit Hit;
					if (Field.GetField().RayCast(FieldPose.WorldToLocal(OtherPoint), Direction, std::numeric_limits<float>::max(), Hit, false))
						Append(FieldPose.LocalToWorld(Hit.Position), OtherPoint, FieldPose.LocalToWorldDirection(Direction), FieldFeature(Field, Hit.Position));
				}
				if (Contacts.size() > FirstContact)
				{
					const DistanceFieldContact Recovery = Contacts.back(); Contacts.assign(1, Recovery); return true;
				}
			}
			for (const auto& Sample : Samples)
			{
				Vector3 P = Sample.Position;
				for (int Iteration = 0; Iteration < Field.GetSettings().WitnessRefinementIterations; ++Iteration)
				{
					Vector3 Q, N; float Distance;
					if (!Closest(Other, OtherPose.WorldToLocal(FieldPose.LocalToWorld(P)), Q, N, Distance)) break;
					const Vector3 OtherPoint = OtherPose.LocalToWorld(Q);
					if (Distance < -InteriorDepth)
					{
						HasInteriorWitness = true; InteriorDepth = -Distance; InteriorField = FieldPose.LocalToWorld(P);
						InteriorOther = OtherPoint; InteriorNormal = -OtherPose.LocalToWorldDirection(N); InteriorFeature = Sample.Feature;
					}
					if (Distance <= Range + Tolerance)
					{
						if (Other->GetShapeType() == PrimitiveType::PLANE || Other->GetShapeType() == PrimitiveType::DISTANCE_FIELD)
							Append(FieldPose.LocalToWorld(P), OtherPoint, -OtherPose.LocalToWorldDirection(N), Sample.Feature);
						else
						{
							DistanceFieldSample AtSurface;
							if (Field.GetField().Sample(P, AtSurface) && AtSurface.Gradient != Vector3::Zero())
							{
								const Vector3 Outward = Unit(AtSurface.Gradient);
								// Reject side/exit faces of the intruder. Those are interior
								// edges of this overlap, not separate collision barriers.
								if (FieldPose.LocalToWorldDirection(Outward).Dot(OtherPose.LocalToWorldDirection(N)) < -Field.GetSettings().MinimumFacingCosine)
								{
									Vector3 QField, NField; float Signed;
									if (Field.Project(FieldPose.WorldToLocal(OtherPoint), QField, NField, Signed))
										Append(FieldPose.LocalToWorld(QField), OtherPoint, FieldPose.LocalToWorldDirection(NField), FieldFeature(Field, QField));
								}
							}
						}
						break;
					}
					Vector3 Next, FieldNormal; float SignedDistance;
					if (!Field.Project(FieldPose.WorldToLocal(OtherPoint), Next, FieldNormal, SignedDistance)
						|| (Next - P).SquareLength() <= Tolerance * Tolerance) break;
					P = Next;
				}
			}
			if (Other->GetShapeType() == PrimitiveType::PLANE || Other->GetShapeType() == PrimitiveType::DISTANCE_FIELD) return false;
			// Support directions supply vertices and curved-shape witnesses even when
			// the intruder is smaller than the spacing between grid-edge crossings.
			for (int Z = -1; Z <= 1; ++Z) for (int Y = -1; Y <= 1; ++Y) for (int X = -1; X <= 1; ++X)
			{
				if (X == 0 && Y == 0 && Z == 0) continue;
				Vector3 P = Support(Other, OtherPose, OtherPose.LocalToWorldDirection(Unit(Vector3((float)X, (float)Y, (float)Z))));
				Vector3 Q, N; float Distance;
				if (!Field.Project(FieldPose.WorldToLocal(P), Q, N, Distance) || Distance > Range + Tolerance) continue;
				Append(FieldPose.LocalToWorld(Q), P, FieldPose.LocalToWorldDirection(N), FieldFeature(Field, Q));
				const Vector3 LocalDirection = OtherPose.WorldToLocalDirection(-FieldPose.LocalToWorldDirection(N));
				// Legacy primitives use opposite face-direction conventions. Surface
				// projection rejects the far face and preserves the supporting segment.
				for (float Sign : { -1.0f, 1.0f })
				{
					SupportFace Face;
					Other->GetSupportFace(Other->GetTransform()->LocalToWorldDirection(LocalDirection * Sign), Face);
					for (int I = 0; I < Face.size(); ++I)
					{
						P = OtherPose.LocalToWorld(Other->GetTransform()->WorldToLocal(Face[I]));
						if (Field.Project(FieldPose.WorldToLocal(P), Q, N, Distance) && Distance <= Range + Tolerance)
							Append(FieldPose.LocalToWorld(Q), P, FieldPose.LocalToWorldDirection(N), FieldFeature(Field, Q));
					}
				}
			}
			bool CoversGrid = true;
			for (int Axis = 0; Axis < Dimensions; ++Axis)
				CoversGrid = CoversGrid && Bounds.Min[Axis] <= Field.GetField().GetBounds().Min[Axis] && Bounds.Max[Axis] >= Field.GetField().GetBounds().Max[Axis];
			if (Contacts.size() == FirstContact && CoversGrid && !Samples.empty() && Other->GetShapeType() != PrimitiveType::DISTANCE_FIELD)
			{
				bool Contained = true;
				for (const auto& Sample : Samples)
				{
					Vector3 Q, N; float Distance;
					if (!Closest(Other, OtherPose.WorldToLocal(FieldPose.LocalToWorld(Sample.Position)), Q, N, Distance) || Distance >= -Tolerance)
					{ Contained = false; break; }
				}
				if (Contained)
				{
					// A field completely inside a convex body needs the whole field to
					// cross a supporting plane. A nearest single sample only moves its
					// leading surface to the exit and leaves the rest trapped inside.
					float Best = std::numeric_limits<float>::max();
					Vector3 BestField, BestOther, BestNormal; uint32_t BestFeature = 0;
					for (int Axis = 0; Axis < Dimensions; ++Axis) for (int Sign : { -1, 1 })
					{
						Vector3 Local = Vector3::Zero(); Local[Axis] = (float)Sign;
						const Vector3 Outward = OtherPose.LocalToWorldDirection(Local);
						const Vector3 OtherPoint = Support(Other, OtherPose, Outward);
						const auto* Furthest = &Samples.front();
						for (const auto& Sample : Samples)
							if (FieldPose.LocalToWorldDirection(Sample.Position - Furthest->Position).Dot(Outward) < 0.0f) Furthest = &Sample;
						const Vector3 FieldPoint = FieldPose.LocalToWorld(Furthest->Position);
						const float Depth = (OtherPoint - FieldPoint).Dot(Outward);
						if (Depth >= 0.0f && Depth < Best) { Best = Depth; BestField = FieldPoint; BestOther = OtherPoint; BestNormal = -Outward; BestFeature = Furthest->Feature; }
					}
					if (Best < std::numeric_limits<float>::max())
					{
						Append(BestField, BestOther, BestNormal, BestFeature); return true;
					}
				}
			}
			// Deep crossings may have only exit-facing witnesses. Retain one deepest
			// overlap in that case, rather than dropping the collision or imposing
			// conflicting rows on both sides of a straddling volume.
			if (Contacts.size() == FirstContact && HasInteriorWitness)
				Append(InteriorField, InteriorOther, InteriorNormal, InteriorFeature);
			return false;
		}
	}
	bool DistanceFieldCollision::HasField(const Geometry* A, const Geometry* B)
	{
		return A && B && (A->GetShapeType() == PrimitiveType::DISTANCE_FIELD || B->GetShapeType() == PrimitiveType::DISTANCE_FIELD);
	}
	bool DistanceFieldCollision::Generate(const Geometry* A, const Transform& PoseA, const Geometry* B, const Transform& PoseB,
		float ContactDistance, std::vector<DistanceFieldContact>& Contacts)
	{
		Contacts.clear(); if (!HasField(A, B) || !std::isfinite(ContactDistance) || ContactDistance < 0.0f) return false;
		if (A->GetShapeType() == PrimitiveType::TRIANGLE_MESH || A->GetShapeType() == PrimitiveType::HEIGHTFIELD
			|| B->GetShapeType() == PrimitiveType::TRIANGLE_MESH || B->GetShapeType() == PrimitiveType::HEIGHTFIELD)
		{
			MeshContactGenerator Generator;
			std::vector<ContactManifold> Patches;
			Generator.Generate(0, 1, QueryState(A, PoseA), QueryState(B, PoseB), Patches, ContactDistance);
			for (const auto& Patch : Patches) for (int I = 0; I < Patch.NumContactPointCount; ++I)
			{
				const auto& Point = Patch.ContactPoints[I];
				DistanceFieldContact Contact; Contact.PointA = Point.PositionWorldA; Contact.PointB = Point.PositionWorldB;
				Contact.Normal = Point.Normal; Contact.Separation = -Point.PenetrationDepth;
				Contact.FeatureA = Point.FeatureA; Contact.FeatureB = Point.FeatureB; Contacts.push_back(Contact);
			}
			return !Contacts.empty();
		}
		const bool Recovered = A->GetShapeType() == PrimitiveType::DISTANCE_FIELD && GenerateField(A, PoseA, B, PoseB, ContactDistance, false, Contacts);
		if (!Recovered && B->GetShapeType() == PrimitiveType::DISTANCE_FIELD) GenerateField(B, PoseB, A, PoseA, ContactDistance, true, Contacts);
		if (Contacts.empty() && ContactDistance > 0.0f)
		{
			DistanceFieldPairDistance Query;
			const auto* FieldA = A->GetShapeObj<DistanceFieldShape>(); const auto* FieldB = B->GetShapeObj<DistanceFieldShape>();
			const float Tolerance = FieldA ? FieldA->GetSurfaceTolerance() : FieldB->GetSurfaceTolerance();
			if (Distance(A, PoseA, B, PoseB, Query) && Query.UpperBound <= ContactDistance + Tolerance)
			{
				const Vector3 Delta = Query.PointB - Query.PointA; const float Gap = Delta.Length();
				if (Gap > 0.0f)
				{
					DistanceFieldContact Contact; Contact.PointA = Query.PointA; Contact.PointB = Query.PointB;
					Contact.Normal = Delta / Gap; Contact.Separation = Gap;
					if (FieldA) Contact.FeatureA = FieldA->GetSurfaceFeature(PoseA.WorldToLocal(Contact.PointA));
					if (FieldB) Contact.FeatureB = FieldB->GetSurfaceFeature(PoseB.WorldToLocal(Contact.PointB));
					Contacts.push_back(Contact);
				}
			}
		}
		return !Contacts.empty();
	}
	bool DistanceFieldCollision::Intersect(const Geometry* A, const Transform& PoseA, const Geometry* B, const Transform& PoseB)
	{
		std::vector<DistanceFieldContact> Contacts;
		Generate(A, PoseA, B, PoseB, 0.0f, Contacts);
		for (const auto& C : Contacts) if (C.Separation <= 0.0f) return true;
		return false;
	}
	bool DistanceFieldCollision::Distance(const Geometry* A, const Transform& PoseA, const Geometry* B, const Transform& PoseB,
		DistanceFieldPairDistance& Result, float DistanceTolerance)
	{
		Result = DistanceFieldPairDistance(); if (!HasField(A, B) || !std::isfinite(DistanceTolerance) || DistanceTolerance < 0.0f) return false;
		const bool FieldIsA = A->GetShapeType() == PrimitiveType::DISTANCE_FIELD;
		const auto* Field = (FieldIsA ? A : B)->GetShapeObj<DistanceFieldShape>();
		const Geometry* Other = FieldIsA ? B : A;
		if (!Field->IsValid() || !HasDistanceTarget(Other)) return false;
		const Transform& FieldPose = FieldIsA ? PoseA : PoseB;
		// Reserve half the requested error for the nested target-field query.
		// Its error must be smaller than the outer tolerance to permit convergence.
		const float Tolerance = DistanceTolerance > 0.0f ? DistanceTolerance : Field->GetField().GetCellSize() * Field->GetSettings().RelativeSweepTolerance;
		GeometryDistanceTarget Target(Other, FieldPose, FieldIsA ? PoseB : PoseA, Tolerance / 2);
		DistanceFieldTargetDistance Query;
		const bool Mesh = Other->GetShapeType() == PrimitiveType::TRIANGLE_MESH || Other->GetShapeType() == PrimitiveType::HEIGHTFIELD;
		const bool Found = Mesh ? MeshDistance(*Field, FieldIsA ? A : B, Other, FieldPose, FieldIsA ? PoseB : PoseA, Query, Result.Stats, Tolerance)
			: Field->GetField().DistanceToTarget(Target, Query, &Result.Stats, Tolerance);
		Result.Stats.NodeVisits += Target.Stats.NodeVisits; Result.Stats.CellTests += Target.Stats.CellTests; Result.Stats.Subdivisions += Target.Stats.Subdivisions;
		Result.WitnessValid = Found;
		Result.LowerBound = Query.LowerBound; Result.UpperBound = Query.UpperBound;
		Result.Converged = Query.Converged; Result.Status = Query.Status;
		if (Found)
		{
			Result.PointA = FieldPose.LocalToWorld(Query.Position); Result.PointB = FieldPose.LocalToWorld(Query.TargetPoint);
			Result.Normal = FieldPose.LocalToWorldDirection(Query.Normal);
			if (!FieldIsA) { std::swap(Result.PointA, Result.PointB); Result.Normal = -Result.Normal; }
		}
		return Found;
	}

	/* Translation changes the distance between two fixed-orientation closed sets
	   by at most speed * time. Each advance consumes only the certified lower bound.
	   A search/work limit can stop motion conservatively; it is exposed separately
	   from an ordinary hit so callers do not mistake a budget decision for exact TOI.
	   The initial overlap test uses solid semantics, including containment: minimum
	   distance from one boundary alone cannot detect a smaller object wholly inside.
	*/
	bool DistanceFieldCollision::SweepBounded(const Geometry* A, const Transform& PoseA, const Geometry* B, const Transform& PoseB,
		const Vector3& LocalDirection, float MaxTime, DistanceFieldSweepResult& Result)
	{
		Result = DistanceFieldSweepResult();
		if (!HasField(A, B) || !std::isfinite(MaxTime) || MaxTime < 0.0f
			|| !std::isfinite(LocalDirection.x) || !std::isfinite(LocalDirection.y) || !std::isfinite(LocalDirection.z)) return false;
		const auto* Field = A->GetShapeObj<DistanceFieldShape>(); if (!Field) Field = B->GetShapeObj<DistanceFieldShape>();
		if (!Field->IsValid() || !HasDistanceTarget(A) || !HasDistanceTarget(B)) return false;
		const Vector3 Direction = PoseA.LocalToWorldDirection(LocalDirection);
		const double Speed = std::sqrt((double)Direction.x * Direction.x + (double)Direction.y * Direction.y + (double)Direction.z * Direction.z);
		std::vector<DistanceFieldContact> Initial;
		Generate(A, PoseA, B, PoseB, 0.0f, Initial);
		for (const auto& Contact : Initial) if (Contact.Separation <= 0.0f)
		{
			Result.Hit = Result.InitialOverlap = true; Result.Status = DistanceFieldSearchStatus::Complete;
			Result.Position = PoseA.WorldToLocal(Contact.PointA); Result.Normal = PoseA.WorldToLocalDirection(-Contact.Normal); return true;
		}
		Result.Status = DistanceFieldSearchStatus::Complete;
		if (!(Speed > 0.0) || MaxTime == 0.0f) return false;
		double Enter = 0.0, Exit = MaxTime;
		if (!SweepInterval(A, PoseA, B, PoseB, Direction, Enter, Exit)) return false;
		float Time = (float)Enter; if (Time > Enter) Time = std::nextafter(Time, 0.0f);
		const float Tolerance = Field->GetField().GetCellSize() * Field->GetSettings().RelativeSweepTolerance;
		DistanceFieldPairDistance Query;
		auto Hit = [&](bool Conservative)
		{
			Result.Hit = true; Result.Conservative = Conservative; Result.Time = Time;
			Result.DistanceError = Query.UpperBound - Query.LowerBound;
			const Vector3 Fallback = PoseA.LocalToWorld(A->GetTransform()->WorldToLocal(A->GetCenter())) + Direction * Time;
			Result.Position = PoseA.WorldToLocal(Query.WitnessValid ? Query.PointA : Fallback);
			Vector3 Normal = Unit(Query.Normal, Unit(-Direction)); if (Normal.Dot(Direction) > 0.0f) Normal = -Normal;
			Result.Normal = PoseA.WorldToLocalDirection(Normal);
			Result.Status = Conservative ? Query.Status : DistanceFieldSearchStatus::Complete;
			return true;
		};
		for (int Iteration = 0; Iteration < Field->GetSettings().MaximumSweepIterations && Time <= Exit; ++Iteration)
		{
			++Result.Iterations; Transform Moving = PoseA; Moving.pos += Direction * Time;
			Distance(A, Moving, B, PoseB, Query, Tolerance / 2);
			Result.Stats.NodeVisits += Query.Stats.NodeVisits; Result.Stats.CellTests += Query.Stats.CellTests; Result.Stats.Subdivisions += Query.Stats.Subdivisions;
			if (Query.Status == DistanceFieldSearchStatus::InvalidInput) { Result.Status = Query.Status; return false; }
			if (!Query.WitnessValid && Query.Converged) return false;
			if (Query.UpperBound <= Tolerance) return Hit(false);
			if (!(Query.LowerBound > 0.0f)) return Hit(true);
			const double Next = Time + Query.LowerBound / Speed;
			if (Next > Exit) return false;
			float Rounded = (float)Next; if (Rounded > Next) Rounded = std::nextafter(Rounded, 0.0f);
			if (Rounded <= Time) return Hit(true);
			if (Iteration + 1 == Field->GetSettings().MaximumSweepIterations) { Query.Status = DistanceFieldSearchStatus::WorkLimit; return Hit(true); }
			Time = Rounded;
		}
		Query.Status = DistanceFieldSearchStatus::WorkLimit; return Hit(true);
	}

	bool DistanceFieldCollision::Sweep(const Geometry* A, const Transform& PoseA, const Geometry* B, const Transform& PoseB,
		const Vector3& LocalDirection, Vector3* Position, Vector3* Normal, float* Time)
	{
		DistanceFieldSweepResult Result;
		if (!SweepBounded(A, PoseA, B, PoseB, LocalDirection, std::numeric_limits<float>::max(), Result)) return false;
		if (Position) *Position = Result.Position;
		if (Normal) *Normal = Result.Normal;
		if (Time) *Time = Result.Time;
		return true;
	}
}
