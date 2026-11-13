#include "ContactGeneration.h"
#include "RigidBody.h"
#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/Plane3.h"
#include "../CollisionPrimitive/AxisAlignedBox3.h"
#include "../CollisionPrimitive/Sphere3.h"
#include "../Geometry/Polygon3.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace Riemann
{
	/* Contact offsets enlarge the search envelope, not the geometry. Rest offsets
	   prescribe a signed surface gap: positive values separate surfaces, negative
	   values permit overlap. The solver error is penetration + restOffset. Anchors
	   remain on the original surfaces so cache refresh does not grow the shapes.
	   Offsets use simulation length units. LengthScale scales the minimum tolerance
	   and motion cap; relative tolerances and solver slop follow the smaller finite
	   shape. UseShapeOffsets bypasses stored offsets for direct demo comparisons.

	   Speculative contact uses the same distance constraints with a motion-expanded
	   envelope. For a positive remaining gap g, v_normal >= -g / dt permits approach
	   without crossing the target gap. Friction and restitution are inactive across
	   an open gap. This is a velocity bound, not a time-of-impact/resweep algorithm;
	   abrupt changes of velocity or contact normal still favor swept CCD. A PBD/XPBD
	   velocity prepass bounds motion before pose prediction, otherwise a thin obstacle
	   could already be crossed when position contacts are first generated.

	   Separated convex pairs use normalized GJK distance and witness reconstruction.
	   Each simplex reduction minimizes squared distance over its Voronoi features;
	   solving at most three small Gram equations makes degeneracies explicit. Edges
	   and barycentric reduction use double precision so nearly parallel support
	   faces do not lose their separating direction through cancellation. Face
	   clipping preserves a supporting patch instead of one arbitrary support corner.
	   See Gino van den Bergen, Collision Detection in Interactive 3D Environments.
	*/
	bool ContactGenerationSettings::IsValid() const
	{
		return std::isfinite(LengthScale) && LengthScale > 0.0f
			&& std::isfinite(RelativeTolerance) && RelativeTolerance > 0.0f
			&& std::isfinite(MinimumTolerance) && MinimumTolerance > 0.0f
			&& std::isfinite(RelativePenetrationSlop) && RelativePenetrationSlop >= 0.0f
			&& std::isfinite(MaximumSpeculativeDistance) && MaximumSpeculativeDistance >= 0.0f
			&& std::isfinite(FaceClipSquaredDistance) && FaceClipSquaredDistance >= 0.0f && DistanceIterations > 0
			&& std::isfinite(LengthScale * MaximumSpeculativeDistance)
			&& std::isfinite(LengthScale * LengthScale * FaceClipSquaredDistance);
	}

	bool ContactGeneration::UsesOffsets(const GeometryWorldState& A, const GeometryWorldState& B,
		const ContactGenerationSettings& Settings)
	{
		if (!A.Geom || !B.Geom || A.Geom->IsTriggerEnabled() || B.Geom->IsTriggerEnabled()) return false;
		return Settings.SpeculativeContacts || (Settings.UseShapeOffsets && (A.Geom->GetContactOffset() != 0.0f || B.Geom->GetContactOffset() != 0.0f
			|| A.Geom->GetRestOffset() != 0.0f || B.Geom->GetRestOffset() != 0.0f));
	}
	float ContactGeneration::ShapeScale(const Geometry* Shape, const ContactGenerationSettings& Settings)
	{
		if (!Shape || Shape->GetShapeType() == PrimitiveType::PLANE) return Settings.LengthScale;
		const float Extent = Shape->GetShapeBounds().GetExtent().Length();
		return std::max(Settings.MinimumTolerance * Settings.LengthScale, Extent);
	}
	float ContactGeneration::PairScale(const GeometryWorldState& A, const GeometryWorldState& B,
		const ContactGenerationSettings& Settings)
	{
		if (A.Geom->GetShapeType() == PrimitiveType::PLANE) return ShapeScale(B.Geom, Settings);
		if (B.Geom->GetShapeType() == PrimitiveType::PLANE) return ShapeScale(A.Geom, Settings);
		return std::min(ShapeScale(A.Geom, Settings), ShapeScale(B.Geom, Settings));
	}
	float ContactGeneration::Tolerance(const GeometryWorldState& A, const GeometryWorldState& B,
		const ContactGenerationSettings& Settings)
	{
		return std::max(Settings.MinimumTolerance * Settings.LengthScale, PairScale(A, B, Settings) * Settings.RelativeTolerance);
	}
	uint32_t ContactGeneration::Feature(const GeometryWorldState& State, const Vector3& Position,
		const ContactGenerationSettings& Settings)
	{
		const PrimitiveType Type = State.Geom->GetShapeType();
		if (Type == PrimitiveType::SPHERE || Type == PrimitiveType::PLANE) return 0;
		if (Type != PrimitiveType::BOX) return UINT32_MAX;
		const auto* Box = State.Geom->GetShapeObj<AxisAlignedBox3>();
		const Vector3 Local = State.ShapeToWorld.WorldToLocal(Position) - Box->GetCenter();
		const Vector3 Extent = Box->GetExtent();
		// Analytic and GJK witnesses share one local feature convention. Scale the
		// classification tolerance with this shape, preserving IDs across length units.
		const float FeatureTolerance = Tolerance(State, State, Settings);
		uint32_t Mask = 0;
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			if (std::abs(std::abs(Local[Axis]) - Extent[Axis]) <= FeatureTolerance)
				Mask |= 1u << (2 * Axis + (Local[Axis] >= 0.0f ? 1 : 0));
		}
		return Mask ? Mask : UINT32_MAX;
	}
	float ContactGeneration::MotionDistance(const GeometryWorldState& State, const ContactGenerationSettings& Settings, float Dt)
	{
		if (!Settings.SpeculativeContacts || !State.Body || !State.MovingRigid || Dt <= 0.0f
			|| State.Geom->IsTriggerEnabled()) return 0.0f;
		const float Radius = State.Geom->GetShapeType() == PrimitiveType::PLANE ? 0.0f
			: (State.ShapeToWorld.pos - State.Body->X).Length() + ShapeScale(State.Geom, Settings);
		return std::min(Settings.MaximumSpeculativeDistance * Settings.LengthScale,
			(State.Body->V.Length() + State.Body->W.Length() * Radius) * Dt);
	}
	float ContactGeneration::GenerationDistance(const GeometryWorldState& A, const GeometryWorldState& B,
		const ContactGenerationSettings& Settings, float Dt)
	{
		if (!UsesOffsets(A, B, Settings)) return 0.0f;
		return (Settings.UseShapeOffsets ? A.Geom->GetContactOffset() + B.Geom->GetContactOffset() : 0.0f)
			+ MotionDistance(A, Settings, Dt) + MotionDistance(B, Settings, Dt);
	}
	Box3 ContactGeneration::Bounds(const GeometryWorldState& State, const ContactGenerationSettings& Settings, float Dt)
	{
		Box3 Result = State.WorldBounds;
		if (State.Geom && !State.Geom->IsTriggerEnabled())
			Result.Thicken((Settings.UseShapeOffsets ? State.Geom->GetContactOffset() : 0.0f) + MotionDistance(State, Settings, Dt));
		return Result;
	}

	namespace
	{
		const int SimplexCapacity = 4;
		const int SpatialDimensions = 3;
		const double LinearSolveTolerance = 1.0e-12;
		struct SupportVertex { Vector3 Difference, A, B; };
		Vector3 Support(const GeometryWorldState& State, const Vector3& Direction)
		{
			return State.BodyToWorld.LocalToWorld(State.Geom->GetSupport(State.BodyToWorld.WorldToLocalDirection(Direction)));
		}
		std::vector<Vector3> Face(const GeometryWorldState& State, const Vector3& Direction)
		{
			std::vector<Vector3> Best;
			float BestProjection = -std::numeric_limits<float>::max();
			for (float Sign : { -1.0f, 1.0f })
			{
				SupportFace Local;
				State.Geom->GetSupportFace(State.BodyToWorld.WorldToLocalDirection(Direction * Sign), Local);
				if (Local.empty()) continue;
				std::vector<Vector3> Points;
				float Projection = 0.0f;
				for (int Index = 0; Index < Local.size(); ++Index)
				{
					Points.push_back(State.BodyToWorld.LocalToWorld(Local[Index]));
					Projection += Points.back().Dot(Direction);
				}
				Projection /= (float)Points.size();
				if (Projection > BestProjection) { BestProjection = Projection; Best.swap(Points); }
			}
			return Best;
		}
		bool ReduceSimplex(SupportVertex* Vertices, int& Count, Vector3& Closest, Vector3& PointA, Vector3& PointB)
		{
			double BestDistance = std::numeric_limits<double>::max();
			double BestWeights[SimplexCapacity] = {};
			int BestMask = 0;
			for (int Mask = 1; Mask < (1 << Count); ++Mask)
			{
				int Indices[SimplexCapacity], Size = 0;
				for (int Index = 0; Index < Count; ++Index) if (Mask & (1 << Index)) Indices[Size++] = Index;
				const Vector3 Origin = Vertices[Indices[0]].Difference;
				double Edges[SpatialDimensions][SpatialDimensions] = {};
				const int Rank = Size - 1;
				for (int Row = 0; Row < Rank; ++Row)
					for (int Axis = 0; Axis < SpatialDimensions; ++Axis)
						Edges[Row][Axis] = (double)Vertices[Indices[Row + 1]].Difference[Axis] - Origin[Axis];
				double Matrix[SpatialDimensions][SpatialDimensions + 1] = {};
				for (int Row = 0; Row < Rank; ++Row)
				for (int Axis = 0; Axis < SpatialDimensions; ++Axis)
				{
					for (int Col = 0; Col < Rank; ++Col) Matrix[Row][Col] += Edges[Row][Axis] * Edges[Col][Axis];
					Matrix[Row][Rank] -= Edges[Row][Axis] * Origin[Axis];
				}
				bool Valid = true;
				for (int Col = 0; Col < Rank; ++Col)
				{
					int Pivot = Col;
					for (int Row = Col + 1; Row < Rank; ++Row)
						if (std::abs(Matrix[Row][Col]) > std::abs(Matrix[Pivot][Col])) Pivot = Row;
					if (std::abs(Matrix[Pivot][Col]) <= LinearSolveTolerance) { Valid = false; break; }
					for (int Entry = Col; Entry <= Rank; ++Entry) std::swap(Matrix[Col][Entry], Matrix[Pivot][Entry]);
					const double Divisor = Matrix[Col][Col];
					for (int Entry = Col; Entry <= Rank; ++Entry) Matrix[Col][Entry] /= Divisor;
					for (int Row = 0; Row < Rank; ++Row)
					{
						if (Row == Col) continue;
						const double Factor = Matrix[Row][Col];
						for (int Entry = Col; Entry <= Rank; ++Entry) Matrix[Row][Entry] -= Factor * Matrix[Col][Entry];
					}
				}
				if (!Valid) continue;
				double Weights[SimplexCapacity] = { 1.0, 0.0, 0.0, 0.0 };
				for (int Row = 0; Row < Rank; ++Row) { Weights[Row + 1] = Matrix[Row][Rank]; Weights[0] -= Weights[Row + 1]; }
				double Point[SpatialDimensions] = {};
				for (int Index = 0; Index < Size; ++Index)
				{
					if (Weights[Index] < 0.0) Valid = false;
					for (int Axis = 0; Axis < SpatialDimensions; ++Axis)
						Point[Axis] += Vertices[Indices[Index]].Difference[Axis] * Weights[Index];
				}
				const double Distance = Point[0] * Point[0] + Point[1] * Point[1] + Point[2] * Point[2];
				if (!Valid || Distance >= BestDistance) continue;
				BestDistance = Distance; BestMask = Mask;
				std::fill(BestWeights, BestWeights + SimplexCapacity, 0.0);
				for (int Index = 0; Index < Size; ++Index) BestWeights[Indices[Index]] = Weights[Index];
			}
			if (!BestMask) return false;
			Closest = PointA = PointB = Vector3::Zero();
			double ReducedPoint[SpatialDimensions] = {};
			int ReducedCount = 0;
			for (int Index = 0; Index < Count; ++Index)
			{
				if (!(BestMask & (1 << Index))) continue;
				const float Weight = (float)BestWeights[Index];
				for (int Axis = 0; Axis < SpatialDimensions; ++Axis)
					ReducedPoint[Axis] += Vertices[Index].Difference[Axis] * BestWeights[Index];
				PointA += Vertices[Index].A * Weight; PointB += Vertices[Index].B * Weight;
				Vertices[ReducedCount++] = Vertices[Index];
			}
			for (int Axis = 0; Axis < SpatialDimensions; ++Axis) Closest[Axis] = (float)ReducedPoint[Axis];
			Count = ReducedCount;
			return true;
		}
		bool ConvexDistance(const GeometryWorldState& A, const GeometryWorldState& B, const ContactGenerationSettings& Settings,
			Vector3& PointA, Vector3& PointB, Vector3& Normal)
		{
			const float Scale = ContactGeneration::PairScale(A, B, Settings);
			const float Tolerance = ContactGeneration::Tolerance(A, B, Settings) / Scale;
			Vector3 Direction = (B.ShapeToWorld.pos - A.ShapeToWorld.pos).SafeUnit();
			if (Direction.IsZero()) Direction = Vector3::UnitX();
			auto Vertex = [&](const Vector3& Axis) -> SupportVertex
			{
				SupportVertex V;
				V.A = Support(A, Axis); V.B = Support(B, -Axis);
				V.Difference = (V.A - V.B) / Scale;
				return V;
			};
			SupportVertex Vertices[SimplexCapacity]; Vertices[0] = Vertex(Direction);
			int Count = 1;
			Vector3 Closest = Vertices[0].Difference;
			PointA = Vertices[0].A; PointB = Vertices[0].B;
			Normal = Direction;
			float SupportDepth = Vertices[0].Difference.Dot(Normal);
			float MinimumSupportDepth = SupportDepth;
			Vector3 SeparatingNormal = Normal;
			for (int Iteration = 0; Iteration < Settings.DistanceIterations; ++Iteration)
			{
				const float Distance = Closest.Length();
				// A simplex on the origin is also the touching case. Keep the best
				// separating plane when its support gap proves negligible penetration;
				// otherwise EPA must recover the normal of an actual overlap.
				if (Distance <= Tolerance)
				{
					if (MinimumSupportDepth <= Tolerance) { Normal = SeparatingNormal; return true; }
					if (Distance == 0.0f || Count == SimplexCapacity) return false;
				}
				Normal = -Closest / Distance;
				const SupportVertex Next = Vertex(Normal);
				SupportDepth = Next.Difference.Dot(Normal);
				if (SupportDepth < MinimumSupportDepth) { MinimumSupportDepth = SupportDepth; SeparatingNormal = Normal; }
				const float Improvement = Closest.Dot(Closest - Next.Difference);
				bool Duplicate = false;
				for (int Index = 0; Index < Count; ++Index)
					Duplicate = Duplicate || (Next.Difference - Vertices[Index].Difference).SquareLength() <= Tolerance * Tolerance;
				if (Duplicate || Improvement <= Tolerance * Distance || Count == SimplexCapacity)
				{
					Normal = -Closest / Distance;
					return SupportDepth <= Tolerance;
				}
				Vertices[Count++] = Next;
				if (!ReduceSimplex(Vertices, Count, Closest, PointA, PointB)) return false;
			}
			return false;
		}
	}

	bool ContactGeneration::GenerateSeparated(int IndexA, int IndexB, const GeometryWorldState& A, const GeometryWorldState& B,
		const ContactGenerationSettings& Settings, float Dt, ContactManifold& Manifold, bool& SeparationKnown)
	{
		SeparationKnown = false;
		const PrimitiveType TypeA = A.Geom->GetShapeType(), TypeB = B.Geom->GetShapeType();
		const bool AnalyticPlane = (TypeA == PrimitiveType::PLANE || TypeB == PrimitiveType::PLANE)
			&& TypeA != TypeB && TypeA != PrimitiveType::TRIANGLE_MESH && TypeB != PrimitiveType::TRIANGLE_MESH
			&& TypeA != PrimitiveType::HEIGHTFIELD && TypeB != PrimitiveType::HEIGHTFIELD;
		const bool AnalyticSphereBox = (TypeA == PrimitiveType::SPHERE && TypeB == PrimitiveType::BOX)
			|| (TypeB == PrimitiveType::SPHERE && TypeA == PrimitiveType::BOX);
		const bool Offsets = UsesOffsets(A, B, Settings);
		if (!Offsets && !AnalyticPlane && !AnalyticSphereBox) return false;
		// An infinite half-space has no finite GJK support. Its bounding slab is
		// suitable for broad phase only: EPA witnesses on that proxy can drift into
		// the floor and poison PCM refresh under heavy stacks. Always project convex
		// support witnesses onto the actual plane, even in the legacy offset mode.
		const float Range = Offsets ? GenerationDistance(A, B, Settings, Dt) : 0;
		const float Epsilon = Offsets ? Tolerance(A, B, Settings) : 0;
		auto Add = [&](const Vector3& PointA, const Vector3& PointB, const Vector3& Normal)
		{
			const float Depth = (PointA - PointB).Dot(Normal);
			if (Depth < -Range - Epsilon) return;
			Contact Point;
			Point.PositionWorldA = PointA; Point.PositionWorldB = PointB;
			Point.PositionLocalA = PointA - (A.Body ? A.Body->X : A.ShapeToWorld.pos);
			Point.PositionLocalB = PointB - (B.Body ? B.Body->X : B.ShapeToWorld.pos);
			Point.Normal = Normal; Normal.GetPerpVectors(Point.Tangent, Point.Binormal);
			Point.PenetrationDepth = Depth;
			Point.FeatureA = Feature(A, PointA, Settings);
			Point.FeatureB = Feature(B, PointB, Settings);
			Manifold.AddNewContact(IndexA, IndexB, Point);
		};
		if (AnalyticSphereBox)
		{
			/* A sphere against a box has an exact closest feature. GJK/EPA's shallow
			   overlap simplex can become degenerate here, intermittently dropping a
			   resting contact and injecting noise into a coupled attachment's load.
			   Clamp in box-local space for exterior face/edge/corner witnesses. An
			   interior center instead exits through the nearest face; its sphere
			   witness lies opposite that exit normal. Both witnesses remain on the
			   physical surfaces, including when offsets admit a separated pair. */
			const bool SphereIsA = TypeA == PrimitiveType::SPHERE;
			const auto& SphereState = SphereIsA ? A : B; const auto& BoxState = SphereIsA ? B : A;
			const auto* Sphere = SphereState.Geom->GetShapeObj<Sphere3>();
			const auto* Box = BoxState.Geom->GetShapeObj<AxisAlignedBox3>();
			const Vector3 Center = SphereState.ShapeToWorld.LocalToWorld(Sphere->Center);
			const Vector3 LocalCenter = BoxState.ShapeToWorld.WorldToLocal(Center);
			Vector3 LocalPoint = Box->ClosestPointToPoint(LocalCenter), Normal = LocalCenter - LocalPoint;
			const double Distance = std::hypot(std::hypot((double)Normal.x, Normal.y), Normal.z);
			if (Distance > 0)
				Normal = Vector3((float)(Normal.x / Distance), (float)(Normal.y / Distance), (float)(Normal.z / Distance));
			else
			{
				const Vector3 Offset = LocalCenter - Box->GetCenter(), Extent = Box->GetExtent();
				int Axis = 0;
				for (int I = 1; I < 3; ++I) if (Extent[I] - std::abs(Offset[I]) < Extent[Axis] - std::abs(Offset[Axis])) Axis = I;
				Normal = Vector3::Zero(); Normal[Axis] = Offset[Axis] < 0 ? -1.0f : 1.0f;
				LocalPoint[Axis] = Box->GetCenter()[Axis] + Normal[Axis] * Extent[Axis];
			}
			Normal = BoxState.ShapeToWorld.LocalToWorldDirection(Normal).SafeUnit();
			const Vector3 BoxPoint = BoxState.ShapeToWorld.LocalToWorld(LocalPoint), SpherePoint = Center - Normal * Sphere->Radius;
			SeparationKnown = true;
			if (SphereIsA) Add(SpherePoint, BoxPoint, -Normal); else Add(BoxPoint, SpherePoint, Normal);
		}
		else if (TypeA == PrimitiveType::PLANE || TypeB == PrimitiveType::PLANE)
		{
			if (TypeA == TypeB) return false;
			const bool PlaneIsA = TypeA == PrimitiveType::PLANE;
			const GeometryWorldState& PlaneState = PlaneIsA ? A : B;
			const GeometryWorldState& Convex = PlaneIsA ? B : A;
			const Plane3* Plane = PlaneState.Geom->GetShapeObj<Plane3>();
			const Vector3 Normal = PlaneState.ShapeToWorld.LocalToWorldDirection(Plane->Normal).SafeUnit();
			const Vector3 Origin = PlaneState.ShapeToWorld.LocalToWorld(Plane->GetOrigin());
			SeparationKnown = true;
			std::vector<Vector3> Points = Face(Convex, -Normal);
			if (Points.empty()) Points.push_back(Support(Convex, -Normal));
			for (const Vector3& Point : Points)
			{
				const Vector3 Projection = Point - Normal * (Point - Origin).Dot(Normal);
				if (PlaneIsA) Add(Projection, Point, Normal); else Add(Point, Projection, -Normal);
			}
		}
		else
		{
			if (TypeA == PrimitiveType::HEIGHTFIELD || TypeA == PrimitiveType::TRIANGLE_MESH
				|| TypeB == PrimitiveType::HEIGHTFIELD || TypeB == PrimitiveType::TRIANGLE_MESH) return false;
			Vector3 PointA, PointB, Normal;
			if (!ConvexDistance(A, B, Settings, PointA, PointB, Normal)) return false;
			SeparationKnown = true;
			if ((PointB - PointA).Length() > Range + Epsilon) return false;
			const std::vector<Vector3> FaceA = Face(A, Normal), FaceB = Face(B, -Normal);
			std::array<Vector3, 2 * MAX_FACE_POINTS> ClippedA, ClippedB;
			int CountA = 0, CountB = 0;
			if (FaceA.size() >= 2 && FaceB.size() >= 3)
				ClipPolygonAgainPolygon3D(FaceA.data(), (int)FaceA.size(), FaceB.data(), (int)FaceB.size(), Normal,
					(Range + Epsilon) * (Range + Epsilon), ClippedA.data(), &CountA, ClippedB.data(), &CountB);
			else if (FaceA.size() >= 3 && FaceB.size() >= 2)
				ClipPolygonAgainPolygon3D(FaceB.data(), (int)FaceB.size(), FaceA.data(), (int)FaceA.size(), -Normal,
					(Range + Epsilon) * (Range + Epsilon), ClippedB.data(), &CountB, ClippedA.data(), &CountA);
			for (int Index = 0; Index < CountA; ++Index) Add(ClippedA[Index], ClippedB[Index], Normal);
			if (!Manifold.NumContactPointCount) Add(PointA, PointB, Normal);
		}
		return Manifold.NumContactPointCount > 0;
	}

	void ContactGeneration::ApplyOffsets(const GeometryWorldState& A, const GeometryWorldState& B,
		const ContactGenerationSettings& Settings, ContactManifold& Manifold)
	{
		if (!UsesOffsets(A, B, Settings)) return;
		const float RestOffset = Settings.UseShapeOffsets ? A.Geom->GetRestOffset() + B.Geom->GetRestOffset() : 0.0f;
		const float Slop = PairScale(A, B, Settings) * Settings.RelativePenetrationSlop;
		for (int Index = 0; Index < Manifold.NumContactPointCount; ++Index)
		{
			Contact& Point = Manifold.ContactPoints[Index];
			Point.PenetrationDepth += RestOffset - Point.RestOffset;
			Point.RestOffset = RestOffset; Point.SolverSlop = Slop;
		}
	}
}
