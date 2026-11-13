
#include "NarrowPhase.h"
#include <algorithm>
#include <array>
#include <cmath>
#include "CollidingContact.h"
#include "RigidBody.h"
#include "../Core/BatchList.h"
#include "../Geometry/Polygon3.h"
#include "../Collision/GeometryObject.h"
#include "../Collision/DistanceFieldCollision.h"
#include "../CollisionPrimitive/DistanceFieldShape.h"
#include "../Collision/GeometryDifference.h"
#include "../Collision/EPAPenetration.h"
#include "../CollisionPrimitive/AxisAlignedBox3.h"
#include "../CollisionPrimitive/GJK.h"
#include "../CollisionPrimitive/MinkowskiSum.h"
#include "../CollisionPrimitive/OrientedBox3.h"
#include "../CollisionPrimitive/Plane3.h"
#include "../CollisionPrimitive/Sphere3.h"

namespace Riemann
{
	static bool IsConcaveGeometry(const Geometry* GeometryObject)
	{
		if (GeometryObject == nullptr)
		{
			return false;
		}
		const PrimitiveType Type = GeometryObject->GetShapeType();
		return Type == PrimitiveType::TRIANGLE_MESH || Type == PrimitiveType::HEIGHTFIELD;
	}

	static Vector3 GetGeometrySupport(const GeometryWorldState& State, const Vector3& Direction)
	{
		if (State.Geom == nullptr)
		{
			return Vector3::Zero();
		}

		if (State.Body == nullptr)
		{
			return State.Geom->GetSupport(Direction);
		}

		const Vector3 LocalDirection = State.BodyToWorld.WorldToLocalDirection(Direction);
		const Vector3 LocalSupport = State.Geom->GetSupport(LocalDirection);
		return State.BodyToWorld.LocalToWorld(LocalSupport);
	}

	static void GetGeometrySupportFace(const GeometryWorldState& State, const Vector3& Direction, SupportFace& Face)
	{
		if (State.Geom == nullptr)
		{
			return;
		}

		if (State.Body == nullptr)
		{
			State.Geom->GetSupportFace(Direction, Face);
			return;
		}

		const Vector3 LocalDirection = State.BodyToWorld.WorldToLocalDirection(Direction);
		State.Geom->GetSupportFace(LocalDirection, Face);
		for (int i = 0; i < Face.size(); ++i)
		{
			Face[i] = State.BodyToWorld.LocalToWorld(Face[i]);
		}
	}

	static Vector3 GetContactRelativePosition(const GeometryWorldState& State, const Vector3& Position)
	{
		if (State.Body)
		{
			return Position - State.Body->X;
		}
		return State.Geom ? Position - State.Geom->GetPosition() : Position;
	}

	static bool ConstructSupportFace(const GeometryWorldState& StateA, const GeometryWorldState& StateB, const Vector3& penetration_normal, Vector3* Face, int& FaceCount, float ClipTolerance)
	{
		SupportFace FaceA, FaceB;
		GetGeometrySupportFace(StateA, -penetration_normal, FaceA);
		GetGeometrySupportFace(StateB, penetration_normal, FaceB);

		bool succ = ClipPolygonAgainPolygon3D(FaceA.data(), FaceA.size(), FaceB.data(), FaceB.size(), penetration_normal, ClipTolerance, Face, &FaceCount, nullptr, nullptr);
		return succ;
	}


	static bool PenetrationTest(const GeometryWorldState& State1, const GeometryWorldState& State2, EPAPenetration& epa, float Scale, bool NormalizeSupport)
	{
		if (IsConcaveGeometry(State1.Geom) || IsConcaveGeometry(State2.Geom))
		{
			return false;
		}
		if (State1.Geom->GetShapeType() == PrimitiveType::BOX
			&& State2.Geom->GetShapeType() == PrimitiveType::BOX)
		{
			const AxisAlignedBox3* Box1 = State1.Geom->GetShapeObj<AxisAlignedBox3>();
			const AxisAlignedBox3* Box2 = State2.Geom->GetShapeObj<AxisAlignedBox3>();
			const OrientedBox3 OBB1(State1.ShapeToWorld.LocalToWorld(Box1->GetCenter()),
				Box1->GetExtent(), State1.ShapeToWorld.quat.ToRotationMatrix3());
			const OrientedBox3 OBB2(State2.ShapeToWorld.LocalToWorld(Box2->GetCenter()),
				Box2->GetExtent(), State2.ShapeToWorld.quat.ToRotationMatrix3());
			if (!OBB1.PenetrateOBB(OBB2, &epa.penetration_normal, &epa.penetration_depth))
			{
				return false;
			}
			epa.penetration_normal.SafeNormalize();
			epa.penetration_depth = std::max(0.0f, epa.penetration_depth);
			epa.status = EPA_status::Valid;
			return true;
		}
		TransformedMinkowskiSum<Geometry, Geometry> shape(
			State1.Geom, State1.BodyToWorld,
			State2.Geom, State2.BodyToWorld);
		// Normalize the Minkowski difference so the legacy GJK/EPA convergence
		// constants represent relative error for explicitly scaled contacts.
		class NormalizedShape : public GjkShape
		{
		public:
			NormalizedShape(const GjkShape& Shape, float Scale, bool NormalizeSupport)
				: m_Shape(Shape), m_Scale(Scale), m_NormalizeSupport(NormalizeSupport) {}
			Vector3 Center() const override { return m_Shape.Center() / m_Scale; }
			Vector3 Support(const Vector3& Direction) const override { return m_Shape.Support(m_NormalizeSupport ? Direction.SafeUnit() : Direction) / m_Scale; }
		private:
			const GjkShape& m_Shape;
			float m_Scale;
			bool m_NormalizeSupport;
		} Normalized(shape, Scale, NormalizeSupport);
		GJKIntersection gjk;
		GJK_status gjk_status = gjk.Solve(&Normalized);
		if (gjk_status != GJK_status::Intersect)
		{
			return false;
		}

		EPA_status epa_result = epa.Solve(gjk.result);
		if (epa_result == EPA_status::Failed || epa_result == EPA_status::FallBack)
		{
			return false;
		}

		epa.penetration_depth *= Scale;
		return true;
	}

	static void BuildContactBasis(Contact* contact)
	{
		if (std::fabs(contact->Normal.x) >= 0.57735f)
		{
			contact->Tangent = Vector3(contact->Normal.y, -contact->Normal.x, 0.0f);
		}
		else
		{
			contact->Tangent = Vector3(0.0f, contact->Normal.z, -contact->Normal.y);
		}
		contact->Tangent.SafeNormalize();
		contact->Binormal = CrossProduct(contact->Normal, contact->Tangent);
	}

	static bool ConstructSpherePlaneManifold(int IndexA, int IndexB, const GeometryWorldState& StateA,
		const GeometryWorldState& StateB, ContactManifold* Manifold)
	{
		// An infinite half-space has no finite support point for GJK/EPA.
		const bool SphereIsA = StateA.Geom->GetShapeType() == PrimitiveType::SPHERE;
		const GeometryWorldState& SphereState = SphereIsA ? StateA : StateB;
		const GeometryWorldState& PlaneState = SphereIsA ? StateB : StateA;
		const Sphere3* Sphere = SphereState.Geom->GetShapeObj<Sphere3>();
		const Plane3* Plane = PlaneState.Geom->GetShapeObj<Plane3>();
		const Vector3 Normal = PlaneState.ShapeToWorld.LocalToWorldDirection(Plane->Normal).SafeUnit();
		const Vector3 Origin = PlaneState.ShapeToWorld.LocalToWorld(Plane->GetOrigin());
		const Vector3 Center = SphereState.ShapeToWorld.LocalToWorld(Sphere->Center);
		const float Depth = Sphere->Radius - (Center - Origin).Dot(Normal);
		if (!std::isfinite(Depth) || Depth < 0.0f || Normal.SquareLength() < 0.5f) return false;
		const Vector3 SpherePoint = Center - Normal * Sphere->Radius;
		const Vector3 PlanePoint = SpherePoint + Normal * Depth;
		Contact Point;
		Point.PositionWorldA = SphereIsA ? SpherePoint : PlanePoint;
		Point.PositionWorldB = SphereIsA ? PlanePoint : SpherePoint;
		Point.PositionLocalA = GetContactRelativePosition(StateA, Point.PositionWorldA);
		Point.PositionLocalB = GetContactRelativePosition(StateB, Point.PositionWorldB);
		Point.Normal = SphereIsA ? -Normal : Normal;
		Point.PenetrationDepth = Depth;
		BuildContactBasis(&Point);
		Manifold->Reset();
		Manifold->AddNewContact(IndexA, IndexB, Point);
		return true;
	}

	static bool ConstructConcaveManifold(int indexA, int indexB, const GeometryWorldState& StateA, const GeometryWorldState& StateB, ContactManifold* manifold)
	{
		const bool ConcaveIsA = IsConcaveGeometry(StateA.Geom);
		const bool ConcaveIsB = IsConcaveGeometry(StateB.Geom);
		if (ConcaveIsA == ConcaveIsB)
		{
			return false;
		}
		const GeometryWorldState& ConcaveState = ConcaveIsA ? StateA : StateB;
		const GeometryWorldState& ConvexState = ConcaveIsA ? StateB : StateA;
		manifold->Reset();

		const Vector3 RelativeDisplacement = ConvexState.Displacement - ConcaveState.Displacement;
		const bool CurrentlyIntersecting = ConvexState.Geom->Intersect(
			ConvexState.ShapeToWorld, ConcaveState.Geom, ConcaveState.ShapeToWorld);
		const PrimitiveType ConvexType = ConvexState.Geom->GetShapeType();
		const bool CanEstimatePenetration = ConvexType == PrimitiveType::BOX
			|| ConvexType == PrimitiveType::SPHERE || ConvexType == PrimitiveType::CAPSULE;
		auto EstimatePenetration = [&](const Vector3& WorldNormal, float InitialDepth) -> float
		{
			if (!CanEstimatePenetration)
			{
				return std::max(0.001f, InitialDepth);
			}
			float Lower = 0.0f;
			float Upper = 0.01f;
			Transform SeparatedTransform = ConvexState.ShapeToWorld;
			SeparatedTransform.pos += WorldNormal * Upper;
			while (Upper < 0.5f && ConvexState.Geom->Intersect(SeparatedTransform, ConcaveState.Geom, ConcaveState.ShapeToWorld))
			{
				Lower = Upper;
				Upper *= 2.0f;
				SeparatedTransform.pos = ConvexState.ShapeToWorld.pos + WorldNormal * Upper;
			}
			for (int Iteration = 0; Iteration < 8; ++Iteration)
			{
				const float Middle = (Lower + Upper) * 0.5f;
				SeparatedTransform.pos = ConvexState.ShapeToWorld.pos + WorldNormal * Middle;
				if (ConvexState.Geom->Intersect(SeparatedTransform, ConcaveState.Geom, ConcaveState.ShapeToWorld))
				{
					Lower = Middle;
				}
				else
				{
					Upper = Middle;
				}
			}
			return std::min(0.2f, std::max(InitialDepth, Upper));
		};

		struct ConcaveContactCandidate
		{
			Vector3 Position = Vector3::Zero();
			Vector3 Normal = Vector3::Zero();
			float Depth = 1.0e30f;
			bool Valid = false;
		};
		ConcaveContactCandidate BestCandidate;
		auto TrySweepCandidate = [&](const Transform& StartTransform, const Vector3& WorldDirection)
		{
			Vector3 LocalPosition;
			Vector3 LocalNormal;
			float HitTime = 0.0f;
			const Vector3 LocalDirection = StartTransform.WorldToLocalDirection(WorldDirection);
			if (!ConvexState.Geom->Sweep(StartTransform, LocalDirection, ConcaveState.Geom,
				ConcaveState.ShapeToWorld, &LocalPosition, &LocalNormal, &HitTime)
				|| HitTime < 0.0f || HitTime > 1.0f)
			{
				return;
			}
			// An initial-overlap hit from the previous pose is not a collision when
			// the current pose has already separated from the surface.
			if (!CurrentlyIntersecting && HitTime <= 1.0e-5f)
			{
				return;
			}

			Vector3 WorldNormal = StartTransform.LocalToWorldDirection(LocalNormal).SafeUnit();
			if (WorldNormal.IsZero())
			{
				return;
			}
			if (WorldNormal.Dot(WorldDirection) > 0.0f)
			{
				WorldNormal = -WorldNormal;
			}
			if (ConcaveState.Geom->GetShapeType() == PrimitiveType::HEIGHTFIELD)
			{
				const Vector3 HeightFieldNormal = ConcaveState.ShapeToWorld.LocalToWorldDirection(Vector3::UnitY());
				if (WorldNormal.Dot(HeightFieldNormal) < 0.0f)
				{
					WorldNormal = -WorldNormal;
				}
			}

			const float ImpactDepth = CurrentlyIntersecting ? 0.001f : std::max(0.001f,
				-RelativeDisplacement.Dot(WorldNormal) * std::max(0.0f, 1.0f - HitTime));
			const float PenetrationDepth = EstimatePenetration(WorldNormal, ImpactDepth);
			if (!BestCandidate.Valid || PenetrationDepth < BestCandidate.Depth)
			{
				BestCandidate.Valid = true;
				BestCandidate.Position = StartTransform.LocalToWorld(LocalPosition);
				BestCandidate.Normal = WorldNormal;
				BestCandidate.Depth = PenetrationDepth;
			}
		};

		if (CurrentlyIntersecting)
		{
			const Vector3 ProbeAxes[3] = {
				ConcaveState.ShapeToWorld.LocalToWorldDirection(Vector3::UnitX()),
				ConcaveState.ShapeToWorld.LocalToWorldDirection(Vector3::UnitY()),
				ConcaveState.ShapeToWorld.LocalToWorldDirection(Vector3::UnitZ())
			};
			for (const Vector3& ProbeAxis : ProbeAxes)
			{
				TrySweepCandidate(ConvexState.ShapeToWorld, ProbeAxis * 0.02f);
				TrySweepCandidate(ConvexState.ShapeToWorld, ProbeAxis * -0.02f);
			}
		}
		else if (RelativeDisplacement.SquareLength() > 1.0e-10f)
		{
			Transform StartTransform = ConvexState.ShapeToWorld;
			StartTransform.pos -= RelativeDisplacement;
			TrySweepCandidate(StartTransform, RelativeDisplacement);
		}
		if (!BestCandidate.Valid)
		{
			return false;
		}

		Contact contact;
		contact.PositionWorldA = BestCandidate.Position;
		contact.PositionWorldB = BestCandidate.Position - BestCandidate.Normal * BestCandidate.Depth;
		contact.Normal = BestCandidate.Normal;
		contact.PenetrationDepth = BestCandidate.Depth;
		if (!ConcaveIsA)
		{
			std::swap(contact.PositionWorldA, contact.PositionWorldB);
			contact.Normal = -contact.Normal;
		}
		BuildContactBasis(&contact);
		contact.PositionLocalA = GetContactRelativePosition(StateA, contact.PositionWorldA);
		contact.PositionLocalB = GetContactRelativePosition(StateB, contact.PositionWorldB);
		manifold->AddNewContact(indexA, indexB, contact);
		return true;
	}

	static void ConstructManifols(int indexA, int indexB, const GeometryWorldState& StateA, const GeometryWorldState& StateB, EPAPenetration& epa, ContactManifold* manifold, float ClipTolerance, bool NormalizeSupport)
	{
		manifold->Reset();

		// http://allenchou.net/2013/12/game-physics-contact-generation-epa/
		Vector3 w0 = Vector3::Zero();
		if (epa.result.dimension == 0)
		{
			w0 = GetGeometrySupport(StateA, epa.penetration_normal);
		}
		else
		{
			for (int i = 0; i < epa.result.dimension; ++i)
			{
				Vector3 pi = GetGeometrySupport(StateA, NormalizeSupport ? epa.result.v[i].dir.SafeUnit() : epa.result.v[i].dir) * epa.result.w[i];
				w0 = w0 + pi;
			}
		}

		const bool UseContactFace = true;
		std::array<Vector3, 2 * MAX_FACE_POINTS> ContactFace;
		int ContactFaceCount = 0;
		bool bHasContactFace = false;
		if (UseContactFace)
		{
			bool succ = ConstructSupportFace(StateA, StateB, epa.penetration_normal, ContactFace.data(), ContactFaceCount, ClipTolerance);
			if (!succ)
			{
				succ = ConstructSupportFace(StateB, StateA, -epa.penetration_normal, ContactFace.data(), ContactFaceCount, ClipTolerance);
			}
			bHasContactFace = succ && ContactFaceCount > 0;
		}

		const int FirstContactIndex = bHasContactFace ? 0 : -1;
		for (int i = FirstContactIndex; i < ContactFaceCount; ++i)
		{
			Vector3 pa = i == -1 ? w0 : ContactFace[i];
			Contact contact;
			contact.PositionLocalA = GetContactRelativePosition(StateA, pa);
			Vector3 pb = pa - epa.penetration_normal * epa.penetration_depth;
			contact.PositionLocalB = GetContactRelativePosition(StateB, pb);
			contact.PositionWorldA = pa;
			contact.PositionWorldB = pb;
			contact.Normal = epa.penetration_normal;
			contact.PenetrationDepth = epa.penetration_depth;
			BuildContactBasis(&contact);

			manifold->AddNewContact(indexA, indexB, contact);
		}
	}


	static bool GenerateContactManifold(int IndexA, int IndexB, const GeometryWorldState& A,
		const GeometryWorldState& B, ContactManifold* Manifold, const ContactGenerationSettings& Settings)
	{
		Manifold->Reset();
		Manifold->indexA = IndexA;
		Manifold->indexB = IndexB;
		EPAPenetration EPA;
		const bool Extended = ContactGeneration::UsesOffsets(A, B, Settings);
		const float Scale = Extended ? ContactGeneration::PairScale(A, B, Settings) : 1.0f;
		const PrimitiveType TypeA = A.Geom->GetShapeType(), TypeB = B.Geom->GetShapeType();
		if ((TypeA == PrimitiveType::SPHERE && TypeB == PrimitiveType::PLANE)
			|| (TypeB == PrimitiveType::SPHERE && TypeA == PrimitiveType::PLANE))
			ConstructSpherePlaneManifold(IndexA, IndexB, A, B, Manifold);
		else if (IsConcaveGeometry(A.Geom) || IsConcaveGeometry(B.Geom))
			ConstructConcaveManifold(IndexA, IndexB, A, B, Manifold);
		else if (PenetrationTest(A, B, EPA, Scale, Extended))
			ConstructManifols(IndexA, IndexB, A, B, EPA, Manifold, Settings.FaceClipSquaredDistance * Settings.LengthScale * Settings.LengthScale, Extended);
		for (int Index = 0; Index < Manifold->NumContactPointCount; ++Index)
		{
			Contact& Point = Manifold->ContactPoints[Index];
			Point.FeatureA = ContactGeneration::Feature(A, Point.PositionWorldA, Settings);
			Point.FeatureB = ContactGeneration::Feature(B, Point.PositionWorldB, Settings);
		}
		return Manifold->NumContactPointCount > 0;
	}

	class NarrowPhase_Contacts : public NarrowPhase
	{
	public:
		NarrowPhase_Contacts(bool Persistent, const ContactPersistenceSettings& Settings, const MeshContactSettings& MeshSettings)
			: m_Persistent(Persistent), m_Cache(Settings), m_Mesh(MeshSettings)
		{
			const int InitialBatches = 1, ManifoldsPerBatch = 256;
			m_ManifoldPool.init(InitialBatches, ManifoldsPerBatch);
		}
		void CollisionDetection(GeometryWorldStateSpan States, const std::vector<OverlapPair>& Overlaps,
			std::vector<ContactManifold*>* Manifolds) override
		{
			m_ManifoldPool.clear(); Manifolds->clear();
			for (const OverlapPair& Pair : Overlaps)
			{
				const GeometryWorldState& A = States[Pair.index1];
				const GeometryWorldState& B = States[Pair.index2];
				if (!A.Geom || !B.Geom) continue;
				std::vector<ContactManifold> Patches;
				const std::vector<uint32_t> CachedIds = m_Persistent ? m_Cache.GetPatchIds(A, B) : std::vector<uint32_t>();
				// Field patches regenerate their geometry while retaining feature/impulse matching.
				bool Refreshed = !CachedIds.empty() && !m_ContactSettings.SpeculativeContacts && !DistanceFieldCollision::HasField(A.Geom, B.Geom);
				for (uint32_t Id : CachedIds)
				{
					if (!Refreshed) break;
					Patches.emplace_back();
					ContactManifold& Patch = Patches.back();
					Patch.indexA = Pair.index1; Patch.indexB = Pair.index2; Patch.PatchId = Id;
					if (!m_Cache.Refresh(A, B, &Patch)) { Refreshed = false; break; }
				}
				if (!Refreshed)
				{
					Patches.clear();
					// Offsets and speculative gaps require per-triangle distances even when
					// the legacy discrete mesh route is selected for ordinary contacts.
					const bool Extended = ContactGeneration::UsesOffsets(A, B, m_ContactSettings);
					if (DistanceFieldCollision::HasField(A.Geom, B.Geom) && (IsConcaveGeometry(A.Geom) || IsConcaveGeometry(B.Geom)))
						m_Mesh.Generate(Pair.index1, Pair.index2, A, B, Patches,
							ContactGeneration::GenerationDistance(A, B, m_ContactSettings, m_Dt),
							ContactGeneration::Tolerance(A, B, m_ContactSettings));
					else if (DistanceFieldCollision::HasField(A.Geom, B.Geom))
					{
						std::vector<DistanceFieldContact> Candidates;
						DistanceFieldCollision::Generate(A.Geom, A.ShapeToWorld, B.Geom, B.ShapeToWorld,
							ContactGeneration::GenerationDistance(A, B, m_ContactSettings, m_Dt), Candidates);
						const auto* Field = A.Geom->GetShapeObj<DistanceFieldShape>();
						if (!Field) Field = B.Geom->GetShapeObj<DistanceFieldShape>();
						const float PatchNormalCosine = Field->GetSettings().PatchNormalCosine;
						const float DepthTolerance = Field->GetField().GetCellSize() * Field->GetSettings().ManifoldDepthToleranceCells;
						std::vector<std::pair<Vector3, float>> Depths;
						for (const auto& Candidate : Candidates)
						{
							auto Existing = std::find_if(Depths.begin(), Depths.end(), [&](const auto& Entry) { return Entry.first.Dot(Candidate.Normal) >= PatchNormalCosine; });
							if (Existing == Depths.end()) Depths.push_back({ Candidate.Normal, Candidate.Separation });
							else Existing->second = std::min(Existing->second, Candidate.Separation);
						}
						for (const auto& Candidate : Candidates)
						{
							// Preserve the deepest supporting footprint. Distant positive-gap
							// samples must not displace active rows merely to widen the patch.
							bool Shallow = false;
							for (const auto& Entry : Depths) if (Entry.first.Dot(Candidate.Normal) >= PatchNormalCosine && Candidate.Separation > Entry.second + DepthTolerance) { Shallow = true; break; }
							if (Shallow) continue;
							Contact Point; Point.PositionWorldA = Candidate.PointA; Point.PositionWorldB = Candidate.PointB;
							Point.Normal = Candidate.Normal; Point.PenetrationDepth = -Candidate.Separation;
							Point.PositionLocalA = GetContactRelativePosition(A, Point.PositionWorldA);
							Point.PositionLocalB = GetContactRelativePosition(B, Point.PositionWorldB);
							Point.FeatureA = Candidate.FeatureA; Point.FeatureB = Candidate.FeatureB; BuildContactBasis(&Point);
							ContactManifold* Patch = nullptr;
							for (auto& Existing : Patches) if (Existing.ContactPoints[0].Normal.Dot(Point.Normal) >= PatchNormalCosine) { Patch = &Existing; break; }
							if (!Patch) { Patches.emplace_back(); Patch = &Patches.back(); Patch->PatchId = (uint32_t)Patches.size(); }
							Patch->AddNewContact(Pair.index1, Pair.index2, Point);
						}
					}
					else if ((m_Mesh.IsEnabled() || Extended) && (IsConcaveGeometry(A.Geom) || IsConcaveGeometry(B.Geom)))
						m_Mesh.Generate(Pair.index1, Pair.index2, A, B, Patches,
							ContactGeneration::GenerationDistance(A, B, m_ContactSettings, m_Dt),
							Extended ? ContactGeneration::Tolerance(A, B, m_ContactSettings) : 0.0f);
					else
					{
						ContactManifold Patch;
						bool SeparationKnown;
						if (ContactGeneration::GenerateSeparated(Pair.index1, Pair.index2, A, B, m_ContactSettings, m_Dt, Patch, SeparationKnown)
							|| (!SeparationKnown && GenerateContactManifold(Pair.index1, Pair.index2, A, B, &Patch, m_ContactSettings))) Patches.push_back(Patch);
					}
					for (ContactManifold& Patch : Patches) ContactGeneration::ApplyOffsets(A, B, m_ContactSettings, Patch);
					if (m_Persistent)
						for (ContactManifold& Patch : Patches) m_Cache.Match(A, B, &Patch, false);
				}
				std::vector<uint32_t> PatchIds;
				for (ContactManifold& Patch : Patches)
				{
					if (!Patch.NumContactPointCount) continue;
					PatchIds.push_back(Patch.PatchId);
					if (m_Persistent) m_Cache.Store(A, B, Patch, 0.0f, !Refreshed);
					ContactManifold* Output = m_ManifoldPool.allocate(); *Output = Patch;
					Manifolds->push_back(Output);
				}
				if (m_Persistent) m_Cache.RetainPatches(A, B, PatchIds);
			}
		}
		void BeginFrame(uint64_t Frame) override { m_Cache.BeginFrame(Frame); }
		void ResetState() override { m_Cache.Reset(); m_Mesh.Reset(); }
		void Invalidate(const Geometry* Shape) override { m_Cache.Invalidate(Shape); m_Mesh.Invalidate(Shape); }
		size_t GetCachedPairCount() const override { return m_Cache.GetPairCount(); }
		void SetTriangleContactsEnabled(bool Enabled) override { m_Mesh.SetEnabled(Enabled); m_Cache.Reset(); }
		void SetContactGeneration(const ContactGenerationSettings& Settings, float Dt) override { m_ContactSettings = Settings; m_Dt = Dt; }
	private:
		ContactGenerationSettings m_ContactSettings;
		float m_Dt = 0.0f;
		bool m_Persistent;
		BatchList<ContactManifold> m_ManifoldPool;
		ContactManifoldManager m_Cache;
		MeshContactGenerator m_Mesh;
	};

	class NarrowPhase_PenetrateTable : public NarrowPhase
	{
	public:
		NarrowPhase_PenetrateTable()
		{
			m_ManifoldPool.init(1, 256);
		}
		virtual ~NarrowPhase_PenetrateTable()
		{

		}

		virtual void CollisionDetection(GeometryWorldStateSpan states,
			const std::vector<OverlapPair>& overlaps,
			std::vector<ContactManifold*>* manifolds) override final
		{
			return;
		}

	private:
		BatchList<ContactManifold>	m_ManifoldPool;
	};

	class NarrowPhase_SAT : public NarrowPhase
	{
	public:
		NarrowPhase_SAT()
		{
		}
		virtual ~NarrowPhase_SAT()
		{
		}

		virtual void CollisionDetection(GeometryWorldStateSpan states,
			const std::vector<OverlapPair>& overlaps,
			std::vector<ContactManifold*>* manifolds) override final
		{
			return;
		}
	};

	NarrowPhase* NarrowPhase::Create_GJKEPA(const MeshContactSettings& MeshSettings)
	{
		return new NarrowPhase_Contacts(false, ContactPersistenceSettings(), MeshSettings);
	}

	NarrowPhase* NarrowPhase::Create_SAT()
	{
		return new NarrowPhase_SAT();
	}

	NarrowPhase* NarrowPhase::Create_PCM(const ContactPersistenceSettings& Settings, const MeshContactSettings& MeshSettings)
	{
		return new NarrowPhase_Contacts(true, Settings, MeshSettings);
	}
}
