
#include <assert.h>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <map>
#include <tuple>
#include <cmath>
#include <stdexcept>
#include "CollidingContact.h"
#include "RigidBody.h"
#include "../Core/StaticArray.h"
#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/Segment3.h"
#include "../CollisionPrimitive/HeightField3.h"
#include "../CollisionPrimitive/Triangle3.h"

namespace Riemann
{
	void ContactManifold::AddNewContact(int a, int b, const Contact& result)
	{
		indexA = a;
		indexB = b;
		const float CoincidenceSquared = 1.0e-10f;
		for (int Index = 0; Index < NumContactPointCount; ++Index)
		{
			if ((ContactPoints[Index].PositionWorldA - result.PositionWorldA).SquareLength() <= CoincidenceSquared
				&& (ContactPoints[Index].PositionWorldB - result.PositionWorldB).SquareLength() <= CoincidenceSquared)
			{
				if (result.PenetrationDepth > ContactPoints[Index].PenetrationDepth) ContactPoints[Index] = result;
				return;
			}
		}
		ContactPoints[NumContactPointCount] = result;
		NumContactPointCount++;

		if (NumContactPointCount > MAX_CONTACT_POINTS)
		{
			// Retain the deepest point, then maximize spatial coverage. Unlike the
			// older four-extreme construction, this also handles collinear patches.
			int Selected[MAX_CONTACT_POINTS] = {};
			bool Used[MAX_CONTACT_POINTS + 1] = {};
			for (int Index = 1; Index < NumContactPointCount; ++Index)
				if (ContactPoints[Index].PenetrationDepth > ContactPoints[Selected[0]].PenetrationDepth) Selected[0] = Index;
			Used[Selected[0]] = true;
			for (int Count = 1; Count < MAX_CONTACT_POINTS; ++Count)
			{
				float BestScore = -1.0f;
				for (int Index = 0; Index < NumContactPointCount; ++Index)
				{
					if (Used[Index]) continue;
					float Score = FLT_MAX;
					for (int Existing = 0; Existing < Count; ++Existing)
					{
						const Vector3 Delta = ContactPoints[Index].PositionWorldA - ContactPoints[Selected[Existing]].PositionWorldA;
						const Vector3 Tangential = Delta - result.Normal * Delta.Dot(result.Normal);
						Score = std::min(Score, Tangential.SquareLength());
					}
					if (Score > BestScore) { BestScore = Score; Selected[Count] = Index; }
				}
				Used[Selected[Count]] = true;
			}
			// Keep the surviving rows in their existing order. Reordering the whole
			// patch at every insertion introduces a directional bias in PGS/PBD.
			for (int Index = 0; Index < MAX_CONTACT_POINTS; ++Index)
				if (!Used[Index]) ContactPoints[Index] = ContactPoints[MAX_CONTACT_POINTS];
			NumContactPointCount = MAX_CONTACT_POINTS;
		}
	}

	bool ContactManifold::MergeManifold()
	{
		assert(NumContactPointCount == MAX_CONTACT_POINTS + 1);

		const float kMinSqrDist = 1e-6f;
		StaticArray<Vector3, MAX_CONTACT_POINTS + 1> projected;
		StaticArray<float, MAX_CONTACT_POINTS + 1> penetration_depth_sq;
		for (int i = 0; i < NumContactPointCount; ++i)
		{
			Contact* contact = &ContactPoints[i];

			const Vector3& local1 = contact->PositionLocalA;
			projected.add(local1 - local1.Dot(contact->Normal) * contact->Normal);

			const Vector3& w1 = contact->PositionWorldA;
			const Vector3& w2 = contact->PositionWorldB;
			penetration_depth_sq.add(std::max(kMinSqrDist, (w1 - w2).SquareLength()));
		}

		int point0 = -1, point1 = -1, point2 = -1, point3 = -1;

		float max_val = -FLT_MAX;
		for (int i = 0; i < NumContactPointCount; ++i)
		{
			// Contact* contact = &ContactPoints[i];
			const float val = std::max(kMinSqrDist, projected[i].SquareLength()) * penetration_depth_sq[i];
			if (max_val < val)
			{
				max_val = val;
				point0 = i;
			}
		}

		// find the contact farthest from c0
		max_val = -FLT_MAX;
		for (int i = 0; i < NumContactPointCount; ++i)
		{
			Contact* contact = &ContactPoints[i];
			if (point0 == i)
				continue;
			const float val = (contact->PositionWorldA - projected[point0]).SquareLength() * penetration_depth_sq[i];
			if (max_val < val)
			{
				max_val = val;
				point1 = i;
			}
		}

		float min_val = 0.0f;
		max_val = 0.0f;

		Vector3 perp = (projected[point1] - projected[point0]).Cross(ContactPoints[0].Normal);
		for (int i = 0; i < projected.size(); ++i)
		{
			if (point0 == i || point1 == i)
				continue;

			const float v = perp.Dot(projected[i] - projected[point0]);
			if (v < min_val)
			{
				min_val = v;
				point2 = i;
			}
			else if (v > max_val)
			{
				max_val = v;
				point3 = i;
			}
		}


		assert(point0 != point1 && point0 != point2 && point0 != point3);
		assert(point1 != point2 && point1 != point3);
		assert(point2 != point3);

		for (int i = 0; i < NumContactPointCount; ++i)
		{
			if (i == point0 || i == point1 || i == point2 || i == point3)
				continue;

			NumContactPointCount--;
			if (i == MAX_CONTACT_POINTS)
			{
				break;
			}

			memcpy(&ContactPoints[i], &ContactPoints[MAX_CONTACT_POINTS], sizeof(Contact));
			break;
		}

		return true;
	}

	bool ContactManifold::MergeManifold2()
	{
		assert(NumContactPointCount == MAX_CONTACT_POINTS + 1);

		// find the contact with deepest penetration depth
		Contact* c0 = nullptr;
		float max_penetration = -FLT_MAX;
		for (int i = 0; i < NumContactPointCount; ++i)
		{
			Contact* contact = &ContactPoints[i];
			if (contact->PenetrationDepth > max_penetration)
			{
				max_penetration = contact->PenetrationDepth;
				c0 = contact;
			}
		}

		// find the contact farthest from c0
		Contact* c1 = nullptr;
		float max_sqr_dist = -FLT_MAX;
		for (int i = 0; i < NumContactPointCount; ++i)
		{
			Contact* contact = &ContactPoints[i];
			if (contact == c0)
				continue;
			const float sqr_dist = (contact->PositionWorldA - c0->PositionWorldA).SquareLength();
			if (sqr_dist > max_sqr_dist)
			{
				max_sqr_dist = sqr_dist;
				c1 = contact;
			}
		}

		// find the contact farthest form segment c0 - c1
		Contact* c2 = nullptr;
		float max_sqr_dist_seq = -FLT_MAX;
		for (int i = 0; i < NumContactPointCount; ++i)
		{
			Contact* contact = &ContactPoints[i];
			if (contact == c0 || contact == c1)
				continue;
			const float dist = Segment3::SqrDistancePointToSegment(contact->PositionWorldA, c0->PositionWorldA, c1->PositionWorldA);
			if (dist > max_sqr_dist_seq)
			{
				max_sqr_dist_seq = dist;
				c2 = contact;
			}
		}

		// find the contact farthest form triangle c0 c1 c2
		Contact* c3 = nullptr;
		float max_sqr_dist_tri = -FLT_MAX;
		for (int i = 0; i < NumContactPointCount; ++i)
		{
			Contact* contact = &ContactPoints[i];
			if (contact == c0 || contact == c1 || contact == c2)
				continue;
			const float dist = Triangle3::SqrDistancePointToTriangle(contact->PositionWorldA, c0->PositionWorldA, c1->PositionWorldA, c2->PositionWorldA);
			if (dist > max_sqr_dist_tri)
			{
				max_sqr_dist_tri = dist;
				c3 = contact;
			}
		}

		assert(c0 != c1 && c0 != c2 && c0 != c3);
		assert(c1 != c2 && c1 != c3);
		assert(c2 != c3);

		for (int i = 0; i < NumContactPointCount; ++i)
		{
			Contact* contact = &ContactPoints[i];
			if (contact == c0 || contact == c1 || contact == c2 || contact == c3)
				continue;

			NumContactPointCount--;
			if (i == MAX_CONTACT_POINTS)
			{
				break;
			}

			memcpy(contact, &ContactPoints[MAX_CONTACT_POINTS], sizeof(Contact));
			break;
		}
		return true;
	}

	ContactPersistenceSettings::ContactPersistenceSettings()
		: MatchDistance(0.03f), BreakDistance(0.005f), NormalCosine(0.98f)
		, RelativeTranslationThreshold(0.02f), RelativeRotationCosine(0.999f)
		, RefreshInterval(4), RetentionFrames(8), WarmStartFactor(0.85f)
		, MinTimeStepRatio(0.25f), MaxTimeStepRatio(4.0f)
	{
	}

	bool ContactPersistenceSettings::IsValid() const
	{
		return std::isfinite(MatchDistance) && MatchDistance > 0.0f
			&& std::isfinite(BreakDistance) && BreakDistance >= 0.0f
			&& std::isfinite(NormalCosine) && NormalCosine >= 0.0f && NormalCosine <= 1.0f
			&& std::isfinite(RelativeTranslationThreshold) && RelativeTranslationThreshold >= 0.0f
			&& std::isfinite(RelativeRotationCosine) && RelativeRotationCosine >= 0.0f && RelativeRotationCosine <= 1.0f
			&& RefreshInterval > 0 && RetentionFrames > 0
			&& std::isfinite(WarmStartFactor) && WarmStartFactor >= 0.0f && WarmStartFactor <= 1.0f
			&& std::isfinite(MinTimeStepRatio) && MinTimeStepRatio > 0.0f
			&& std::isfinite(MaxTimeStepRatio) && MaxTimeStepRatio >= MinTimeStepRatio;
	}

	/* Persistent contacts follow the local-anchor construction in Erin Catto,
	   "Contact Manifolds" (GDC 2007). Refresh and regeneration are separate steps.
	   Store anchors in each shape frame, not the world-space moment
	   arms called PositionLocalA/B by the legacy solver. Relative motion invalidates
	   a patch; common rigid motion does not. Matching is one-to-one, using feature
	   identity when available and bounded local-anchor distance otherwise.
	   Geometry and impulse caches use the same representation but separate owners.
	   https://box2d.org/publications/ */
	struct ContactManifoldManager::Storage
	{
		struct Point
		{
			Contact Value;
			Vector3 AnchorA, AnchorB, NormalA, TangentA;
		};
		struct Pair
		{
			GeometryHandle HandleA, HandleB;
			RigidBody* BodyA = nullptr;
			RigidBody* BodyB = nullptr;
			uint64_t RevisionA = 0, RevisionB = 0;
			uint64_t SurfaceRevisionA = 0, SurfaceRevisionB = 0;
			uint64_t LastFrame = 0, GeneratedFrame = 0;
			Transform LocalPoseA, LocalPoseB, RelativePose;
			std::vector<Point> Points;
			float Dt = 0.0f;
		};
		using Key = std::tuple<uintptr_t, uintptr_t, uint32_t>;
		ContactPersistenceSettings Settings;
		std::map<Key, Pair> Pairs;
		uint64_t Frame = 1, NextId = 1;
		static uint64_t SurfaceRevision(const Geometry* Shape)
		{
			const HeightField3* Field = Shape ? Shape->GetShapeObj<HeightField3>() : nullptr;
			return Field ? Field->GetRevision() : 0;
		}

		static bool Reverse(const GeometryWorldState& A, const GeometryWorldState& B)
		{
			// Registry order is independent of heap layout. Choosing the normal's
			// reference frame by address makes warm-start eligibility allocation-dependent.
			if (A.Handle.IsValid() && B.Handle.IsValid() && A.Handle != B.Handle)
				return std::make_pair(A.Handle.Index, A.Handle.Generation) > std::make_pair(B.Handle.Index, B.Handle.Generation);
			return reinterpret_cast<uintptr_t>(A.Geom) > reinterpret_cast<uintptr_t>(B.Geom);
		}
		static Key MakeKey(const GeometryWorldState& A, const GeometryWorldState& B, uint32_t PatchId)
		{
			const uintptr_t First = reinterpret_cast<uintptr_t>(A.Geom), Second = reinterpret_cast<uintptr_t>(B.Geom);
			return std::make_tuple(std::min(First, Second), std::max(First, Second), PatchId);
		}
		static Contact Flip(Contact Value)
		{
			std::swap(Value.PositionWorldA, Value.PositionWorldB);
			std::swap(Value.PositionLocalA, Value.PositionLocalB);
			std::swap(Value.FeatureA, Value.FeatureB);
			Value.Normal = -Value.Normal;
			Value.Tangent = -Value.Tangent;
			// Preserve a right-handed basis while reversing the physical impulse.
			Value.totalImpulseBinormal = -Value.totalImpulseBinormal;
			return Value;
		}
		Pair* Find(const GeometryWorldState& A, const GeometryWorldState& B, uint32_t PatchId)
		{
			auto It = Pairs.find(MakeKey(A, B, PatchId));
			if (It == Pairs.end() || !A.Geom || !B.Geom) return nullptr;
			Pair& P = It->second;
			if (P.HandleA != A.Handle || P.HandleB != B.Handle || P.BodyA != A.Body || P.BodyB != B.Body
				|| P.RevisionA != A.Geom->GetCollisionRevision() || P.RevisionB != B.Geom->GetCollisionRevision()
				|| P.SurfaceRevisionA != SurfaceRevision(A.Geom) || P.SurfaceRevisionB != SurfaceRevision(B.Geom)
				|| P.LocalPoseA != *A.Geom->GetTransform() || P.LocalPoseB != *B.Geom->GetTransform())
			{
				Pairs.erase(It);
				return nullptr;
			}
			return &P;
		}
	};

	ContactManifoldManager::ContactManifoldManager(const ContactPersistenceSettings& Settings)
		: m_Storage(new Storage)
	{
		if (!Settings.IsValid()) throw std::invalid_argument("Invalid contact persistence settings");
		m_Storage->Settings = Settings;
	}
	ContactManifoldManager::~ContactManifoldManager() = default;
	void ContactManifoldManager::Reset() { m_Storage->Pairs.clear(); }
	size_t ContactManifoldManager::GetPairCount() const
	{
		size_t Count = 0;
		std::pair<uintptr_t, uintptr_t> Previous(0, 0);
		for (const auto& Entry : m_Storage->Pairs)
		{
			const auto Pair = std::make_pair(std::get<0>(Entry.first), std::get<1>(Entry.first));
			if (Pair != Previous) { ++Count; Previous = Pair; }
		}
		return Count;
	}

	std::vector<uint32_t> ContactManifoldManager::GetPatchIds(const GeometryWorldState& A, const GeometryWorldState& B) const
	{
		std::vector<uint32_t> Ids;
		const auto Key = Storage::MakeKey(A, B, 0);
		for (auto It = m_Storage->Pairs.lower_bound(Key); It != m_Storage->Pairs.end(); ++It)
		{
			if (std::get<0>(It->first) != std::get<0>(Key) || std::get<1>(It->first) != std::get<1>(Key)) break;
			Ids.push_back(std::get<2>(It->first));
		}
		return Ids;
	}

	void ContactManifoldManager::RetainPatches(const GeometryWorldState& A, const GeometryWorldState& B,
		const std::vector<uint32_t>& PatchIds)
	{
		for (uint32_t Id : GetPatchIds(A, B))
			if (std::find(PatchIds.begin(), PatchIds.end(), Id) == PatchIds.end())
				m_Storage->Pairs.erase(Storage::MakeKey(A, B, Id));
	}

	void ContactManifoldManager::BeginFrame(uint64_t Frame)
	{
		Storage& S = *m_Storage;
		if (Frame < S.Frame) Reset();
		S.Frame = Frame;
		for (auto It = S.Pairs.begin(); It != S.Pairs.end();)
		{
			if (Frame - It->second.LastFrame > S.Settings.RetentionFrames) It = S.Pairs.erase(It);
			else ++It;
		}
	}

	void ContactManifoldManager::Invalidate(const Geometry* Shape)
	{
		const uintptr_t Id = reinterpret_cast<uintptr_t>(Shape);
		for (auto It = m_Storage->Pairs.begin(); It != m_Storage->Pairs.end();)
		{
			if (std::get<0>(It->first) == Id || std::get<1>(It->first) == Id) It = m_Storage->Pairs.erase(It);
			else ++It;
		}
	}

	bool ContactManifoldManager::Refresh(const GeometryWorldState& StateA, const GeometryWorldState& StateB,
		ContactManifold* Manifold)
	{
		Storage& S = *m_Storage;
		const bool Reversed = Storage::Reverse(StateA, StateB);
		const GeometryWorldState& A = Reversed ? StateB : StateA;
		const GeometryWorldState& B = Reversed ? StateA : StateB;
		if (!Manifold) return false;
		Storage::Pair* P = S.Find(A, B, Manifold->PatchId);
		if (!P || !Manifold) return false;
		const Transform Relative = A.ShapeToWorld.TransformInv(B.ShapeToWorld);
		const bool Regenerate = S.Frame - P->GeneratedFrame >= S.Settings.RefreshInterval
			|| (Relative.pos - P->RelativePose.pos).SquareLength()
				> S.Settings.RelativeTranslationThreshold * S.Settings.RelativeTranslationThreshold
			|| std::abs(Relative.quat.Dot(P->RelativePose.quat)) < S.Settings.RelativeRotationCosine;
		Manifold->Reset();
		for (const Storage::Point& Cached : P->Points)
		{
			Contact C = Cached.Value;
			C.PositionWorldA = A.ShapeToWorld.LocalToWorld(Cached.AnchorA);
			C.PositionWorldB = B.ShapeToWorld.LocalToWorld(Cached.AnchorB);
			C.Normal = A.ShapeToWorld.LocalToWorldDirection(Cached.NormalA);
			C.Tangent = A.ShapeToWorld.LocalToWorldDirection(Cached.TangentA);
			C.Binormal = C.Normal.Cross(C.Tangent);
			const Vector3 Difference = C.PositionWorldA - C.PositionWorldB;
			const float SurfaceDepth = Difference.Dot(C.Normal);
			C.PenetrationDepth = SurfaceDepth + C.RestOffset;
			const Vector3 Drift = Difference - C.Normal * SurfaceDepth;
			if (C.PenetrationDepth < -S.Settings.BreakDistance
				|| Drift.SquareLength() > S.Settings.MatchDistance * S.Settings.MatchDistance) continue;
			C.PositionLocalA = C.PositionWorldA - (A.Body ? A.Body->X : A.ShapeToWorld.pos);
			C.PositionLocalB = C.PositionWorldB - (B.Body ? B.Body->X : B.ShapeToWorld.pos);
			C.CachedDt = P->Dt;
			Manifold->AddNewContact(Manifold->indexA, Manifold->indexB, Reversed ? Storage::Flip(C) : C);
		}
		return !Regenerate && Manifold->NumContactPointCount == (int)P->Points.size()
			&& Manifold->NumContactPointCount > 0;
	}

	int ContactManifoldManager::Match(const GeometryWorldState& StateA, const GeometryWorldState& StateB,
		ContactManifold* Manifold, bool Impulses)
	{
		Storage& S = *m_Storage;
		const bool Reversed = Storage::Reverse(StateA, StateB);
		const GeometryWorldState& A = Reversed ? StateB : StateA;
		const GeometryWorldState& B = Reversed ? StateA : StateB;
		if (!Manifold) return 0;
		Storage::Pair* P = S.Find(A, B, Manifold->PatchId);
		if (!P || !Manifold || (Impulses && P->LastFrame == S.Frame)) return 0;
		bool Used[MAX_CONTACT_POINTS] = {};
		int Matches = 0;
		for (int Index = 0; Index < Manifold->NumContactPointCount; ++Index)
		{
			Contact C = Reversed ? Storage::Flip(Manifold->ContactPoints[Index]) : Manifold->ContactPoints[Index];
			const Vector3 LocalA = A.ShapeToWorld.WorldToLocal(C.PositionWorldA);
			const Vector3 LocalB = B.ShapeToWorld.WorldToLocal(C.PositionWorldB);
			float BestDistance = 2.0f * S.Settings.MatchDistance * S.Settings.MatchDistance;
			int Best = -1;
			for (int Old = 0; Old < (int)P->Points.size(); ++Old)
			{
				const Storage::Point& Point = P->Points[(size_t)Old];
				if (Used[Old] || C.Normal.Dot(A.ShapeToWorld.LocalToWorldDirection(Point.NormalA)) < S.Settings.NormalCosine) continue;
				if ((C.FeatureA != UINT32_MAX && Point.Value.FeatureA != UINT32_MAX && C.FeatureA != Point.Value.FeatureA)
					|| (C.FeatureB != UINT32_MAX && Point.Value.FeatureB != UINT32_MAX && C.FeatureB != Point.Value.FeatureB)) continue;
				const float Distance = (LocalA - Point.AnchorA).SquareLength() + (LocalB - Point.AnchorB).SquareLength();
				if (Distance < BestDistance) { BestDistance = Distance; Best = Old; }
			}
			if (Best < 0) continue;
			Used[Best] = true;
			const Contact& Old = P->Points[(size_t)Best].Value;
			C.PersistentId = Old.PersistentId;
			if (Impulses)
			{
				const Vector3 Impulse = Old.Normal * Old.totalImpulseNormal
					+ Old.Tangent * Old.totalImpulseTangent + Old.Binormal * Old.totalImpulseBinormal;
				C.totalImpulseNormal = std::max(0.0f, Impulse.Dot(C.Normal));
				C.totalImpulseTangent = Impulse.Dot(C.Tangent);
				C.totalImpulseBinormal = Impulse.Dot(C.Binormal);
				C.CachedDt = P->Dt;
			}
			Manifold->ContactPoints[Index] = Reversed ? Storage::Flip(C) : C;
			++Matches;
		}
		return Matches;
	}

	void ContactManifoldManager::Store(const GeometryWorldState& StateA, const GeometryWorldState& StateB,
		ContactManifold& Manifold, float Dt, bool Regenerated, bool Accumulate)
	{
		if (!StateA.Geom || !StateB.Geom) return;
		Storage& S = *m_Storage;
		const bool Reversed = Storage::Reverse(StateA, StateB);
		const GeometryWorldState& A = Reversed ? StateB : StateA;
		const GeometryWorldState& B = Reversed ? StateA : StateB;
		if (Manifold.NumContactPointCount == 0)
		{
			S.Pairs.erase(Storage::MakeKey(A, B, Manifold.PatchId));
			return;
		}
		Storage::Pair* Previous = S.Find(A, B, Manifold.PatchId);
		std::vector<Storage::Point> OldPoints;
		if (Accumulate && Previous && Previous->LastFrame == S.Frame) OldPoints = Previous->Points;
		Storage::Pair& P = S.Pairs[Storage::MakeKey(A, B, Manifold.PatchId)];
		P.HandleA = A.Handle; P.HandleB = B.Handle;
		P.BodyA = A.Body; P.BodyB = B.Body;
		P.RevisionA = A.Geom->GetCollisionRevision(); P.RevisionB = B.Geom->GetCollisionRevision();
		P.SurfaceRevisionA = Storage::SurfaceRevision(A.Geom); P.SurfaceRevisionB = Storage::SurfaceRevision(B.Geom);
		P.LocalPoseA = *A.Geom->GetTransform(); P.LocalPoseB = *B.Geom->GetTransform();
		P.LastFrame = S.Frame; P.Dt = Dt;
		if (Regenerated)
		{
			P.GeneratedFrame = S.Frame;
			P.RelativePose = A.ShapeToWorld.TransformInv(B.ShapeToWorld);
		}
		P.Points.clear();
		bool Used[MAX_CONTACT_POINTS] = {};
		for (int Index = 0; Index < Manifold.NumContactPointCount; ++Index)
		{
			Contact& Input = Manifold.ContactPoints[Index];
			if (!Input.PersistentId) Input.PersistentId = S.NextId++;
			Storage::Point Point;
			Point.Value = Reversed ? Storage::Flip(Input) : Input;
			Point.AnchorA = A.ShapeToWorld.WorldToLocal(Point.Value.PositionWorldA);
			Point.AnchorB = B.ShapeToWorld.WorldToLocal(Point.Value.PositionWorldB);
			Point.NormalA = A.ShapeToWorld.WorldToLocalDirection(Point.Value.Normal);
			Point.TangentA = A.ShapeToWorld.WorldToLocalDirection(Point.Value.Tangent);
			float BestDistance = 2.0f * S.Settings.MatchDistance * S.Settings.MatchDistance;
			int Best = -1;
			for (int OldIndex = 0; OldIndex < (int)OldPoints.size(); ++OldIndex)
			{
				const Storage::Point& Old = OldPoints[(size_t)OldIndex];
				if (Used[OldIndex] || Point.NormalA.Dot(Old.NormalA) < S.Settings.NormalCosine) continue;
				if ((Point.Value.FeatureA != UINT32_MAX && Old.Value.FeatureA != UINT32_MAX && Point.Value.FeatureA != Old.Value.FeatureA)
					|| (Point.Value.FeatureB != UINT32_MAX && Old.Value.FeatureB != UINT32_MAX && Point.Value.FeatureB != Old.Value.FeatureB)) continue;
				const float Distance = (Point.AnchorA - Old.AnchorA).SquareLength() + (Point.AnchorB - Old.AnchorB).SquareLength();
				if (Distance < BestDistance) { BestDistance = Distance; Best = OldIndex; }
			}
			if (Best >= 0)
			{
				Used[Best] = true;
				const Storage::Point& Old = OldPoints[(size_t)Best];
				const Vector3 Impulse = Old.Value.Normal * Old.Value.totalImpulseNormal
					+ Old.Value.Tangent * Old.Value.totalImpulseTangent + Old.Value.Binormal * Old.Value.totalImpulseBinormal;
				Point.Value.totalImpulseNormal += std::max(0.0f, Impulse.Dot(Point.Value.Normal));
				Point.Value.totalImpulseTangent += Impulse.Dot(Point.Value.Tangent);
				Point.Value.totalImpulseBinormal += Impulse.Dot(Point.Value.Binormal);
			}
			P.Points.push_back(Point);
		}
	}

}
