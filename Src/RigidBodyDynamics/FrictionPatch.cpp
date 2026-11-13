#include "FrictionPatch.h"
#include "CollidingContact.h"
#include "Jacobian.h"
#include "RigidBody.h"
#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/HeightField3.h"
#include "../LinearSystem/EigenValueDecomposition.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

namespace Riemann
{
	/* A friction patch shares the summed normal load of a coherent contact region.
	   At most two separated anchors represent its tangential force and resisting
	   moment. Solve their coupled tangential effective-mass block (up to 4 x 4),
	   then project each anchor onto a circular Coulomb bound. Unlike independent tangent
	   clamps, that bound is invariant under rotation of the tangent basis.

	   Sticking anchors keep body-local attachment points across frames. Their
	   tangential separation supplies a bounded drift correction. Exceeding the
	   static cone releases the anchor into dynamic friction; reattachment requires
	   both low slip speed and an admissible static impulse. Sliding anchors are
	   relocated to the current contact footprint on the next frame.

	   Normal solvers remain separate numerical routes. They publish accumulated
	   normal impulses before each friction block; friction changes the same velocity
	   state before their next sweep. Warm starting occurs once per frame. A second
	   solve after pose correction retains the already-applied impulse and load.
	   Persistent state is invalidated by shape generations, collision revisions,
	   terrain edits, local-pose changes, excessive drift or loss of contact.
	   The patch block runs on the host; GPU normal solves synchronize with it at
	   each sweep. This optional route prioritizes shared behavior across backends
	   and incurs extra synchronization compared with batched contact-only LCP.

	   This is a block projected Gauss-Seidel approximation to Coulomb contact,
	   with equal load allocation between the patch's anchors. It does not model
	   rolling resistance or adhesion. Related background: Erin Catto, "Iterative
	   Dynamics with Temporal Coherence" (GDC 2005).
	*/
	namespace
	{
		bool Finite(const Vector3& V)
		{
			return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z);
		}
		float InverseMass(RigidBody* Body)
		{
			RigidBodyDynamic* Dynamic = Body ? Body->CastDynamic() : nullptr;
			return Dynamic && !Dynamic->Freezing ? Dynamic->InvMass : 0.0f;
		}
		Vector3 InverseInertia(RigidBody* Body, const Vector3& V)
		{
			RigidBodyDynamic* Dynamic = Body ? Body->CastDynamic() : nullptr;
			return Dynamic && !Dynamic->Freezing ? Body->GetInverseInertia_WorldSpace() * V : Vector3::Zero();
		}
		uint64_t SurfaceRevision(Geometry* Shape)
		{
			const HeightField3* Field = Shape->GetShapeObj<HeightField3>();
			return Field ? Field->GetRevision() : 0;
		}
		Vector3 LimitLength(const Vector3& V, float Limit)
		{
			const float Length = V.Length();
			return Length > Limit && Length > 0.0f ? V * (Limit / Length) : V;
		}
	}

	bool FrictionPatchSettings::IsValid() const
	{
		return std::isfinite(AnchorMatchDistance) && AnchorMatchDistance > 0.0f
			&& std::isfinite(AnchorBreakDistance) && AnchorBreakDistance >= AnchorMatchDistance
			&& std::isfinite(MinimumAnchorSeparation) && MinimumAnchorSeparation > 0.0f
			&& std::isfinite(ContactTolerance) && ContactTolerance >= 0.0f
			&& std::isfinite(NormalCosine) && NormalCosine >= 0.0f && NormalCosine <= 1.0f
			&& std::isfinite(PositionBias) && PositionBias >= 0.0f && PositionBias <= 1.0f
			&& std::isfinite(MaximumCorrectionSpeed) && MaximumCorrectionSpeed >= 0.0f
			&& std::isfinite(RestickSpeed) && RestickSpeed >= 0.0f
			&& std::isfinite(WarmStartFactor) && WarmStartFactor >= 0.0f && WarmStartFactor <= 1.0f
			&& std::isfinite(MinimumTimeStepRatio) && MinimumTimeStepRatio > 0.0f
			&& std::isfinite(MaximumTimeStepRatio) && MaximumTimeStepRatio >= MinimumTimeStepRatio
			&& std::isfinite(MinimumEffectiveMass) && MinimumEffectiveMass > 0.0f
			&& std::isfinite(RelativeRankTolerance) && RelativeRankTolerance > 0.0f
			&& RelativeRankTolerance < 1.0f && RetentionFrames > 0;
	}

	struct FrictionPatchSolver::Storage
	{
		struct Anchor
		{
			Vector3 LocalA, LocalB;
			Vector3 Impulse = Vector3::Zero();
			Vector3 CachedImpulseA = Vector3::Zero();
			Vector3 WarmImpulse = Vector3::Zero();
			bool WarmPending = false;
			bool Sliding = false;
		};
		struct Patch
		{
			RigidBody* A = nullptr;
			RigidBody* B = nullptr;
			GeometryHandle HandleA, HandleB;
			Transform LocalPoseA, LocalPoseB;
			uint64_t RevisionA = 0, RevisionB = 0, SurfaceA = 0, SurfaceB = 0;
			uint64_t Frame = 0, SolveId = 0;
			Vector3 NormalA = Vector3::Zero();
			std::vector<Anchor> Anchors;
			float Dt = 0.0f;
			float PreviousNormal = 0.0f;
			float TotalNormal = 0.0f;
		};
		struct Reference
		{
			Contact* Point;
			bool Reversed;
			Vector3 PositionA() const { return Reversed ? Point->PositionWorldB : Point->PositionWorldA; }
			Vector3 PositionB() const { return Reversed ? Point->PositionWorldA : Point->PositionWorldB; }
			Vector3 Midpoint() const { return (PositionA() + PositionB()) * 0.5f; }
		};
		using Key = std::tuple<uintptr_t, uintptr_t, uint32_t, size_t>;
		struct ActivePatch
		{
			Key Id;
			Geometry* ShapeA;
			Geometry* ShapeB;
			Vector3 Normal;
			std::vector<Reference> Points;
		};
		FrictionPatchSettings Settings;
		FrictionPatchStatistics Statistics;
		uint64_t Frame = 1, SolveId = 0;
		bool WarmStartingEnabled = true;
		bool CarryNormalLoad = true;
		RagdollVelocityContext* ReducedRagdolls = nullptr;
		std::map<Key, Patch> Patches;
		std::vector<ActivePatch> Active;
		std::unordered_map<RigidBody*, GeneralizedVelocity*> Velocities;

		GeneralizedVelocity Read(RigidBody* Body) const
		{
			RigidBodyDynamic* Dynamic = Body->CastDynamic();
			if (Body->mRigidType == RigidType::Static || (Dynamic && Dynamic->Freezing))
				return { Vector3::Zero(), Vector3::Zero() };
			const auto Found = Velocities.find(Body);
			return Found == Velocities.end() ? GeneralizedVelocity{ Body->V, Body->W } : *Found->second;
		}
		void Apply(RigidBody* Body, const Vector3& Offset, const Vector3& Impulse)
		{
			const float Mass = InverseMass(Body);
			RigidBodyDynamic* Dynamic = Body->CastDynamic();
			if (!Dynamic || Dynamic->Freezing || Impulse.SquareLength() == 0.0f) return;
			if (Body->ReducedRagdoll && ReducedRagdolls)
			{
				RagdollImpulseResponse Response;
				ReducedRagdolls->Response(Body, Impulse, Offset.Cross(Impulse), nullptr, Vector3::Zero(), Vector3::Zero(), Response);
				Response.Apply(1); Body->CastDynamic()->Wakeup(); return;
			}
			const Vector3 Angular = InverseInertia(Body, Offset.Cross(Impulse));
			const auto Found = Velocities.find(Body);
			if (Found == Velocities.end()) { Body->V += Impulse * Mass; Body->W += Angular; }
			else { Found->second->v += Impulse * Mass; Found->second->w += Angular; }
			Body->CastDynamic()->Wakeup();
		}
		Vector3 RelativeVelocity(const Patch& P, const Vector3& RA, const Vector3& RB) const
		{
			const GeneralizedVelocity A = Read(P.A), B = Read(P.B);
			return B.v + B.w.Cross(RB) - A.v - A.w.Cross(RA);
		}
		void Attach(Anchor& A, const Patch& P, const Vector3& Point)
		{
			A.LocalA = P.A->WorldToBodyLocal(Point);
			A.LocalB = P.B->WorldToBodyLocal(Point);
		}
	};

	FrictionPatchSolver::FrictionPatchSolver(const FrictionPatchSettings& Settings) : m_Storage(new Storage)
	{
		SetSettings(Settings);
	}
	FrictionPatchSolver::~FrictionPatchSolver() = default;
	void FrictionPatchSolver::SetSettings(const FrictionPatchSettings& Settings)
	{
		if (!Settings.IsValid()) throw std::invalid_argument("Invalid friction patch settings");
		m_Storage->Settings = Settings;
		Reset();
	}
	const FrictionPatchSettings& FrictionPatchSolver::GetSettings() const { return m_Storage->Settings; }
	void FrictionPatchSolver::SetWarmStartingEnabled(bool Enabled)
	{
		if (m_Storage->WarmStartingEnabled == Enabled) return;
		m_Storage->WarmStartingEnabled = Enabled;
		Reset();
	}
	const FrictionPatchStatistics& FrictionPatchSolver::GetStatistics() const { return m_Storage->Statistics; }
	void FrictionPatchSolver::Reset()
	{
		m_Storage->Patches.clear(); m_Storage->Active.clear(); m_Storage->Velocities.clear();
		m_Storage->Statistics = FrictionPatchStatistics();
	}
	void FrictionPatchSolver::BeginFrame(uint64_t Frame)
	{
		Storage& S = *m_Storage;
		if (Frame < S.Frame) Reset();
		S.Frame = Frame;
		for (auto It = S.Patches.begin(); It != S.Patches.end();)
		{
			if (Frame - It->second.Frame > S.Settings.RetentionFrames) It = S.Patches.erase(It);
			else ++It;
		}
	}
	void FrictionPatchSolver::BeginSolve(const std::vector<RigidBody*>& Bodies, GeneralizedVelocity* Velocities, bool CarryNormalLoad, RagdollVelocityContext* ReducedRagdolls)
	{
		Storage& S = *m_Storage;
		++S.SolveId;
		S.CarryNormalLoad = CarryNormalLoad; S.ReducedRagdolls = ReducedRagdolls;
		S.Active.clear(); S.Velocities.clear();
		if (S.Settings.Enabled && Velocities)
			for (size_t Index = 0; Index < Bodies.size(); ++Index) S.Velocities.emplace(Bodies[Index], Velocities + Index);
	}
	void FrictionPatchSolver::Prepare(const std::vector<Geometry*>& Geometries,
		const std::vector<ContactManifold*>& Manifolds, float Dt)
	{
		if (!m_Storage->Settings.Enabled) return;
		std::vector<FrictionContact> Points;
		for (ContactManifold* Manifold : Manifolds)
		{
			if (!Manifold || Manifold->indexA < 0 || Manifold->indexB < 0
				|| (size_t)Manifold->indexA >= Geometries.size() || (size_t)Manifold->indexB >= Geometries.size()) continue;
			for (int Index = 0; Index < std::min(MAX_CONTACT_POINTS, Manifold->NumContactPointCount); ++Index)
				Points.push_back({ Geometries[Manifold->indexA], Geometries[Manifold->indexB], Manifold->PatchId, &Manifold->ContactPoints[Index] });
		}
		Prepare(Points, Dt);
	}

	void FrictionPatchSolver::Prepare(const std::vector<FrictionContact>& Contacts, float Dt)
	{
		Storage& S = *m_Storage;
		S.Active.clear(); S.Statistics = FrictionPatchStatistics();
		if (!S.Settings.Enabled || !std::isfinite(Dt) || Dt <= 0.0f) return;
		std::map<Storage::Key, size_t> ActiveIndices;
		for (const FrictionContact& Input : Contacts)
		{
			if (!Input.ShapeA || !Input.ShapeB || !Input.Point || !Finite(Input.Point->Normal)
				|| !Finite(Input.Point->PositionWorldA) || !Finite(Input.Point->PositionWorldB)) continue;
			if (!Input.ShapeA->IsSimulationEnabled() || !Input.ShapeB->IsSimulationEnabled()
				|| Input.ShapeA->IsTriggerEnabled() || Input.ShapeB->IsTriggerEnabled()) continue;
			RigidBody* A = Input.ShapeA->GetParent<RigidBody>();
			RigidBody* B = Input.ShapeB->GetParent<RigidBody>();
			if (!A || !B || A == B) continue;
			const bool WritableA = A->CastDynamic() && !A->CastDynamic()->Freezing;
			const bool WritableB = B->CastDynamic() && !B->CastDynamic()->Freezing;
			if (!WritableA && !WritableB) continue;
			const uintptr_t AddressA = reinterpret_cast<uintptr_t>(Input.ShapeA), AddressB = reinterpret_cast<uintptr_t>(Input.ShapeB);
			const bool Reversed = AddressA > AddressB;
			const Vector3 Normal = (Reversed ? -Input.Point->Normal : Input.Point->Normal).SafeUnit();
			if (Normal.SquareLength() == 0.0f) continue;
			size_t Cluster = 0;
			for (;; ++Cluster)
			{
				const Storage::Key Id(std::min(AddressA, AddressB), std::max(AddressA, AddressB), Input.PatchId, Cluster);
				const auto Found = ActiveIndices.find(Id);
				if (Found != ActiveIndices.end())
				{
					Storage::ActivePatch& Active = S.Active[Found->second];
					if (Active.Normal.Dot(Normal) < S.Settings.NormalCosine) continue;
					Active.Points.push_back({ Input.Point, Reversed });
				}
				else
				{
					ActiveIndices.emplace(Id, S.Active.size());
					S.Active.push_back({ Id, Reversed ? Input.ShapeB : Input.ShapeA,
						Reversed ? Input.ShapeA : Input.ShapeB, Normal, { { Input.Point, Reversed } } });
				}
				break;
			}
		}

		for (Storage::ActivePatch& Active : S.Active)
		{
			RigidBody* A = Active.ShapeA->GetParent<RigidBody>();
			RigidBody* B = Active.ShapeB->GetParent<RigidBody>();
			Storage::Patch& P = S.Patches[Active.Id];
			const GeometryHandle HandleA = A->FindGeometryHandle(Active.ShapeA), HandleB = B->FindGeometryHandle(Active.ShapeB);
			const bool Valid = P.A == A && P.B == B && P.HandleA == HandleA && P.HandleB == HandleB
				&& S.Frame - P.Frame <= 1
				&& P.RevisionA == Active.ShapeA->GetCollisionRevision() && P.RevisionB == Active.ShapeB->GetCollisionRevision()
				&& P.SurfaceA == SurfaceRevision(Active.ShapeA) && P.SurfaceB == SurfaceRevision(Active.ShapeB)
				&& P.LocalPoseA == *Active.ShapeA->GetTransform() && P.LocalPoseB == *Active.ShapeB->GetTransform()
				&& (A->Q * P.NormalA).Dot(Active.Normal) >= S.Settings.NormalCosine;
			if (!Valid) P = Storage::Patch();
			P.A = A; P.B = B; P.HandleA = HandleA; P.HandleB = HandleB;
			P.RevisionA = Active.ShapeA->GetCollisionRevision(); P.RevisionB = Active.ShapeB->GetCollisionRevision();
			P.SurfaceA = SurfaceRevision(Active.ShapeA); P.SurfaceB = SurfaceRevision(Active.ShapeB);
			P.LocalPoseA = *Active.ShapeA->GetTransform(); P.LocalPoseB = *Active.ShapeB->GetTransform();
			const bool NewFrame = P.Frame != S.Frame || !Valid;
			if (P.SolveId != S.SolveId || NewFrame) P.PreviousNormal = NewFrame || !S.CarryNormalLoad ? 0.0f : P.TotalNormal;
			const std::vector<Storage::Anchor> Previous = P.Anchors;
			if (NewFrame)
			{
				// Choose the widest pair to retain a useful friction moment arm.
				size_t First = 0, Second = 0;
				float Span = S.Settings.MinimumAnchorSeparation * S.Settings.MinimumAnchorSeparation;
				for (size_t I = 0; I < Active.Points.size(); ++I)
					for (size_t J = I + 1; J < Active.Points.size(); ++J)
					{
						const float Distance = (Active.Points[I].Midpoint() - Active.Points[J].Midpoint()).SquareLength();
						if (Distance > Span) { Span = Distance; First = I; Second = J; }
					}
				P.Anchors.clear();
				std::vector<bool> Used(Previous.size(), false);
				const size_t Candidates[] = { First, Second };
				const size_t Count = First == Second ? 1 : 2;
				for (size_t Index = 0; Index < Count; ++Index)
				{
					const Vector3 Point = Active.Points[Candidates[Index]].Midpoint();
					Storage::Anchor Anchor;
					S.Attach(Anchor, P, Point);
					float BestDistance = S.Settings.AnchorMatchDistance * S.Settings.AnchorMatchDistance * 2.0f;
					size_t Best = Previous.size();
					for (size_t Old = 0; Old < Previous.size(); ++Old)
					{
						if (Used[Old]) continue;
						const Vector3 PA = A->BodyLocalToWorld(Previous[Old].LocalA), PB = B->BodyLocalToWorld(Previous[Old].LocalB);
						if ((PB - PA).SquareLength() > S.Settings.AnchorBreakDistance * S.Settings.AnchorBreakDistance) continue;
						const float Distance = (PA - Point).SquareLength() + (PB - Point).SquareLength();
						if (Distance < BestDistance) { Best = Old; BestDistance = Distance; }
					}
					if (Best < Previous.size())
					{
						Used[Best] = true;
						Anchor = Previous[Best];
						if (Anchor.Sliding) S.Attach(Anchor, P, Point);
						const float Ratio = S.WarmStartingEnabled && P.Dt > 0.0f ? std::max(S.Settings.MinimumTimeStepRatio,
							std::min(S.Settings.MaximumTimeStepRatio, Dt / P.Dt)) : 0.0f;
						Anchor.WarmImpulse = A->Q * Anchor.CachedImpulseA * (S.Settings.WarmStartFactor * Ratio);
						Anchor.WarmImpulse -= Active.Normal * Anchor.WarmImpulse.Dot(Active.Normal);
						Anchor.WarmPending = true;
						Anchor.Impulse = Vector3::Zero();
						++S.Statistics.RetainedAnchors;
					}
					// A new contact first attempts the static cone. Applied external
					// forces can create a small pre-solve velocity even at perfect rest.
					P.Anchors.push_back(Anchor);
				}
			}
			P.NormalA = A->Q.Conjugate() * Active.Normal;
			P.Frame = S.Frame; P.SolveId = S.SolveId; P.Dt = Dt;
			S.Statistics.Anchors += P.Anchors.size();
		}
		S.Statistics.Patches = S.Active.size();
	}

	float FrictionPatchSolver::Solve(float Dt)
	{
		Storage& S = *m_Storage;
		if (!S.Settings.Enabled || !std::isfinite(Dt) || Dt <= 0.0f) return 0.0f;
		S.Statistics.SlidingAnchors = 0;
		float Error = 0.0f;
		for (Storage::ActivePatch& Active : S.Active)
		{
			Storage::Patch& P = S.Patches[Active.Id];
			float NormalLoad = P.PreviousNormal;
			bool Touching = false;
			for (const Storage::Reference& Ref : Active.Points)
			{
				Ref.Point->totalImpulseTangent = Ref.Point->totalImpulseBinormal = 0.0f;
				if (Ref.Point->PenetrationDepth >= -S.Settings.ContactTolerance && std::isfinite(Ref.Point->totalImpulseNormal))
				{
					Touching = true;
					NormalLoad += std::max(0.0f, Ref.Point->totalImpulseNormal);
				}
			}
			if (!Touching) NormalLoad = 0.0f;
			P.TotalNormal = NormalLoad;
			if (P.Anchors.empty()) continue;
			const float LoadPerAnchor = NormalLoad / (float)P.Anchors.size();
			const float DynamicFriction = std::sqrt(std::max(0.0f, P.A->GetFrictionDynamic() * P.B->GetFrictionDynamic()));
			const float StaticFriction = std::max(DynamicFriction,
				std::sqrt(std::max(0.0f, P.A->GetFrictionStatic() * P.B->GetFrictionStatic())));
			Vector3 Tangent, Binormal;
			Active.Normal.GetPerpVectors(Tangent, Binormal);
			const Vector3 Axes[] = { Tangent, Binormal };
			const size_t TangentDimensions = 2, MaximumRows = 4;
			const size_t Dimension = P.Anchors.size() * TangentDimensions;
			struct TangentRow { Vector3 Axis, AngularA, AngularB; double Speed, Bias; };
			TangentRow Rows[MaximumRows];
			// Warm-start all anchors before measuring any row velocity. Otherwise
			// row order would change the load shared by redundant friction anchors.
			for (Storage::Anchor& Anchor : P.Anchors)
			{
				const Vector3 PA = P.A->BodyLocalToWorld(Anchor.LocalA), PB = P.B->BodyLocalToWorld(Anchor.LocalB);
				const Vector3 RA = PA - P.A->X, RB = PB - P.B->X;
				if (Anchor.WarmPending)
				{
					Anchor.Impulse = LimitLength(Anchor.WarmImpulse, (Anchor.Sliding ? DynamicFriction : StaticFriction) * LoadPerAnchor);
					S.Apply(P.A, RA, -Anchor.Impulse); S.Apply(P.B, RB, Anchor.Impulse);
					Anchor.WarmPending = false;
				}
			}
			for (size_t Index = 0; Index < P.Anchors.size(); ++Index)
			{
				const Storage::Anchor& Anchor = P.Anchors[Index];
				const Vector3 PA = P.A->BodyLocalToWorld(Anchor.LocalA), PB = P.B->BodyLocalToWorld(Anchor.LocalB);
				const Vector3 RA = PA - P.A->X, RB = PB - P.B->X;
				const Vector3 Relative = S.RelativeVelocity(P, RA, RB);
				Vector3 Bias = Vector3::Zero();
				if (!Anchor.Sliding)
				{
					const Vector3 Difference = PB - PA;
					Bias = LimitLength((Difference - Active.Normal * Difference.Dot(Active.Normal)) * (S.Settings.PositionBias / Dt),
						S.Settings.MaximumCorrectionSpeed);
				}
				for (size_t Axis = 0; Axis < TangentDimensions; ++Axis)
					Rows[Index * TangentDimensions + Axis] = { Axes[Axis], RA.Cross(Axes[Axis]), RB.Cross(Axes[Axis]),
						Relative.Dot(Axes[Axis]), Bias.Dot(Axes[Axis]) };
			}
			// A patch's tangential rows are often redundant. Solve for the minimum-
			// norm TOTAL impulse with a spectral pseudoinverse. A regularized inverse
			// would amplify tiny, incompatible anchor drift in the null space into
			// large internal forces and incorrectly break static friction.
			double Block[MaximumRows][MaximumRows + 2] = {};
			double OldImpulse[MaximumRows] = {};
			for (size_t I = 0; I < Dimension; ++I)
				OldImpulse[I] = P.Anchors[I / TangentDimensions].Impulse.Dot(Rows[I].Axis);
			RagdollImpulseResponse Responses[MaximumRows];
			const bool ReducedRagdoll = S.ReducedRagdolls && (P.A->ReducedRagdoll || P.B->ReducedRagdoll);
			if (ReducedRagdoll) for (size_t J = 0; J < Dimension; ++J)
				S.ReducedRagdolls->Response(P.A, -Rows[J].Axis, -Rows[J].AngularA, P.B, Rows[J].Axis, Rows[J].AngularB, Responses[J]);
			double Scale = 0.0;
			for (size_t I = 0; I < Dimension; ++I)
			{
				for (size_t J = 0; J < Dimension; ++J)
					Block[I][J] = ReducedRagdoll ? S.ReducedRagdolls->Project(Responses[J], P.A, -Rows[I].Axis, -Rows[I].AngularA, P.B, Rows[I].Axis, Rows[I].AngularB)
						: (InverseMass(P.A) + InverseMass(P.B)) * Rows[I].Axis.Dot(Rows[J].Axis)
						+ Rows[I].AngularA.Dot(InverseInertia(P.A, Rows[J].AngularA))
						+ Rows[I].AngularB.Dot(InverseInertia(P.B, Rows[J].AngularB));
				Scale = std::max(Scale, Block[I][I]);
				Block[I][Dimension] = -Rows[I].Speed - Rows[I].Bias;
				Block[I][Dimension + 1] = -Rows[I].Speed;
				for (size_t J = 0; J < Dimension; ++J)
				{
					Block[I][Dimension] += Block[I][J] * OldImpulse[J];
					Block[I][Dimension + 1] += Block[I][J] * OldImpulse[J];
				}
			}
			if (!std::isfinite(Scale) || Scale <= S.Settings.MinimumEffectiveMass) continue;
			double Normalized[MaximumRows * MaximumRows], Eigenvalues[MaximumRows], Eigenvectors[MaximumRows * MaximumRows];
			for (size_t I = 0; I < Dimension; ++I)
				for (size_t J = 0; J < Dimension; ++J) Normalized[I * Dimension + J] = Block[I][J] / Scale;
			Maths::LinearAlgebra::SymmetricEigenSolver<double>()(Normalized, (int)Dimension, Eigenvalues, Eigenvectors);
			double Solution[MaximumRows][2] = {};
			for (size_t Mode = 0; Mode < Dimension; ++Mode)
			{
				if (Eigenvalues[Mode] <= S.Settings.RelativeRankTolerance * Eigenvalues[0]) continue;
				for (size_t Target = 0; Target < 2; ++Target)
				{
					double Projection = 0.0;
					for (size_t Row = 0; Row < Dimension; ++Row) Projection += Eigenvectors[Row * Dimension + Mode] * Block[Row][Dimension + Target];
					Projection /= Eigenvalues[Mode] * Scale;
					for (size_t Row = 0; Row < Dimension; ++Row) Solution[Row][Target] += Eigenvectors[Row * Dimension + Mode] * Projection;
				}
			}
			for (size_t Index = 0; Index < P.Anchors.size(); ++Index)
			{
				Storage::Anchor& Anchor = P.Anchors[Index];
				const Vector3 PA = P.A->BodyLocalToWorld(Anchor.LocalA), PB = P.B->BodyLocalToWorld(Anchor.LocalB);
				const Vector3 RA = PA - P.A->X, RB = PB - P.B->X;
				const size_t RowT = Index * TangentDimensions, RowB = RowT + 1;
				const Vector3 Slip = Tangent * (float)Rows[RowT].Speed + Binormal * (float)Rows[RowB].Speed;
				const float LambdaT = (float)Solution[RowT][0];
				const float LambdaB = (float)Solution[RowB][0];
				Vector3 Candidate = Tangent * LambdaT + Binormal * LambdaB;
				const bool CanStick = !Anchor.Sliding || Slip.Length() <= S.Settings.RestickSpeed;
				if (CanStick && Candidate.Length() <= StaticFriction * LoadPerAnchor)
				{
					if (Anchor.Sliding) S.Attach(Anchor, P, (PA + PB) * 0.5f);
					Anchor.Sliding = false;
				}
				else
				{
					Anchor.Sliding = true;
					Candidate = Tangent * (float)Solution[RowT][1] + Binormal * (float)Solution[RowB][1];
					Candidate = LimitLength(Candidate, DynamicFriction * LoadPerAnchor);
				}
				const Vector3 Delta = Candidate - Anchor.Impulse;
				S.Apply(P.A, RA, -Delta); S.Apply(P.B, RB, Delta);
				Error = std::max(Error, Delta.SquareLength());
				Anchor.Impulse = Candidate;
				Anchor.CachedImpulseA = P.A->Q.Conjugate() * Candidate;
				if (Anchor.Sliding) ++S.Statistics.SlidingAnchors;
				// Report at the closest geometric point. The solver itself applies the
				// impulse at its persistent anchor, whose torque can differ slightly.
				size_t Closest = 0;
				float Distance = (Active.Points[0].Midpoint() - (PA + PB) * 0.5f).SquareLength();
				for (size_t Index = 1; Index < Active.Points.size(); ++Index)
				{
					const float Next = (Active.Points[Index].Midpoint() - (PA + PB) * 0.5f).SquareLength();
					if (Next < Distance) { Distance = Next; Closest = Index; }
				}
				const Storage::Reference& Ref = Active.Points[Closest];
				const Vector3 Reported = Ref.Reversed ? -Candidate : Candidate;
				Ref.Point->totalImpulseTangent += Reported.Dot(Ref.Point->Tangent);
				Ref.Point->totalImpulseBinormal += Reported.Dot(Ref.Point->Binormal);
			}
		}
		return Error;
	}
}
