#include "SupportStabilization.h"
#include "CollidingContact.h"
#include "Constraints.h"
#include "RigidBody.h"
#include "../Collision/GeometryObject.h"
#include <algorithm>
#include <cmath>

namespace Riemann
{
	/* Shock propagation trades exact momentum conservation for support stability.
	   A breadth-first traversal assigns distance from immovable boundaries. Late
	   Gauss-Seidel sweeps visit low levels first and reduce the response of the
	   lower-level endpoint. At zero parent scale, previously solved supports cannot
	   be pushed back into the floor by an arbitrarily heavy child. Equal-level
	   edges retain both physical weights; unsupported islands receive no changes.
	   See Guendelman, Bridson and Fedkiw, "Nonconvex Rigid Bodies with Stacking"
	   (SIGGRAPH 2003).

	   This optional CPU stage runs after each solver/backend has committed its
	   physical substep. It does not replace SI/LCP/PBD/XPBD or temporal refinement.
	   Contacts and hard joints share one support order. Contact rows retain local
	   witnesses and re-evaluate lever arms and separation as poses change. Position
	   projection is split from velocity correction, so depenetration does not add
	   kinetic energy through a dt^-1 velocity reconstruction. The velocity sweep
	   removes inward normal motion without suppressing restitution already solved.
	   The normal/tangent block first tries sticking inside a Coulomb disk, then
	   sliding at its dynamic-friction radius, coupled to the normal active set.
	   These tangent impulses use the artificial normal load and remain separate
	   from the physical solver's cached friction patches.

	   Body mass/inertia, physical impulses and warm-start caches remain unchanged.
	   Joint drives are disabled only in this artificial stage; their physical
	   MaxForce*dt budget and reaction ledger belong to the ordinary solve. Artificial
	   changes are exposed in SupportStabilizationStats, not joint breaking loads.
	   Compliant joints are excluded to preserve their intended spring response.
	   Projection is bounded for contacts, uses existing joint trust regions, and
	   is not a substitute for CCD. Fast impacts, unrooted piles, cyclic supports,
	   and force-accurate load measurement still require the ordinary physical solver.
	   Artificial support impulses are not charged to physical contact-impulse caps.
	   Disable this stage for impulse-limited interactions or when the full supported
	   load must appear in joint reports.
	   Use PCM manifolds for face support: a single witness cannot resist every
	   tipping direction. Full collision detection runs once before this stage.
	*/
	namespace
	{
		constexpr float MinimumRowWeight = 1.0e-9f;
		constexpr float MinimumRotation = 1.0e-7f;
		bool Movable(RigidBody* Body)
		{
			const auto* Dynamic = Body ? Body->CastDynamic() : nullptr;
			return Dynamic && !Dynamic->Freezing && Dynamic->InvMass > 0
				&& (!Dynamic->IslandSleepManaged || !Dynamic->Sleeping);
		}
		float InverseMass(RigidBody* Body, float Scale)
		{
			return Movable(Body) ? Body->InvMass * Scale : 0;
		}
		Matrix3 InverseInertia(RigidBody* Body, float Scale)
		{
			return Movable(Body) ? Body->GetInverseInertia_WorldSpace() * Scale : Matrix3::Zero();
		}
		void Rotate(RigidBody* Body, const Vector3& Rotation)
		{
			const float Angle = Rotation.Length();
			if (Angle <= MinimumRotation) return;
			Quaternion Delta; Delta.FromRotationAxis(Rotation / Angle, Angle);
			Body->Q = (Delta * Body->Q).Unit();
		}
		Vector3 PointVelocity(RigidBody* Body, const Vector3& Offset)
		{
			if (Body->mRigidType == RigidType::Static || (Body->CastDynamic() && Body->CastDynamic()->Freezing))
				return Vector3::Zero();
			return Body->V + Body->W.Cross(Offset);
		}
		struct ContactRow
		{
			Vector3 LocalA, LocalB, InitialDelta, Normal;
			float Depth, PositionImpulse = 0, VelocityImpulse = 0;
		};
		struct PreparedEdge
		{
			SupportGraph::Edge Edge;
			float ScaleA = 1, ScaleB = 1;
			std::vector<ContactRow> Rows;
			Vector3 FrictionImpulse = Vector3::Zero();
		};
		// Enumerate active normal rows in a tiny contact LCP. Redundant flat patches
		// are rank deficient; a full-rank subset still resolves all witnesses. In
		// the velocity pass, solve both tangent rows in the same block. Alternating
		// normal and friction solves at low budgets can leave a spurious tipping
		// torque, even when every individual scalar row appears well conditioned.
		bool SolveContactBlock(PreparedEdge& Prepared, const SupportStabilizationSettings& Settings, float Dt, bool Position)
		{
			constexpr int MaximumRows = MAX_CONTACT_POINTS + 2;
			constexpr double RelativeRankTolerance = 1.0e-6;
			constexpr double ComplementarityTolerance = 1.0e-6;
			const int Normals = (int)Prepared.Rows.size();
			if (Normals == 0) return true;
			auto* A = Prepared.Edge.A; auto* B = Prepared.Edge.B;
			const float WA = InverseMass(A, Prepared.ScaleA), WB = InverseMass(B, Prepared.ScaleB);
			const Matrix3 InertiaA = InverseInertia(A, Prepared.ScaleA), InertiaB = InverseInertia(B, Prepared.ScaleB);
			const double MuDynamic = std::sqrt(std::max(0.0f, A->GetFrictionDynamic() * B->GetFrictionDynamic()));
			const double MuStatic = std::max(MuDynamic, (double)std::sqrt(std::max(0.0f, A->GetFrictionStatic() * B->GetFrictionStatic())));
			const bool Friction = !Position && MuStatic > 0;
			const int Count = Normals + (Friction ? 2 : 0);
			Vector3 Linear[MaximumRows], JA[MaximumRows], JB[MaximumRows], IA[MaximumRows], IB[MaximumRows];
			double Matrix[MaximumRows][MaximumRows] = {}, Bias[MaximumRows] = {}, Old[MaximumRows] = {}, Solution[MaximumRows] = {};
			Vector3 CenterA = Vector3::Zero(), CenterB = Vector3::Zero(), Normal = Vector3::Zero();
			for (int I = 0; I < Normals; ++I)
			{
				const auto& Row = Prepared.Rows[I];
				const Vector3 RA = A->Q * Row.LocalA, RB = B->Q * Row.LocalB;
				CenterA += RA; CenterB += RB; Normal += Row.Normal;
				Linear[I] = Row.Normal; JA[I] = -RA.Cross(Row.Normal); JB[I] = RB.Cross(Row.Normal);
				const float Depth = Row.Depth - ((B->X + RB - A->X - RA) - Row.InitialDelta).Dot(Row.Normal);
				Bias[I] = Position ? Settings.ContactTolerance - Depth
					: (PointVelocity(B, RB) - PointVelocity(A, RA)).Dot(Row.Normal) + std::max(0.0f, -Depth) / Dt;
				Old[I] = Position ? Row.PositionImpulse : Row.VelocityImpulse;
			}
			if (Friction)
			{
				CenterA /= (float)Normals; CenterB /= (float)Normals; Normal = Normal.SafeUnit();
				const Vector3 Axis = std::abs(Normal.x) < std::abs(Normal.y) ? Vector3::UnitX() : Vector3::UnitY();
				Linear[Normals] = Normal.Cross(Axis).SafeUnit(); Linear[Normals + 1] = Normal.Cross(Linear[Normals]);
				const Vector3 Relative = PointVelocity(B, CenterB) - PointVelocity(A, CenterA);
				for (int I = Normals; I < Count; ++I)
				{
					JA[I] = -CenterA.Cross(Linear[I]); JB[I] = CenterB.Cross(Linear[I]);
					Bias[I] = Relative.Dot(Linear[I]); Old[I] = Prepared.FrictionImpulse.Dot(Linear[I]);
				}
			}
			for (int I = 0; I < Count; ++I) { IA[I] = InertiaA * JA[I]; IB[I] = InertiaB * JB[I]; }
			double MatrixScale = 0, ResidualScale = 1;
			for (int I = 0; I < Count; ++I)
			{
				for (int J = 0; J < Count; ++J)
				{
					Matrix[I][J] = (double)(WA + WB) * Linear[I].Dot(Linear[J]) + JA[I].Dot(IA[J]) + JB[I].Dot(IB[J]);
					Bias[I] -= Matrix[I][J] * Old[J];
				}
				MatrixScale = std::max(MatrixScale, Matrix[I][I]); ResidualScale = std::max(ResidualScale, std::abs(Bias[I]));
			}
			if (MatrixScale <= MinimumRowWeight) return true;
			const double ImpulseTolerance = ComplementarityTolerance * ResidualScale / MatrixScale;
			const double ResidualTolerance = ComplementarityTolerance * ResidualScale;
			double SlipT = Friction ? Bias[Normals] : 0, SlipU = Friction ? Bias[Normals + 1] : 0;
			bool Feasible = false;
			for (int Mode = 0; Mode < (Friction ? 2 : 1) && !Feasible; ++Mode)
			{
				const bool Sliding = Mode == 1;
				if (Sliding)
				{
					const double Speed = std::sqrt(SlipT * SlipT + SlipU * SlipU);
					if (Speed <= ResidualTolerance) break;
					SlipT /= Speed; SlipU /= Speed;
				}
				for (int Mask = 0; Mask < (1 << Normals); ++Mask)
				{
					int Active[MaximumRows], Size = 0;
					for (int I = 0; I < Count; ++I) { Solution[I] = 0; if (I < Normals && (Mask & (1 << I))) Active[Size++] = I; }
					if (Friction && !Sliding) { Active[Size++] = Normals; Active[Size++] = Normals + 1; }
					double Augmented[MaximumRows][MaximumRows + 1] = {};
					for (int I = 0; I < Size; ++I)
					{
						for (int J = 0; J < Size; ++J)
						{
							Augmented[I][J] = Matrix[Active[I]][Active[J]];
							if (Sliding) Augmented[I][J] -= MuDynamic * (Matrix[Active[I]][Normals] * SlipT + Matrix[Active[I]][Normals + 1] * SlipU);
						}
						Augmented[I][Size] = -Bias[Active[I]];
					}
					bool FullRank = true;
					for (int Pivot = 0; Pivot < Size; ++Pivot)
					{
						int Best = Pivot;
						for (int I = Pivot + 1; I < Size; ++I) if (std::abs(Augmented[I][Pivot]) > std::abs(Augmented[Best][Pivot])) Best = I;
						if (std::abs(Augmented[Best][Pivot]) <= MatrixScale * RelativeRankTolerance) { FullRank = false; break; }
						for (int J = Pivot; J <= Size; ++J) std::swap(Augmented[Pivot][J], Augmented[Best][J]);
						for (int I = Pivot + 1; I < Size; ++I)
						{
							const double Factor = Augmented[I][Pivot] / Augmented[Pivot][Pivot];
							for (int J = Pivot; J <= Size; ++J) Augmented[I][J] -= Factor * Augmented[Pivot][J];
						}
					}
					if (!FullRank) continue;
					Feasible = true;
					for (int I = Size - 1; I >= 0; --I)
					{
						double Value = Augmented[I][Size];
						for (int J = I + 1; J < Size; ++J) Value -= Augmented[I][J] * Solution[Active[J]];
						Solution[Active[I]] = Value / Augmented[I][I];
						if (!std::isfinite(Solution[Active[I]]) || (Active[I] < Normals && Solution[Active[I]] < -ImpulseTolerance)) Feasible = false;
					}
					if (!Feasible) continue;
					double NormalImpulse = 0;
					for (int I = 0; I < Normals; ++I) { Solution[I] = std::max(0.0, Solution[I]); NormalImpulse += Solution[I]; }
					if (Sliding) { Solution[Normals] = -MuDynamic * NormalImpulse * SlipT; Solution[Normals + 1] = -MuDynamic * NormalImpulse * SlipU; }
					for (int I = 0; I < Normals; ++I)
					{
						double Residual = Bias[I]; for (int J = 0; J < Count; ++J) Residual += Matrix[I][J] * Solution[J];
						if (Residual < -ResidualTolerance || (Solution[I] > ImpulseTolerance && std::abs(Residual) > ResidualTolerance)) Feasible = false;
					}
					if (Feasible && Friction && !Sliding)
					{
						const double Magnitude = std::sqrt(Solution[Normals] * Solution[Normals] + Solution[Normals + 1] * Solution[Normals + 1]);
						if (Magnitude > MuStatic * NormalImpulse + ImpulseTolerance)
						{
							SlipT = -Solution[Normals]; SlipU = -Solution[Normals + 1]; Feasible = false;
						}
					}
					if (Feasible) break;
				}
			}
			if (!Feasible) return false;
			Vector3 Translation = Vector3::Zero(), AngularA = Vector3::Zero(), AngularB = Vector3::Zero();
			for (int I = 0; I < Count; ++I)
			{
				const float Delta = (float)(Solution[I] - Old[I]);
				Translation += Linear[I] * Delta; AngularA += IA[I] * Delta; AngularB += IB[I] * Delta;
			}
			float Scale = 1;
			if (Position)
			{
				const float Length = std::max(WA, WB) * Translation.Length();
				const float Angle = std::max(AngularA.Length(), AngularB.Length());
				if (Length > Settings.MaximumPositionCorrection) Scale = std::min(Scale, Settings.MaximumPositionCorrection / Length);
				if (Angle > Settings.MaximumAngularCorrection) Scale = std::min(Scale, Settings.MaximumAngularCorrection / Angle);
				A->X -= Translation * (WA * Scale); B->X += Translation * (WB * Scale);
				Rotate(A, AngularA * Scale); Rotate(B, AngularB * Scale);
			}
			else
			{
				A->V -= Translation * WA; B->V += Translation * WB;
				A->W += AngularA; B->W += AngularB;
			}
			for (int I = 0; I < Normals; ++I)
			{
				float& Lambda = Position ? Prepared.Rows[I].PositionImpulse : Prepared.Rows[I].VelocityImpulse;
				Lambda = (float)(Old[I] + Scale * (Solution[I] - Old[I]));
			}
			if (Friction) Prepared.FrictionImpulse = Linear[Normals] * (float)Solution[Normals] + Linear[Normals + 1] * (float)Solution[Normals + 1];
			if (!Translation.IsZero() || !AngularA.IsZero() || !AngularB.IsZero())
			{
				if (WA > 0) A->CastDynamic()->Wakeup(); if (WB > 0) B->CastDynamic()->Wakeup();
			}
			return true;
		}
		void SolveContact(PreparedEdge& Prepared, ContactRow& Row, const SupportStabilizationSettings& Settings, float Dt, bool Position)
		{
			auto* A = Prepared.Edge.A; auto* B = Prepared.Edge.B;
			const Vector3 RA = A->Q * Row.LocalA, RB = B->Q * Row.LocalB;
			const Vector3 AngularA = -RA.Cross(Row.Normal), AngularB = RB.Cross(Row.Normal);
			const float WA = InverseMass(A, Prepared.ScaleA), WB = InverseMass(B, Prepared.ScaleB);
			const Vector3 IA = InverseInertia(A, Prepared.ScaleA) * AngularA;
			const Vector3 IB = InverseInertia(B, Prepared.ScaleB) * AngularB;
			const float Weight = WA + WB + AngularA.Dot(IA) + AngularB.Dot(IB);
			if (Weight <= MinimumRowWeight) return;
			const float Depth = Row.Depth - ((B->X + RB - A->X - RA) - Row.InitialDelta).Dot(Row.Normal);
			float& Lambda = Position ? Row.PositionImpulse : Row.VelocityImpulse;
			const float Residual = Position ? Settings.ContactTolerance - Depth
				: (PointVelocity(B, RB) - PointVelocity(A, RA)).Dot(Row.Normal) + std::max(0.0f, -Depth) / Dt;
			float Delta = std::max(0.0f, Lambda - Residual / Weight) - Lambda;
			if (Position)
			{
				const float Translation = std::max(WA, WB) * std::abs(Delta);
				const float Angle = std::max(IA.Length(), IB.Length()) * std::abs(Delta);
				float Scale = 1;
				if (Translation > Settings.MaximumPositionCorrection) Scale = std::min(Scale, Settings.MaximumPositionCorrection / Translation);
				if (Angle > Settings.MaximumAngularCorrection) Scale = std::min(Scale, Settings.MaximumAngularCorrection / Angle);
				Delta *= Scale;
				A->X -= Row.Normal * (WA * Delta); B->X += Row.Normal * (WB * Delta);
				Rotate(A, IA * Delta); Rotate(B, IB * Delta);
			}
			else
			{
				A->V -= Row.Normal * (WA * Delta); B->V += Row.Normal * (WB * Delta);
				A->W += IA * Delta; B->W += IB * Delta;
			}
			Lambda += Delta;
			if (Delta != 0)
			{
				if (WA > 0) A->CastDynamic()->Wakeup();
				if (WB > 0) B->CastDynamic()->Wakeup();
			}
		}
	}

	bool SupportStabilizationSettings::IsValid() const
	{
		return PositionIterations > 0 && VelocityIterations > 0
			&& std::isfinite(ParentInverseMassScale) && ParentInverseMassScale >= 0 && ParentInverseMassScale <= 1
			&& std::isfinite(ContactTolerance) && ContactTolerance >= 0
			&& std::isfinite(MaximumPositionCorrection) && MaximumPositionCorrection > 0
			&& std::isfinite(MaximumAngularCorrection) && MaximumAngularCorrection > 0;
	}
	int SupportGraph::GetLevel(const RigidBody* Body) const
	{
		const auto It = m_Levels.find(Body); return It == m_Levels.end() ? -1 : It->second;
	}
	void SupportGraph::Build(const std::vector<Geometry*>& Geometries, const std::vector<ContactManifold*>& Contacts,
		const std::vector<PositionConstraint*>& Joints, float ContactTolerance)
	{
		m_Levels.clear(); m_Edges.clear();
		for (auto* Contact : Contacts)
		{
			if (!Contact || Contact->indexA < 0 || Contact->indexB < 0 || (size_t)Contact->indexA >= Geometries.size()
				|| (size_t)Contact->indexB >= Geometries.size()) continue;
			Geometry* GA = Geometries[Contact->indexA]; Geometry* GB = Geometries[Contact->indexB];
			if (!GA || !GB || GA->IsTriggerEnabled() || GB->IsTriggerEnabled() || !GA->IsSimulationEnabled() || !GB->IsSimulationEnabled()) continue;
			auto* A = GA->GetParent<RigidBody>(); auto* B = GB->GetParent<RigidBody>();
			if (!A || !B || A == B || (!Movable(A) && !Movable(B))) continue;
			bool Touching = false;
			for (int I = 0; I < std::min(Contact->NumContactPointCount, MAX_CONTACT_POINTS); ++I)
				Touching |= Contact->ContactPoints[I].PenetrationDepth >= -ContactTolerance;
			if (Touching) m_Edges.push_back({ A, B, Contact, nullptr });
		}
		for (auto* Joint : Joints)
		{
			if (!Joint || !Joint->IsEnabled() || Joint->GetLinearCompliance() > 0 || Joint->GetAngularCompliance() > 0) continue;
			auto* A = Joint->GetBodyA(); auto* B = Joint->GetBodyB();
			if (A && B && A != B && (Movable(A) || Movable(B))) m_Edges.push_back({ A, B, nullptr, Joint });
		}
		std::unordered_map<RigidBody*, std::vector<RigidBody*>> Neighbors;
		std::vector<RigidBody*> Queue;
		for (const auto& Edge : m_Edges)
		{
			Neighbors[Edge.A].push_back(Edge.B); Neighbors[Edge.B].push_back(Edge.A);
			for (auto* Body : { Edge.A, Edge.B }) if (m_Levels.emplace(Body, Movable(Body) ? -1 : 0).second && !Movable(Body)) Queue.push_back(Body);
		}
		for (size_t Index = 0; Index < Queue.size(); ++Index)
			for (auto* Next : Neighbors[Queue[Index]]) if (m_Levels[Next] < 0)
			{
				m_Levels[Next] = m_Levels[Queue[Index]] + 1; Queue.push_back(Next);
			}
		for (auto& Edge : m_Edges) { Edge.LevelA = GetLevel(Edge.A); Edge.LevelB = GetLevel(Edge.B); }
		m_Edges.erase(std::remove_if(m_Edges.begin(), m_Edges.end(), [](const Edge& E) { return E.LevelA < 0 || E.LevelB < 0; }), m_Edges.end());
		std::stable_sort(m_Edges.begin(), m_Edges.end(), [](const Edge& A, const Edge& B)
		{
			return std::max(A.LevelA, A.LevelB) < std::max(B.LevelA, B.LevelB);
		});
	}
	void StabilizeSupports(const SupportGraph& Graph, const SupportStabilizationSettings& Settings, float Dt, SupportStabilizationStats& Stats)
	{
		if (!Settings.Enabled || !(Dt > 0)) return;
		struct Snapshot { Vector3 X, V, W; };
		std::unordered_map<RigidBody*, Snapshot> Before;
		std::vector<PreparedEdge> Edges;
		for (const auto& Edge : Graph.GetEdges())
		{
			PreparedEdge P; P.Edge = Edge;
			if (Edge.LevelA < Edge.LevelB) P.ScaleA = Settings.ParentInverseMassScale;
			if (Edge.LevelB < Edge.LevelA) P.ScaleB = Settings.ParentInverseMassScale;
			Stats.MaximumLevel = std::max(Stats.MaximumLevel, std::max(Edge.LevelA, Edge.LevelB));
			for (auto* Body : { Edge.A, Edge.B }) Before.emplace(Body, Snapshot{ Body->X, Body->V, Body->W });
			if (Edge.Contact) for (int Index = 0; Index < std::min(Edge.Contact->NumContactPointCount, MAX_CONTACT_POINTS); ++Index)
			{
				const auto& C = Edge.Contact->ContactPoints[Index];
				ContactRow R;
				R.LocalA = Edge.A->Q.Conjugate() * (C.PositionWorldA - Edge.A->X);
				R.LocalB = Edge.B->Q.Conjugate() * (C.PositionWorldB - Edge.B->X);
				R.InitialDelta = C.PositionWorldB - C.PositionWorldA; R.Normal = C.Normal; R.Depth = C.PenetrationDepth;
				P.Rows.push_back(R);
			}
			Edges.push_back(std::move(P));
		}
		for (bool Positions : { true, false })
			for (int Iteration = 0; Iteration < (Positions ? Settings.PositionIterations : Settings.VelocityIterations); ++Iteration)
				for (auto& Edge : Edges)
				{
					if (Edge.Edge.Joint) { Edge.Edge.Joint->Stabilize(Dt, Edge.ScaleA, Edge.ScaleB, Positions); ++Stats.JointPasses; }
					if (!SolveContactBlock(Edge, Settings, Dt, Positions))
					{
						++Stats.ContactBlockFallbacks;
						// A failed coupled solve falls back to normal-only PGS. Remove any
						// tangent impulse from earlier sweeps before changing that model.
						if (!Positions && !Edge.FrictionImpulse.IsZero())
						{
							auto* A = Edge.Edge.A; auto* B = Edge.Edge.B;
							Vector3 RA = Vector3::Zero(), RB = Vector3::Zero();
							for (const auto& Row : Edge.Rows) { RA += A->Q * Row.LocalA; RB += B->Q * Row.LocalB; }
							RA /= (float)Edge.Rows.size(); RB /= (float)Edge.Rows.size();
							A->V += Edge.FrictionImpulse * InverseMass(A, Edge.ScaleA); B->V -= Edge.FrictionImpulse * InverseMass(B, Edge.ScaleB);
							A->W += InverseInertia(A, Edge.ScaleA) * RA.Cross(Edge.FrictionImpulse);
							B->W -= InverseInertia(B, Edge.ScaleB) * RB.Cross(Edge.FrictionImpulse);
							Edge.FrictionImpulse.SetZero();
						}
						for (auto& Row : Edge.Rows) SolveContact(Edge, Row, Settings, Dt, Positions);
					}
					Stats.ContactPointPasses += (int)Edge.Rows.size();
				}
		for (const auto& Pair : Before)
		{
			Stats.MaximumPositionChange = std::max(Stats.MaximumPositionChange, (Pair.first->X - Pair.second.X).Length());
			Stats.MaximumLinearVelocityChange = std::max(Stats.MaximumLinearVelocityChange, (Pair.first->V - Pair.second.V).Length());
			Stats.MaximumAngularVelocityChange = std::max(Stats.MaximumAngularVelocityChange, (Pair.first->W - Pair.second.W).Length());
		}
	}
}
