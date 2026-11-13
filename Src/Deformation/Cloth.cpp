#include "Cloth.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

#include "../RigidBodyDynamics/RigidBody.h"
#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/StaticMesh.h"
#include "../CollisionPrimitive/TriangleMesh.h"

namespace Riemann
{
	namespace
	{
		constexpr float ReferenceTimeStep = 1.0f / 60, MinimumGridSize = 0.01f, MinimumThickness = 0.001f;
		bool Finite(const Vector3& P) { return std::isfinite(P.x) && std::isfinite(P.y) && std::isfinite(P.z); }
	}
	ClothSettings::ClothSettings()
		: Rows(16)
		, Columns(16)
		, SolverIterations(12)
		, Width(8.0f)
		, Height(8.0f)
		, Thickness(0.08f)
		, ParticleInverseMass(1.0f)
		, Damping(0.995f)
		, Gravity(0.0f, -9.8f, 0.0f)
	{
	}

	ClothSystem::ClothSystem(const ClothSettings& settings, const Vector3& center, const Vector3& horizontalAxis, const Vector3& verticalAxis)
		: m_Settings(settings)
		, m_RenderGeometry(nullptr)
	{
		ValidateSettings();
		if (!Finite(center) || !Finite(horizontalAxis) || !Finite(verticalAxis)) throw std::invalid_argument("Invalid cloth grid frame");
		m_Settings.Rows = std::max(2, m_Settings.Rows);
		m_Settings.Columns = std::max(2, m_Settings.Columns);
		m_Settings.SolverIterations = std::max(1, m_Settings.SolverIterations);
		m_Settings.Width = std::max(MinimumGridSize, m_Settings.Width);
		m_Settings.Height = std::max(MinimumGridSize, m_Settings.Height);
		m_Settings.Thickness = std::max(MinimumThickness, m_Settings.Thickness);
		m_Settings.ParticleInverseMass = std::max(0.0f, m_Settings.ParticleInverseMass);
		m_Settings.Damping = std::max(0.0f, std::min(1.0f, m_Settings.Damping));

		Vector3 horizontal = horizontalAxis.SafeUnit();
		Vector3 vertical = verticalAxis.SafeUnit();
		if (horizontal.IsZero()) horizontal = Vector3::UnitX();
		if (vertical.IsZero()) vertical = Vector3::UnitY();
		vertical = (vertical - horizontal * vertical.Dot(horizontal)).SafeUnit();
		if (vertical.IsZero())
		{
			Vector3 unused;
			horizontal.GetPerpVectors(vertical, unused);
		}

		const size_t Count = (size_t)m_Settings.Rows * (size_t)m_Settings.Columns;
		if (Count > (size_t)std::numeric_limits<int>::max()) throw std::invalid_argument("Cloth grid has too many vertices");
		ClothMesh Mesh; Mesh.Positions.resize(Count); Mesh.MaterialCoordinates.resize(Count); Mesh.InverseMasses.resize(Count, m_Settings.ParticleInverseMass);
		for (int row = 0; row < m_Settings.Rows; ++row)
		{
			for (int column = 0; column < m_Settings.Columns; ++column)
			{
				const float u = (float)column / (float)(m_Settings.Columns - 1) - 0.5f;
				const float v = 0.5f - (float)row / (float)(m_Settings.Rows - 1);
				const int Index = ParticleIndex(row, column);
				Mesh.Positions[Index] = center + horizontal * (u * m_Settings.Width) + vertical * (v * m_Settings.Height);
				Mesh.MaterialCoordinates[Index] = Vector2(u, v);
				if (row + 1 < m_Settings.Rows && column + 1 < m_Settings.Columns)
				{
					const int Below = ParticleIndex(row + 1, column);
					Mesh.Triangles.emplace_back(Index, Below, Index + 1); Mesh.Triangles.emplace_back(Index + 1, Below, Below + 1);
				}
			}
		}

		for (int row = 0; row < m_Settings.Rows; ++row)
		{
			for (int column = 0; column < m_Settings.Columns; ++column)
			{
				const int particle = ParticleIndex(row, column);
				if (column + 1 < m_Settings.Columns) Mesh.DistanceLinks.emplace_back(particle, ParticleIndex(row, column + 1));
				if (row + 1 < m_Settings.Rows) Mesh.DistanceLinks.emplace_back(particle, ParticleIndex(row + 1, column));
				if (row + 1 < m_Settings.Rows && column + 1 < m_Settings.Columns)
				{
					Mesh.DistanceLinks.emplace_back(particle, ParticleIndex(row + 1, column + 1));
					Mesh.DistanceLinks.emplace_back(ParticleIndex(row, column + 1), ParticleIndex(row + 1, column));
				}
				if (column + 2 < m_Settings.Columns) Mesh.DistanceLinks.emplace_back(particle, ParticleIndex(row, column + 2));
				if (row + 2 < m_Settings.Rows) Mesh.DistanceLinks.emplace_back(particle, ParticleIndex(row + 2, column));
			}
		}
		Initialize(Mesh);

		BuildRenderMesh();
	}
	ClothSystem::ClothSystem(const ClothSettings& Settings, const ClothMesh& Mesh) : m_Settings(Settings), m_RenderGeometry(nullptr)
	{
		ValidateSettings(); m_Settings.Rows = m_Settings.Columns = 0; Initialize(Mesh); BuildRenderMesh();
	}
	void ClothSystem::ValidateSettings()
	{
		if (!std::isfinite(m_Settings.Width) || !std::isfinite(m_Settings.Height) || !std::isfinite(m_Settings.Thickness) || m_Settings.Thickness < 0
			|| !std::isfinite(m_Settings.ParticleInverseMass) || m_Settings.ParticleInverseMass < 0 || !std::isfinite(m_Settings.Damping)
			|| m_Settings.Damping < 0 || m_Settings.Damping > 1 || !Finite(m_Settings.Gravity) || m_Settings.Substeps < 1
			|| !std::isfinite(m_Settings.SurfaceDensity) || m_Settings.SurfaceDensity <= 0 || !m_Settings.Material.IsValid()
			|| !m_Settings.Wind.IsValid() || !m_Settings.AnimationDrive.IsValid() || !m_Settings.Collision.IsValid()
			|| (m_Settings.Model != ClothModel::DistanceLinks && m_Settings.Model != ClothModel::TriangleStrain))
			throw std::invalid_argument("Invalid cloth settings");
		m_Settings.SolverIterations = std::max(1, m_Settings.SolverIterations);
	}
	void ClothSystem::Initialize(const ClothMesh& Mesh)
	{
		m_Topology.reset(new ClothTopology(Mesh)); m_Particles.resize(Mesh.Positions.size());
		for (size_t I = 0; I < m_Particles.size(); ++I)
		{
			auto& P = m_Particles[I]; P.Position = P.PreviousPosition = P.PinPosition = Mesh.Positions[I]; P.Pinned = false;
			P.InverseMass = !Mesh.InverseMasses.empty() ? Mesh.InverseMasses[I]
				: 1 / (m_Settings.SurfaceDensity * m_Topology->GetVertexAreas()[I]);
			if (!std::isfinite(P.InverseMass)) throw std::invalid_argument("Unrepresentable cloth particle mass");
		}
		for (const auto& Edge : m_Topology->GetDistanceLinks()) m_Links.push_back({ Edge.A, Edge.B, Edge.Length });
		m_StrainMultipliers.resize(m_Topology->GetTriangles().size(), Vector3::Zero()); m_BendMultipliers.resize(m_Topology->GetBends().size(), 0);
		m_Animation = m_AnimationStart = m_AnimationEnd = Mesh.Positions;
		m_AnimationVelocity.resize(m_Particles.size(), Vector3::Zero()); m_AnimationMultipliers = m_AnimationVelocity;
		m_AnimationWeights.resize(m_Particles.size(), 1);
		m_Collision.reset(new ClothCollision(*this));
	}

	ClothSystem::~ClothSystem()
	{
		delete m_RenderGeometry;
		m_RenderGeometry = nullptr;
	}

	bool ClothSystem::PinParticle(int row, int column, const Vector3& worldPosition)
	{
		if (row < 0 || row >= m_Settings.Rows || column < 0 || column >= m_Settings.Columns)
		{
			return false;
		}
		return PinParticle(ParticleIndex(row, column), worldPosition);
	}
	bool ClothSystem::PinParticle(int Index, const Vector3& worldPosition)
	{
		if (!ValidIndex(Index) || !Finite(worldPosition)) return false;
		ClothParticle& particle = m_Particles[Index];
		particle.Pinned = true;
		particle.PinPosition = worldPosition;
		particle.Position = worldPosition;
		particle.PreviousPosition = worldPosition;
		UpdateRenderMesh();
		return true;
	}

	bool ClothSystem::UnpinParticle(int row, int column)
	{
		if (row < 0 || row >= m_Settings.Rows || column < 0 || column >= m_Settings.Columns)
		{
			return false;
		}
		return UnpinParticle(ParticleIndex(row, column));
	}
	bool ClothSystem::UnpinParticle(int Index)
	{
		if (!ValidIndex(Index)) return false;
		m_Particles[Index].Pinned = false;
		return true;
	}
	float ClothSystem::InverseMass(int Index) const { return m_Particles[Index].Pinned ? 0 : m_Particles[Index].InverseMass; }
	bool ClothSystem::SetParticlePosition(int Index, const Vector3& Position, bool ResetVelocity)
	{
		if (!ValidIndex(Index) || !Finite(Position)) return false;
		auto& P = m_Particles[Index]; P.PreviousPosition = ResetVelocity ? Position : P.PreviousPosition + Position - P.Position;
		P.Position = Position; if (P.Pinned) P.PinPosition = Position; UpdateRenderMesh(); return true;
	}
	bool ClothSystem::SetParticleVelocity(int Index, const Vector3& Velocity)
	{
		if (!ValidIndex(Index) || !Finite(Velocity) || InverseMass(Index) <= 0) return false;
		if (m_PreviousDt <= 0) m_PreviousDt = ReferenceTimeStep;
		m_Particles[Index].PreviousPosition = m_Particles[Index].Position - Velocity * m_PreviousDt; return true;
	}
	Vector3 ClothSystem::GetParticleVelocity(int Index) const
	{
		return ValidIndex(Index) && m_PreviousDt > 0 ? (m_Particles[Index].Position - m_Particles[Index].PreviousPosition) / m_PreviousDt : Vector3::Zero();
	}
	void ClothSystem::SetMaterial(const ClothMaterial& Material)
	{
		if (!Material.IsValid()) throw std::invalid_argument("Invalid cloth material"); m_Settings.Material = Material;
	}
	void ClothSystem::SetModel(ClothModel Model)
	{
		if (Model != ClothModel::DistanceLinks && Model != ClothModel::TriangleStrain)
			throw std::invalid_argument("Invalid cloth model");
		if (m_Settings.Model == Model) return;
		m_Settings.Model = Model;
		std::fill(m_StrainMultipliers.begin(), m_StrainMultipliers.end(), Vector3::Zero());
		std::fill(m_BendMultipliers.begin(), m_BendMultipliers.end(), 0.0f);
	}
	void ClothSystem::SetSolverParameters(int Iterations, int Substeps, float Damping)
	{
		if (Iterations < 1 || Substeps < 1 || !std::isfinite(Damping) || Damping < 0 || Damping > 1)
			throw std::invalid_argument("Invalid cloth solver parameters");
		m_Settings.SolverIterations = Iterations;
		m_Settings.Substeps = Substeps;
		m_Settings.Damping = Damping;
	}
	void ClothSystem::SetWind(const ClothWind& Wind)
	{
		if (!Wind.IsValid()) throw std::invalid_argument("Invalid cloth wind"); m_Settings.Wind = Wind;
	}
	void ClothSystem::SetAnimationDrive(const ClothAnimationDrive& Drive)
	{
		if (!Drive.IsValid()) throw std::invalid_argument("Invalid cloth animation drive"); m_Settings.AnimationDrive = Drive;
	}
	void ClothSystem::SetAnimationWeights(const std::vector<float>& Weights)
	{
		if (Weights.size() != m_Particles.size()) throw std::invalid_argument("Wrong cloth animation weight count");
		for (float Weight : Weights) if (!std::isfinite(Weight) || Weight < 0 || Weight > 1) throw std::invalid_argument("Invalid cloth animation weight");
		m_AnimationWeights = Weights;
	}
	void ClothSystem::SetAnimationPose(const std::vector<Vector3>& WorldPositions, float BlendTime)
	{
		if (WorldPositions.size() != m_Particles.size() || !std::isfinite(BlendTime) || BlendTime < 0) throw std::invalid_argument("Invalid cloth animation sample");
		for (const auto& P : WorldPositions) if (!Finite(P)) throw std::invalid_argument("Non-finite cloth animation sample");
		m_AnimationStart = m_Animation; m_AnimationEnd = WorldPositions; m_AnimationTime = 0; m_AnimationDuration = BlendTime;
		if (BlendTime == 0) { m_Animation = m_AnimationStart = WorldPositions; m_ResetAnimationVelocity = true; }
	}

	bool ClothSystem::AddSphereCollider(RigidBody* body, float radius, const Vector3& worldCenter)
	{
		if (body == nullptr || !std::isfinite(radius) || radius <= 0.0f || !Finite(worldCenter))
		{
			return false;
		}
		if (!m_Collision->AddCapsule(body, radius, worldCenter, worldCenter, true)) return false;
		m_SphereColliders.erase(std::remove_if(m_SphereColliders.begin(), m_SphereColliders.end(),
			[body](const SphereCollider& Collider) { return Collider.Body == body; }), m_SphereColliders.end());
		SphereCollider collider;
		collider.Body = body;
		collider.LocalCenter = body->WorldToBodyLocal(worldCenter);
		collider.Radius = radius;
		m_SphereColliders.push_back(collider);
		return true;
	}

	bool ClothSystem::RemoveSphereCollider(RigidBody* body)
	{
		m_Collision->RemoveColliders(body, true);
		const size_t oldSize = m_SphereColliders.size();
		m_SphereColliders.erase(std::remove_if(m_SphereColliders.begin(), m_SphereColliders.end(),
			[body](const SphereCollider& collider) { return collider.Body == body; }), m_SphereColliders.end());
		return oldSize != m_SphereColliders.size();
	}
	bool ClothSystem::AddCollider(Geometry* Shape) { return m_Collision->AddCollider(Shape); }
	bool ClothSystem::AddCapsuleCollider(RigidBody* Body, float Radius, const Vector3& A, const Vector3& B) { return m_Collision->AddCapsule(Body, Radius, A, B); }
	bool ClothSystem::RemoveCollider(const Geometry* Shape) { return m_Collision->RemoveCollider(Shape); }
	bool ClothSystem::RemoveColliders(RigidBody* Body)
	{
		const bool Removed = m_Collision->RemoveColliders(Body); RemoveSphereCollider(Body); return Removed;
	}
	void ClothSystem::SetCollisionSettings(const ClothCollisionSettings& Settings)
	{
		if (!Settings.IsValid()) throw std::invalid_argument("Invalid cloth collision settings"); m_Settings.Collision = Settings;
	}
	size_t ClothSystem::GetColliderCount() const { return m_Collision->GetColliderCount(); }
	const ClothCollisionStats& ClothSystem::GetCollisionStats() const { return m_Collision->GetStats(); }

	void ClothSystem::Simulate(float dt)
	{
		SimulateBatch({ this }, dt);
	}
	void ClothSystem::SimulateGroup(const std::vector<ClothSystem*>& Cloths, float Dt)
	{
		std::vector<ClothSystem*> Shared;
		for (size_t I = 0; I < Cloths.size(); ++I)
		{
			if (!Cloths[I] || std::find(Cloths.begin(), Cloths.begin() + I, Cloths[I]) != Cloths.begin() + I)
				throw std::invalid_argument("Cloth simulation groups require distinct non-null objects");
			if (Cloths[I]->m_Settings.Collision.InterCollision) Shared.push_back(Cloths[I]);
		}
		for (auto* Cloth : Cloths) if (!Cloth->m_Settings.Collision.InterCollision) Cloth->Simulate(Dt);
		if (!Shared.empty()) SimulateBatch(Shared, Dt);
	}
	void ClothSystem::SimulateBatch(const std::vector<ClothSystem*>& Cloths, float Dt)
	{
		/* Pair contacts couple two evolving surfaces, so fully stepping one cloth
		   before the other would mix time levels. Predict every cloth, alternate
		   its material/rigid rows with shared contact rows, then commit velocities.
		   Opted-in cloths use the largest requested substep count as a common clock;
		   their individual counts become minimum quality requests, not different
		   physical times. Independent cloths retain their own clock. Material rows
		   retain each cloth's iteration budget; contact iterations use the maximum
		   so lower-budget cloths still react to the other surface's later corrections.
		   This phase boundary also permits parallel independent batches in the future. */
		if (!std::isfinite(Dt) || Dt <= 0 || Cloths.empty()) return;
		int Substeps = 1, Iterations = 1; std::vector<ClothCollision*> Collisions;
		for (auto* Cloth : Cloths)
		{
			Substeps = std::max(Substeps, Cloth->m_Settings.Substeps); Iterations = std::max(Iterations, Cloth->m_Settings.SolverIterations);
			Collisions.push_back(Cloth->m_Collision.get());
		}
		// An unresolved sweep rejects the whole coupled cloth frame. Keep physical
		// and animation state so a caller can increase its budget and retry the same
		// motion. This transaction covers the cloth batch, not the rigid world.
		struct FrameState
		{
			std::vector<ClothParticle> Particles;
			std::vector<Vector3> Animation, AnimationVelocity;
			float PreviousDt, AnimationTime;
			bool ResetAnimationVelocity;
		};
		bool Continuous = false; for (auto* Cloth : Cloths) Continuous = Continuous || Cloth->m_Settings.Collision.Continuous;
		auto Capture = [&]()
		{
			std::vector<FrameState> State;
			if (Continuous) for (auto* Cloth : Cloths) State.push_back({ Cloth->m_Particles, Cloth->m_Animation, Cloth->m_AnimationVelocity,
				Cloth->m_PreviousDt, Cloth->m_AnimationTime, Cloth->m_ResetAnimationVelocity });
			return State;
		};
		auto Restore = [&](std::vector<FrameState>& Saved)
		{
			for (size_t I = 0; I < Saved.size(); ++I)
			{
				auto& State = Saved[I]; auto* Cloth = Cloths[I];
				Cloth->m_Particles.swap(State.Particles); Cloth->m_Animation.swap(State.Animation); Cloth->m_AnimationVelocity.swap(State.AnimationVelocity);
				Cloth->m_PreviousDt = State.PreviousDt; Cloth->m_AnimationTime = State.AnimationTime; Cloth->m_ResetAnimationVelocity = State.ResetAnimationVelocity;
			}
		};
		auto Saved = Capture();
		for (auto* Collision : Collisions) Collision->BeginFrame();
		try
		{
			for (auto* Collision : Collisions) Substeps = std::max(Substeps, Collision->RequiredSubsteps());
			int Refinements = 0, MaximumRefinements = 0, MaximumDepth = 0;
			for (auto* Cloth : Cloths) if (Cloth->m_Settings.Collision.Continuous)
			{
				MaximumRefinements = std::max(MaximumRefinements, Cloth->m_Settings.Collision.MaximumContactRefinements);
				MaximumDepth = std::max(MaximumDepth, Cloth->m_Settings.Collision.MaximumContactRefinementDepth);
			}
			struct Interval { double Start, End; int Depth = 0; };
			std::vector<Interval> Pending;
			for (int Step = Substeps; Step > 0; --Step) Pending.push_back({ (double)(Step - 1) / Substeps, (double)Step / Substeps });
			while (!Pending.empty())
			{
				const auto Range = Pending.back(); Pending.pop_back();
				const float StepTime = (float)(Dt * (Range.End - Range.Start));
				if (!(StepTime * StepTime > 0)) throw std::invalid_argument("Cloth substep interval underflow");
				auto Before = Capture();
				for (auto* Collision : Collisions) Collision->BeginSubstep((float)Range.Start, (float)Range.End);
				bool Resolved = false, UnresolvedSweep = false;
				try
				{
					for (auto* Cloth : Cloths) Cloth->PrepareStep(StepTime);
					for (int Iteration = 0; Iteration < Iterations; ++Iteration)
					{
						for (auto* Cloth : Cloths) Cloth->SolveIteration(StepTime, Iteration);
						ClothCollision::SolveInteractions(Collisions, StepTime);
					}
					Resolved = ClothCollision::FinishContinuous(Collisions, StepTime);
				}
				catch (const ClothCollision::SweepFailure&) { UnresolvedSweep = true; }
				if (!Resolved)
				{
					// Impact planes are local approximations. On a failed
					// coupled projection, retry only this physical interval at half
					// its duration. All participants and animation samples rewind
					// together; no rejected correction becomes a physical velocity.
					const double Middle = (Range.Start + Range.End) / 2;
					if (Refinements >= MaximumRefinements || Range.Depth >= MaximumDepth || (float)Middle <= (float)Range.Start || (float)Middle >= (float)Range.End)
					{
						if (UnresolvedSweep) throw ClothCollision::SweepFailure();
						throw std::runtime_error("Cloth continuous contacts exhausted their substep refinement budget");
					}
					Restore(Before); ++Refinements; ClothCollision::RecordRefinement(Collisions);
					Pending.push_back({ Middle, Range.End, Range.Depth + 1 }); Pending.push_back({ Range.Start, Middle, Range.Depth + 1 }); continue;
				}
				for (auto* Cloth : Cloths) Cloth->m_PreviousDt = StepTime;
			}
		}
		catch (...)
		{
			// Rows borrow particles from other cloths. Never retain those borrows
			// after a failed step, when the caller may remove either participant.
			for (auto* Collision : Collisions) Collision->EndFrame(false);
			Restore(Saved);
			throw;
		}
		for (auto* Collision : Collisions) Collision->EndFrame();
		for (auto* Cloth : Cloths) Cloth->UpdateRenderMesh();
	}
	void ClothSystem::PrepareStep(float Dt)
	{
		/* The distance route uses links cooked by ClothTopology. The material route
		   performs nonlinear Gauss-Seidel XPBD with multipliers reset once per
		   substep, never per iteration. Compliance is divided by dt^2 exactly once.
		   Particle masses are lumped from surface area unless explicit mesh inverse
		   masses override that distribution. Both models use the same particle data.
		   PreviousPosition encodes velocity over m_PreviousDt, so changing world
		   substeps does not silently multiply speed. Damping is retention at 60 Hz.
		   Animation springs and wind are external forces; internal material rows
		   conserve linear/angular momentum through their analytic gradients. */
		SampleAnimation(Dt); if (m_Settings.Material.TethersEnabled) UpdateTethers();
		std::fill(m_StrainMultipliers.begin(), m_StrainMultipliers.end(), Vector3::Zero());
		std::fill(m_BendMultipliers.begin(), m_BendMultipliers.end(), 0);
		std::fill(m_AnimationMultipliers.begin(), m_AnimationMultipliers.end(), Vector3::Zero());
		for (auto& Tether : m_Tethers) Tether.Lambda = 0;
		const float Retention = std::pow(m_Settings.Damping, Dt / ReferenceTimeStep); m_StepVelocities.resize(m_Particles.size());
		for (size_t I = 0; I < m_Particles.size(); ++I)
		{
			const auto& P = m_Particles[I]; Vector3 Velocity = GetParticleVelocity((int)I) * Retention;
			if (P.Pinned) Velocity = (P.PinPosition - P.Position) / Dt;
			else if (P.InverseMass <= 0) Velocity = Vector3::Zero();
			else
			{
				Velocity += m_Settings.Gravity * Dt;
				if (m_Settings.AnimationDrive.Enabled && m_AnimationWeights[I] > 0)
					Velocity = m_AnimationVelocity[I] + (Velocity - m_AnimationVelocity[I])
						* std::exp(-m_Settings.AnimationDrive.Damping * m_AnimationWeights[I] * Dt);
			}
			m_StepVelocities[I] = Velocity;
		}
		ApplyWind(Dt);
		for (size_t I = 0; I < m_Particles.size(); ++I)
		{
			auto& P = m_Particles[I]; P.PreviousPosition = P.Position;
			if (P.Pinned) P.Position = P.PinPosition;
			else if (P.InverseMass > 0) P.Position += m_StepVelocities[I] * Dt;
		}
	}
	void ClothSystem::SolveIteration(float Dt, int Iteration)
	{
		if (Iteration < m_Settings.SolverIterations)
		{
			if (m_Settings.Model == ClothModel::DistanceLinks) SolveDistanceConstraints(); else SolveMaterial(Dt);
			SolveTethers(Dt); SolveAnimation(Dt);
		}
		SolvePins();
		if (m_Settings.Collision.SurfaceContacts) m_Collision->Solve(Dt); else SolveSphereCollisions();
	}
	void ClothSystem::SolveMaterial(float Dt)
	{
		const auto& Material = m_Settings.Material; const auto& Triangles = m_Topology->GetTriangles();
		for (size_t I = 0; I < Triangles.size(); ++I)
		{
			const auto& Triangle = Triangles[I];
			for (int Channel = 0; Channel < 3; ++Channel)
			{
				if (Channel < 2 ? !Material.StretchEnabled : !Material.ShearEnabled) continue;
				Vector3 Positions[3], Gradients[3]; for (int J = 0; J < 3; ++J) Positions[J] = m_Particles[Triangle.Particles[J]].Position;
				float Value; if (!ClothTopology::EvaluateStrain(Triangle, Positions, Channel, Value, Gradients)) continue;
				const float Compliance = Channel == 0 ? Material.StretchUCompliance : Channel == 1 ? Material.StretchVCompliance : Material.ShearCompliance;
				const float Alpha = Compliance / (Triangle.Area * Dt * Dt); float Denominator = 0;
				for (int J = 0; J < 3; ++J) Denominator += InverseMass(Triangle.Particles[J]) * Gradients[J].SquareLength();
				if (!(Denominator > 0)) continue;
				float& Lambda = m_StrainMultipliers[I][Channel]; const float Delta = (-Value - Alpha * Lambda) / (Denominator + Alpha); Lambda += Delta;
				for (int J = 0; J < 3; ++J) m_Particles[Triangle.Particles[J]].Position += Gradients[J] * (InverseMass(Triangle.Particles[J]) * Delta);
			}
		}
		if (!Material.BendingEnabled) return;
		const auto& Bends = m_Topology->GetBends();
		for (size_t I = 0; I < Bends.size(); ++I)
		{
			const auto& Bend = Bends[I]; Vector3 Positions[4], Gradients[4];
			for (int J = 0; J < 4; ++J) Positions[J] = m_Particles[Bend.Particles[J]].Position;
			float Angle; if (!ClothTopology::EvaluateBend(Bend, Positions, Angle, Gradients)) continue;
			const float Value = std::remainder(Angle - Bend.RestAngle, 2 * PI), Alpha = Material.BendingCompliance / (Bend.Weight * Dt * Dt);
			float Denominator = 0; for (int J = 0; J < 4; ++J) Denominator += InverseMass(Bend.Particles[J]) * Gradients[J].SquareLength();
			if (!(Denominator > 0)) continue;
			float& Lambda = m_BendMultipliers[I];
			const float Residual = std::max(-Material.MaxBendingStep, std::min(Material.MaxBendingStep, -Value - Alpha * Lambda));
			const float Delta = Residual / (Denominator + Alpha); Lambda += Delta;
			for (int J = 0; J < 4; ++J) m_Particles[Bend.Particles[J]].Position += Gradients[J] * (InverseMass(Bend.Particles[J]) * Delta);
		}
	}
	void ClothSystem::UpdateTethers()
	{
		bool Changed = m_TetherAnchors.size() != m_Particles.size(); m_TetherAnchors.resize(m_Particles.size());
		for (size_t I = 0; I < m_Particles.size(); ++I)
		{
			const unsigned char Anchor = InverseMass((int)I) <= 0; Changed |= Anchor != m_TetherAnchors[I]; m_TetherAnchors[I] = Anchor;
		}
		if (!Changed) return;
		/* Multi-source Dijkstra measures distance along REST mesh edges. Euclidean
		   distance across a folded garment would make a too-short tether. Each free
		   vertex keeps its nearest fixed anchor; disconnected unanchored panels have
		   no tether. Moving an anchor does not recook lengths, but pin/unpin does. */
		std::vector<std::vector<std::pair<int, float>>> Neighbors(m_Particles.size());
		for (const auto& E : m_Topology->GetEdges()) { Neighbors[E.A].push_back({ E.B, E.Length }); Neighbors[E.B].push_back({ E.A, E.Length }); }
		std::vector<float> Distance(m_Particles.size(), std::numeric_limits<float>::infinity()); std::vector<int> Anchor(m_Particles.size(), -1);
		using Entry = std::pair<float, int>; std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> Queue;
		for (int I = 0; I < GetParticleCount(); ++I) if (m_TetherAnchors[I]) { Distance[I] = 0; Anchor[I] = I; Queue.push({ 0, I }); }
		while (!Queue.empty())
		{
			const auto Entry = Queue.top(); Queue.pop(); const int Vertex = Entry.second; if (Entry.first != Distance[Vertex]) continue;
			for (const auto& Neighbor : Neighbors[Vertex])
			{
				const float Candidate = Entry.first + Neighbor.second; const int Next = Neighbor.first;
				if (Candidate < Distance[Next]) { Distance[Next] = Candidate; Anchor[Next] = Anchor[Vertex]; Queue.push({ Candidate, Next }); }
			}
		}
		m_Tethers.clear();
		for (int I = 0; I < GetParticleCount(); ++I) if (!m_TetherAnchors[I] && Anchor[I] >= 0) m_Tethers.push_back({ I, Anchor[I], Distance[I] });
	}
	void ClothSystem::SolveTethers(float Dt)
	{
		const auto& M = m_Settings.Material; if (!M.TethersEnabled) return; const float Alpha = M.TetherCompliance / (Dt * Dt);
		for (auto& T : m_Tethers)
		{
			const Vector3 Delta = m_Particles[T.Particle].Position - m_Particles[T.Anchor].Position; const float Length = Delta.Length(), W = InverseMass(T.Particle);
			if (!(Length > 0) || !(W > 0)) continue;
			const float Value = Length - M.TetherScale * T.Length, Previous = T.Lambda;
			T.Lambda = std::min(0.0f, Previous + (-Value - Alpha * Previous) / (W + Alpha));
			m_Particles[T.Particle].Position += Delta * (W * (T.Lambda - Previous) / Length);
		}
	}
	void ClothSystem::SampleAnimation(float Dt)
	{
		m_AnimationTime = std::min(m_AnimationTime + Dt, m_AnimationDuration);
		const float Fraction = m_AnimationDuration > 0 ? m_AnimationTime / m_AnimationDuration : 1;
		for (size_t I = 0; I < m_Particles.size(); ++I)
		{
			const Vector3 Next = m_AnimationStart[I] + (m_AnimationEnd[I] - m_AnimationStart[I]) * Fraction;
			m_AnimationVelocity[I] = m_ResetAnimationVelocity ? Vector3::Zero() : (Next - m_Animation[I]) / Dt; m_Animation[I] = Next;
			if (m_Settings.AnimationDrive.Enabled && m_Settings.AnimationDrive.MovePins && m_AnimationWeights[I] > 0 && m_Particles[I].Pinned)
				m_Particles[I].PinPosition = Next;
		}
		m_ResetAnimationVelocity = false;
	}
	void ClothSystem::SolveAnimation(float Dt)
	{
		const auto& Drive = m_Settings.AnimationDrive; if (!Drive.Enabled) return;
		for (size_t I = 0; I < m_Particles.size(); ++I)
		{
			const float Weight = m_AnimationWeights[I], W = InverseMass((int)I); if (!(Weight > 0) || !(W > 0)) continue;
			const float Alpha = Drive.Compliance / (Weight * Dt * Dt);
			const Vector3 Value = m_Particles[I].Position - m_Animation[I];
			const Vector3 Delta = (-Value - m_AnimationMultipliers[I] * Alpha) / (W + Alpha);
			m_AnimationMultipliers[I] += Delta; m_Particles[I].Position += Delta * W;
		}
	}
	void ClothSystem::ApplyWind(float Dt)
	{
		const auto& Wind = m_Settings.Wind; if (Wind.Density == 0 || (Wind.NormalDrag == 0 && Wind.TangentialDrag == 0)) return;
		/* Quadratic normal pressure and tangential drag use current triangle area
		   and relative air velocity, equally distributed to its three vertices.
		   Freeze each drag coefficient for one implicit velocity solve. This limits
		   drag to approaching the wind velocity instead of explicitly overshooting
		   it on light or coarse triangles. With stationary air, drag removes energy;
		   reversing triangle winding leaves the pressure force unchanged. */
		for (const auto& Triangle : m_Topology->GetTriangles())
		{
			const auto& T = Triangle.Particles; const Vector3 Cross = (m_Particles[T.b].Position - m_Particles[T.a].Position).Cross(m_Particles[T.c].Position - m_Particles[T.a].Position);
			const float TwiceArea = Cross.Length(); if (!(TwiceArea > 0)) continue;
			const Vector3 Normal = Cross / TwiceArea;
			const Vector3 Relative = (m_StepVelocities[T.a] + m_StepVelocities[T.b] + m_StepVelocities[T.c]) / 3 - Wind.Velocity;
			const float NormalSpeed = Relative.Dot(Normal); const Vector3 Tangent = Relative - Normal * NormalSpeed;
			const float NormalRate = Wind.Density * TwiceArea * Wind.NormalDrag * std::abs(NormalSpeed) / 4;
			const float TangentRate = Wind.Density * TwiceArea * Wind.TangentialDrag * Tangent.Length() / 4;
			const float EffectiveInverseMass = (InverseMass(T.a) + InverseMass(T.b) + InverseMass(T.c)) / (3 * 3);
			const Vector3 Impulse = -Normal * (NormalSpeed * NormalRate * Dt / (1 + NormalRate * Dt * EffectiveInverseMass))
				- Tangent * (TangentRate * Dt / (1 + TangentRate * Dt * EffectiveInverseMass));
			for (int J = 0; J < 3; ++J) m_StepVelocities[T[J]] += Impulse * (InverseMass(T[J]) / 3);
		}
	}

	void ClothSystem::SolveDistanceConstraints()
	{
		for (const DistanceLink& link : m_Links)
		{
			ClothParticle& particleA = m_Particles[(size_t)link.ParticleA];
			ClothParticle& particleB = m_Particles[(size_t)link.ParticleB];
			const Vector3 delta = particleB.Position - particleA.Position;
			const float distance = delta.Length();
			if (distance <= 1e-7f) continue;
			const float weightA = particleA.Pinned ? 0.0f : particleA.InverseMass;
			const float weightB = particleB.Pinned ? 0.0f : particleB.InverseMass;
			const float weightSum = weightA + weightB;
			if (weightSum <= 1e-7f) continue;
			const Vector3 correction = delta * ((distance - link.RestLength) / distance);
			particleA.Position += correction * (weightA / weightSum);
			particleB.Position -= correction * (weightB / weightSum);
		}
	}

	void ClothSystem::SolvePins()
	{
		for (ClothParticle& particle : m_Particles)
		{
			if (particle.Pinned) particle.Position = particle.PinPosition;
		}
	}

	void ClothSystem::SolveSphereCollisions()
	{
		for (const SphereCollider& collider : m_SphereColliders)
		{
			if (collider.Body == nullptr) continue;
			const Vector3 center = collider.Body->BodyLocalToWorld(collider.LocalCenter);
			const float minDistance = collider.Radius + m_Settings.Thickness;
			for (ClothParticle& particle : m_Particles)
			{
				if (particle.Pinned || particle.InverseMass <= 0.0f) continue;
				Vector3 delta = particle.Position - center;
				float distance = delta.Length();
				if (distance >= minDistance) continue;
				if (distance <= 1e-7f)
				{
					delta = Vector3::UnitZ();
					distance = 1.0f;
				}
				particle.Position = center + delta * (minDistance / distance);
			}
		}
	}

	void ClothSystem::BuildRenderMesh()
	{
		m_RenderGeometry = GeometryFactory::CreateTriangleMesh();
		m_RenderGeometry->SetQueryEnabled(false);
		m_RenderGeometry->SetSimulationEnabled(false);
		TriangleMesh* mesh = m_RenderGeometry->GetShapeObj<TriangleMesh>();
		if (m_Particles.size() <= (size_t)std::numeric_limits<uint16_t>::max() + 1) mesh->Flags |= INDICES_16_BIT;
		for (const ClothParticle& particle : m_Particles) mesh->AddVertex(particle.Position);
		for (const auto& Triangle : m_Topology->GetTriangles()) mesh->AddTriangle(Triangle.Particles.a, Triangle.Particles.b, Triangle.Particles.c);
		mesh->Compact();
		m_RenderGeometry->SetShapeBounds(mesh->BoundingVolume);
	}

	void ClothSystem::UpdateRenderMesh()
	{
		TriangleMesh* mesh = m_RenderGeometry ? m_RenderGeometry->GetShapeObj<TriangleMesh>() : nullptr;
		if (mesh == nullptr || mesh->NumVertices != m_Particles.size()) return;
		mesh->BoundingVolume.SetEmpty();
		for (size_t i = 0; i < m_Particles.size(); ++i)
		{
			mesh->Vertices[i] = m_Particles[i].Position;
			mesh->BoundingVolume.Encapsulate(m_Particles[i].Position);
		}
		mesh->mNormals.clear();
		m_RenderGeometry->SetShapeBounds(mesh->BoundingVolume);
	}
}

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace Riemann
{
	/* The mesh is an oriented, edge-manifold triangle surface in world space.
	   Boundaries, holes, disconnected panels and non-planar rest shapes are valid.
	   Rest data is immutable; simulation positions and XPBD multipliers belong to
	   ClothSystem. This same edge/triangle adjacency can serve collision exclusions
	   without coupling the material model to a particular collision algorithm.

	   Optional UVs choose the warp direction, not the rest length: the derivative
	   dX/du defines an orthonormal material frame in the 3-D rest triangle. Without
	   UVs its first edge defines warp. F = sum(x_i outer grad_i) maps that frame to
	   the deformed triangle. The three constraints are Green strain components:
	   (Fu.Fu-1)/2, (Fv.Fv-1)/2 and Fu.Fv. Independent compliances allow orthotropic
	   stretch and shear. Membrane energy is A*C^2/(2*compliance), so XPBD uses
	   compliance/A. Compliance has units m/N; zero means an inextensible channel.
	   This uncoupled strain energy has no Poisson parameter or plasticity.

	   Bending uses the signed angle about a shared edge, retaining curved rest
	   shapes and distinguishing positive and negative folds. Its discrete-shell
	   weight is 3*edgeLength^2/(areaLeft+areaRight); bending compliance has units
	   1/(N*m). Angle gradients follow from rotating each face normal about the
	   edge. Translation invariance supplies the remaining endpoint gradient.
	   A collapsed current triangle has no well-defined normal and is skipped;
	   degenerate rest triangles are rejected during topology construction.

	   References: Muller et al., "Strain Based Dynamics" (2014); Macklin et al.,
	   "XPBD: Position-Based Simulation of Compliant Constrained Dynamics" (2016);
	   Grinspun et al., "Discrete Shells" (2003). Stretch and bending use separate
	   material modes.
	   Skinning/garment import is intentionally outside this runtime mesh contract;
	   a future asset adapter must supply welded simulation topology and UV frames. */
	namespace
	{
		constexpr float RelativeDegeneracy = 1e-6f;
		constexpr float Pi = 3.14159265358979323846f;
		bool Nonnegative(float X) { return std::isfinite(X) && X >= 0; }
		struct EdgeFaces { int A, B, Opposite, Face; bool Paired = false; };
	}
	bool ClothMaterial::IsValid() const
	{
		return Nonnegative(StretchUCompliance) && Nonnegative(StretchVCompliance) && Nonnegative(ShearCompliance)
			&& Nonnegative(BendingCompliance) && Nonnegative(TetherCompliance) && std::isfinite(TetherScale) && TetherScale >= 1
			&& std::isfinite(MaxBendingStep) && MaxBendingStep > 0 && MaxBendingStep <= Pi;
	}
	bool ClothWind::IsValid() const { return Finite(Velocity) && Nonnegative(Density) && Nonnegative(NormalDrag) && Nonnegative(TangentialDrag); }
	bool ClothAnimationDrive::IsValid() const { return Nonnegative(Compliance) && Nonnegative(Damping); }
	ClothTopology::ClothTopology(const ClothMesh& Mesh)
	{
		const size_t Count = Mesh.Positions.size();
		if (Count < 3 || Count > (size_t)std::numeric_limits<int>::max() || Mesh.Triangles.empty()
			|| (!Mesh.MaterialCoordinates.empty() && Mesh.MaterialCoordinates.size() != Count)
			|| (!Mesh.InverseMasses.empty() && Mesh.InverseMasses.size() != Count)) throw std::invalid_argument("Invalid cloth mesh sizes");
		for (const auto& P : Mesh.Positions) if (!Finite(P)) throw std::invalid_argument("Non-finite cloth vertex");
		for (const auto& UV : Mesh.MaterialCoordinates) if (!std::isfinite(UV.x) || !std::isfinite(UV.y)) throw std::invalid_argument("Non-finite cloth material coordinate");
		for (float W : Mesh.InverseMasses) if (!Nonnegative(W)) throw std::invalid_argument("Invalid cloth inverse mass");
		std::map<std::pair<int, int>, EdgeFaces> Edges;
		std::set<std::array<int, 3>> Faces; m_VertexAreas.resize(Count, 0);
		for (const auto& Indices : Mesh.Triangles)
		{
			std::array<int, 3> Key = { Indices.a, Indices.b, Indices.c }; std::sort(Key.begin(), Key.end());
			if (Key[0] < 0 || (size_t)Key[2] >= Count || Key[0] == Key[1] || Key[1] == Key[2] || !Faces.insert(Key).second)
				throw std::invalid_argument("Invalid or duplicate cloth triangle");
			const Vector3 E1 = Mesh.Positions[Indices.b] - Mesh.Positions[Indices.a], E2 = Mesh.Positions[Indices.c] - Mesh.Positions[Indices.a];
			const Vector3 Cross = E1.Cross(E2); const float TwiceArea = Cross.Length();
			if (!std::isfinite(TwiceArea) || TwiceArea <= RelativeDegeneracy * std::max(E1.SquareLength(), E2.SquareLength()))
				throw std::invalid_argument("Degenerate cloth rest triangle");
			Vector3 U = E1.Unit(); const Vector3 Normal = Cross / TwiceArea;
			if (!Mesh.MaterialCoordinates.empty())
			{
				const Vector2 D1 = Mesh.MaterialCoordinates[Indices.b] - Mesh.MaterialCoordinates[Indices.a];
				const Vector2 D2 = Mesh.MaterialCoordinates[Indices.c] - Mesh.MaterialCoordinates[Indices.a];
				const float Determinant = D1.Cross(D2);
				if (!std::isfinite(Determinant) || std::abs(Determinant) <= RelativeDegeneracy * std::max(D1.SquareLength(), D2.SquareLength()))
					throw std::invalid_argument("Degenerate cloth material coordinates");
				U = ((E1 * D2.y - E2 * D1.y) / Determinant).Unit();
			}
			const Vector3 V = Normal.Cross(U);
			const float X1 = E1.Dot(U), Y1 = E1.Dot(V), X2 = E2.Dot(U), Y2 = E2.Dot(V), Determinant = X1 * Y2 - X2 * Y1;
			ClothTriangle Triangle; Triangle.Particles = Indices; Triangle.Area = TwiceArea / 2;
			Triangle.GradientU = Vector3(Y1 - Y2, Y2, -Y1) / Determinant;
			Triangle.GradientV = Vector3(X2 - X1, -X2, X1) / Determinant;
			if (!Finite(Triangle.GradientU) || !Finite(Triangle.GradientV)) throw std::invalid_argument("Unrepresentable cloth rest frame");
			const int Face = (int)m_Triangles.size(); m_Triangles.push_back(Triangle);
			for (int J = 0; J < 3; ++J)
			{
				const int A = Indices[J], B = Indices[(J + 1) % 3], Opposite = Indices[(J + 2) % 3];
				m_VertexAreas[A] += Triangle.Area / 3;
				const auto EdgeKey = std::make_pair(std::min(A, B), std::max(A, B)); auto It = Edges.find(EdgeKey);
				if (It == Edges.end()) { Edges.emplace(EdgeKey, EdgeFaces{ A, B, Opposite, Face }); continue; }
				auto& First = It->second;
				if (First.Paired || First.A != B) throw std::invalid_argument("Non-manifold or inconsistently wound cloth edge");
				First.Paired = true;
				ClothBend Bend; Bend.Particles[0] = First.A; Bend.Particles[1] = First.B; Bend.Particles[2] = First.Opposite; Bend.Particles[3] = Opposite;
				Bend.RestAreas = Vector2(m_Triangles[First.Face].Area, Triangle.Area);
				Bend.Weight = 3 * (Mesh.Positions[A] - Mesh.Positions[B]).SquareLength() / (Bend.RestAreas.x + Bend.RestAreas.y);
				Vector3 Positions[4], Gradients[4]; for (int K = 0; K < 4; ++K) Positions[K] = Mesh.Positions[Bend.Particles[K]];
				if (!EvaluateBend(Bend, Positions, Bend.RestAngle, Gradients)) throw std::invalid_argument("Degenerate cloth rest hinge");
				m_Bends.push_back(Bend);
			}
		}
		for (const auto& Item : Edges) m_Edges.push_back({ Item.first.first, Item.first.second, (Mesh.Positions[Item.first.first] - Mesh.Positions[Item.first.second]).Length() });
		std::set<std::pair<int, int>> LinkPairs;
		auto AddDistanceLink = [&](int A, int B)
		{
			if (A < 0 || B < 0 || (size_t)A >= Count || (size_t)B >= Count || A == B)
				throw std::invalid_argument("Invalid cloth distance link");
			const auto Key = std::minmax(A, B);
			if (!LinkPairs.insert(Key).second) return;
			const float Length = (Mesh.Positions[B] - Mesh.Positions[A]).Length();
			if (!std::isfinite(Length) || Length <= 0) throw std::invalid_argument("Invalid cloth distance link length");
			m_DistanceLinks.push_back({ A, B, Length });
		};
		if (!Mesh.DistanceLinks.empty()) for (const auto& Link : Mesh.DistanceLinks) AddDistanceLink(Link.first, Link.second);
		else
		{
			for (const auto& Edge : m_Edges) AddDistanceLink(Edge.A, Edge.B);
			for (const auto& Bend : m_Bends) AddDistanceLink(Bend.Particles[2], Bend.Particles[3]);
		}
		for (float Area : m_VertexAreas) if (!std::isfinite(Area) || Area <= 0) throw std::invalid_argument("Unused or unrepresentable cloth vertex");
	}
	bool ClothTopology::EvaluateStrain(const ClothTriangle& Triangle, const Vector3 Positions[3], int Channel, float& Value, Vector3 Gradients[3])
	{
		if (Channel < 0 || Channel > 2) return false;
		// Subtract a reference point before differentiation to avoid cancellation
		// when a small cloth is translated far from the world origin.
		const Vector3 E1 = Positions[1] - Positions[0], E2 = Positions[2] - Positions[0];
		const Vector3 U = E1 * Triangle.GradientU.y + E2 * Triangle.GradientU.z;
		const Vector3 V = E1 * Triangle.GradientV.y + E2 * Triangle.GradientV.z;
		Value = Channel == 0 ? (U.SquareLength() - 1) / 2 : Channel == 1 ? (V.SquareLength() - 1) / 2 : U.Dot(V);
		for (int I = 0; I < 3; ++I) Gradients[I] = Channel == 0 ? U * Triangle.GradientU[I]
			: Channel == 1 ? V * Triangle.GradientV[I] : V * Triangle.GradientU[I] + U * Triangle.GradientV[I];
		return std::isfinite(Value);
	}
	bool ClothTopology::EvaluateBend(const ClothBend& Bend, const Vector3 Positions[4], float& Angle, Vector3 Gradients[4])
	{
		const Vector3 Edge = Positions[1] - Positions[0], Left = Positions[2] - Positions[0], Right = Positions[3] - Positions[0];
		const Vector3 CrossL = Edge.Cross(Left), CrossR = Right.Cross(Edge);
		const float Length = Edge.Length(), AreaL = CrossL.Length(), AreaR = CrossR.Length();
		// Each face has its own scale: a small valid triangle next to a much larger
		// one must not be classified as collapsed by the neighbor's area.
		if (!(Length > 0) || AreaL <= 2 * RelativeDegeneracy * Bend.RestAreas.x || AreaR <= 2 * RelativeDegeneracy * Bend.RestAreas.y) return false;
		const Vector3 N1 = CrossL / AreaL, N2 = CrossR / AreaR;
		Angle = std::atan2((Edge / Length).Dot(N1.Cross(N2)), N1.Dot(N2));
		Gradients[2] = N1 * (-Length / AreaL); Gradients[3] = N2 * (-Length / AreaR);
		Gradients[1] = -(Gradients[2] * Left.Dot(Edge) + Gradients[3] * Right.Dot(Edge)) / Edge.SquareLength();
		Gradients[0] = -Gradients[1] - Gradients[2] - Gradients[3];
		return std::isfinite(Angle) && Finite(Gradients[0]) && Finite(Gradients[1]) && Finite(Gradients[2]) && Finite(Gradients[3]);
	}
}

#include "Cloth.h"
#include "../Collision/SurfaceCollision.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>

namespace Riemann
{
	/* Cloth surface contacts and swept impact constraints
	   -------------------------------------
	   Cloth topology, collider snapshots and contact rows have separate lifetimes.
	   Geometry is copied at registration; only the owning rigid body is borrowed.
	   Removing a body must unregister its colliders before destroying that body.
	   Shape edits require re-registration. Detached geometry is a static snapshot.

	   Rounded shapes use filled-triangle/segment distance, so a small capsule can
	   touch the middle of a large cloth face without touching any cloth vertex.
	   Meshes use vertex/triangle pairs in both directions and edge/edge pairs.
	   Closed convexes additionally recover embedded vertices using signed planes.
	   The spatial index only selects candidates; SurfaceCollision supplies actual
	   witnesses and SurfaceContactConstraint supplies mass-weighted XPBD response.
	   See Bridson, Fedkiw and Anderson, "Robust Treatment of Collisions, Contact
	   and Friction for Cloth Animation" (SIGGRAPH 2002) for feature-based contact.

	   Multipliers persist across iterations of ONE physical substep. Discrete
	   witnesses are refreshed as material constraints bend the contact surface.
	   Friction measures slip since the substep start, including rigid translation
	   and rotation, rather than treating the current witness as a new rest point
	   every iteration. A rounded shape's material surface also moves when its
	   centerline does not: rotate the radius offset to recover that tangential slip.
	   Bodies are prescribed motion here; rigid reaction belongs to the shared
	   two-way deformable coupling stage. Rows are a local geometric linearization.

	   Continuous mode resweeps the previous physical position to every corrected
	   iterate. The impact witness is fixed during ONE projection, then refreshed
	   from that sweep rather than from the potentially crossed endpoint. Freezing
	   a finite feature as an infinite plane for the entire substep can contradict
	   later witnesses when cloth wraps around a curved collider. Retire rows when
	   their corrected paths no longer meet, including departures from swept bounds.
	   Contacts present at time zero use current closest weights, oriented toward
	   their starting normal; this permits overlap recovery without changing sides.
	   Friction references start at impact, preserving tangential free flight.

	   Coupled iterations solve material, normal and Coulomb friction constraints.
	   Final feasibility passes add only mass-weighted normal corrections: repeated
	   friction projections can otherwise undo separation against another surface.
	   Refresh both swept constraints and endpoint distances before accepting a
	   substep. Stagnation alone does not prove separation. ClothSystem bisects and
	   retries rejected physical intervals, including unresolved geometric sweeps,
	   with explicit total/depth budgets and transactional frame rollback on failure.
	   Continuous barriers are hard; contact compliance applies to discrete mode.
	   This remains an iterative impact-plane method, not a rigid impact-zone solve.
	   Statistics count rejected attempts too. ProjectionLimitReached records any
	   exhausted projection attempt; residuals describe the last validated substep.
	   Rigid poses follow shortest-arc slerp between frame samples. Angular substeps
	   bound their chord approximation; a sagitta envelope covers the remaining arc.
	   Full revolutions absent from those pose samples cannot be reconstructed. */
	namespace
	{
		Transform Interpolate(const Transform& A, const Transform& B, float T)
		{
			return Transform(A.pos + (B.pos - A.pos) * T, Quaternion::Slerp(A.quat, B.quat, T));
		}
		std::vector<Index3> Triangles(const ClothSystem& Cloth)
		{
			std::vector<Index3> Result; for (const auto& T : Cloth.GetTopology().GetTriangles()) Result.push_back(T.Particles); return Result;
		}
		struct Feature
		{
			int Count = 1, Indices[3] = { 0, 0, 0 };
			explicit Feature(int Vertex) { Indices[0] = Vertex; }
			explicit Feature(const SurfaceEdge& E) : Count(2) { Indices[0] = E.A; Indices[1] = E.B; }
			explicit Feature(const Index3& T) : Count(3) { for (int I = 0; I < Count; ++I) Indices[I] = T[I]; }
		};
		struct Collider : SurfaceCollider { bool LegacySphere=false; };
		enum class ContactKind { VertexTriangle, TriangleVertex, EdgeEdge, Rounded, Interior };
		using ContactKey = std::tuple<size_t, ContactKind, int, int>;
		struct ContactRow
		{
			Vector3 FixedPoint = Vector3::Zero();
			bool Continuous = false, Seen = true;
			double SurfaceGap = 0;
			float Tolerance = 0;
			std::unique_ptr<SurfaceContactConstraint> Constraint;
			void Solve(float Dt, bool SeparationOnly = false)
			{
				if (SeparationOnly && Continuous) Constraint->ProjectSeparation(); else Constraint->Solve(Dt);
			}
		};
		void RefreshInitialContact(const SurfaceContact& Endpoint, SurfaceContact& Contact)
		{
			if (Contact.Time != 0) return;
			const Vector3 Normal = Contact.Normal;
			Contact = Endpoint;
			if (Contact.Normal.IsZero()) Contact.Normal = Normal;
			else if (Contact.Normal.Dot(Normal) < 0) Contact.Normal = -Contact.Normal;
			Contact.InitialOverlap = true;
		}
		void RefreshRow(ContactRow& Row, const SurfaceContactParticle* A, int CountA, const SurfaceContactParticle* B, int CountB,
			const SurfaceContact& Contact, double Thickness, const SurfaceContactMaterial& Material, double SurfaceGap, float Tolerance)
		{
			Row.Seen = true; Row.SurfaceGap = SurfaceGap; Row.Tolerance = Tolerance;
			auto& Constraint = Row.Constraint;
			if (Constraint) Constraint->Refresh(A, CountA, B, CountB, Contact, Thickness);
			else Constraint.reset(new SurfaceContactConstraint(A, CountA, B, CountB, Contact, Thickness, Material));
		}
	}
	bool ClothCollisionSettings::IsValid() const
	{
		return std::isfinite(ContactTolerance) && ContactTolerance > 0 && Material.IsValid()
			&& SweepIterations > 0 && SweepRefinements >= 0 && ContinuousIterations > 0
			&& MaximumContactRefinements >= 0
			&& MaximumContactRefinementDepth > 0
			&& MaximumAngularSubsteps > 0
			&& std::isfinite(MaximumColliderAngle) && MaximumColliderAngle > 0 && MaximumColliderAngle <= Pi;
	}
	struct ClothCollision::Implementation
	{
		ClothSystem& Cloth;
		SurfaceCollisionMesh Index;
		std::vector<Vector3> Positions, PreviousPositions;
		std::vector<std::unique_ptr<Collider>> Colliders;
		std::map<ContactKey, std::unique_ptr<ContactRow>> Rows;
		using InteractionKey = std::tuple<size_t, SurfacePairType, int, int>;
		std::map<InteractionKey, std::unique_ptr<ContactRow>> InteractionRows;
		Box3 Bounds;
		ClothCollisionStats Stats;
		explicit Implementation(ClothSystem& Owner) : Cloth(Owner), Index(Owner.GetParticleCount(), Triangles(Owner)) {}
		float InverseMass(int I) const { const auto& P = Cloth.GetParticle(I); return P.Pinned ? 0 : P.InverseMass; }
		void UpdateSurface(bool Swept = false)
		{
			Positions.resize(Cloth.GetParticleCount()); PreviousPositions.resize(Positions.size());
			for (int I = 0; I < Cloth.GetParticleCount(); ++I)
			{
				Positions[I] = Cloth.GetParticle(I).Position; PreviousPositions[I] = Cloth.GetParticle(I).PreviousPosition;
			}
			const float Margin = Cloth.GetSettings().Thickness + Cloth.GetSettings().Collision.ContactTolerance;
			Index.Update(Swept ? PreviousPositions : Positions, Positions, Margin);
			Bounds = Box3(Positions.data(), Positions.size());
			if (Swept) for (const auto& P : PreviousPositions) Bounds.Encapsulate(P);
			Bounds.Min -= Vector3(Margin); Bounds.Max += Vector3(Margin);
		}
		bool FindContact(const SurfaceMotion& A, const SurfaceMotion& B, double Thickness,
			const ClothCollisionSettings& Settings, SurfaceContact& Contact)
		{
			SurfaceCollisionSettings Query; Query.Thickness = Thickness; Query.DistanceTolerance = Settings.ContactTolerance;
			Query.RelativeTolerance = 0; Query.MaxIterations = Settings.SweepIterations;
			if (Settings.Continuous)
			{
				if (!SurfaceCollision::SweepRefined(A, B, Query, Settings.SweepRefinements, Contact)) return false;
				if (Contact.Conservative)
				{
					++Stats.UnresolvedSweeps;
					throw SweepFailure();
				}
				return true;
			}
			Contact = SurfaceCollision::Closest(A.End, A.Count, B.End, B.Count);
			if (Contact.Distance > Thickness + Settings.ContactTolerance) return false;
			return true;
		}
		void AddInteraction(Implementation& Other, size_t OtherIndex, const SurfaceCandidate& Pair)
		{
			const bool Self = this == &Other;
			++Stats.Candidates; if (!Self) ++Other.Stats.Candidates;
			const InteractionKey Key(OtherIndex, Pair.Type, Pair.A, Pair.B); const auto Existing = InteractionRows.find(Key);
			const auto& SettingsA = Cloth.GetSettings(); const auto& SettingsB = Other.Cloth.GetSettings();
			auto QuerySettings = SettingsA.Collision.Continuous ? SettingsA.Collision : SettingsB.Collision;
			QuerySettings.ContactTolerance = std::max(SettingsA.Collision.ContactTolerance, SettingsB.Collision.ContactTolerance);
			if (SettingsA.Collision.Continuous && SettingsB.Collision.Continuous)
			{
				QuerySettings.SweepIterations = std::max(SettingsA.Collision.SweepIterations, SettingsB.Collision.SweepIterations);
				QuerySettings.SweepRefinements = std::max(SettingsA.Collision.SweepRefinements, SettingsB.Collision.SweepRefinements);
			}
			const Feature A = Pair.Type == SurfacePairType::VertexTriangle ? Feature(Pair.A)
				: Pair.Type == SurfacePairType::TriangleVertex ? Feature(Index.GetTriangles()[Pair.A]) : Feature(Index.GetEdges()[Pair.A]);
			const Feature B = Pair.Type == SurfacePairType::TriangleVertex ? Feature(Pair.B)
				: Pair.Type == SurfacePairType::VertexTriangle ? Feature(Other.Index.GetTriangles()[Pair.B]) : Feature(Other.Index.GetEdges()[Pair.B]);
			SurfaceMotion MA, MB; MA.Count = A.Count; MB.Count = B.Count;
			Vector3 OldA[3] = { Vector3::Zero(), Vector3::Zero(), Vector3::Zero() };
			Vector3 OldB[3] = { Vector3::Zero(), Vector3::Zero(), Vector3::Zero() };
			for (int I = 0; I < A.Count; ++I) { MA.Start[I] = MA.End[I] = Positions[A.Indices[I]]; OldA[I] = Cloth.GetParticle(A.Indices[I]).PreviousPosition; }
			for (int I = 0; I < B.Count; ++I) { MB.Start[I] = MB.End[I] = Other.Positions[B.Indices[I]]; OldB[I] = Other.Cloth.GetParticle(B.Indices[I]).PreviousPosition; }
			if (QuerySettings.Continuous)
			{
				for (int I = 0; I < A.Count; ++I) MA.Start[I] = OldA[I];
				for (int I = 0; I < B.Count; ++I) MB.Start[I] = OldB[I];
			}
			const double Thickness = (double)SettingsA.Thickness + SettingsB.Thickness; SurfaceContact Contact;
			if (!FindContact(MA, MB, Thickness, QuerySettings, Contact))
			{
				if (QuerySettings.Continuous && Existing != InteractionRows.end()) InteractionRows.erase(Existing);
				return;
			}
			const auto Endpoint = QuerySettings.Continuous ? SurfaceCollision::Closest(MA.End, MA.Count, MB.End, MB.Count) : Contact;
			const double SurfaceGap = Endpoint.Distance - Thickness;
			if (QuerySettings.Continuous) RefreshInitialContact(Endpoint, Contact);
			if (Contact.Normal.IsZero())
			{
				SurfaceCollisionSettings Query; Query.Thickness = Thickness; Query.DistanceTolerance = QuerySettings.ContactTolerance; Query.RelativeTolerance = 0;
				const auto Old = SurfaceCollision::Closest(OldA, A.Count, OldB, B.Count);
				if (!SurfaceCollision::Sweep(MA, MB, Query, Contact, Old.Normal)) return;
			}
			SurfaceContactParticle PA[3], PB[3]; Contact.PointA = Contact.PointB = Vector3::Zero();
			const float Time = QuerySettings.Continuous ? (float)Contact.Time : 0;
			for (int I = 0; I < A.Count; ++I)
			{
				PA[I] = { &Cloth.GetParticle(A.Indices[I]).Position, InverseMass(A.Indices[I]) };
				Contact.PointA += (OldA[I] + (MA.End[I] - OldA[I]) * Time) * Contact.WeightsA[I];
			}
			for (int I = 0; I < B.Count; ++I)
			{
				PB[I] = { &Other.Cloth.GetParticle(B.Indices[I]).Position, Other.InverseMass(B.Indices[I]) };
				Contact.PointB += (OldB[I] + (MB.End[I] - OldB[I]) * Time) * Contact.WeightsB[I];
			}
			// The softer compliance and geometric-mean friction preserve identical
			// materials in self contact. Zero friction on either side stays frictionless.
			const auto& MaterialA = SettingsA.Collision.Material; const auto& MaterialB = SettingsB.Collision.Material;
			SurfaceContactMaterial Material; Material.Compliance = std::max(MaterialA.Compliance, MaterialB.Compliance);
			Material.StaticFriction = std::sqrt(MaterialA.StaticFriction) * std::sqrt(MaterialB.StaticFriction);
			Material.DynamicFriction = std::sqrt(MaterialA.DynamicFriction) * std::sqrt(MaterialB.DynamicFriction);
			if (QuerySettings.Continuous) Material.Compliance = 0;
			if (Existing != InteractionRows.end()) { RefreshRow(*Existing->second, PA, A.Count, PB, B.Count, Contact, Thickness, Material, SurfaceGap, QuerySettings.ContactTolerance); return; }
			std::unique_ptr<ContactRow> Row(new ContactRow); Row->Continuous = QuerySettings.Continuous; Row->SurfaceGap = SurfaceGap; Row->Tolerance = QuerySettings.ContactTolerance;
			Row->Constraint.reset(new SurfaceContactConstraint(PA, A.Count, PB, B.Count, Contact, Thickness, Material));
			InteractionRows.emplace(Key, std::move(Row));
			if (QuerySettings.Continuous) { ++Stats.ContinuousContacts; if (!Self) ++Other.Stats.ContinuousContacts; }
			++Stats.Contacts;
			if (Self) ++Stats.SelfContacts;
			else { ++Stats.InterContacts; ++Other.Stats.InterContacts; ++Other.Stats.Contacts; }
		}
		void AddContact(size_t ColliderIndex, ContactKind Kind, int FeatureA, int FeatureB, const Feature& A, const Feature& B)
		{
			const ContactKey Key(ColliderIndex, Kind, FeatureA, FeatureB); const auto Existing = Rows.find(Key);
			auto& C = *Colliders[ColliderIndex]; ++Stats.Candidates;
			SurfaceMotion MA, MB; MA.Count = A.Count; MB.Count = B.Count;
			Vector3 OldA[3] = { Vector3::Zero(), Vector3::Zero(), Vector3::Zero() };
			Vector3 OldB[3] = { Vector3::Zero(), Vector3::Zero(), Vector3::Zero() };
			for (int I = 0; I < A.Count; ++I) { MA.Start[I] = MA.End[I] = Positions[A.Indices[I]]; OldA[I] = Cloth.GetParticle(A.Indices[I]).PreviousPosition; }
			for (int I = 0; I < B.Count; ++I) { MB.Start[I] = MB.End[I] = C.End[B.Indices[I]]; OldB[I] = C.Start[B.Indices[I]]; }
			const auto& Settings = Cloth.GetSettings(); const bool Continuous = Settings.Collision.Continuous;
			if (Continuous)
			{
				for (int I = 0; I < A.Count; ++I) MA.Start[I] = OldA[I];
				for (int I = 0; I < B.Count; ++I) MB.Start[I] = OldB[I];
			}
			const double Thickness = (double)Settings.Thickness + C.Radius + C.ArcBound; SurfaceContact Contact;
			if (!FindContact(MA, MB, Thickness, Settings.Collision, Contact))
			{
				if (Continuous && Existing != Rows.end()) Rows.erase(Existing);
				return;
			}
			const auto Endpoint = Continuous ? SurfaceCollision::Closest(MA.End, MA.Count, MB.End, MB.Count) : Contact;
			const double SurfaceGap = Endpoint.Distance - Thickness;
			if (Continuous) RefreshInitialContact(Endpoint, Contact);
			if (Contact.Normal.IsZero())
			{
				SurfaceCollisionSettings Query; Query.Thickness = Thickness; Query.DistanceTolerance = Settings.Collision.ContactTolerance; Query.RelativeTolerance = 0;
				const auto Old = SurfaceCollision::Closest(OldA, A.Count, OldB, B.Count);
				if (!SurfaceCollision::Sweep(MA, MB, Query, Contact, Old.Normal)) return;
			}
			SurfaceContactParticle PA[3], PB[3]; Vector3 StartA = Vector3::Zero(), StartB = Vector3::Zero();
			const float Time = Continuous ? (float)Contact.Time : 0;
			for (int I = 0; I < A.Count; ++I)
			{
				PA[I] = { &Cloth.GetParticle(A.Indices[I]).Position, InverseMass(A.Indices[I]) };
				StartA += (OldA[I] + (MA.End[I] - OldA[I]) * Time) * Contact.WeightsA[I];
			}
			for (int I = 0; I < B.Count; ++I)
			{
				PB[I] = { &C.End[B.Indices[I]], 0 }; StartB += (OldB[I] + (MB.End[I] - OldB[I]) * Time) * Contact.WeightsB[I];
			}
			const Transform ReferencePose = Continuous ? Interpolate(C.StepStart, C.StepEnd, Time) : C.StepStart;
			const Vector3 ReferenceNormal = Continuous ? Contact.Normal : ReferencePose.quat * (C.StepEnd.quat.Conjugate() * Contact.Normal);
			const Vector3 EndNormal = Continuous ? C.StepEnd.quat * (ReferencePose.quat.Conjugate() * Contact.Normal) : Contact.Normal;
			Contact.PointA = StartA + EndNormal * C.Radius; Contact.PointB = StartB + ReferenceNormal * C.Radius;
			auto Material = Settings.Collision.Material; if (Continuous) Material.Compliance = 0;
			if (Existing != Rows.end())
			{
				RefreshRow(*Existing->second, PA, A.Count, PB, B.Count, Contact, Thickness, Material, SurfaceGap, Settings.Collision.ContactTolerance); return;
			}
			std::unique_ptr<ContactRow> Row(new ContactRow); Row->Continuous = Continuous; Row->SurfaceGap = SurfaceGap; Row->Tolerance = Settings.Collision.ContactTolerance;
			Row->Constraint.reset(new SurfaceContactConstraint(PA, A.Count, PB, B.Count, Contact, Thickness, Material));
			if (Continuous) ++Stats.ContinuousContacts;
			Rows.emplace(Key, std::move(Row)); ++Stats.Contacts;
		}
		std::vector<bool> RecoverInterior(size_t ColliderIndex)
		{
			auto& C = *Colliders[ColliderIndex]; std::vector<bool> Inside(Positions.size(), false);
			const auto& Settings = Cloth.GetSettings(); const bool Continuous = Settings.Collision.Continuous;
			if (C.Planes.empty()) return Inside;
			for (size_t I = 0; I < Positions.size(); ++I)
			{
				if (Continuous)
				{
					const Vector3 Start = C.StepStart.WorldToLocal(PreviousPositions[I]);
					bool StartedInside = true;
					for (const auto& Plane : C.Planes) if (Plane.Normal.Dot(Start - Plane.Point) >= 0) { StartedInside = false; break; }
					// An incoming trajectory must use its entry face, not the nearest
					// exit plane at the predicted endpoint. Only recover old embeddings.
					if (!StartedInside) continue;
					// An initially embedded vertex may leave freely. Sweeping its exit
					// as an incoming hit would build an inward barrier and trap it.
					Inside[I] = true;
				}
				const Vector3 Local = C.StepEnd.WorldToLocal(Positions[I]); float Separation = -std::numeric_limits<float>::infinity(); int Face = -1;
				for (size_t J = 0; J < C.Planes.size(); ++J)
				{
					const auto& P = C.Planes[J]; const float Distance = P.Normal.Dot(Local - P.Point);
					if (Distance > Separation) { Separation = Distance; Face = (int)J; }
				}
				if (Face < 0 || Separation >= (Continuous ? Settings.Thickness : 0)) continue;
				Inside[I] = true;
				const ContactKey Key(ColliderIndex, ContactKind::Interior, (int)I, 0); const auto Existing = Rows.find(Key);
				if (Existing != Rows.end() && !Continuous) continue;
				const auto& Plane = C.Planes[Face]; const Vector3 Projected = Local - Plane.Normal * Separation;
				std::unique_ptr<ContactRow> NewRow; if (Existing == Rows.end()) NewRow.reset(new ContactRow);
				auto* Row = NewRow ? NewRow.get() : Existing->second.get(); Row->FixedPoint = C.StepEnd.LocalToWorld(Projected); Row->Continuous = Continuous;
				SurfaceContact Contact; Contact.Normal = C.StepEnd.quat * Plane.Normal;
				Contact.WeightsA = Contact.WeightsB = Vector3::UnitX();
				Contact.PointA = Cloth.GetParticle((int)I).PreviousPosition; Contact.PointB = C.StepStart.LocalToWorld(Projected);
				const SurfaceContactParticle A = { &Cloth.GetParticle((int)I).Position, InverseMass((int)I) }, B = { &Row->FixedPoint, 0 };
				auto Material = Settings.Collision.Material; if (Continuous) Material.Compliance = 0;
				RefreshRow(*Row, &A, 1, &B, 1, Contact, Settings.Thickness, Material, Separation - Settings.Thickness, Settings.Collision.ContactTolerance);
				if (NewRow) { Rows.emplace(Key, std::move(NewRow)); ++Stats.Contacts; }
			}
			return Inside;
		}
		void Detect(size_t ColliderIndex)
		{
			auto& C = *Colliders[ColliderIndex];
			if (C.Radius > 0)
			{
				Box3 Bounds(C.End.data(), C.End.size());
				if (Cloth.GetSettings().Collision.Continuous) for (const auto& P : C.Start) Bounds.Encapsulate(P);
				Bounds.Min -= Vector3(C.Radius + C.ArcBound); Bounds.Max += Vector3(C.Radius + C.ArcBound);
				std::vector<int> Hits; Index.QueryTriangles(Bounds, Hits);
				const Feature B = C.End.size() == 1 ? Feature(0) : Feature(SurfaceEdge{ 0, 1 });
				for (int I : Hits)
				{
					const auto& T = Index.GetTriangles()[I];
					AddContact(ColliderIndex, ContactKind::Rounded, I, 0, Feature(T), B);
					if (B.Count == 1) continue;
					// A parallel capsule can cover a line of closest points. One
					// witness cannot support both ends as a face tilts. Endpoint/face
					// and edge/segment witnesses bound that contact interval, including
					// long capsules whose endpoints both lie outside the cloth face.
					for (int End = 0; End < 2; ++End) AddContact(ColliderIndex, ContactKind::Rounded, I, 1 + End, Feature(T), Feature(End));
					for (int Edge = 0; Edge < 3; ++Edge)
						AddContact(ColliderIndex, ContactKind::Rounded, I, 3 + Edge, Feature(SurfaceEdge{ T[Edge], T[(Edge + 1) % 3] }), B);
				}
				return;
			}
			const auto Inside = RecoverInterior(ColliderIndex);
			std::vector<SurfaceCandidate> Candidates; Index.FindCandidates(*C.Index, Candidates);
			for (const auto& Pair : Candidates)
			{
				if (Pair.Type == SurfacePairType::VertexTriangle)
				{
					if (!Inside[Pair.A]) AddContact(ColliderIndex, ContactKind::VertexTriangle, Pair.A, Pair.B, Feature(Pair.A), Feature(C.Triangles[Pair.B]));
				}
				else if (Pair.Type == SurfacePairType::TriangleVertex)
				{
					const auto& T = Index.GetTriangles()[Pair.A];
					if (!Inside[T.a] && !Inside[T.b] && !Inside[T.c]) AddContact(ColliderIndex, ContactKind::TriangleVertex, Pair.A, Pair.B, Feature(T), Feature(Pair.B));
				}
				else
				{
					const auto& E = Index.GetEdges()[Pair.A];
					if (!Inside[E.A] && !Inside[E.B]) AddContact(ColliderIndex, ContactKind::EdgeEdge, Pair.A, Pair.B, Feature(E), Feature(C.Index->GetEdges()[Pair.B]));
				}
			}
		}
		void DiscoverContacts()
		{
			const bool Continuous = Cloth.GetSettings().Collision.Continuous;
			if (Continuous) for (auto& Row : Rows) if (Row.second->Continuous) Row.second->Seen = false;
			UpdateSurface(Continuous);
			for (size_t I = 0; I < Colliders.size(); ++I) Detect(I);
			for (auto It = Rows.begin(); It != Rows.end();)
				if (It->second->Continuous && !It->second->Seen) It = Rows.erase(It); else ++It;
		}
	};
	ClothCollision::ClothCollision(ClothSystem& Cloth) : m_Impl(new Implementation(Cloth)) {}
	ClothCollision::~ClothCollision() = default;
	bool ClothCollision::AddCapsule(RigidBody* Body, float Radius, const Vector3& A, const Vector3& B, bool LegacySphere)
	{
		std::unique_ptr<Collider> C(new Collider); C->LegacySphere=LegacySphere;
		if (!C->InitializeCapsule(Body,Radius,A,B)) return false;
		if (LegacySphere) RemoveColliders(Body,true);
		m_Impl->Rows.clear(); m_Impl->Colliders.push_back(std::move(C)); return true;
	}
	bool ClothCollision::AddCollider(Geometry* Shape)
	{
		std::unique_ptr<Collider> C(new Collider); if (!C->Initialize(Shape)) return false;
		RemoveCollider(Shape); m_Impl->Rows.clear(); m_Impl->Colliders.push_back(std::move(C)); return true;
	}
	bool ClothCollision::RemoveCollider(const Geometry* Shape)
	{
		if (!Shape) return false;
		auto& C = m_Impl->Colliders; const auto Old = C.size(); m_Impl->Rows.clear();
		C.erase(std::remove_if(C.begin(), C.end(), [Shape](const std::unique_ptr<Collider>& P) { return P->Source == Shape; }), C.end()); return Old != C.size();
	}
	bool ClothCollision::RemoveColliders(RigidBody* Body, bool LegacySpheresOnly)
	{
		auto& C = m_Impl->Colliders; const auto Old = C.size(); m_Impl->Rows.clear();
		C.erase(std::remove_if(C.begin(), C.end(), [=](const std::unique_ptr<Collider>& P) { return P->Body == Body && (!LegacySpheresOnly || P->LegacySphere); }), C.end()); return Old != C.size();
	}
	size_t ClothCollision::GetColliderCount() const { return m_Impl->Colliders.size(); }
	const ClothCollisionStats& ClothCollision::GetStats() const { return m_Impl->Stats; }
	void ClothCollision::BeginFrame()
	{
		m_Impl->InteractionRows.clear();
		m_Impl->Stats = ClothCollisionStats();
		for (auto& C : m_Impl->Colliders) C->CaptureFrame();
	}
	void ClothCollision::BeginSubstep(float StartFraction, float EndFraction)
	{
		m_Impl->Rows.clear(); m_Impl->InteractionRows.clear();
		const bool Continuous = m_Impl->Cloth.GetSettings().Collision.Continuous;
		++m_Impl->Stats.Substeps;
		for (auto& C : m_Impl->Colliders) C->Update(StartFraction,EndFraction,Continuous);
	}
	void ClothCollision::Solve(float Dt, bool SeparationOnly)
	{
		auto& S = *m_Impl; if (S.Colliders.empty()) return;
		S.DiscoverContacts();
		for (auto& Pair : S.Rows) Pair.second->Solve(Dt, SeparationOnly);
	}
	void ClothCollision::SolveInteractions(const std::vector<ClothCollision*>& Collisions, float Dt, bool Project, bool SeparationOnly)
	{
		/* All surfaces are refitted after material and rigid-contact projection.
		   Discover every row before writing any particle, so broad/narrow phases
		   see one coherent iterate. A canonical owner solves each inter-object row
		   once, with real inverse masses on both sides; no reaction is lost at an
		   object boundary. Self contacts always exclude shared vertices, optionally
		   also the one-ring neighborhood. Contact thickness is the sum of radii.
		   The ordered row map retains multipliers only within this physical substep.
		   Per-cloth statistics include a shared contact on BOTH participating cloths.
		   Independent object pairs could run in parallel after coloring shared writes. */
		bool Swept = false;
		for (auto* Collision : Collisions) Swept = Swept || Collision->m_Impl->Cloth.GetSettings().Collision.Continuous;
		for (auto* Collision : Collisions)
		{
			for (auto& Row : Collision->m_Impl->InteractionRows) if (Row.second->Continuous) Row.second->Seen = false;
			const auto& Settings = Collision->m_Impl->Cloth.GetSettings().Collision;
			if (Settings.SelfCollision || Settings.InterCollision) Collision->m_Impl->UpdateSurface(Swept);
		}
		std::vector<SurfaceCandidate> Candidates;
		for (size_t I = 0; I < Collisions.size(); ++I)
		{
			auto& A = *Collisions[I]->m_Impl; const auto& Settings = A.Cloth.GetSettings().Collision;
			if (Settings.SelfCollision)
			{
				A.Index.FindSelfCandidates(Candidates, Settings.ExcludeSelfNeighbors);
				for (const auto& Pair : Candidates) A.AddInteraction(A, I, Pair);
			}
			if (!Settings.InterCollision) continue;
			for (size_t J = I + 1; J < Collisions.size(); ++J)
			{
				auto& B = *Collisions[J]->m_Impl;
				if (!B.Cloth.GetSettings().Collision.InterCollision || !A.Bounds.Intersect(B.Bounds)) continue;
				A.Index.FindCandidates(B.Index, Candidates);
				for (const auto& Pair : Candidates) A.AddInteraction(B, J, Pair);
			}
		}
		for (auto* Collision : Collisions)
		{
			auto& Rows = Collision->m_Impl->InteractionRows;
			for (auto It = Rows.begin(); It != Rows.end();)
				if (It->second->Continuous && !It->second->Seen) It = Rows.erase(It); else ++It;
			if (Project) for (auto& Row : Rows) Row.second->Solve(Dt, SeparationOnly);
		}
	}
	int ClothCollision::RequiredSubsteps() const
	{
		const auto& Settings = m_Impl->Cloth.GetSettings().Collision;
		if (!Settings.Continuous || !Settings.SurfaceContacts) return 1;
		double Count = 1;
		for (const auto& C : m_Impl->Colliders)
			Count = std::max(Count, std::ceil((double)C->FrameAngle() / Settings.MaximumColliderAngle));
		if (Count > Settings.MaximumAngularSubsteps) throw std::runtime_error("Cloth collider rotation exceeds its angular substep budget");
		return (int)Count;
	}
	bool ClothCollision::FinishContinuous(const std::vector<ClothCollision*>& Collisions, float Dt)
	{
		int Passes = 0;
		for (auto* Collision : Collisions)
		{
			const auto& Settings = Collision->m_Impl->Cloth.GetSettings().Collision;
			if (Settings.Continuous && (Settings.SurfaceContacts || Settings.SelfCollision || Settings.InterCollision))
				Passes = std::max(Passes, Settings.ContinuousIterations);
		}
		if (Passes == 0) return true;
		auto Separated = [&]()
		{
			// Refresh every witness after the LAST projection. A small position
			// change can mean conflicting rows have stagnated, not that they agree.
			for (auto* Collision : Collisions)
			{
				auto& S = *Collision->m_Impl;
				if (!S.Cloth.GetSettings().Collision.Continuous || !S.Cloth.GetSettings().Collision.SurfaceContacts) continue;
				S.DiscoverContacts();
			}
			SolveInteractions(Collisions, Dt, false);
			bool Valid = true;
			for (auto* Collision : Collisions)
			{
				auto& S = *Collision->m_Impl; const auto& Settings = S.Cloth.GetSettings().Collision;
				double Violation = 0, Penetration = 0;
				auto Inspect = [&](const ContactRow& Row)
				{
					const double Error = -Row.Constraint->GetNormalGap();
					Violation = std::max(Violation, Error);
					Penetration = std::max(Penetration, -Row.SurfaceGap);
					if (std::max(Error, -Row.SurfaceGap) > Row.Tolerance) Valid = false;
				};
				if (Settings.Continuous && Settings.SurfaceContacts) for (const auto& Row : S.Rows) Inspect(*Row.second);
				for (const auto& Row : S.InteractionRows) if (Row.second->Continuous) Inspect(*Row.second);
				S.Stats.MaximumConstraintViolation = Violation; S.Stats.MaximumSurfacePenetration = Penetration;
			}
			return Valid;
		};
		std::vector<Vector3> Before;
		for (int Pass = 0; Pass < Passes; ++Pass)
		{
			Before.clear();
			for (auto* Collision : Collisions) for (int I = 0; I < Collision->m_Impl->Cloth.GetParticleCount(); ++I)
				Before.push_back(Collision->m_Impl->Cloth.GetParticle(I).Position);
			// Rigid and pair rows can move a vertex into another collider. Resweep
			// the corrected path before accepting convergence of the coupled batch.
			for (auto* Collision : Collisions)
				if (Collision->m_Impl->Cloth.GetSettings().Collision.SurfaceContacts) Collision->Solve(Dt, true);
			SolveInteractions(Collisions, Dt, true, true);
			bool Converged = true; size_t Offset = 0;
			for (auto* Collision : Collisions)
			{
				auto& S = *Collision->m_Impl; ++S.Stats.ContinuousPasses;
				const float Tolerance = S.Cloth.GetSettings().Collision.ContactTolerance;
				for (int I = 0; I < S.Cloth.GetParticleCount(); ++I)
					if ((S.Cloth.GetParticle(I).Position - Before[Offset++]).Length() > Tolerance) Converged = false;
			}
			if (Converged && Separated()) return true;
		}
		if (Separated()) return true;
		for (auto* Collision : Collisions) Collision->m_Impl->Stats.ProjectionLimitReached = true;
		return false;
	}
	void ClothCollision::RecordRefinement(const std::vector<ClothCollision*>& Collisions)
	{
		for (auto* Collision : Collisions) ++Collision->m_Impl->Stats.ContactRefinements;
	}
	void ClothCollision::EndFrame(bool Commit)
	{
		m_Impl->Rows.clear(); m_Impl->InteractionRows.clear();
		if (Commit) for (auto& C : m_Impl->Colliders) C->CommitFrame();
	}
}
