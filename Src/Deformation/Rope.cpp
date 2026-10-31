#include "Rope.h"

#include <algorithm>
#include <cmath>
#include <cfloat>

#include "../RigidBodyDynamics/RigidBody.h"
#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/AxisAlignedBox3.h"
#include "../CollisionPrimitive/Sphere3.h"

namespace Riemann
{
	namespace
	{
		bool Finite(const Vector3& value)
		{
			return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
		}

		bool Finite(const Quaternion& value)
		{
			return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z)
				&& std::isfinite(value.w) && value.Dot(value) > 1.0e-12f;
		}

		float Clamp(float value, float lower, float upper)
		{
			return std::max(lower, std::min(value, upper));
		}

		float ParticleWeight(const RopeParticle& particle)
		{
			return !particle.Pinned && std::isfinite(particle.InverseMass)
				? std::max(0.0f, particle.InverseMass) : 0.0f;
		}

		Quaternion AdvanceRotation(const Quaternion& rotation, const Vector3& velocity, float dt)
		{
			return (rotation + (Quaternion(0.0f, velocity) * rotation) * (0.5f * dt)).Unit();
		}

		Vector3 RotationVelocity(const Quaternion& start, const Quaternion& end, float dt)
		{
			Quaternion delta = (end * start.Conjugate()).Unit();
			if (delta.w < 0.0f) delta = -delta;
			// Invert the normalized quaternion update used by rigid-body integration.
			const float scale = 2.0f / (dt * std::max(0.05f, delta.w));
			return Vector3(delta.x, delta.y, delta.z) * scale;
		}

		struct SegmentContact
		{
			float Fraction;
			float Distance;
			Vector3 Normal;
			Vector3 Surface;
		};

		SegmentContact SphereContact(const Vector3& a, const Vector3& b, const Transform& pose, float radius)
		{
			SegmentContact result;
			const Vector3 edge = b - a;
			const float lengthSquared = edge.SquareLength();
			result.Fraction = lengthSquared > 1.0e-12f ? Clamp((pose.pos - a).Dot(edge) / lengthSquared, 0.0f, 1.0f) : 0.0f;
			const Vector3 offset = a + edge * result.Fraction - pose.pos;
			const float length = offset.Length();
			result.Normal = length > 1.0e-7f ? offset / length : edge.Cross(Vector3::UnitY()).SafeUnit();
			if (result.Normal.IsZero()) result.Normal = Vector3::UnitX();
			result.Distance = length - radius;
			result.Surface = pose.pos + result.Normal * radius;
			return result;
		}

		SegmentContact BoxContact(const Vector3& a, const Vector3& b, const Transform& pose, const Vector3& extent)
		{
			const Vector3 start = pose.WorldToLocal(a);
			const Vector3 direction = pose.WorldToLocal(b) - start;
			float cuts[8] = { 0.0f, 1.0f };
			int count = 2;
			for (int axis = 0; axis < 3; ++axis)
			{
				if (std::abs(direction[axis]) <= 1.0e-12f) continue;
				for (int side = -1; side <= 1; side += 2)
				{
					const float t = (extent[axis] * side - start[axis]) / direction[axis];
					if (t > 0.0f && t < 1.0f) cuts[count++] = t;
				}
			}
			std::sort(cuts, cuts + count);
			float bestDistance = FLT_MAX;
			float bestFraction = 0.0f;
			for (int interval = 0; interval + 1 < count; ++interval)
			{
				const float midpoint = (cuts[interval] + cuts[interval + 1]) * 0.5f;
				float numerator = 0.0f;
				float denominator = 0.0f;
				for (int axis = 0; axis < 3; ++axis)
				{
					const float coordinate = start[axis] + direction[axis] * midpoint;
					if (coordinate >= -extent[axis] && coordinate <= extent[axis]) continue;
					const float boundary = coordinate < 0.0f ? -extent[axis] : extent[axis];
					numerator += direction[axis] * (start[axis] - boundary);
					denominator += direction[axis] * direction[axis];
				}
				// Squared segment-to-box distance is quadratic between slab crossings.
				const float fraction = denominator > 1.0e-12f
					? Clamp(-numerator / denominator, cuts[interval], cuts[interval + 1]) : midpoint;
				const Vector3 point = start + direction * fraction;
				Vector3 offset = Vector3::Zero();
				for (int axis = 0; axis < 3; ++axis) offset[axis] = point[axis] - Clamp(point[axis], -extent[axis], extent[axis]);
				const float squared = offset.SquareLength();
				if (squared < bestDistance)
				{
					bestDistance = squared;
					bestFraction = fraction;
				}
			}
			SegmentContact result;
			result.Fraction = bestFraction;
			const Vector3 point = start + direction * bestFraction;
			Vector3 surface = point;
			for (int axis = 0; axis < 3; ++axis) surface[axis] = Clamp(point[axis], -extent[axis], extent[axis]);
			Vector3 normal = point - surface;
			result.Distance = normal.Length();
			if (result.Distance > 1.0e-7f)
			{
				normal /= result.Distance;
			}
			else
			{
				int axis = 0;
				for (int i = 1; i < 3; ++i)
					if (extent[i] - std::abs(point[i]) < extent[axis] - std::abs(point[axis])) axis = i;
				normal = Vector3::Zero();
				normal[axis] = point[axis] < 0.0f ? -1.0f : 1.0f;
				result.Distance = -(extent[axis] - std::abs(point[axis]));
				surface[axis] = normal[axis] * extent[axis];
			}
			result.Normal = pose.LocalToWorldDirection(normal);
			result.Surface = pose.LocalToWorld(surface);
			return result;
		}
	}

	RopeSettings::RopeSettings()
		: SegmentCount(16)
		, SolverIterations(12)
		, ParticleRadius(0.08f)
		, ParticleInverseMass(1.0f)
		, RestLength(0.0f)
		, Damping(0.995f)
		, StretchCompliance(0.0f)
		, CollisionFriction(0.1f)
		, TensionOnly(false)
		, ContinuousCollision(true)
		, Gravity(0.0f, -9.8f, 0.0f)
	{
	}

	RopeSystem::RopeSystem(const RopeSettings& settings, const Vector3& start, const Vector3& end)
		: m_Settings(settings)
		, m_SegmentRestLength(0.0f)
		, m_PreviousDt(1.0f / 60.0f)
	{
		m_Settings.SegmentCount = std::max(1, m_Settings.SegmentCount);
		m_Settings.SolverIterations = std::max(1, m_Settings.SolverIterations);
		m_Settings.ParticleRadius = std::isfinite(m_Settings.ParticleRadius) ? std::max(0.001f, m_Settings.ParticleRadius) : 0.08f;
		m_Settings.ParticleInverseMass = std::isfinite(m_Settings.ParticleInverseMass) ? std::max(0.0f, m_Settings.ParticleInverseMass) : 1.0f;
		m_Settings.Damping = std::isfinite(m_Settings.Damping) ? Clamp(m_Settings.Damping, 0.0f, 1.0f) : 0.995f;
		m_Settings.StretchCompliance = std::isfinite(m_Settings.StretchCompliance) ? std::max(0.0f, m_Settings.StretchCompliance) : 0.0f;
		m_Settings.CollisionFriction = std::isfinite(m_Settings.CollisionFriction) ? Clamp(m_Settings.CollisionFriction, 0.0f, 1.0f) : 0.1f;
		if (!Finite(m_Settings.Gravity)) m_Settings.Gravity = Vector3::Zero();
		const Vector3 safeStart = Finite(start) ? start : Vector3::Zero();
		const Vector3 safeEnd = Finite(end) ? end : safeStart;
		m_SegmentRestLength = m_Settings.RestLength > 0.0f
			&& std::isfinite(m_Settings.RestLength) ? m_Settings.RestLength
			: (safeEnd - safeStart).Length() / (float)m_Settings.SegmentCount;

		m_Particles.resize((size_t)m_Settings.SegmentCount + 1);
		m_CollisionStartPositions.resize(m_Particles.size());
		m_StretchMultipliers.resize((size_t)m_Settings.SegmentCount, 0.0f);
		m_RenderGeometries.reserve(m_Particles.size());
		for (int i = 0; i <= m_Settings.SegmentCount; ++i)
		{
			const float fraction = (float)i / (float)m_Settings.SegmentCount;
			RopeParticle& particle = m_Particles[(size_t)i];
			particle.Position = safeStart + (safeEnd - safeStart) * fraction;
			particle.PreviousPosition = particle.Position;
			particle.PinPosition = particle.Position;
			particle.InverseMass = m_Settings.ParticleInverseMass;
			particle.Pinned = false;
			m_CollisionStartPositions[(size_t)i] = particle.Position;
			particle.RenderGeometry = GeometryFactory::CreateSphere(particle.Position, m_Settings.ParticleRadius);
			particle.RenderGeometry->SetQueryEnabled(false);
			particle.RenderGeometry->SetSimulationEnabled(false);
			m_RenderGeometries.push_back(particle.RenderGeometry);
		}
	}

	RopeSystem::~RopeSystem()
	{
		for (Geometry* geometry : m_RenderGeometries)
		{
			delete geometry;
		}
		m_RenderGeometries.clear();
	}

	bool RopeSystem::PinParticle(int index, const Vector3& worldPosition)
	{
		if (index < 0 || index >= GetParticleCount() || !Finite(worldPosition))
		{
			return false;
		}
		RopeParticle& particle = m_Particles[(size_t)index];
		particle.Pinned = true;
		particle.PinPosition = worldPosition;
		particle.Position = worldPosition;
		particle.PreviousPosition = worldPosition;
		UpdateRenderGeometries();
		return true;
	}

	bool RopeSystem::UnpinParticle(int index)
	{
		if (index < 0 || index >= GetParticleCount())
		{
			return false;
		}
		m_Particles[(size_t)index].Pinned = false;
		return true;
	}

	bool RopeSystem::AttachRigidBody(int index, RigidBody* body, const Vector3& worldAnchor)
	{
		if (index < 0 || index >= GetParticleCount() || body == nullptr || !Finite(worldAnchor))
		{
			return false;
		}
		DetachRigidBody(index);
		BodyAttachment attachment;
		attachment.ParticleIndex = index;
		attachment.Body = body;
		attachment.LocalAnchor = body->WorldToBodyLocal(worldAnchor);
		m_BodyAttachments.push_back(attachment);
		return true;
	}

	bool RopeSystem::DetachRigidBody(int index)
	{
		const size_t oldSize = m_BodyAttachments.size();
		m_BodyAttachments.erase(std::remove_if(m_BodyAttachments.begin(), m_BodyAttachments.end(),
			[index](const BodyAttachment& attachment) { return attachment.ParticleIndex == index; }), m_BodyAttachments.end());
		return m_BodyAttachments.size() != oldSize;
	}

	bool RopeSystem::DetachRigidBody(RigidBody* body)
	{
		const size_t oldSize = m_BodyAttachments.size();
		m_BodyAttachments.erase(std::remove_if(m_BodyAttachments.begin(), m_BodyAttachments.end(),
			[body](const BodyAttachment& attachment) { return attachment.Body == body; }), m_BodyAttachments.end());
		return m_BodyAttachments.size() != oldSize;
	}

	bool RopeSystem::SetParticleVelocity(int index, const Vector3& velocity)
	{
		if (index < 0 || index >= GetParticleCount() || !Finite(velocity)) return false;
		RopeParticle& particle = m_Particles[(size_t)index];
		particle.PreviousPosition = particle.Position - velocity * m_PreviousDt;
		return true;
	}

	Vector3 RopeSystem::GetParticleVelocity(int index) const
	{
		if (index < 0 || index >= GetParticleCount()) return Vector3::Zero();
		const RopeParticle& particle = m_Particles[(size_t)index];
		return particle.Pinned ? Vector3::Zero() : (particle.Position - particle.PreviousPosition) / m_PreviousDt;
	}

	bool RopeSystem::AddSphereCollider(RigidBody* body, float radius, const Vector3& worldCenter)
	{
		if (!Finite(worldCenter) || !std::isfinite(radius) || radius <= 0.0f) return false;
		if (body && (!Finite(body->X) || !Finite(body->Q))) return false;
		Collider collider;
		collider.Body = body;
		collider.Source = nullptr;
		collider.LocalTransform = Transform(body ? body->WorldToBodyLocal(worldCenter) : worldCenter);
		collider.WorldTransform = collider.PreviousTransform = Transform(worldCenter);
		collider.HalfExtent = Vector3::Zero();
		collider.Radius = radius;
		collider.Sphere = true;
		m_Colliders.push_back(collider);
		return true;
	}

	bool RopeSystem::AddBoxCollider(RigidBody* body, const Vector3& halfExtent, const Transform& worldTransform)
	{
		if (!Finite(halfExtent) || halfExtent.x <= 0.0f || halfExtent.y <= 0.0f || halfExtent.z <= 0.0f
			|| !Finite(worldTransform.pos) || !Finite(worldTransform.quat)) return false;
		if (body && (!Finite(body->X) || !Finite(body->Q))) return false;
		Collider collider;
		collider.Body = body;
		collider.Source = nullptr;
		collider.WorldTransform = collider.PreviousTransform = Transform(worldTransform.pos, worldTransform.quat.Unit());
		collider.LocalTransform = body ? Transform(body->WorldToBodyLocal(worldTransform.pos),
			(body->Q.Conjugate() * worldTransform.quat).Unit()) : collider.WorldTransform;
		collider.HalfExtent = halfExtent;
		collider.Radius = 0.0f;
		collider.Sphere = false;
		m_Colliders.push_back(collider);
		return true;
	}

	bool RopeSystem::AddCollider(Geometry* geometry)
	{
		if (!geometry) return false;
		RigidBody* body = geometry->GetParent<RigidBody>();
		const Transform pose = body ? body->GetGeometryTransform(geometry) : *geometry->GetTransform();
		bool added = false;
		if (const Sphere3* sphere = geometry->GetShapeObj<Sphere3>())
			added = AddSphereCollider(body, sphere->Radius, pose.LocalToWorld(sphere->Center));
		else if (const AxisAlignedBox3* box = geometry->GetShapeObj<AxisAlignedBox3>())
			added = AddBoxCollider(body, box->GetExtent(), Transform(pose.LocalToWorld(box->GetCenter()), pose.quat));
		if (added)
		{
			RemoveCollider(geometry);
			m_Colliders.back().Source = geometry;
		}
		return added;
	}

	bool RopeSystem::RemoveCollider(const Geometry* geometry)
	{
		if (!geometry) return false;
		const size_t count = m_Colliders.size();
		m_Colliders.erase(std::remove_if(m_Colliders.begin(), m_Colliders.end(),
			[geometry](const Collider& collider) { return collider.Source == geometry; }), m_Colliders.end());
		return count != m_Colliders.size();
	}

	bool RopeSystem::RemoveColliders(RigidBody* body)
	{
		const size_t count = m_Colliders.size();
		m_Colliders.erase(std::remove_if(m_Colliders.begin(), m_Colliders.end(),
			[body](const Collider& collider) { return collider.Body == body; }), m_Colliders.end());
		return count != m_Colliders.size();
	}

	void RopeSystem::ClearColliders()
	{
		m_Colliders.clear();
	}

	void RopeSystem::Simulate(float dt)
	{
		if (!std::isfinite(dt) || dt <= 1.0e-8f) return;
		m_BodyStartStates.clear();
		for (const BodyAttachment& attachment : m_BodyAttachments)
		{
			RigidBodyDynamic* body = attachment.Body ? attachment.Body->CastDynamic() : nullptr;
			if (body && !body->Freezing && body->InvMass > 0.0f)
			{
				const auto found = std::find_if(m_BodyStartStates.begin(), m_BodyStartStates.end(),
					[body](const BodyState& entry) { return entry.Body == body; });
				if (found == m_BodyStartStates.end())
				{
					BodyState state;
					state.Body = body;
					state.Position = body->X;
					state.Rotation = body->Q;
					m_BodyStartStates.push_back(state);
					body->X += body->V * dt;
					body->Q = AdvanceRotation(body->Q, body->W, dt);
				}
			}
		}
		for (Collider& collider : m_Colliders)
		{
			collider.WorldTransform = collider.Body ? Transform(collider.Body->BodyLocalToWorld(collider.LocalTransform.pos),
				collider.Body->Q * collider.LocalTransform.quat) : collider.LocalTransform;
		}

		const float dtSquared = dt * dt;
		const float damping = std::pow(m_Settings.Damping, dt * 60.0f);
		for (RopeParticle& particle : m_Particles)
		{
			if (ParticleWeight(particle) <= 0.0f)
			{
				particle.PreviousPosition = particle.Position;
				continue;
			}
			const Vector3 displacement = (particle.Position - particle.PreviousPosition) * (damping * dt / m_PreviousDt);
			particle.PreviousPosition = particle.Position;
			particle.Position += displacement + m_Settings.Gravity * dtSquared;
		}

		std::fill(m_StretchMultipliers.begin(), m_StretchMultipliers.end(), 0.0f);
		for (int iteration = 0; iteration < m_Settings.SolverIterations; ++iteration)
		{
			SolveDistanceConstraints(dt);
			SolvePins();
			SolveBodyAttachments();
			SolveCollisions();
		}

		for (const BodyState& state : m_BodyStartStates)
		{
			RigidBodyDynamic* body = state.Body->CastDynamic();
			const Vector3 linearVelocity = (body->X - state.Position) / dt;
			const Vector3 angularVelocity = RotationVelocity(state.Rotation, body->Q, dt);
			const bool changed = (linearVelocity - body->V).SquareLength() + (angularVelocity - body->W).SquareLength() > 1.0e-10f;
			body->V = linearVelocity;
			body->W = angularVelocity;
			body->X = state.Position;
			body->Q = state.Rotation;
			if (changed) body->Wakeup();
		}
		for (size_t i = 0; i < m_Particles.size(); ++i) m_CollisionStartPositions[i] = m_Particles[i].Position;
		for (Collider& collider : m_Colliders) collider.PreviousTransform = collider.WorldTransform;
		m_PreviousDt = dt;
		UpdateRenderGeometries();
	}

	void RopeSystem::SolveDistanceConstraints(float dt)
	{
		const float alpha = m_Settings.StretchCompliance / (dt * dt);
		for (int i = 0; i < m_Settings.SegmentCount; ++i)
		{
			RopeParticle& particleA = m_Particles[(size_t)i];
			RopeParticle& particleB = m_Particles[(size_t)i + 1];
			const Vector3 delta = particleB.Position - particleA.Position;
			const float distance = delta.Length();
			if (distance <= 1e-7f)
			{
				continue;
			}
			const float weightA = ParticleWeight(particleA);
			const float weightB = ParticleWeight(particleB);
			const float weightSum = weightA + weightB;
			if (weightSum <= 1e-7f)
			{
				continue;
			}
			float& multiplier = m_StretchMultipliers[(size_t)i];
			float increment = (-(distance - m_SegmentRestLength) - alpha * multiplier) / (weightSum + alpha);
			if (m_Settings.TensionOnly) increment = std::min(0.0f, multiplier + increment) - multiplier;
			multiplier += increment;
			const Vector3 correction = delta * (increment / distance);
			particleA.Position -= correction * weightA;
			particleB.Position += correction * weightB;
		}
	}

	void RopeSystem::SolvePins()
	{
		for (RopeParticle& particle : m_Particles)
		{
			if (particle.Pinned)
			{
				particle.Position = particle.PinPosition;
			}
		}
	}

	void RopeSystem::SolveBodyAttachments()
	{
		for (const BodyAttachment& attachment : m_BodyAttachments)
		{
			if (attachment.Body == nullptr || attachment.ParticleIndex < 0 || attachment.ParticleIndex >= GetParticleCount())
			{
				continue;
			}
			RopeParticle& particle = m_Particles[(size_t)attachment.ParticleIndex];
			RigidBodyDynamic* dynamicBody = attachment.Body->CastDynamic();
			const float particleWeight = ParticleWeight(particle);
			const float bodyWeight = dynamicBody && !dynamicBody->Freezing ? dynamicBody->InvMass : 0.0f;
			const float weightSum = particleWeight + bodyWeight;
			if (weightSum <= 1e-7f)
			{
				continue;
			}
			const Vector3 bodyAnchor = attachment.Body->BodyLocalToWorld(attachment.LocalAnchor);
			const Vector3 error = particle.Position - bodyAnchor;
			Vector3 arm = bodyAnchor - attachment.Body->X;
			const Matrix3 inverseInertia = bodyWeight > 0.0f ? dynamicBody->GetInverseInertia_WorldSpace() : Matrix3::Zero();
			const Matrix3 cross = Matrix3::Skew(arm);
			Matrix3 effectiveMass = Matrix3(weightSum, weightSum, weightSum) - cross * inverseInertia * cross;
			Vector3 impulse;
			if (!effectiveMass.Solve(error, impulse)) continue;
			particle.Position -= impulse * particleWeight;
			if (bodyWeight > 0.0f)
			{
				dynamicBody->X += impulse * bodyWeight;
				const Vector3 angularCorrection = inverseInertia * arm.Cross(impulse);
				dynamicBody->Q = AdvanceRotation(dynamicBody->Q, angularCorrection, 1.0f);
			}
		}
	}

	void RopeSystem::ApplyCollision(int indexA, int indexB, float fraction, const Vector3& normal,
		float depth, const Vector3& colliderDisplacement)
	{
		if (depth <= 0.0f || !std::isfinite(depth) || !Finite(normal)) return;
		RopeParticle& a = m_Particles[(size_t)indexA];
		RopeParticle& b = m_Particles[(size_t)indexB];
		const float gradientA = indexA == indexB ? 1.0f : 1.0f - fraction;
		const float gradientB = indexA == indexB ? 0.0f : fraction;
		const float weightA = ParticleWeight(a);
		const float weightB = ParticleWeight(b);
		const float denominator = weightA * gradientA * gradientA + weightB * gradientB * gradientB;
		if (denominator <= 1.0e-10f) return;
		const Vector3 correctionA = normal * (depth * weightA * gradientA / denominator);
		const Vector3 correctionB = normal * (depth * weightB * gradientB / denominator);
		a.Position += correctionA;
		a.PreviousPosition += correctionA;
		if (indexA != indexB)
		{
			b.Position += correctionB;
			b.PreviousPosition += correctionB;
		}

		// Remove incoming relative motion without adding energy from depenetration.
		const Vector3 displacement = (a.Position - a.PreviousPosition) * gradientA
			+ (b.Position - b.PreviousPosition) * gradientB - colliderDisplacement;
		const float normalMotion = displacement.Dot(normal);
		const Vector3 tangent = displacement - normal * normalMotion;
		const float tangentLength = tangent.Length();
		const float friction = tangentLength > 1.0e-8f
			? std::min(1.0f, m_Settings.CollisionFriction * (depth + std::max(0.0f, -normalMotion)) / tangentLength) : 0.0f;
		const Vector3 removed = normal * std::min(0.0f, normalMotion) + tangent * friction;
		a.PreviousPosition += removed * (weightA * gradientA / denominator);
		if (indexA != indexB) b.PreviousPosition += removed * (weightB * gradientB / denominator);
	}

	void RopeSystem::SolveSegmentCollision(int indexA, int indexB, const Collider& collider)
	{
		RopeParticle& a = m_Particles[(size_t)indexA];
		RopeParticle& b = m_Particles[(size_t)indexB];
		if (ParticleWeight(a) + ParticleWeight(b) <= 0.0f) return;
		const Vector3 startA = m_CollisionStartPositions[(size_t)indexA];
		const Vector3 startB = m_CollisionStartPositions[(size_t)indexB];
		const Vector3 motionA = a.Position - startA;
		const Vector3 motionB = b.Position - startB;
		auto ContactAt = [&collider](const Vector3& pointA, const Vector3& pointB, const Transform& pose)
		{
			return collider.Sphere ? SphereContact(pointA, pointB, pose, collider.Radius)
				: BoxContact(pointA, pointB, pose, collider.HalfExtent);
		};
		const float radius = m_Settings.ParticleRadius;
		if (m_Settings.ContinuousCollision)
		{
			const float rotationAngle = collider.Sphere ? 0.0f : 2.0f * std::acos(Clamp(
				std::abs(collider.PreviousTransform.quat.Dot(collider.WorldTransform.quat)), 0.0f, 1.0f));
			const float speedBound = std::max(motionA.Length(), motionB.Length())
				+ (collider.WorldTransform.pos - collider.PreviousTransform.pos).Length()
				+ rotationAngle * collider.HalfExtent.Length();
			SegmentContact contact = ContactAt(startA, startB, collider.PreviousTransform);
			if (speedBound > 1.0e-8f && contact.Distance >= radius - 1.0e-5f)
			{
				float time = 0.0f;
				Transform pose = collider.PreviousTransform;
				for (int iteration = 0; iteration < 80; ++iteration)
				{
					if (contact.Distance <= radius + 1.0e-5f)
					{
						const Vector3 localSurface = pose.WorldToLocal(contact.Surface);
						const Vector3 surface = collider.WorldTransform.LocalToWorld(localSurface);
						const Vector3 normal = collider.WorldTransform.LocalToWorldDirection(pose.WorldToLocalDirection(contact.Normal));
						const Vector3 endPoint = a.Position + (b.Position - a.Position) * contact.Fraction;
						const Vector3 surfaceMotion = surface - collider.PreviousTransform.LocalToWorld(localSurface);
						ApplyCollision(indexA, indexB, contact.Fraction, normal,
							radius - (endPoint - surface).Dot(normal), surfaceMotion);
						break;
					}
					// The closest segment point moves no faster than its fastest endpoint.
					// Adding collider translation and angular sweep keeps this conservative.
					const float advance = (contact.Distance - radius) / speedBound;
					if (advance <= 0.0f || time + advance > 1.0f) break;
					time += advance;
					pose = Transform(collider.PreviousTransform.pos
						+ (collider.WorldTransform.pos - collider.PreviousTransform.pos) * time,
						Quaternion::Slerp(collider.PreviousTransform.quat, collider.WorldTransform.quat, time));
					contact = ContactAt(startA + motionA * time, startB + motionB * time, pose);
				}
			}
		}
		const SegmentContact contact = ContactAt(a.Position, b.Position, collider.WorldTransform);
		const Vector3 localSurface = collider.WorldTransform.WorldToLocal(contact.Surface);
		ApplyCollision(indexA, indexB, contact.Fraction, contact.Normal, radius - contact.Distance,
			contact.Surface - collider.PreviousTransform.LocalToWorld(localSurface));
	}

	void RopeSystem::SolveCollisions()
	{
		for (const Collider& collider : m_Colliders)
		{
			if (!Finite(collider.WorldTransform.pos) || !Finite(collider.WorldTransform.quat)) continue;
			for (int i = 0; i < GetParticleCount(); ++i) SolveSegmentCollision(i, i, collider);
			for (int i = 0; i < GetSegmentCount(); ++i) SolveSegmentCollision(i, i + 1, collider);
		}
	}

	void RopeSystem::UpdateRenderGeometries()
	{
		for (RopeParticle& particle : m_Particles)
		{
			if (particle.RenderGeometry)
			{
				particle.RenderGeometry->SetPosition(particle.Position);
			}
		}
	}
}
