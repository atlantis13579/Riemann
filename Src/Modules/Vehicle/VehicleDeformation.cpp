#include "VehicleDeformation.h"

#include <algorithm>
#include <cmath>

#include "../../Collision/GeometryObject.h"
#include "../../RigidBodyDynamics/PhysicsWorld.h"
#include "../../RigidBodyDynamics/RigidBody.h"

namespace Riemann
{
	namespace
	{
		bool Finite(const Vector3& value)
		{
			return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
		}

		bool ValidTransform(const Transform& value)
		{
			const float length = value.quat.SquareLength();
			return Finite(value.pos) && std::isfinite(length) && length > 1.0e-12f;
		}

		double Length(const Vector3& value)
		{
			return std::sqrt(double(value.x) * value.x + double(value.y) * value.y + double(value.z) * value.z);
		}

		float Positive(float value, float fallback)
		{
			return std::isfinite(value) && value > 0.0f ? value : fallback;
		}

		float NonNegative(float value, float fallback)
		{
			return std::isfinite(value) && value >= 0.0f ? value : fallback;
		}

		VehicleDeformationSettings Sanitize(const VehicleDeformationSettings& settings)
		{
			VehicleDeformationSettings result = settings;
			const VehicleDeformationSettings defaults;
			for (int axis = 0; axis < 3; ++axis)
			{
				result.HalfExtent[axis] = std::max(0.05f, std::min(10000.0f,
					Positive(settings.HalfExtent[axis], defaults.HalfExtent[axis])));
			}
			// Keep a non-collinear internal frame and bound the cost of the tetrahedral cage.
			result.Cells.x = std::max(4, std::min(8, settings.Cells.x));
			result.Cells.y = std::max(2, std::min(4, settings.Cells.y));
			result.Cells.z = std::max(4, std::min(16, settings.Cells.z));
			result.CabinHeight = std::min(4.0f * result.HalfExtent.y, NonNegative(settings.CabinHeight, 0.0f));
			result.ShellMass = std::max(0.01f, std::min(1.0e6f, Positive(settings.ShellMass, defaults.ShellMass)));
			result.CrushZoneLength = std::min(0.75f * result.HalfExtent.z, NonNegative(settings.CrushZoneLength, 0));
			result.ImpactRadius = Positive(settings.ImpactRadius, defaults.ImpactRadius);
			result.ImpulseScale = NonNegative(settings.ImpulseScale, defaults.ImpulseScale);
			result.MinImpactImpulse = NonNegative(settings.MinImpactImpulse, defaults.MinImpactImpulse);
			result.MaxDeformationImpulse = NonNegative(settings.MaxDeformationImpulse, defaults.MaxDeformationImpulse);
			// The chassis carries gravity and rigid motion. Applying either again to this
			// local cage would sag or accelerate the visual shell relative to the body.
			result.Material.Gravity = Vector3::Zero();
			result.Material.GroundEnabled = false;
			return result;
		}
	}

	VehicleDeformationSettings::VehicleDeformationSettings()
		: HalfExtent(1.0f, 0.4f, 2.2f)
		, Cells(4, 2, 8)
		, CabinHeight(0.0f)
		, ShellMass(12.0f)
		, ImpactRadius(0.85f)
		, ImpulseScale(0.1f)
		, MinImpactImpulse(20.0f)
		, MaxDeformationImpulse(40.0f)
	{
		Material.Substeps = 4;
		Material.SolverIterations = 12;
		Material.EdgeCompliance = 3.0e-4f;
		Material.VolumeCompliance = 2.0e-5f;
		Material.Damping = 7.0f;
		Material.Gravity = Vector3::Zero();
		Material.YieldStrain = 0.04f;
		Material.PlasticCreep = 35.0f;
		Material.MaxPlasticStrain = 0.45f;
		Material.GroundEnabled = false;
	}

	VehicleDeformation::VehicleDeformation(const VehicleDeformationSettings& settings)
		: m_Settings(Sanitize(settings))
		, m_Cage(m_Settings.Material)
	{
		m_Settings.Material = m_Cage.GetSettings();
		BuildCage();
		SyncRenderGeometry();
	}

	VehicleDeformation::~VehicleDeformation()
	{
		Detach();
	}

	void VehicleDeformation::BuildCage()
	{
		const int nx = m_Settings.Cells.x, ny = m_Settings.Cells.y, nz = m_Settings.Cells.z;
		const int count = (nx + 1) * (ny + 1) * (nz + 1);
		const float cabinEnd = m_Settings.HalfExtent.z - m_Settings.CrushZoneLength;
		int pinned = 0;
		for (int z = 1; z < nz; ++z)
			if (std::fabs(2.0f * z / nz - 1.0f) <= 0.3f) pinned += (nx - 1) * (ny - 1);
		const float inverseMass = (count - pinned) / m_Settings.ShellMass;
		auto Index = [nx, ny](int x, int y, int z) { return x + (nx + 1) * (y + (ny + 1) * z); };
		for (int z = 0; z <= nz; ++z)
		for (int y = 0; y <= ny; ++y)
		for (int x = 0; x <= nx; ++x)
		{
			Vector3 position((2.0f * x / nx - 1.0f) * m_Settings.HalfExtent.x,
				(2.0f * y / ny - 1.0f) * m_Settings.HalfExtent.y,
				(2.0f * z / nz - 1.0f) * m_Settings.HalfExtent.z);
			const float profile = std::max(0.0f, std::min(1.0f,
				(0.75f - std::fabs(position.z / m_Settings.HalfExtent.z)) / 0.35f));
			// This vertical warp keeps the bottom fixed and preserves positive cell volume.
			position.y += m_Settings.CabinHeight * profile * (float(y) / ny);
			if (m_Settings.CrushZoneLength > 0)
			{
				const float end = std::max(0.0f, (std::abs(position.z) - cabinEnd) / m_Settings.CrushZoneLength);
				const float height = float(y) / ny;
				position.x *= 1.0f - 0.09f * end * end - 0.18f * profile * height * height;
				position.y -= m_Settings.HalfExtent.y * end * end * height * 0.35f;
			}
			const int index = m_Cage.AddParticle(position, inverseMass);
			if (x > 0 && x < nx && y > 0 && y < ny && std::fabs(2.0f * z / nz - 1.0f) <= 0.3f)
				m_Cage.PinParticle(index, position);
		}
		for (int z = 0; z < nz; ++z)
		for (int y = 0; y < ny; ++y)
		for (int x = 0; x < nx; ++x)
		{
			const int a = Index(x, y, z), b = Index(x + 1, y, z);
			const int c = Index(x, y + 1, z), d = Index(x + 1, y + 1, z);
			const int e = Index(x, y, z + 1), f = Index(x + 1, y, z + 1);
			const int g = Index(x, y + 1, z + 1), h = Index(x + 1, y + 1, z + 1);
			const float midZ = 0.5f * (m_Cage.GetParticle(a).Position.z + m_Cage.GetParticle(h).Position.z);
			const bool safetyCell = m_Settings.CrushZoneLength > 0 && std::abs(midZ) <= cabinEnd;
			const float compliance = safetyCell ? 0.0f : m_Settings.Material.VolumeCompliance;
			m_Cage.AddTetrahedron(a, b, d, h, compliance);
			m_Cage.AddTetrahedron(a, d, c, h, compliance);
			m_Cage.AddTetrahedron(a, c, g, h, compliance);
			m_Cage.AddTetrahedron(a, g, e, h, compliance);
			m_Cage.AddTetrahedron(a, e, f, h, compliance);
			m_Cage.AddTetrahedron(a, f, b, h, compliance);
		}
		if (m_Settings.CrushZoneLength > 0)
		{
			// Braced cabin members retain their rest lengths. Toward each bumper
			// the beams buckle more readily in compression than in tension.
			const auto& edges = m_Cage.GetEdges();
			for (int i = 0; i < (int)edges.size(); ++i)
			{
				const float z = 0.5f * (m_Cage.GetParticle(edges[i].ParticleA).Position.z
					+ m_Cage.GetParticle(edges[i].ParticleB).Position.z);
				const float depth = std::max(0.0f, (std::abs(z) - cabinEnd) / m_Settings.CrushZoneLength);
				const float compression = m_Settings.Material.EdgeCompliance * depth;
				m_Cage.SetEdgeCompliance(i, compression, compression * 0.12f);
			}
		}
		std::vector<Index3> triangles;
		auto Quad = [&](int a, int b, int c, int d, const Vector3& outward)
		{
			const Vector3 normal = (m_Cage.GetParticle(b).Position - m_Cage.GetParticle(a).Position)
				.Cross(m_Cage.GetParticle(c).Position - m_Cage.GetParticle(a).Position);
			if (normal.Dot(outward) < 0.0f) std::swap(b, d);
			triangles.emplace_back(a, b, c);
			triangles.emplace_back(a, c, d);
		};
		for (int side = 0; side < 2; ++side)
		{
			const int x = side * nx, y = side * ny, z = side * nz;
			const float sign = side ? 1.0f : -1.0f;
			for (int j = 0; j < ny; ++j)
			for (int k = 0; k < nz; ++k)
				Quad(Index(x, j, k), Index(x, j + 1, k), Index(x, j + 1, k + 1), Index(x, j, k + 1), Vector3(sign, 0, 0));
			for (int i = 0; i < nx; ++i)
			for (int k = 0; k < nz; ++k)
				Quad(Index(i, y, k), Index(i, y, k + 1), Index(i + 1, y, k + 1), Index(i + 1, y, k), Vector3(0, sign, 0));
			for (int i = 0; i < nx; ++i)
			for (int j = 0; j < ny; ++j)
				Quad(Index(i, j, z), Index(i + 1, j, z), Index(i + 1, j + 1, z), Index(i, j + 1, z), Vector3(0, 0, sign));
		}
		m_Cage.SetSurfaceTriangles(triangles);
	}

	bool VehicleDeformation::Attach(RigidBody* body, const Transform& cageToBody)
	{
		if (!body || !body->CastDynamic() || (body->Parent && body->Parent != this)
			|| !ValidTransform(cageToBody) || !ValidTransform(Transform(body->X, body->Q))) return false;
		if (!ValidTransform(Transform(body->BodyLocalToWorld(cageToBody.pos), body->Q * cageToBody.quat.Unit()))) return false;
		Detach();
		m_Body = body;
		m_CageToBody = Transform(cageToBody.pos, cageToBody.quat.Unit());
		body->Parent = this;
		SyncRenderGeometry();
		return true;
	}

	bool VehicleDeformation::CreateFrontRails(PhysicsWorld& world, float yieldForce)
	{
		if (!m_Body || m_Body->ReducedRagdoll || world.IsSimulating() || m_RailWorld || m_Settings.CrushZoneLength <= 0)
			return false;
		std::vector<Geometry*> shapes;
		world.GetAllGeometries(&shapes);
		if (std::none_of(shapes.begin(), shapes.end(), [&](const Geometry* g) { return g->GetParent<RigidBody>() == m_Body; }))
			return false;
		const Transform frame = GetWorldTransform();
		const Vector3 extent = m_Settings.HalfExtent;
		const float stroke = m_Settings.CrushZoneLength * 0.8f;
		const float invMass = 1.0f / (m_Settings.ShellMass * 0.20f);
		ScalarJointDrive drive;
		drive.Settings.PositionEnabled = true;
		drive.Settings.MaxForce = yieldForce;
		drive.Settings.Stiffness = yieldForce / (stroke * 0.04f);
		drive.Settings.Damping = 2.0f * std::sqrt(drive.Settings.Stiffness / invMass);
		if (stroke < 1.0e-4f || !(yieldForce > 0) || !drive.IsValid()) return false;
		m_RailWorld = &world;
		for (int i = 0; i < 2; ++i)
		{
			auto& rail = m_Rails[i];
			const Vector3 halfSize(0.45f * extent.x, 0.32f * extent.y, 0.05f * stroke);
			rail.LocalCenter = Vector3((i ? 1 : -1) * halfSize.x, -0.25f * extent.y, extent.z - halfSize.z);
			const Vector3 center = frame.LocalToWorld(rail.LocalCenter);
			RigidBodyParam param;
			param.rigidType = RigidType::Dynamic;
			param.linearVelocity = m_Body->V + m_Body->W.Cross(center - m_Body->X);
			param.angularVelocity = m_Body->W;
			param.maxContactImpulse = m_Body->CastDynamic()->MaxContactImpulse;
			param.disableGravity = m_Body->CastDynamic()->DisableGravity;
			auto* bumperShape = GeometryFactory::CreateOBB(center, halfSize, frame.quat);
			bumperShape->SetFilterData(m_Body->Geometries().front()->GetFilterData());
			rail.Bumper = world.CreateRigidBody(bumperShape, param);
			rail.Bumper->InvInertia *= invMass / rail.Bumper->InvMass;
			rail.Bumper->InvMass = invMass;
			rail.Bumper->SetRestitution(0);
			rail.Bumper->IgnoreCollisionWith(m_Body);
			rail.Slider = world.CreateSliderConstraint(m_Body, rail.Bumper, center,
				frame.LocalToWorldDirection(Vector3::UnitZ()), -stroke, 0.0f);
			rail.Slider->SetDrive(drive);
		}
		m_Rails[0].Bumper->IgnoreCollisionWith(m_Rails[1].Bumper);
		for (int i = 0; i < m_Cage.GetParticleCount(); ++i)
		{
			const Vector3 p = m_Cage.GetParticle(i).InitialPosition;
			if (p.z == extent.z && p.y <= 0)
			{
				m_BumperNodes.push_back(i);
				m_Cage.PinParticle(i, p);
			}
		}
		return true;
	}

	const RigidBody* VehicleDeformation::GetFrontBumper(int side) const
	{
		return side >= 0 && side < 2 ? m_Rails[side].Bumper : nullptr;
	}

	void VehicleDeformation::UpdateFrontRails()
	{
		float travel[2];
		for (int i = 0; i < 2; ++i)
		{
			auto* slider = m_Rails[i].Slider;
			travel[i] = std::max(slider->GetMinDistance(), std::min(0.0f, slider->GetCurrentPosition()));
			// Elastic-perfectly plastic return mapping: unload elastically from
			// the new rest position after exceeding the yield displacement F/k.
			auto drive = slider->GetDrive();
			const float rest = travel[i] + drive.Settings.MaxForce / drive.Settings.Stiffness;
			if (rest < drive.TargetPosition)
			{
				drive.TargetPosition = rest;
				slider->SetDrive(drive);
			}
		}
		for (int i : m_BumperNodes)
		{
			const Vector3 p = m_Cage.GetParticle(i).InitialPosition;
			const float t = std::max(0.0f, std::min(1.0f, 0.5f + p.x / (2.0f * m_Rails[1].LocalCenter.x)));
			m_Cage.SetPinTarget(i, p + Vector3(0, 0, (1.0f - t) * travel[0] + t * travel[1]));
		}
	}

	void VehicleDeformation::Detach()
	{
		if (m_RailWorld)
		{
			for (auto& rail : m_Rails)
			{
				if (m_RailWorld->RemoveRigidBody(rail.Bumper))
				{
					rail.Bumper->ReleaseGeometries();
					delete rail.Bumper;
				}
				rail = CrushRail();
			}
			for (int i : m_BumperNodes) m_Cage.UnpinParticle(i);
			m_BumperNodes.clear();
			m_RailWorld = nullptr;
		}
		if (m_Body)
		{
			SyncRenderGeometry();
			if (m_Body->Parent == this) m_Body->Parent = nullptr;
			m_Body = nullptr;
		}
		m_Impacts.clear();
	}

	Transform VehicleDeformation::GetWorldTransform() const
	{
		return m_Body ? Transform(m_Body->BodyLocalToWorld(m_CageToBody.pos), m_Body->Q * m_CageToBody.quat)
			: m_LastWorldTransform;
	}

	bool VehicleDeformation::QueueImpact(const Vector3& worldPoint, const Vector3& worldImpulse)
	{
		if (!m_Body || !Finite(worldPoint) || !Finite(worldImpulse)
			|| m_Settings.ImpulseScale <= 0.0f || m_Settings.MaxDeformationImpulse <= 0.0f) return false;
		const double magnitude = Length(worldImpulse);
		if (magnitude <= 0.0 || magnitude < m_Settings.MinImpactImpulse) return false;
		const Transform world = GetWorldTransform();
		Impact impact;
		impact.Point = world.WorldToLocal(worldPoint);
		impact.Impulse = world.WorldToLocalDirection(worldImpulse);
		if (!Finite(impact.Point) || !Finite(impact.Impulse)) return false;
		bool affected = false;
		for (const DeformableParticle& particle : m_Cage.GetParticles())
		{
			if (!particle.Pinned && Length(particle.Position - impact.Point) < m_Settings.ImpactRadius)
			{
				affected = true;
				break;
			}
		}
		if (!affected) return false;
		// Contact arrays are temporary. Capture the local event at callback time,
		// before the chassis can move to a different pose.
		m_Impacts.push_back(impact);
		return true;
	}

	void VehicleDeformation::OnContact(const PhysicsContactInfo& contactInfo)
	{
		if (!m_Body || !contactInfo.ContactPoints || contactInfo.NumContactPoints <= 0) return;
		const bool isA = contactInfo.BodyA == m_Body;
		const bool isB = contactInfo.BodyB == m_Body;
		if (isA == isB) return;
		for (int i = 0; i < contactInfo.NumContactPoints; ++i)
		{
			const PhysicsContactPoint& point = contactInfo.ContactPoints[i];
			QueueImpact(isA ? point.PositionWorldA : point.PositionWorldB, isA ? -point.Impulse : point.Impulse);
		}
	}

	void VehicleDeformation::Simulate(float dt)
	{
		if (!std::isfinite(dt) || dt <= 0.0f || dt / m_Settings.Material.Substeps < 1.0e-8f) return;
		double magnitude = 0.0;
		for (const Impact& impact : m_Impacts) magnitude += Length(impact.Impulse);
		const double scale = magnitude > 0.0 ? std::min(double(m_Settings.ImpulseScale),
			double(m_Settings.MaxDeformationImpulse) / magnitude) : 0.0;
		for (const Impact& impact : m_Impacts)
		{
			const Vector3 impulse(float(impact.Impulse.x * scale), float(impact.Impulse.y * scale), float(impact.Impulse.z * scale));
			m_Cage.AddRadialImpulse(impact.Point, m_Settings.ImpactRadius, impulse);
		}
		m_Impacts.clear();
		if (m_RailWorld) UpdateFrontRails();
		m_Cage.Simulate(dt);
		SyncRenderGeometry();
	}

	void VehicleDeformation::Reset()
	{
		const Transform frame = GetWorldTransform();
		for (const auto& rail : m_Rails)
		{
			if (!rail.Bumper) continue;
			rail.Bumper->X = frame.LocalToWorld(rail.LocalCenter);
			rail.Bumper->Q = frame.quat;
			rail.Bumper->W = m_Body->W;
			rail.Bumper->V = m_Body->V + m_Body->W.Cross(rail.Bumper->X - m_Body->X);
			auto drive = rail.Slider->GetDrive();
			drive.TargetPosition = 0;
			rail.Slider->SetDrive(drive);
		}
		m_Impacts.clear();
		m_Cage.Reset();
		++m_Revision;
		SyncRenderGeometry();
	}

	void VehicleDeformation::SyncRenderGeometry()
	{
		Geometry* geometry = GetRenderGeometry();
		if (!geometry) return;
		const Transform world = GetWorldTransform();
		bool changed = m_LastWorldTransform != world || m_LastPositions.size() != (size_t)m_Cage.GetParticleCount();
		m_LastPositions.resize(m_Cage.GetParticleCount());
		for (int i = 0; i < m_Cage.GetParticleCount(); ++i)
		{
			const Vector3 position = m_Cage.GetParticle(i).Position;
			changed = changed || m_LastPositions[i] != position;
			m_LastPositions[i] = position;
		}
		m_LastWorldTransform = world;
		geometry->SetTransform(world.pos, world.quat);
		geometry->SetSimulationEnabled(false);
		geometry->SetQueryEnabled(false);
		if (changed) ++m_Revision;
	}

	float VehicleDeformation::GetMaxDisplacement() const
	{
		float maximum = 0.0f;
		for (const DeformableParticle& particle : m_Cage.GetParticles())
			maximum = std::max(maximum, (particle.Position - particle.InitialPosition).Length());
		return maximum;
	}

	float VehicleDeformation::GetMaxPlasticStrain() const
	{
		float maximum = 0.0f;
		for (const DeformableEdge& edge : m_Cage.GetEdges())
			maximum = std::max(maximum, std::fabs(edge.RestLength / edge.InitialRestLength - 1.0f));
		return maximum;
	}
}
