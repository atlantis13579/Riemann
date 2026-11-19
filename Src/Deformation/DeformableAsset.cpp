#include "DeformableBody.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace Riemann
{
	/* Initialization adopts validated connectivity in a single batch. Repeated
	   AddTetrahedron/SetMaterial calls would rescan edges and material membership
	   for each element. Prepared edges are indexed once, and per-instance state is
	   separate from the immutable shared asset. Placement recomputes world-rest
	   gradients from the actual float positions, rejecting precision collapse.
	   Materials and pin states remain editable; asset topology remains immutable.
	   Render selection changes only the existing render geometry, preserving its
	   identity and every simulation particle/contact/attachment. */
	void DeformableBody::InitializeAsset(const TetrahedralAsset& Asset, const Transform& Pose)
	{
		RequireIdle();
		if (!Asset.IsValid() || !m_Particles.empty() || !m_Materials.empty() || !m_Attachments.empty() || !m_AttachmentUsers.empty() || m_RigidAttachmentCount)
			throw std::invalid_argument("Initialize a prepared asset on an empty deformable body");
		const double Q2 = (double)Pose.quat.x * Pose.quat.x + (double)Pose.quat.y * Pose.quat.y + (double)Pose.quat.z * Pose.quat.z + (double)Pose.quat.w * Pose.quat.w;
		constexpr double PlacementQuaternionTolerance = 1e-4;
		if (!std::isfinite(Q2) || std::abs(Q2 - 1) > PlacementQuaternionTolerance || !std::isfinite(Pose.pos.x) || !std::isfinite(Pose.pos.y) || !std::isfinite(Pose.pos.z))
			throw std::invalid_argument("Invalid prepared asset placement");
		std::vector<DeformableParticle> Particles; Particles.reserve(Asset.GetPositions().size());
		for (size_t I = 0; I < Asset.GetPositions().size(); ++I)
		{
			DeformableParticle P; P.Position = Pose.LocalToWorld(Asset.GetPositions()[I]);
			if (!std::isfinite(P.Position.x) || !std::isfinite(P.Position.y) || !std::isfinite(P.Position.z)) throw std::overflow_error("Asset placement position overflow");
			P.PreviousPosition = P.InitialPosition = P.PinPosition = P.Position; P.Velocity = P.Force = Vector3::Zero();
			P.InverseMass = Asset.GetInverseMasses()[I]; P.Pinned = Asset.GetPins()[I]; Particles.push_back(P);
		}
		std::map<std::array<int, 2>, int> EdgeIndices; std::vector<DeformableEdge> Edges; Edges.reserve(Asset.GetEdges().size());
		for (const auto& E : Asset.GetEdges())
		{
			DeformableEdge Edge; Edge.ParticleA = E[0]; Edge.ParticleB = E[1]; Edge.Explicit = Edge.Enabled = false;
			Edge.RestLength = Edge.InitialRestLength = (Particles[E[1]].Position - Particles[E[0]].Position).Length();
			if (!std::isfinite(Edge.RestLength) || Edge.RestLength <= 0) throw std::invalid_argument("Asset placement collapses an edge");
			Edge.TensionCompliance = Edge.Compliance = m_Settings.EdgeCompliance; Edge.Lambda = 0;
			EdgeIndices.emplace(E, (int)Edges.size()); Edges.push_back(Edge);
		}
		std::vector<DeformableTetrahedron> Tets; std::vector<std::unique_ptr<MaterialElement>> MaterialElements;
		Tets.reserve(Asset.GetElements().size()); MaterialElements.reserve(Asset.GetElements().size());
		for (const auto& E : Asset.GetElements())
		{
			DeformableTetrahedron Tet; Vector3 P[4];
			for (int I = 0; I < 4; ++I) { Tet.Particles[I] = E.Vertices[I]; P[I] = Particles[E.Vertices[I]].Position; }
			const auto Double = [](const Vector3& V) { return DeformationGradient::DVector(V.x, V.y, V.z); };
			if ((Double(P[1]) - Double(P[0])).Dot((Double(P[2]) - Double(P[0])).Cross(Double(P[3]) - Double(P[0]))) <= 0)
				throw std::invalid_argument("Asset placement changes tetrahedral orientation");
			std::unique_ptr<MaterialElement> Material(new MaterialElement); Material->Shape = TetrahedralRestShape::Create(P, m_Settings.MaterialSolver);
			double Longest = 0; for (int I = 0; I < 4; ++I) for (int J = I + 1; J < 4; ++J)
			{ const auto D = Double(P[I]) - Double(P[J]); Longest = std::max(Longest, std::hypot(std::hypot(D.x, D.y), D.z)); }
			if (6 * std::sqrt(2.0) * Material->Shape.Volume / (Longest * Longest * Longest) < Asset.GetSettings().MinimumQuality)
				throw std::invalid_argument("Asset placement loses element quality");
			Tet.RestVolume = Tet.InitialRestVolume = (float)Material->Shape.Volume; Tet.Compliance = m_Settings.VolumeCompliance; Tet.Lambda = 0; Tet.Material = E.Material;
			if (!std::isfinite(Tet.RestVolume) || Tet.RestVolume <= 0) throw std::overflow_error("Asset placement volume overflow");
			int Slot = 0; for (int I = 0; I < 4; ++I) for (int J = I + 1; J < 4; ++J)
			{ std::array<int, 2> Key = { Tet.Particles[I], Tet.Particles[J] }; std::sort(Key.begin(), Key.end()); Tet.Edges[Slot++] = EdgeIndices.at(Key); }
			Tets.push_back(Tet); MaterialElements.push_back(std::move(Material));
		}
		// Allocate all fallible auxiliary arrays before publishing the new topology.
		auto Materials = Asset.GetMaterials(); auto Surface = Asset.GetBoundary(); std::vector<unsigned char> Ground(Particles.size(), 0); std::vector<float> Corrections(Particles.size(), 0);
		std::vector<int> Active; Active.reserve(Tets.size()); for (size_t I = 0; I < Tets.size(); ++I) Active.push_back((int)I);
		m_Asset = Asset; m_Particles.swap(Particles); m_Edges.swap(Edges); m_Tetrahedra.swap(Tets); m_Materials.swap(Materials);
		m_MaterialElements.swap(MaterialElements); m_ActiveMaterialTetrahedra.swap(Active); m_SurfaceTriangles.swap(Surface);
		m_GroundContacts.swap(Ground); m_GroundCorrections.swap(Corrections); m_RenderTopologyDirty = true; UpdateRenderGeometry();
	}
	void DeformableBody::UseEmbeddedRenderMesh(bool Enabled)
	{
		RequireIdle(); if (!m_Asset.IsValid()) throw std::logic_error("Embedded rendering requires a prepared asset");
		if (m_EmbeddedRenderMesh == Enabled) return;
		m_EmbeddedRenderMesh = Enabled; m_RenderTopologyDirty = true; UpdateRenderGeometry();
	}
}
