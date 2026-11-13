#include "DeformableBody.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Riemann
{
	/* Material indices select an alternative constitutive route per tetrahedron.
	   An auto-generated edge is active if ANY incident element still uses the
	   legacy model. Explicitly authored springs remain active by user choice.
	   Switching a material never rebases its rest shape to the current deformation.
	   Plastic history remains in the legacy springs/volumes; these FEM models are
	   elastic. Density assignment is an explicit operation and preserves pin flags.
	   Assigning heterogeneous densities lumps each element's mass equally onto its
	   four vertices. Material edits do not silently change momentum through mass.
	   Prepared shapes and nonlinear state are allocated only for assigned elements;
	   the active list also keeps the legacy-only step out of those caches. */
	int DeformableBody::AddMaterial(const TetrahedralMaterial& Material)
	{
		RequireIdle(); if (!Material.IsValid()) throw std::invalid_argument("Invalid tetrahedral material");
		m_Materials.push_back(Material); return (int)m_Materials.size() - 1;
	}
	void DeformableBody::SetMaterial(int Index, const TetrahedralMaterial& Material)
	{
		RequireIdle(); if (Index < 0 || (size_t)Index >= m_Materials.size() || !Material.IsValid()) throw std::invalid_argument("Invalid material edit");
		m_Materials[Index] = Material;
		for (int I : m_ActiveMaterialTetrahedra) if (m_Tetrahedra[I].Material == Index) m_MaterialElements[I]->State = TetrahedralMaterialState();
	}
	void DeformableBody::RefreshElasticEdges()
	{
		for (auto& Edge : m_Edges) Edge.Enabled = Edge.Explicit;
		for (const auto& Tet : m_Tetrahedra) if (Tet.Material < 0) for (int I : Tet.Edges) if (I >= 0) m_Edges[I].Enabled = true;
		m_ActiveMaterialTetrahedra.clear();
		for (size_t I = 0; I < m_Tetrahedra.size(); ++I) if (m_Tetrahedra[I].Material >= 0) m_ActiveMaterialTetrahedra.push_back((int)I);
	}
	void DeformableBody::SetTetrahedronMaterial(int Tetrahedron, int Material)
	{
		RequireIdle();
		if (Tetrahedron < 0 || (size_t)Tetrahedron >= m_Tetrahedra.size() || Material < -1 || (Material >= 0 && (size_t)Material >= m_Materials.size()))
			throw std::invalid_argument("Invalid tetrahedral material assignment");
		auto& Tet = m_Tetrahedra[Tetrahedron];
		if (Material >= 0)
		{
			Vector3 Rest[4]; for (int I = 0; I < 4; ++I) Rest[I] = m_Particles[Tet.Particles[I]].InitialPosition;
			const auto Shape = TetrahedralRestShape::Create(Rest, m_Settings.MaterialSolver);
			if (!m_MaterialElements[Tetrahedron]) m_MaterialElements[Tetrahedron].reset(new MaterialElement);
			m_MaterialElements[Tetrahedron]->Shape = Shape; m_MaterialElements[Tetrahedron]->State = TetrahedralMaterialState();
		}
		else m_MaterialElements[Tetrahedron].reset();
		Tet.Material = Material; Tet.Lambda = 0; RefreshElasticEdges();
	}
	void DeformableBody::ApplyMaterialDensities(double LegacyDensity)
	{
		RequireIdle(); if (!std::isfinite(LegacyDensity) || LegacyDensity <= 0) throw std::invalid_argument("Invalid legacy element density");
		std::vector<double> Masses(m_Particles.size(), 0); std::vector<float> InverseMasses(m_Particles.size());
		for (const auto& Tet : m_Tetrahedra)
		{
			Vector3 Rest[4]; for (int I = 0; I < 4; ++I) Rest[I] = m_Particles[Tet.Particles[I]].InitialPosition;
			const auto Shape = TetrahedralRestShape::Create(Rest, m_Settings.MaterialSolver);
			const double Density = Tet.Material < 0 ? LegacyDensity : m_Materials[Tet.Material].Density;
			const double Mass = Shape.Volume * Density / 4;
			if (!std::isfinite(Mass) || Mass <= 0) throw std::overflow_error("Tetrahedral element mass overflow");
			for (int I : Tet.Particles) Masses[I] += Mass;
		}
		for (size_t I = 0; I < Masses.size(); ++I)
		{
			InverseMasses[I] = m_Particles[I].InverseMass;
			if (Masses[I] > 0)
			{
				InverseMasses[I] = (float)(1 / Masses[I]);
				if (!std::isfinite(Masses[I]) || !std::isfinite(InverseMasses[I]) || InverseMasses[I] <= 0) throw std::overflow_error("Tetrahedral density mass overflow");
			}
		}
		for (size_t I = 0; I < Masses.size(); ++I) m_Particles[I].InverseMass = InverseMasses[I];
	}
	TetrahedralMaterialResponse DeformableBody::GetTetrahedronMaterialResponse(int Tetrahedron) const
	{
		const auto& Tet = m_Tetrahedra.at(Tetrahedron); if (Tet.Material < 0) throw std::invalid_argument("Element has no constitutive material");
		Vector3 Positions[4]; for (int I = 0; I < 4; ++I) Positions[I] = m_Particles[Tet.Particles[I]].Position;
		return TetrahedralElasticity::Evaluate(GetTetrahedronRestShape(Tetrahedron).Evaluate(Positions), m_Materials[Tet.Material]);
	}
	const TetrahedralRestShape& DeformableBody::GetTetrahedronRestShape(int Tetrahedron) const
	{
		const auto& Element = m_MaterialElements.at(Tetrahedron);
		if (!Element) throw std::invalid_argument("Element has no prepared material rest shape");
		return Element->Shape;
	}
	double DeformableBody::GetMaterialEnergy() const
	{
		double Energy = 0;
		for (int I : m_ActiveMaterialTetrahedra) Energy += m_MaterialElements[I]->Shape.Volume * GetTetrahedronMaterialResponse(I).EnergyDensity;
		return Energy;
	}
	void DeformableBody::SetProjectiveMaterialSolver(bool Enabled)
	{
		RequireIdle(); if (m_Settings.ProjectiveMaterials == Enabled) return;
		m_Settings.ProjectiveMaterials = Enabled;
		if (m_ProjectiveMaterials) m_ProjectiveMaterials->Solver.End();
		for (int I : m_ActiveMaterialTetrahedra) m_MaterialElements[I]->State = TetrahedralMaterialState();
	}
	const ProjectiveDynamicsStats& DeformableBody::GetProjectiveMaterialStats() const
	{
		static const ProjectiveDynamicsStats Empty;
		return m_ProjectiveMaterials ? m_ProjectiveMaterials->Solver.GetStats() : Empty;
	}
	void DeformableBody::BeginMaterialStep(float Dt)
	{
		if (m_ProjectiveMaterials) { m_ProjectiveMaterials->Solver.End(); m_ProjectiveMaterials->Elements.clear(); }
		if (!m_Settings.ProjectiveMaterials) return;
		bool Active = false; for (int I : m_ActiveMaterialTetrahedra) if (m_Materials[m_Tetrahedra[I].Material].Model == TetrahedralMaterialModel::Corotated) Active = true;
		if (!Active) return;
		if (!m_ProjectiveMaterials) m_ProjectiveMaterials.reset(new ProjectiveMaterialData);
		auto& Data = *m_ProjectiveMaterials; Data.Particles.resize(m_Particles.size()); Data.Elements.clear();
		for (size_t I = 0; I < m_Particles.size(); ++I) Data.Particles[I] = { &m_Particles[I].Position, GetInverseMass((int)I) };
		for (int I : m_ActiveMaterialTetrahedra)
		{
			const auto& Tet = m_Tetrahedra[I]; const auto& Material = m_Materials[Tet.Material];
			if (Material.Model != TetrahedralMaterialModel::Corotated) continue;
			ProjectiveDynamicsElement Element; std::copy(Tet.Particles,Tet.Particles+4,Element.Particles.begin());
			Element.Rest = m_MaterialElements[I]->Shape; Element.ShearModulus = Material.ShearModulus(); Data.Elements.push_back(Element);
		}
		Data.Solver.Begin(Data.Particles,Data.Elements,Dt,m_Settings.ProjectiveSolver,m_Settings.MaterialSolver);
	}
	void DeformableBody::SolveMaterials(float Dt)
	{
		if (m_Settings.ProjectiveMaterials && m_ProjectiveMaterials && !m_ProjectiveMaterials->Elements.empty()) m_ProjectiveMaterials->Solver.Project();
		for (int Index : m_ActiveMaterialTetrahedra)
		{
			auto& Tet = m_Tetrahedra[Index]; auto& Element = *m_MaterialElements[Index];
			Vector3* Positions[4]; float Masses[4];
			for (int I = 0; I < 4; ++I) { Positions[I] = &m_Particles[Tet.Particles[I]].Position; Masses[I] = GetInverseMass(Tet.Particles[I]); }
			const auto& Material = m_Materials[Tet.Material];
			const auto Channels = m_Settings.ProjectiveMaterials && Material.Model == TetrahedralMaterialModel::Corotated ? TetrahedralSolveChannels::Volume : TetrahedralSolveChannels::All;
			TetrahedralElasticity::Solve(Positions, Masses, Element.Shape, Material, Element.State, Dt, m_Settings.MaterialSolver, Channels);
		}
	}
}
