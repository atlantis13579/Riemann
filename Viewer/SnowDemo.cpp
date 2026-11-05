#include "Viewer.h"
#include "RenderThread.h"
#include "SnowSurfaceBuilder.h"
#include "../Renderer/imgui.h"
#include <cstdio>
#include <stdexcept>

namespace Riemann
{
	bool WorldViewer::ActivateSnowDemo(bool Sand)
	{
		m_SnowQuality=SnowQuality::Realtime;m_SnowPaused=false;m_SnowStatus.clear();m_SnowMeshReady=false;
		if(Sand)
		{
			m_SnowSimulation.reset();m_SandSimulation.reset(new SandSimulation());m_SandSimulation->Reset(m_SnowQuality);
			if(SandSimulation::IsBackendAvailable(SnowBackend::SIMD)) m_SandSimulation->SetBackend(SnowBackend::SIMD);
		}
		else
		{
			m_SandSimulation.reset();m_SnowSimulation.reset(new SnowSimulation());
			m_SnowSimulation->GetSettings().Material.Viscosity=2;m_SnowSimulation->Reset(m_SnowQuality);
			for(auto Backend:{SnowBackend::CUDA,SnowBackend::Metal,SnowBackend::ComputeShader,SnowBackend::SIMD,SnowBackend::CPU})
				if(SnowSimulation::IsBackendAvailable(Backend) && m_SnowSimulation->SetBackend(Backend)) break;
		}
		return true;
	}
	void WorldViewer::DeactivateSnowDemo()
	{
		if((m_SnowSimulation || m_SandSimulation) && m_RenderThread) m_RenderThread->Submit([](Renderer& R) { R.DeleteMesh("snow.surface");R.DeleteMesh("snow.capsule"); });
		m_SnowSimulation.reset();m_SandSimulation.reset();m_SnowMeshReady=false;m_SnowRenderedFraction=-2;m_SnowStatus.clear();
		m_SnowSolveMilliseconds=m_SnowSurfaceMilliseconds=0;
	}
	void WorldViewer::MoveSnowCapsule(int X,int Y,int Width,int Height)
	{
		if((!m_SnowSimulation && !m_SandSimulation) || Width<=0 || Height<=0 || IsImguiPanelHovered(X,Y,Width,Height)) return;
		const Ray3 Ray=BuildSceneRay(X,Y,Width,Height);
		if(fabsf(Ray.Dir.y)<1.e-5f) return;
		const auto& Capsule=m_SandSimulation ? m_SandSimulation->GetCapsule() : m_SnowSimulation->GetCapsule();
		const float T=(Capsule.Center.y-Capsule.HalfLength-Capsule.Radius-Ray.Origin.y)/Ray.Dir.y;
		if(T<=0 || T>1000) return;
		const Vector3 P=Ray.Origin+Ray.Dir*T;
		if(m_SandSimulation) m_SandSimulation->SetCapsuleTarget(P.x,P.z);else m_SnowSimulation->SetCapsuleTarget(P.x,P.z);
	}
	void WorldViewer::UpdateSnowDemo(float Dt)
	{
		if((!m_SnowSimulation && !m_SandSimulation) || m_SnowPaused) return;
		const auto Start=std::chrono::steady_clock::now();
		try { if(m_SandSimulation) m_SandSimulation->Step(Dt);else m_SnowSimulation->Step(Dt); }
		catch(const std::exception& E) { m_SnowStatus=E.what();m_SnowPaused=true;return; }
		m_SnowSolveMilliseconds=std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-Start).count();
		SubmitSnowSurface();
	}
	void WorldViewer::SubmitSnowSurface()
	{
		if((!m_SnowSimulation && !m_SandSimulation) || !m_RenderThread) return;
		const auto Start=std::chrono::steady_clock::now();
		auto Surface=std::make_shared<RenderMeshDesc>();
		if(!(m_SandSimulation ? BuildSandSurface(*m_SandSimulation,*Surface) : BuildSnowSurface(*m_SnowSimulation,m_SnowQuality,*Surface))) { m_SnowStatus="Material surface capacity exceeded";return; }
		m_SnowSurfaceMilliseconds=std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-Start).count();
		const bool Create=!m_SnowMeshReady,MaterialChanged=m_SnowRenderedFraction!=Surface->SnowFraction;
		const auto& Tool=m_SandSimulation ? m_SandSimulation->GetCapsule() : m_SnowSimulation->GetCapsule();
		RenderMeshDesc Capsule;if(Create) Capsule=BuildSnowCapsule(Tool);
		const auto& C=Tool.Center;
		const Transform CapsuleTransform(Vector3(C.x,C.y,C.z));
		m_RenderThread->Submit([Surface,Create,MaterialChanged,Capsule,CapsuleTransform](Renderer& R) {
			if(Create || MaterialChanged) R.AddMesh(*Surface);
			else R.UpdateMeshGeometry(Surface->Id.c_str(),Surface->Vertices.data(),(int)Surface->Vertices.size(),Surface->Indices.data(),(int)Surface->Indices.size());
			if(Create) R.AddMesh(Capsule);
			RenderTransformUpdate Update;Update.Id="snow.capsule";Update.WorldTransform=CapsuleTransform;R.UpdateTransform(Update);
		});
		m_SnowMeshReady=true;m_SnowRenderedFraction=Surface->SnowFraction;
	}
	void WorldViewer::DrawSnowPanel(SnowPanelState State)
	{
		imguiLabel(State.Fraction < 0.5f ? "3D Sand - Surface Sweep" : "3D Snow");imguiValue("Move mouse over ground");imguiValue("RMB: orbit camera");
		const char* Qualities[]={"Realtime","High detail"};const char* Transfers[]={"MPM / PIC-FLIP","APIC MPM","MLS-MPM"};
		const char* Names[]={"CPU","SIMD","CUDA","Compute Shader","Metal"};
		std::string BackendLabels[5];const char* Backends[5];
		for(int I=0;I<5;++I) { BackendLabels[I]=std::string(Names[I])+(State.Available[I] ? "" : " (unavailable)");Backends[I]=BackendLabels[I].c_str(); }
		bool Change=false;
		imguiLabel("Quality");Change=imguiCombo("",&State.Quality,Qualities,2)||Change;
		imguiValue("Changing quality resets bed");
		if(State.Fraction>=0.5f) { imguiLabel("Transfer");Change=imguiCombo("",&State.Transfer,Transfers,3)||Change; }
		else { imguiValue("Granular height field");imguiValue("Repose angle: 32 degrees"); }
		imguiLabel("Backend");Change=imguiCombo("",&State.Backend,Backends,5)||Change;
		Change=imguiSlider("Viscosity",&State.Viscosity,0,15,0.1f)||Change;
		const bool Reset=imguiButton("Reset material bed");
		if(imguiButton(State.Paused ? "Resume" : "Pause")) { State.Paused=!State.Paused;Change=true; }
		char Text[96];snprintf(Text,sizeof(Text),State.Fraction<0.5f ? "Surface cells: %zu" : "Particles: %zu",State.ParticleCount);imguiValue(Text);
		snprintf(Text,sizeof(Text),"Solve: %.1f ms",State.SolveMilliseconds);imguiValue(Text);
		snprintf(Text,sizeof(Text),"Surface: %.1f ms",State.SurfaceMilliseconds);imguiValue(Text);
		if(State.TimeLimited) imguiValue("Simulation time budget active");
		if(!State.Status.empty()) imguiValue(State.Status.c_str());
		if(Change || Reset)
		{
			std::lock_guard<std::mutex> Lock(m_ImguiMutex);
			if(State.SceneGeneration==m_ImguiSnowPanel.SceneGeneration)
			{ m_ImguiPendingSnowPanel=State;m_ImguiPendingSnowChange=true;m_ImguiPendingSnowReset=m_ImguiPendingSnowReset||Reset; }
		}
		imguiSeparatorLine();
	}
	void WorldViewer::ApplySnowCommands()
	{
		SnowPanelState State;bool Change,Reset;
		{
			std::lock_guard<std::mutex> Lock(m_ImguiMutex);
			State=m_ImguiPendingSnowPanel;Change=m_ImguiPendingSnowChange;Reset=m_ImguiPendingSnowReset;
			m_ImguiPendingSnowChange=m_ImguiPendingSnowReset=false;
		}
		if(!Change || (!m_SnowSimulation && !m_SandSimulation) || State.SceneGeneration!=m_SceneGeneration) return;
		const SnowQuality Quality=(SnowQuality)State.Quality;
		if(m_SandSimulation)
		{
			m_SandSimulation->GetSettings().Viscosity=State.Viscosity;
			if((SnowBackend)State.Backend!=m_SandSimulation->GetBackend()) m_SandSimulation->SetBackend((SnowBackend)State.Backend);
			if(Reset || Quality!=m_SnowQuality) m_SandSimulation->Reset(Quality);
		}
		else
		{
			auto& S=m_SnowSimulation->GetSettings();S.Material.Viscosity=State.Viscosity;S.Transfer=(MPMTransfer)State.Transfer;
			if((SnowBackend)State.Backend!=m_SnowSimulation->GetBackend()) m_SnowSimulation->SetBackend((SnowBackend)State.Backend);
			if(Reset || Quality!=m_SnowQuality) m_SnowSimulation->Reset(Quality);
		}
		m_SnowPaused=State.Paused;
		if(Reset || Quality!=m_SnowQuality) { m_SnowQuality=Quality;m_SnowStatus.clear();m_SnowMeshReady=false; }
		SubmitSnowSurface();
	}
	void WorldViewer::UpdateSnowPanelState()
	{
		m_ImguiSnowPanel.Active=m_SnowSimulation!=nullptr || m_SandSimulation!=nullptr;
		if(m_SandSimulation)
		{
			auto& P=m_ImguiSnowPanel;P.SceneGeneration=m_SceneGeneration;P.Paused=m_SnowPaused;P.Quality=(int)m_SnowQuality;
			P.Backend=(int)m_SandSimulation->GetBackend();P.Fraction=0;P.Viscosity=m_SandSimulation->GetSettings().Viscosity;
			P.ParticleCount=m_SandSimulation->GetHeights().size();P.SolveMilliseconds=m_SnowSolveMilliseconds;P.SurfaceMilliseconds=m_SnowSurfaceMilliseconds;
			P.TimeLimited=false;P.Status=m_SnowStatus;
			if(!m_SandSimulation->GetBackendError().empty()) P.Status=m_SandSimulation->GetBackendError();
			for(int I=0;I<5;++I) P.Available[I]=SandSimulation::IsBackendAvailable((SnowBackend)I);
			return;
		}
		if(!m_SnowSimulation) return;
		auto& P=m_ImguiSnowPanel;const auto& S=m_SnowSimulation->GetSettings();
		P.SceneGeneration=m_SceneGeneration;P.Paused=m_SnowPaused;P.Quality=(int)m_SnowQuality;
		P.Backend=(int)m_SnowSimulation->GetBackend();P.Transfer=(int)S.Transfer;P.Fraction=S.Material.SnowFraction;P.Viscosity=S.Material.Viscosity;
		P.ParticleCount=m_SnowSimulation->GetParticles().size();P.SolveMilliseconds=m_SnowSolveMilliseconds;P.SurfaceMilliseconds=m_SnowSurfaceMilliseconds;
		P.TimeLimited=m_SnowSimulation->GetStatistics().TimeLimited;P.Status=m_SnowStatus;
		if(!m_SnowSimulation->GetBackendError().empty()) P.Status=m_SnowSimulation->GetBackendError();
		for(int I=0;I<5;++I) P.Available[I]=SnowSimulation::IsBackendAvailable((SnowBackend)I);
	}
}
