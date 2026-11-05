#include "SandSimulation.h"
#include "SandGPUBackend.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

#if defined(__SSE2__) || defined(_M_X64)
#include <emmintrin.h>
#define SAND_SSE 1
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define SAND_NEON 1
#endif

namespace Riemann
{
	namespace
	{
		const int OffsetX[8] = {-1,1,0,0,-1,1,-1,1};
		const int OffsetZ[8] = {0,0,-1,1,-1,1,1,-1};
		const int Opposite[8] = {1,0,3,2,5,4,7,6};
		float Clamp(float V,float A,float B) { return std::max(A,std::min(B,V)); }
	}
	SandSimulation::SandSimulation() = default;
	SandSimulation::~SandSimulation() = default;
	bool SandSimulation::IsBackendAvailable(SnowBackend Backend) { return SnowSimulation::IsBackendAvailable(Backend); }
	bool SandSimulation::SetBackend(SnowBackend Backend)
	{
		if(!IsBackendAvailable(Backend)) { m_BackendError="Requested sand backend is unavailable";return false; }
		auto GPU=Backend==SnowBackend::CUDA ? CreateSandCUDA() :
			(Backend==SnowBackend::ComputeShader || Backend==SnowBackend::Metal ? CreateSandCompute() : nullptr);
		if(Backend!=SnowBackend::CPU && Backend!=SnowBackend::SIMD && !GPU)
		{ m_BackendError="Requested sand backend was not built";return false; }
		m_GPU=std::move(GPU);m_Backend=Backend;m_BackendError.clear();return true;
	}
	void SandSimulation::Reset(SnowQuality Quality)
	{
		m_Resolution=Quality==SnowQuality::Realtime ? 64 : 128;
		const int Count=m_Resolution*m_Resolution;
		m_Heights.resize(Count);m_Limits.assign(Count,1.5f);m_Flux.resize(Count*8);m_Next.resize(Count);m_Weights.resize(Count);
		const float Dx=GetCellSize();
		for(int Z=0;Z<m_Resolution;++Z) for(int X=0;X<m_Resolution;++X)
		{
			const float PX=-4+(X+0.5f)*Dx,PZ=-4+(Z+0.5f)*Dx;
			m_Heights[Z*m_Resolution+X]=1.0f+0.015f*sinf(PX*4+PZ*2)*sinf(PZ*3);
		}
		m_Capsule={};m_Capsule.HalfLength=0.85f;m_Capsule.Radius=0.45f;
		m_Capsule.Center={-3,2.05f,0};m_Target=m_Capsule.Center;
		Displace();m_BackendError.clear();
	}
	void SandSimulation::SetCapsuleTarget(float X,float Z)
	{
		if(std::isfinite(X+Z)) m_Target={X,0,Z};
	}
	double SandSimulation::GetVolume() const
	{
		double Volume=0;for(float H:m_Heights) Volume+=H;
		return Volume*GetCellSize()*GetCellSize();
	}
	void SandSimulation::Displace()
	{
		const float Dx=GetCellSize(),Radius=m_Capsule.Radius;
		const float Speed=sqrtf(m_Capsule.Velocity.x*m_Capsule.Velocity.x+m_Capsule.Velocity.z*m_Capsule.Velocity.z);
		double Removed=0,Weights=0;
		for(int Z=0;Z<m_Resolution;++Z) for(int X=0;X<m_Resolution;++X)
		{
			const int I=Z*m_Resolution+X;
			const float RX=-4+(X+0.5f)*Dx-m_Capsule.Center.x,RZ=-4+(Z+0.5f)*Dx-m_Capsule.Center.z;
			const float Distance=sqrtf(RX*RX+RZ*RZ);
			float Limit=1.5f;
			if(m_Capsule.Enabled && Distance<Radius)
				Limit=Clamp(m_Capsule.Center.y-m_Capsule.HalfLength-sqrtf(Radius*Radius-Distance*Distance),0,Limit);
			m_Limits[I]=Limit;
			const float Excess=std::max(0.0f,m_Heights[I]-Limit);m_Heights[I]-=Excess;Removed+=Excess;
			const float Direction=Speed>0.001f && Distance>0 ? (RX*m_Capsule.Velocity.x+RZ*m_Capsule.Velocity.z)/(Distance*Speed) : 0;
			// A swept tool moves grains forward and sideways, never into an adjacent tray.
			const float Weight=m_Capsule.Enabled && Distance>Radius+Dx && Distance<Radius+0.65f ?
				(0.15f+std::max(0.0f,Direction))*std::max(0.0f,1.5f-m_Heights[I]) : 0;
			m_Weights[I]=Weight;Weights+=Weight;
		}
		// Redistribute only into available capacity; no mass is discarded at walls or under the tool.
		for(int Pass=0;Removed>1.e-9 && Pass<4;++Pass)
		{
			if(Weights<1.e-9 || Pass>0)
			{
				Weights=0;
				for(size_t I=0;I<m_Heights.size();++I) { m_Weights[I]=std::max(0.0f,m_Limits[I]-m_Heights[I]);Weights+=m_Weights[I]; }
			}
			if(Weights<Removed-1.e-5) // Global capacity is only required on the fallback pass.
			{
				if(Pass>0) throw std::runtime_error("Sand tray capacity exceeded");
			}
			if(Weights<=0) continue;
			double Deposited=0;
			for(size_t I=0;I<m_Heights.size();++I)
			{
				const float Add=std::min(m_Limits[I]-m_Heights[I],(float)(Removed*m_Weights[I]/Weights));
				m_Heights[I]+=Add;Deposited+=Add;
			}
			Removed=std::max(0.0,Removed-Deposited);
		}
	}
	void SandSimulation::RelaxCPU(int Iterations,float Rate,bool Simd)
	{
		const int N=m_Resolution;
		const float Slope=tanf(m_Settings.ReposeAngle*3.14159265359f/180)*GetCellSize();
		for(int Iteration=0;Iteration<Iterations;++Iteration)
		{
			for(int Z=0;Z<N;++Z) for(int X=0;X<N;++X)
			{
				const int I=Z*N+X;float Difference[8],Flow[8],Sum=0;
				for(int K=0;K<8;++K)
				{
					const int NX=X+OffsetX[K],NZ=Z+OffsetZ[K],J=NZ*N+NX;
					Difference[K]=NX>=0 && NX<N && NZ>=0 && NZ<N && m_Heights[J]<m_Limits[J]-1.e-6f ?
						m_Heights[I]-m_Heights[J]-Slope*(K<4 ? 1.0f : 1.41421356237f) : 0;
				}
				if(Simd)
				{
					for(int K=0;K<8;K+=4)
					{
#if SAND_SSE
						_mm_storeu_ps(Flow+K,_mm_mul_ps(_mm_max_ps(_mm_loadu_ps(Difference+K),_mm_setzero_ps()),_mm_set1_ps(Rate)));
#elif SAND_NEON
						vst1q_f32(Flow+K,vmulq_n_f32(vmaxq_f32(vld1q_f32(Difference+K),vdupq_n_f32(0)),Rate));
#else
						for(int L=0;L<4;++L) Flow[K+L]=std::max(0.0f,Difference[K+L])*Rate;
#endif
					}
				}
				else for(int K=0;K<8;++K) Flow[K]=std::max(0.0f,Difference[K])*Rate;
				for(float F:Flow) Sum+=F;
				const float Scale=Sum>m_Heights[I] ? m_Heights[I]/Sum : 1;
				for(int K=0;K<8;++K) m_Flux[I*8+K]=Flow[K]*Scale;
			}
			for(int Z=0;Z<N;++Z) for(int X=0;X<N;++X)
			{
				const int I=Z*N+X;float H=m_Heights[I];
				for(int K=0;K<8;++K)
				{
					H-=m_Flux[I*8+K];const int NX=X+OffsetX[K],NZ=Z+OffsetZ[K];
					if(NX>=0 && NX<N && NZ>=0 && NZ<N) H+=m_Flux[(NZ*N+NX)*8+Opposite[K]];
				}
				m_Next[I]=std::max(0.0f,H);
			}
			m_Heights.swap(m_Next);
		}
	}
	void SandSimulation::Step(float DeltaTime)
	{
		if(!std::isfinite(DeltaTime) || DeltaTime<=0 || m_Heights.empty()) return;
		if(!std::isfinite(m_Settings.ReposeAngle+m_Settings.Viscosity) || m_Settings.ReposeAngle<1 || m_Settings.ReposeAngle>60 || m_Settings.Viscosity<0)
			throw std::invalid_argument("Invalid sand friction or viscosity");
		const float Dt=std::min(DeltaTime,1.0f/15),Dx=GetCellSize(),Margin=m_Capsule.Radius+Dx;
		const MPM::Vector Target(Clamp(m_Target.x,-4+Margin,4-Margin),m_Capsule.Center.y,Clamp(m_Target.z,-4+Margin,4-Margin));
		MPM::Vector Delta=Target-m_Capsule.Center;const float Distance=Delta.Length();
		if(Distance>5*Dt) Delta=Delta*(5*Dt/Distance);
		const int Steps=std::max(1,(int)ceilf(Delta.Length()/(Dx*0.5f)));
		m_Capsule.Velocity=Delta/Dt;
		for(int I=0;I<Steps;++I) { m_Capsule.Center=m_Capsule.Center+Delta/(float)Steps;Displace(); }
		// Explicit transport scales with dx squared, keeping relaxation speed consistent across quality levels.
		const float Frequency=m_Resolution==64 ? 120.0f : 480.0f;
		const int Iterations=std::max(1,(int)ceilf(Dt*Frequency));
		const float Rate=0.12f*std::min(1.0f,Dt*Frequency/Iterations)/(1+0.25f*m_Settings.Viscosity);
		if(m_GPU)
		{
			const float Slope=tanf(m_Settings.ReposeAngle*3.14159265359f/180)*Dx;
			if(!m_GPU->Relax(m_Heights,m_Limits,m_Resolution,Slope,Rate,Iterations,m_BackendError))
			{ m_GPU.reset();m_Backend=SnowBackend::CPU;RelaxCPU(Iterations,Rate,false); }
		}
		else RelaxCPU(Iterations,Rate,m_Backend==SnowBackend::SIMD);
		Displace();
	}
#if !defined(RIEMANN_SNOW_CUDA)
	std::unique_ptr<SandGPUBackend> CreateSandCUDA() { return {}; }
#endif
#if !defined(RIEMANN_SNOW_COMPUTE)
	std::unique_ptr<SandGPUBackend> CreateSandCompute() { return {}; }
#endif
}
