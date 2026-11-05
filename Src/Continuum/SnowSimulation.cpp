#include "SnowSimulation.h"
#include "SnowGPUBackend.h"
#include "MPMContact.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>

#if defined(__SSE2__) || defined(_M_X64)
#include <emmintrin.h>
#define RIEMANN_MPM_SSE 1
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define RIEMANN_MPM_NEON 1
#endif

namespace Riemann
{
	using MPM::Vector;
	using MPM::Matrix;
	namespace
	{
		struct Stencil
		{
			int Index[28]{};
			float Weight[28]{}, Offset[3][28]{}, Gradient[3][28]{};
		};
		Stencil MakeStencil(const SnowParticle& P, const SnowSettings& S, int Width, int Height)
		{
			Stencil St;
			const float Inverse = 1 / S.CellSize;
			const Vector Grid = (P.Position - S.BoundsMin) * Inverse;
			int Base[3]; Vector F;
			for (int A = 0; A < 3; ++A) { Base[A] = (int)floorf(Grid[A] - 0.5f); F[A] = Grid[A] - Base[A]; }
			for (int Z = 0; Z < 3; ++Z) for (int Y = 0; Y < 3; ++Y) for (int X = 0; X < 3; ++X)
			{
				const int K = (Z * 3 + Y) * 3 + X;
				const float WX = MPM::Weight(X,F.x), WY = MPM::Weight(Y,F.y), WZ = MPM::Weight(Z,F.z);
				St.Index[K] = ((Base[2] + Z) * Height + Base[1] + Y) * Width + Base[0] + X;
				St.Weight[K] = WX * WY * WZ;
				St.Offset[0][K] = (X-F.x)*S.CellSize; St.Offset[1][K] = (Y-F.y)*S.CellSize; St.Offset[2][K] = (Z-F.z)*S.CellSize;
				if (S.Transfer == MPMTransfer::MLS)
					for (int A = 0; A < 3; ++A) St.Gradient[A][K] = 4 * Inverse * Inverse * St.Weight[K] * St.Offset[A][K];
				else
				{
					St.Gradient[0][K] = MPM::Derivative(X,F.x)*WY*WZ*Inverse;
					St.Gradient[1][K] = WX*MPM::Derivative(Y,F.y)*WZ*Inverse;
					St.Gradient[2][K] = WX*WY*MPM::Derivative(Z,F.z)*Inverse;
				}
			}
			return St;
		}
		// SIMD evaluates four stencil nodes together and scatters their contributions.
		void TransferBatch(const SnowParticle& P, const Matrix& C, const Matrix& Stress,
			const Stencil& S, int K, float* Mass, float Momentum[3][4], float Force[3][4])
		{
#if RIEMANN_MPM_SSE
			const __m128 M = _mm_mul_ps(_mm_loadu_ps(S.Weight + K), _mm_set1_ps(P.Mass));
			_mm_storeu_ps(Mass, M);
			for (int A = 0; A < 3; ++A)
			{
				__m128 Velocity = _mm_set1_ps(P.Velocity[A]), F = _mm_setzero_ps();
				for (int B = 0; B < 3; ++B)
				{
					Velocity = _mm_add_ps(Velocity, _mm_mul_ps(_mm_set1_ps(C(A,B)), _mm_loadu_ps(S.Offset[B] + K)));
					F = _mm_add_ps(F, _mm_mul_ps(_mm_set1_ps(Stress(A,B)), _mm_loadu_ps(S.Gradient[B] + K)));
				}
				_mm_storeu_ps(Momentum[A], _mm_mul_ps(M, Velocity));
				_mm_storeu_ps(Force[A], _mm_mul_ps(_mm_set1_ps(-P.Volume), F));
			}
#elif RIEMANN_MPM_NEON
			const float32x4_t M = vmulq_n_f32(vld1q_f32(S.Weight + K), P.Mass);
			vst1q_f32(Mass, M);
			for (int A = 0; A < 3; ++A)
			{
				float32x4_t Velocity = vdupq_n_f32(P.Velocity[A]), F = vdupq_n_f32(0);
				for (int B = 0; B < 3; ++B)
				{
					Velocity = vaddq_f32(Velocity, vmulq_n_f32(vld1q_f32(S.Offset[B] + K), C(A,B)));
					F = vaddq_f32(F, vmulq_n_f32(vld1q_f32(S.Gradient[B] + K), Stress(A,B)));
				}
				vst1q_f32(Momentum[A], vmulq_f32(M, Velocity));
				vst1q_f32(Force[A], vmulq_n_f32(F, -P.Volume));
			}
#else
			for (int Lane = 0; Lane < 4; ++Lane)
			{
				Mass[Lane] = S.Weight[K+Lane] * P.Mass;
				for (int A = 0; A < 3; ++A)
				{
					float V = P.Velocity[A], F = 0;
					for (int B = 0; B < 3; ++B) { V += C(A,B)*S.Offset[B][K+Lane]; F += Stress(A,B)*S.Gradient[B][K+Lane]; }
					Momentum[A][Lane] = Mass[Lane]*V; Force[A][Lane] = -P.Volume*F;
				}
			}
#endif
		}
	}

	SnowSimulation::SnowSimulation(const SnowSettings& Settings) : m_Settings(Settings) { PrepareGrid(); }
	SnowSimulation::~SnowSimulation() = default;
	bool SnowSimulation::IsBackendAvailable(SnowBackend Backend)
	{
		if (Backend == SnowBackend::CPU) return true;
#if RIEMANN_MPM_SSE || RIEMANN_MPM_NEON
		if (Backend == SnowBackend::SIMD) return true;
#endif
		if (Backend == SnowBackend::CUDA) return IsSnowCUDAAvailable();
		if (Backend == SnowBackend::ComputeShader || Backend == SnowBackend::Metal) return IsSnowComputeAvailable(Backend == SnowBackend::Metal);
		return false;
	}
	bool SnowSimulation::SetBackend(SnowBackend Backend)
	{
		if (!IsBackendAvailable(Backend)) { m_BackendError = "Requested snow backend is unavailable"; return false; }
		std::unique_ptr<SnowGPUBackend> GPU;
		if (Backend == SnowBackend::CUDA) GPU = CreateSnowCUDA();
		if (Backend == SnowBackend::ComputeShader || Backend == SnowBackend::Metal) GPU = CreateSnowCompute();
		if (Backend > SnowBackend::SIMD && !GPU) { m_BackendError = "Snow GPU backend creation failed"; return false; }
		m_GPU = std::move(GPU); m_Backend = Backend; m_BackendError.clear(); return true;
	}
	void SnowSimulation::PrepareGrid()
	{
		const auto& S = m_Settings;
		const auto& M = S.Material;
		const Vector Span = S.BoundsMax - S.BoundsMin;
		if (!std::isfinite(S.CellSize) || S.CellSize <= 0 || !std::isfinite(Span.x+Span.y+Span.z) ||
			Span.x < 6*S.CellSize || Span.y < 6*S.CellSize || Span.z < 6*S.CellSize ||
			(double)Span.x*Span.y*Span.z / (S.CellSize*S.CellSize*S.CellSize) > 2000000 ||
			!std::isfinite(S.Density) || S.Density <= 0 || !std::isfinite(S.FlipRatio) || S.FlipRatio < 0 || S.FlipRatio > 1 ||
			!std::isfinite(S.GroundFriction) || S.GroundFriction < 0 || S.GroundFriction > 2 ||
			!std::isfinite(S.FrameBudgetMilliseconds) || S.FrameBudgetMilliseconds < 0 || S.MaximumSubsteps < 1 || S.MaximumSubsteps > 256 ||
			!std::isfinite(S.Gravity.x+S.Gravity.y+S.Gravity.z) || S.Transfer < MPMTransfer::MPM || S.Transfer > MPMTransfer::MLS ||
			!std::isfinite(M.YoungModulus) || M.YoungModulus <= 0 || !std::isfinite(M.PoissonRatio) || M.PoissonRatio < 0 || M.PoissonRatio > 0.45f ||
			!std::isfinite(M.SnowFraction) || M.SnowFraction < 0 || M.SnowFraction > 1 || !std::isfinite(M.Viscosity) || M.Viscosity < 0 || M.Viscosity > 1000 ||
			!std::isfinite(M.Hardening) || M.Hardening < 0 || M.Hardening > 30 ||
			!std::isfinite(M.CompressionYield) || M.CompressionYield <= 0 || M.CompressionYield > 0.5f ||
			!std::isfinite(M.StretchYield) || M.StretchYield <= 0 || M.StretchYield > 0.5f ||
			!std::isfinite(M.FrictionAngle) || M.FrictionAngle < 0 || M.FrictionAngle > 60)
			throw std::invalid_argument("Invalid 3D MPM settings");
		m_Width = (int)ceilf(Span.x/S.CellSize)+1; m_Height = (int)ceilf(Span.y/S.CellSize)+1; m_Depth = (int)ceilf(Span.z/S.CellSize)+1;
		m_Grid.resize((size_t)m_Width*m_Height*m_Depth);
		for (auto& P : m_Particles) for (int A = 0; A < 3; ++A)
		{
			const float Confined = MPM::Clamp(P.Position[A], S.BoundsMin[A]+1.5f*S.CellSize, S.BoundsMax[A]-1.5f*S.CellSize);
			if (P.Position[A] != Confined) { P.Position[A] = Confined; P.Velocity = {}; P.Affine = {}; }
		}
	}
	void SnowSimulation::Clear()
	{ m_Particles.clear(); m_GPU.reset(); m_PendingTime = 0; m_Statistics = {}; }
	void SnowSimulation::Reset(SnowQuality Quality)
	{
		Clear();
		const bool Sand = m_Settings.Material.SnowFraction < 0.5f;
		// Dense, dry sand stores little recoverable strain. PIC dissipates unresolved
		// grain collisions instead of recycling them as coherent APIC elastic waves.
		m_Settings.Density = Sand ? 1600.0f : 200.0f;
		m_Settings.Material.YoungModulus = Sand ? 2000000.0f : 40000.0f;
		m_Settings.Material.FrictionAngle = Sand ? 32.0f : 35.0f;
		m_Settings.GroundFriction = Sand ? 0.65f : 1.0f;
		m_Settings.Transfer = Sand ? MPMTransfer::MPM : MPMTransfer::MLS;
		m_Settings.FlipRatio = Sand ? 0.0f : 0.95f;
		m_Settings.CellSize = Quality == SnowQuality::Realtime ? (Sand ? 0.35f : 0.6f) : 0.22f;
		// Match the particle contact domain to the visible tray's inner faces.
		m_Settings.BoundsMin.x = m_Settings.BoundsMin.z = -4.0f-1.5f*m_Settings.CellSize;
		m_Settings.BoundsMax.x = m_Settings.BoundsMax.z = 4.0f+1.5f*m_Settings.CellSize;
		m_Settings.BoundsMin.y = -1.5f*m_Settings.CellSize;
		m_Settings.FrameBudgetMilliseconds = Quality == SnowQuality::Realtime ? 6.0f : 0;
		m_Settings.MaximumSubsteps = Quality == SnowQuality::Realtime ? 24 : 64;
		m_Capsule = {}; m_CapsuleTarget = {};
		m_Capsule.HalfLength = 0.85f;
		// Sand is skimmed near the free surface; snow keeps the ground-level plough.
		m_Capsule.Center.y = Sand ? 2.05f : 1.05f;
		m_Capsule.Friction = Sand ? 0.25f : 0.55f;
		AddBox({-3.5f,0.1f,-3.5f},{3.5f,1.2f,3.5f},m_Settings.CellSize*0.5f);
		m_Particles.erase(std::remove_if(m_Particles.begin(),m_Particles.end(),[&](const SnowParticle& P) {
			return (P.Position-MPM::CapsuleClosest(P.Position,m_Capsule)).Length() < m_Capsule.Radius+0.25f*m_Settings.CellSize;
		}),m_Particles.end());
		GatherStatistics();
	}
	void SnowSimulation::AddBox(Vector Minimum, Vector Maximum, float Spacing, Vector Velocity)
	{
		PrepareGrid();
		if (!std::isfinite(Spacing) || Spacing <= 0 || !std::isfinite(Minimum.x+Minimum.y+Minimum.z+Maximum.x+Maximum.y+Maximum.z+Velocity.x+Velocity.y+Velocity.z))
			throw std::invalid_argument("Invalid snow particle seeding parameters");
		for (int A = 0; A < 3; ++A)
		{
			Minimum[A] = fmaxf(Minimum[A],m_Settings.BoundsMin[A]+2*m_Settings.CellSize);
			Maximum[A] = fminf(Maximum[A],m_Settings.BoundsMax[A]-2*m_Settings.CellSize);
			if (Minimum[A] >= Maximum[A]) return;
		}
		const Vector Span = Maximum-Minimum;
		if ((double)Span.x*Span.y*Span.z/(Spacing*Spacing*Spacing)+m_Particles.size() > 1000000)
			throw std::invalid_argument("Snow particle capacity exceeded");
		uint32_t Seed = 0x9e3779b9u;
		const auto Jitter = [&]() { Seed ^= Seed<<13; Seed ^= Seed>>17; Seed ^= Seed<<5; return ((Seed&65535u)/65535.0f-0.5f)*0.3f*Spacing; };
		for (float Z=Minimum.z+0.5f*Spacing;Z<Maximum.z;Z+=Spacing)
			for (float Y=Minimum.y+0.5f*Spacing;Y<Maximum.y;Y+=Spacing)
				for (float X=Minimum.x+0.5f*Spacing;X<Maximum.x;X+=Spacing)
				{
					SnowParticle P; P.Position = {X+Jitter(),Y+Jitter(),Z+Jitter()}; P.Velocity = Velocity;
					P.Volume = Spacing*Spacing*Spacing; P.Mass = P.Volume*m_Settings.Density; m_Particles.push_back(P);
				}
		GatherStatistics();
	}
	void SnowSimulation::StepCPU(float Dt, bool Simd)
	{
		std::fill(m_Grid.begin(),m_Grid.end(),GridNode{});
		for (const auto& P : m_Particles)
		{
			const Stencil St = MakeStencil(P,m_Settings,m_Width,m_Height);
			const Matrix Stress = MPM::Stress(P.Elastic,P.Affine,P.PlasticVolume,m_Settings.Material);
			const Matrix C = m_Settings.Transfer == MPMTransfer::MPM ? Matrix{} : P.Affine;
			if (Simd)
			{
				for (int K=0;K<28;K+=4)
				{
					float Mass[4], Momentum[3][4], Force[3][4]; TransferBatch(P,C,Stress,St,K,Mass,Momentum,Force);
					for (int Lane=0;Lane<4 && K+Lane<27;++Lane)
					{
						auto& Node = m_Grid[St.Index[K+Lane]]; Node.Mass += Mass[Lane];
						for (int A=0;A<3;++A) { Node.Momentum[A] += Momentum[A][Lane]; Node.Force[A] += Force[A][Lane]; }
					}
				}
			}
			else for (int K=0;K<27;++K)
			{
				auto& Node = m_Grid[St.Index[K]];
				const float Mass = St.Weight[K]*P.Mass;
				const Vector Offset(St.Offset[0][K],St.Offset[1][K],St.Offset[2][K]);
				const Vector Gradient(St.Gradient[0][K],St.Gradient[1][K],St.Gradient[2][K]);
				Node.Mass += Mass; Node.Momentum += (P.Velocity+C*Offset)*Mass; Node.Force += (Stress*Gradient)*(-P.Volume);
			}
		}
		const float Dx = m_Settings.CellSize;
		for (int Z=0;Z<m_Depth;++Z) for (int Y=0;Y<m_Height;++Y) for (int X=0;X<m_Width;++X)
		{
			auto& Node = m_Grid[(Z*m_Height+Y)*m_Width+X];
			if (Node.Mass <= 0) continue;
			Node.OldVelocity = Node.Momentum/Node.Mass;
			Vector Velocity = Node.OldVelocity+(Node.Force/Node.Mass+m_Settings.Gravity)*Dt;
			const Vector Position = m_Settings.BoundsMin+Vector((float)X,(float)Y,(float)Z)*Dx;
			const int Coordinate[3] = {X,Y,Z};
			// The first node above ground still transfers momentum from particles at the floor.
			if (Y < 3) Velocity = MPM::GroundContact(Velocity,m_Settings.GroundFriction);
			for (int A=0;A<3;++A)
				if ((Coordinate[A]<2 && Velocity[A]<0) || (Position[A]>m_Settings.BoundsMax[A]-2*Dx && Velocity[A]>0)) Velocity[A]=0;
			Node.Momentum = MPM::Contact(Position,Velocity,m_Capsule,0.25f*Dx);
		}
		for (auto& P : m_Particles)
		{
			const Stencil St = MakeStencil(P,m_Settings,m_Width,m_Height);
			Vector PIC, Change; Matrix Gradient, Affine;
			for (int K=0;K<27;++K)
			{
				const auto& Node = m_Grid[St.Index[K]];
				const Vector Weighted = Node.Momentum*St.Weight[K];
				PIC += Weighted; Change += (Node.Momentum-Node.OldVelocity)*St.Weight[K];
				Gradient = Gradient+MPM::Outer(Node.Momentum,{St.Gradient[0][K],St.Gradient[1][K],St.Gradient[2][K]});
				Affine = Affine+MPM::Outer(Weighted,{St.Offset[0][K],St.Offset[1][K],St.Offset[2][K]})*(4/(Dx*Dx));
			}
			P.Velocity = m_Settings.Transfer == MPMTransfer::MPM ? PIC*(1-m_Settings.FlipRatio)+(P.Velocity+Change)*m_Settings.FlipRatio : PIC;
			P.Affine = Affine;
			P.Elastic = MPM::Project((Matrix::Identity()+Gradient*Dt)*P.Elastic,P.PlasticVolume,m_Settings.Material,&P.LogVolumeCorrection);
			P.Position += PIC*Dt;
			MPM::Confine(P.Position,P.Velocity,m_Settings,m_Capsule);
		}
	}
	void SnowSimulation::GatherStatistics()
	{
		m_Statistics.TotalMass = 0; m_Statistics.Momentum = {}; m_Statistics.MaximumSpeed = 0;
		for (const auto& P : m_Particles)
		{
			if (!std::isfinite(P.Position.x+P.Position.y+P.Position.z+P.Velocity.x+P.Velocity.y+P.Velocity.z+P.Elastic.Determinant()+P.PlasticVolume))
				throw std::runtime_error("3D MPM produced a non-finite particle state");
			m_Statistics.TotalMass += P.Mass; m_Statistics.Momentum += P.Velocity*P.Mass;
			m_Statistics.MaximumSpeed = fmaxf(m_Statistics.MaximumSpeed,P.Velocity.Length());
		}
	}
	void SnowSimulation::Step(float DeltaTime)
	{
		if (!std::isfinite(DeltaTime) || DeltaTime<0) throw std::invalid_argument("Invalid 3D MPM timestep");
		PrepareGrid();
		if (!std::isfinite(m_CapsuleTarget.x+m_CapsuleTarget.z+m_Capsule.Center.x+m_Capsule.Center.y+m_Capsule.Center.z+m_Capsule.Radius+m_Capsule.HalfLength+m_Capsule.Friction) ||
			m_Capsule.Radius<=0 || m_Capsule.HalfLength<0 || m_Capsule.Friction<0) throw std::invalid_argument("Invalid snow capsule");
		const auto Start = std::chrono::steady_clock::now();
		m_Statistics.Substeps = 0; m_Statistics.SimulatedTime = 0;
		m_PendingTime = fminf(m_PendingTime+DeltaTime,0.25f);
		if (m_Particles.empty()) m_PendingTime=0;
		while (m_PendingTime>1.e-7f && m_Statistics.Substeps<m_Settings.MaximumSubsteps && !m_Particles.empty())
		{
			GatherStatistics();
			float Mu,Lambda; MPM::Lame(m_Settings.Material,0.4f,Mu,Lambda);
			const float Wave = sqrtf((Lambda+2*Mu)/m_Settings.Density), Dx=m_Settings.CellSize;
			const float CFL = 0.35f*Dx/(Wave+m_Statistics.MaximumSpeed+5);
			const float Viscous = 0.15f*m_Settings.Density*Dx*Dx/fmaxf(m_Settings.Material.Viscosity,0.001f);
			const float Dt = std::min({m_PendingTime,CFL,Viscous,1.0f/120});
			Vector Target=m_CapsuleTarget; Target.y=m_Capsule.Center.y;
			for (int A : {0,2}) Target[A]=MPM::Clamp(Target[A],m_Settings.BoundsMin[A]+m_Capsule.Radius+2*Dx,m_Settings.BoundsMax[A]-m_Capsule.Radius-2*Dx);
			m_Capsule.Velocity=(Target-m_Capsule.Center)/Dt;
			const float Speed=m_Capsule.Velocity.Length();
			if (Speed>5) m_Capsule.Velocity=m_Capsule.Velocity*(5/Speed);
			m_Capsule.Center+=m_Capsule.Velocity*Dt;
			if (m_Backend>SnowBackend::SIMD)
			{
				if (!m_GPU) m_GPU=m_Backend==SnowBackend::CUDA ? CreateSnowCUDA() : CreateSnowCompute();
				if (!m_GPU || !m_GPU->Step(m_Settings,m_Capsule,Dt,m_Particles,m_BackendError))
				{
					if (m_BackendError.empty()) m_BackendError="Snow GPU backend failed";
					m_Backend=SnowBackend::CPU; m_GPU.reset(); StepCPU(Dt,false);
				}
			}
			else StepCPU(Dt,m_Backend==SnowBackend::SIMD);
			m_PendingTime-=Dt; ++m_Statistics.Substeps; m_Statistics.SimulatedTime+=Dt;
			if (m_Settings.FrameBudgetMilliseconds>0 && std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-Start).count()>=m_Settings.FrameBudgetMilliseconds) break;
		}
		m_Statistics.PendingTime=m_PendingTime; m_Statistics.TimeLimited=m_PendingTime>1.e-5f; GatherStatistics();
	}
}
