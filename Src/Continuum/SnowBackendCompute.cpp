#include "SnowGPUBackend.h"
#include "ComputeDevice.h"
#include "SnowKernels.h"
#include <stdexcept>

#if defined(RIEMANN_SNOW_COMPUTE)
namespace Riemann
{
	namespace
	{
		using namespace SnowCompute;
		class ComputeSnow final : public SnowGPUBackend
		{
			std::unique_ptr<Device> m_Device;
			SharedBuffer m_Particles,m_Grid,m_Heads,m_Next,m_Stress;
			SharedProgram m_Clear,m_Prepare,m_ToGrid,m_ToParticles;
			std::vector<SnowParticle> m_Readback;
		public:
			bool Step(const SnowSettings& S,const SnowCapsule& Capsule,float Dt,std::vector<SnowParticle>& Particles,std::string& Error) override
			{
				try
				{
					if(!m_Device)
					{
						m_Device=CreateDevice();if(!m_Device) throw std::runtime_error("Snow compute support was not built");
						m_Clear=m_Device->CreateProgram(SnowKernels,"SnowClear");m_Prepare=m_Device->CreateProgram(SnowKernels,"SnowPrepare");
						m_ToGrid=m_Device->CreateProgram(SnowKernels,"SnowGrid");m_ToParticles=m_Device->CreateProgram(SnowKernels,"SnowParticles");
					}
					if(Particles.empty()) return true;
					const int Count=(int)Particles.size(),Width=(int)ceilf((S.BoundsMax.x-S.BoundsMin.x)/S.CellSize)+1;
					const int Height=(int)ceilf((S.BoundsMax.y-S.BoundsMin.y)/S.CellSize)+1,Depth=(int)ceilf((S.BoundsMax.z-S.BoundsMin.z)/S.CellSize)+1;
					const int Cells=Width*Height*Depth;Constants P;
					P.Integer(0,Count);P.Integer(1,Width);P.Integer(2,Height);P.Integer(3,Depth);
					for(int A=0;A<3;++A) { P.Float(4+A,S.BoundsMin[A]);P.Float(8+A,S.BoundsMax[A]);P.Float(12+A,S.Gravity[A]);P.Float(28+A,Capsule.Center[A]);P.Float(32+A,Capsule.Velocity[A]); }
					P.Float(7,S.CellSize);P.Float(11,Dt);P.Integer(15,(int)S.Transfer);
					const auto& M=S.Material;
					P.Float(16,M.YoungModulus);P.Float(17,M.PoissonRatio);P.Float(18,M.SnowFraction);P.Float(19,M.Viscosity);
					P.Float(20,M.Hardening);P.Float(21,M.CompressionYield);P.Float(22,M.StretchYield);P.Float(23,M.FrictionAngle);
					P.Float(24,S.FlipRatio);P.Integer(25,Capsule.Enabled ? 1 : 0);P.Float(26,Capsule.Radius);P.Float(27,Capsule.HalfLength);P.Float(31,Capsule.Friction);
					P.Float(35,S.GroundFriction);
					static_assert(sizeof(SnowParticle)==28*sizeof(float),"Snow shader particle layout mismatch");
					Reserve(*m_Device,m_Particles,Count*sizeof(SnowParticle));Reserve(*m_Device,m_Grid,Cells*8*sizeof(float));
					Reserve(*m_Device,m_Heads,Cells*sizeof(int));Reserve(*m_Device,m_Next,Count*sizeof(int));Reserve(*m_Device,m_Stress,Count*9*sizeof(float));
					m_Readback.resize(Count);m_Device->Upload(m_Particles,Particles.data(),Count*sizeof(SnowParticle));
					const Bindings B{m_Particles,m_Grid,m_Heads,m_Next,m_Stress};
					m_Device->Dispatch(m_Clear,B,P,Cells);m_Device->Dispatch(m_Prepare,B,P,Count);
					m_Device->Dispatch(m_ToGrid,B,P,Cells);m_Device->Dispatch(m_ToParticles,B,P,Count);
					m_Device->Download({{m_Particles,m_Readback.data(),Count*sizeof(SnowParticle)}});
					for(const auto& Particle:m_Readback)
						if(!std::isfinite(Particle.Position.x+Particle.Position.y+Particle.Position.z+Particle.Velocity.x+Particle.Velocity.y+Particle.Velocity.z+Particle.Elastic.Determinant()+Particle.PlasticVolume))
							throw std::runtime_error("Compute snow produced a non-finite state");
					Particles.swap(m_Readback);Error.clear();return true;
				}
				catch(const std::exception& E) { if(m_Device) m_Device->Finish();Error=E.what();return false; }
			}
		};
	}
	bool IsSnowComputeAvailable(bool MetalOnly)
	{
		static const SnowCompute::BackendInfo Info=[] {
			try { auto Device=SnowCompute::CreateDevice();if(Device) return Device->GetInfo(); }
			catch(const std::exception&) {}
			return SnowCompute::BackendInfo{};
		}();
		return Info.Available && (!MetalOnly || Info.API=="Metal");
	}
	std::unique_ptr<SnowGPUBackend> CreateSnowCompute() { return std::make_unique<ComputeSnow>(); }
}
#endif
