#include "SnowGPUBackend.h"
#include "MPMContact.h"
#include <cuda_runtime.h>
#include <stdexcept>

namespace Riemann
{
	namespace
	{
		using MPM::Vector;
		using MPM::Matrix;
		void Check(cudaError_t Status) { if (Status != cudaSuccess) throw std::runtime_error(cudaGetErrorString(Status)); }
		int SnowDevice()
		{
			static const int Device = [] {
				int Count = 0; if (cudaGetDeviceCount(&Count) != cudaSuccess) return -1;
				for (int I = 0; I < Count; ++I) { cudaDeviceProp P{}; if (cudaGetDeviceProperties(&P,I) == cudaSuccess && P.major >= 7 && P.maxThreadsPerBlock >= 256) return I; }
				return -1;
			}();
			return Device;
		}
		template<class T> struct Buffer
		{
			T* Data = nullptr; size_t Capacity = 0;
			~Buffer() { if (Data) cudaFree(Data); }
			void Reserve(size_t Count)
			{
				if (Count <= Capacity) return;
				T* Next = nullptr; Check(cudaMalloc((void**)&Next,Count*sizeof(T)));
				if (Data) cudaFree(Data); Data = Next; Capacity = Count;
			}
		};
		struct Node { Vector Velocity, OldVelocity; float Mass = 0; };
		__global__ void Prepare(const SnowParticle* Particles, Matrix* Stresses, int* Heads, int* Next, int Count, int Width, int Height, SnowSettings S)
		{
			const int Id=blockIdx.x*blockDim.x+threadIdx.x; if(Id>=Count) return;
			const auto& P=Particles[Id]; Stresses[Id]=MPM::Stress(P.Elastic,P.Affine,P.PlasticVolume,S.Material);
			const Vector G=(P.Position-S.BoundsMin)/S.CellSize;
			const int X=(int)floorf(G.x-0.5f), Y=(int)floorf(G.y-0.5f), Z=(int)floorf(G.z-0.5f);
			Next[Id]=atomicExch(Heads+(Z*Height+Y)*Width+X,Id);
		}
		__global__ void ToGrid(const SnowParticle* Particles, const Matrix* Stresses, const int* Heads, const int* Next,
			Node* Grid, int Count, int Width, int Height, int Depth, float Dt, SnowSettings S, SnowCapsule Capsule)
		{
			const int Id=blockIdx.x*blockDim.x+threadIdx.x; if(Id>=Width*Height*Depth) return;
			const int X=Id%Width,Y=(Id/Width)%Height,Z=Id/(Width*Height);
			const float Inverse=1/S.CellSize;
			float Mass=0;Vector Momentum,Force;
			for(int BZ=max(0,Z-2);BZ<=Z;++BZ) for(int BY=max(0,Y-2);BY<=Y;++BY) for(int BX=max(0,X-2);BX<=X;++BX)
			{
				int Visits=0;
				for(int Index=Heads[(BZ*Height+BY)*Width+BX];Index>=0 && Index<Count && Visits++<Count;Index=Next[Index])
				{
					const auto& P=Particles[Index];
					const Vector F=(P.Position-S.BoundsMin)*Inverse-Vector((float)BX,(float)BY,(float)BZ);
					const int I=X-BX,J=Y-BY,K=Z-BZ;
					const float WX=MPM::Weight(I,F.x),WY=MPM::Weight(J,F.y),WZ=MPM::Weight(K,F.z),W=WX*WY*WZ;
					const Vector D=(Vector((float)I,(float)J,(float)K)-F)*S.CellSize;
					const Vector Gradient=S.Transfer==MPMTransfer::MLS ? D*(4*Inverse*Inverse*W) :
						Vector(MPM::Derivative(I,F.x)*WY*WZ,WX*MPM::Derivative(J,F.y)*WZ,WX*WY*MPM::Derivative(K,F.z))*Inverse;
					const Matrix C=S.Transfer==MPMTransfer::MPM ? Matrix{} : P.Affine;
					const float Contribution=W*P.Mass;
					Mass+=Contribution; Momentum+=(P.Velocity+C*D)*Contribution; Force+=(Stresses[Index]*Gradient)*(-P.Volume);
				}
			}
			Node N;N.Mass=Mass;
			if(Mass>0)
			{
				N.OldVelocity=Momentum/Mass;Vector V=N.OldVelocity+(Force/Mass+S.Gravity)*Dt;
				const Vector Position=S.BoundsMin+Vector((float)X,(float)Y,(float)Z)*S.CellSize;
				const int Coordinate[3]={X,Y,Z};
				// Include the first node above ground in the floor contact layer.
				if(Y<3) V=MPM::GroundContact(V,S.GroundFriction);
				for(int A=0;A<3;++A) if((Coordinate[A]<2 && V[A]<0) || (Position[A]>S.BoundsMax[A]-2*S.CellSize && V[A]>0)) V[A]=0;
				N.Velocity=MPM::Contact(Position,V,Capsule,0.25f*S.CellSize);
			}
			Grid[Id]=N;
		}
		__global__ void ToParticles(SnowParticle* Particles, const Node* Grid, int Count, int Width, int Height, float Dt, SnowSettings S, SnowCapsule Capsule)
		{
			const int Id=blockIdx.x*blockDim.x+threadIdx.x;if(Id>=Count) return;
			SnowParticle P=Particles[Id];const float Inverse=1/S.CellSize;
			const Vector G=(P.Position-S.BoundsMin)*Inverse;
			const int BX=(int)floorf(G.x-0.5f),BY=(int)floorf(G.y-0.5f),BZ=(int)floorf(G.z-0.5f);
			const Vector F=G-Vector((float)BX,(float)BY,(float)BZ);
			Vector PIC,Change;Matrix Gradient,Affine;
			for(int K=0;K<3;++K) for(int J=0;J<3;++J) for(int I=0;I<3;++I)
			{
				const Node N=Grid[((BZ+K)*Height+BY+J)*Width+BX+I];
				const float WX=MPM::Weight(I,F.x),WY=MPM::Weight(J,F.y),WZ=MPM::Weight(K,F.z),W=WX*WY*WZ;
				const Vector D=(Vector((float)I,(float)J,(float)K)-F)*S.CellSize;
				const Vector DW=S.Transfer==MPMTransfer::MLS ? D*(4*Inverse*Inverse*W) :
					Vector(MPM::Derivative(I,F.x)*WY*WZ,WX*MPM::Derivative(J,F.y)*WZ,WX*WY*MPM::Derivative(K,F.z))*Inverse;
				const Vector Weighted=N.Velocity*W;PIC+=Weighted;Change+=(N.Velocity-N.OldVelocity)*W;
				Gradient=Gradient+MPM::Outer(N.Velocity,DW);Affine=Affine+MPM::Outer(Weighted,D)*(4*Inverse*Inverse);
			}
			P.Velocity=S.Transfer==MPMTransfer::MPM ? PIC*(1-S.FlipRatio)+(P.Velocity+Change)*S.FlipRatio : PIC;
			P.Affine=Affine;P.Elastic=MPM::Project((Matrix::Identity()+Gradient*Dt)*P.Elastic,P.PlasticVolume,S.Material,&P.LogVolumeCorrection);
			P.Position+=PIC*Dt;MPM::Confine(P.Position,P.Velocity,S,Capsule);Particles[Id]=P;
		}
		class CUDASnow final : public SnowGPUBackend
		{
			Buffer<SnowParticle> m_Particles;Buffer<Matrix> m_Stress;Buffer<Node> m_Grid;Buffer<int> m_Heads,m_Next;
			cudaStream_t m_Stream=nullptr;std::vector<SnowParticle> m_Readback;
		public:
			~CUDASnow() override { if(m_Stream) cudaStreamDestroy(m_Stream); }
			bool Step(const SnowSettings& S,const SnowCapsule& Capsule,float Dt,std::vector<SnowParticle>& Particles,std::string& Error) override
			{
				try
				{
					Check(cudaSetDevice(SnowDevice()));if(!m_Stream) Check(cudaStreamCreateWithFlags(&m_Stream,cudaStreamNonBlocking));
					if(Particles.empty()) return true;
					const int Count=(int)Particles.size(),Width=(int)ceilf((S.BoundsMax.x-S.BoundsMin.x)/S.CellSize)+1;
					const int Height=(int)ceilf((S.BoundsMax.y-S.BoundsMin.y)/S.CellSize)+1,Depth=(int)ceilf((S.BoundsMax.z-S.BoundsMin.z)/S.CellSize)+1;
					const int Cells=Width*Height*Depth;
					m_Particles.Reserve(Count);m_Stress.Reserve(Count);m_Next.Reserve(Count);m_Grid.Reserve(Cells);m_Heads.Reserve(Cells);m_Readback.resize(Count);
					Check(cudaMemcpyAsync(m_Particles.Data,Particles.data(),Count*sizeof(SnowParticle),cudaMemcpyHostToDevice,m_Stream));
					Check(cudaMemsetAsync(m_Heads.Data,0xff,Cells*sizeof(int),m_Stream));
					Prepare<<<(Count+255)/256,256,0,m_Stream>>>(m_Particles.Data,m_Stress.Data,m_Heads.Data,m_Next.Data,Count,Width,Height,S);Check(cudaGetLastError());
					ToGrid<<<(Cells+255)/256,256,0,m_Stream>>>(m_Particles.Data,m_Stress.Data,m_Heads.Data,m_Next.Data,m_Grid.Data,Count,Width,Height,Depth,Dt,S,Capsule);Check(cudaGetLastError());
					ToParticles<<<(Count+255)/256,256,0,m_Stream>>>(m_Particles.Data,m_Grid.Data,Count,Width,Height,Dt,S,Capsule);Check(cudaGetLastError());
					Check(cudaMemcpyAsync(m_Readback.data(),m_Particles.Data,Count*sizeof(SnowParticle),cudaMemcpyDeviceToHost,m_Stream));Check(cudaStreamSynchronize(m_Stream));
					for(const auto& P:m_Readback)
						if(!std::isfinite(P.Position.x+P.Position.y+P.Position.z+P.Velocity.x+P.Velocity.y+P.Velocity.z+P.Elastic.Determinant()+P.PlasticVolume)) throw std::runtime_error("CUDA snow produced a non-finite state");
					Particles.swap(m_Readback);Error.clear();return true;
				}
				catch(const std::exception& E) { if(m_Stream) cudaStreamSynchronize(m_Stream);Error=E.what();return false; }
			}
		};
	}
	bool IsSnowCUDAAvailable() { return SnowDevice()>=0; }
	std::unique_ptr<SnowGPUBackend> CreateSnowCUDA() { return std::make_unique<CUDASnow>(); }
}
