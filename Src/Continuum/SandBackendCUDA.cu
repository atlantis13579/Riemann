#include "SandGPUBackend.h"
#include <cuda_runtime.h>
#include <cmath>
#include <stdexcept>

namespace Riemann
{
	namespace
	{
		void Check(cudaError_t Status) { if(Status!=cudaSuccess) throw std::runtime_error(cudaGetErrorString(Status)); }
		__device__ int OffsetX(int K) { return K==0 || K==4 || K==6 ? -1 : (K==1 || K==5 || K==7 ? 1 : 0); }
		__device__ int OffsetZ(int K) { return K==2 || K==4 || K==7 ? -1 : (K==3 || K==5 || K==6 ? 1 : 0); }
		__global__ void Flux(const float* Heights,const float* Limits,float* Flows,int N,float Slope,float Rate)
		{
			const int I=blockIdx.x*blockDim.x+threadIdx.x;if(I>=N*N) return;
			const int X=I%N,Z=I/N;const float H=Heights[I];float F[8],Sum=0;
			for(int K=0;K<8;++K)
			{
				const int NX=X+OffsetX(K),NZ=Z+OffsetZ(K),J=NZ*N+NX;
				F[K]=NX>=0 && NX<N && NZ>=0 && NZ<N && Heights[J]<Limits[J]-1.e-6f ?
					fmaxf(0,H-Heights[J]-Slope*(K<4 ? 1.0f : 1.41421356237f))*Rate : 0;
				Sum+=F[K];
			}
			const float Scale=Sum>H ? H/Sum : 1;
			for(int K=0;K<8;++K) Flows[I*8+K]=F[K]*Scale;
		}
		__global__ void Gather(const float* Heights,const float* Flows,float* Next,int N)
		{
			const int I=blockIdx.x*blockDim.x+threadIdx.x;if(I>=N*N) return;
			const int X=I%N,Z=I/N;float H=Heights[I];
			for(int K=0;K<8;++K)
			{
				H-=Flows[I*8+K];const int NX=X+OffsetX(K),NZ=Z+OffsetZ(K);
				if(NX>=0 && NX<N && NZ>=0 && NZ<N) H+=Flows[(NZ*N+NX)*8+(K^1)];
			}
			Next[I]=fmaxf(0,H);
		}
		class CudaSand final : public SandGPUBackend
		{
			float *m_Height=nullptr,*m_Next=nullptr,*m_Flux=nullptr,*m_Limits=nullptr;
			size_t m_Capacity=0;cudaStream_t m_Stream=nullptr;int m_Device=-1;
			std::vector<float> m_Readback;
			void Release()
			{
				cudaFree(m_Height);cudaFree(m_Next);cudaFree(m_Flux);cudaFree(m_Limits);
				m_Height=m_Next=m_Flux=m_Limits=nullptr;m_Capacity=0;
			}
		public:
			~CudaSand() override
			{
				if(m_Device>=0) cudaSetDevice(m_Device);
				if(m_Stream) cudaStreamSynchronize(m_Stream);
				Release();if(m_Stream) cudaStreamDestroy(m_Stream);
			}
			bool Relax(std::vector<float>& Heights,const std::vector<float>& Limits,int Resolution,
				float Slope,float Rate,int Iterations,std::string& Error) override
			{
				try
				{
					if(m_Device<0)
					{
						int Count=0;Check(cudaGetDeviceCount(&Count));
						for(int I=0;I<Count;++I) { cudaDeviceProp P{};Check(cudaGetDeviceProperties(&P,I));if(P.major>=7) { m_Device=I;break; } }
						if(m_Device<0) throw std::runtime_error("No compatible CUDA sand device");
					}
					Check(cudaSetDevice(m_Device));if(!m_Stream) Check(cudaStreamCreate(&m_Stream));
					const int Count=Resolution*Resolution;const size_t Bytes=Count*sizeof(float);
					if(m_Capacity<Bytes)
					{
						Release();Check(cudaMalloc((void**)&m_Height,Bytes));Check(cudaMalloc((void**)&m_Next,Bytes));
						Check(cudaMalloc((void**)&m_Limits,Bytes));Check(cudaMalloc((void**)&m_Flux,Bytes*8));m_Capacity=Bytes;
					}
					Check(cudaMemcpyAsync(m_Height,Heights.data(),Bytes,cudaMemcpyHostToDevice,m_Stream));
					Check(cudaMemcpyAsync(m_Limits,Limits.data(),Bytes,cudaMemcpyHostToDevice,m_Stream));
					for(int I=0;I<Iterations;++I)
					{
						Flux<<<(Count+255)/256,256,0,m_Stream>>>(m_Height,m_Limits,m_Flux,Resolution,Slope,Rate);
						Gather<<<(Count+255)/256,256,0,m_Stream>>>(m_Height,m_Flux,m_Next,Resolution);
						std::swap(m_Height,m_Next);
					}
					Check(cudaGetLastError());m_Readback.resize(Count);
					Check(cudaMemcpyAsync(m_Readback.data(),m_Height,Bytes,cudaMemcpyDeviceToHost,m_Stream));Check(cudaStreamSynchronize(m_Stream));
					for(float H:m_Readback) if(!std::isfinite(H) || H<0) throw std::runtime_error("CUDA sand produced invalid heights");
					Heights.swap(m_Readback);Error.clear();return true;
				}
				catch(const std::exception& E) { if(m_Stream) cudaStreamSynchronize(m_Stream);Error=E.what();return false; }
			}
		};
	}
	std::unique_ptr<SandGPUBackend> CreateSandCUDA() { return std::make_unique<CudaSand>(); }
}
