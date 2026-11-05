#pragma once

namespace Riemann
{
	static const char* const SandKernels = R"SHADER(
int SandX(int k) { return k==0 || k==4 || k==6 ? -1 : (k==1 || k==5 || k==7 ? 1 : 0); }
int SandZ(int k) { return k==2 || k==4 || k==7 ? -1 : (k==3 || k==5 || k==6 ? 1 : 0); }
KERNEL(SandFlux)
{
	int i=(int)tid.x,n=PI(constants,0);if(i>=n*n) return;
	int x=i%n,z=i/n;float h=ReadFloat(b0,i),sum=0;float flow[8];
	for(int k=0;k<8;++k)
	{
		int nx=x+SandX(k),nz=z+SandZ(k),j=nz*n+nx;float f=0;
		if(nx>=0 && nx<n && nz>=0 && nz<n)
			if(ReadFloat(b0,j)<ReadFloat(b2,j)-1.e-6)
				f=max(0.0,h-ReadFloat(b0,j)-PF(constants,1)*(k<4 ? 1.0 : 1.41421356237))*PF(constants,2);
		flow[k]=f;sum+=f;
	}
	float scale=sum>h ? h/sum : 1.0;
	for(int k=0;k<8;++k) WriteFloat(b1,i*8+k,flow[k]*scale);
}
KERNEL(SandGather)
{
	int i=(int)tid.x,n=PI(constants,0);if(i>=n*n) return;
	int x=i%n,z=i/n;float h=ReadFloat(b0,i);
	for(int k=0;k<8;++k)
	{
		h-=ReadFloat(b1,i*8+k);int nx=x+SandX(k),nz=z+SandZ(k);
		if(nx>=0 && nx<n && nz>=0 && nz<n) h+=ReadFloat(b1,(nz*n+nx)*8+(k^1));
	}
	WriteFloat(b3,i,max(0.0,h));
}
)SHADER";
}
