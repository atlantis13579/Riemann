#pragma once

namespace Riemann
{
namespace SnowCompute
{
	// Native HLSL and Metal share the material model, transfers, and contact rules.
	static const char* const SnowKernels = R"MPM(
struct MMatrix { float3 R0; float3 R1; float3 R2; };
MMatrix MMake(float3 A,float3 B,float3 C) { MMatrix M;M.R0=A;M.R1=B;M.R2=C;return M; }
MMatrix MZero() { return MMake(float3(0,0,0),float3(0,0,0),float3(0,0,0)); }
MMatrix MIdentity() { return MMake(float3(1,0,0),float3(0,1,0),float3(0,0,1)); }
float3 MColumn(MMatrix M,int C) { return float3(M.R0[C],M.R1[C],M.R2[C]); }
float3 VSet(float3 V,int C,float X) { if(C==0) V.x=X;else if(C==1) V.y=X;else V.z=X;return V; }
MMatrix MSetColumn(MMatrix M,int C,float3 V) { M.R0=VSet(M.R0,C,V.x);M.R1=VSet(M.R1,C,V.y);M.R2=VSet(M.R2,C,V.z);return M; }
MMatrix MAdd(MMatrix A,MMatrix B) { return MMake(A.R0+B.R0,A.R1+B.R1,A.R2+B.R2); }
MMatrix MSub(MMatrix A,MMatrix B) { return MMake(A.R0-B.R0,A.R1-B.R1,A.R2-B.R2); }
MMatrix MScale(MMatrix A,float S) { return MMake(A.R0*S,A.R1*S,A.R2*S); }
MMatrix MTranspose(MMatrix A) { return MMake(MColumn(A,0),MColumn(A,1),MColumn(A,2)); }
float3 MApply(MMatrix A,float3 V) { return float3(dot(A.R0,V),dot(A.R1,V),dot(A.R2,V)); }
MMatrix MMultiply(MMatrix A,MMatrix B) { MMatrix T=MTranspose(B);return MMake(MApply(T,A.R0),MApply(T,A.R1),MApply(T,A.R2)); }
float MDet(MMatrix A) { return dot(A.R0,cross(A.R1,A.R2)); }
MMatrix MOuter(float3 A,float3 B) { return MMake(A.x*B,A.y*B,A.z*B); }
MMatrix MDiagonal(float3 V) { return MMake(float3(V.x,0,0),float3(0,V.y,0),float3(0,0,V.z)); }
float3 Read3(BUFFER B,uint I) { return float3(ReadFloat(B,I),ReadFloat(B,I+1),ReadFloat(B,I+2)); }
void Write3(BUFFER B,uint I,float3 V) { WriteFloat(B,I,V.x);WriteFloat(B,I+1,V.y);WriteFloat(B,I+2,V.z); }
MMatrix ReadMatrix(BUFFER B,uint I) { return MMake(Read3(B,I),Read3(B,I+3),Read3(B,I+6)); }
void WriteMatrix(BUFFER B,uint I,MMatrix M) { Write3(B,I,M.R0);Write3(B,I+3,M.R1);Write3(B,I+6,M.R2); }
struct MSVD { MMatrix U;MMatrix V;float3 Stretch; };
MSVD MDecompose(MMatrix F)
{
	MMatrix V=MIdentity();
	for(int Sweep=0;Sweep<6;++Sweep)
		for(int P=0;P<2;++P) for(int Q=P+1;Q<3;++Q)
		{
			float3 A=MColumn(F,P),B=MColumn(F,Q);
			float AA=dot(A,A),BB=dot(B,B),AB=dot(A,B);
			if(abs(AB)<=1.e-7*sqrt(AA*BB) || abs(AB)<1.e-20) continue;
			float Tau=(BB-AA)/(2*AB),T=(Tau>=0 ? 1.0 : -1.0)/(abs(Tau)+sqrt(1+Tau*Tau));
			float C=1/sqrt(1+T*T),S=C*T;
			F=MSetColumn(F,P,A*C-B*S);F=MSetColumn(F,Q,A*S+B*C);
			float3 VP=MColumn(V,P),VQ=MColumn(V,Q);
			V=MSetColumn(V,P,VP*C-VQ*S);V=MSetColumn(V,Q,VP*S+VQ*C);
		}
	MSVD Result;Result.V=V;
	Result.Stretch=max(float3(length(MColumn(F,0)),length(MColumn(F,1)),length(MColumn(F,2))),float3(1.e-6,1.e-6,1.e-6));
	for(int SP=0;SP<2;++SP) for(int SQ=SP+1;SQ<3;++SQ)
		if(Result.Stretch[SP]<Result.Stretch[SQ])
		{
			float Temp=Result.Stretch[SP];Result.Stretch=VSet(Result.Stretch,SP,Result.Stretch[SQ]);Result.Stretch=VSet(Result.Stretch,SQ,Temp);
			float3 A=MColumn(F,SP),B=MColumn(Result.V,SP);
			F=MSetColumn(F,SP,MColumn(F,SQ));F=MSetColumn(F,SQ,A);
			Result.V=MSetColumn(Result.V,SP,MColumn(Result.V,SQ));Result.V=MSetColumn(Result.V,SQ,B);
		}
	float3 U0=MColumn(F,0)/Result.Stretch.x;
	if(length(U0)<0.5) U0=float3(1,0,0);
	float3 U1=MColumn(F,1)-U0*dot(U0,MColumn(F,1));
	if(length(U1)<1.e-6) U1=cross(U0,abs(U0.x)<0.8 ? float3(1,0,0) : float3(0,1,0));
	U1/=max(length(U1),1.e-6);
	float3 U2=cross(U0,U1)*(MDet(Result.V)<0 ? -1.0 : 1.0);
	Result.U=MTranspose(MMake(U0,U1,U2));return Result;
}
float2 MLame(float Jp,DATA)
{
	float E=PF(data,16)*exp(clamp(PF(data,20)*(1-Jp)*PF(data,18),-2.0,1.6)),Nu=PF(data,17);
	return float2(E/(2*(1+Nu)),E*Nu/((1+Nu)*(1-2*Nu)));
}
MMatrix MStress(MMatrix F,MMatrix C,float Jp,DATA)
{
	float2 L=MLame(Jp,data);MSVD S=MDecompose(F);
	MMatrix R=MMultiply(S.U,MTranspose(S.V));float J=max(MDet(F),1.e-6);
	MMatrix Snow=MAdd(MScale(MMultiply(MSub(F,R),MTranspose(F)),2*L.x),MScale(MIdentity(),L.y*J*(J-1)));
	float3 Strain=log(S.Stretch);float Trace=Strain.x+Strain.y+Strain.z;
	MMatrix Sand=MMultiply(MMultiply(S.U,MDiagonal(2*L.x*Strain+L.y*Trace)),MTranspose(S.U));
	return MAdd(MAdd(MScale(Snow,PF(data,18)),MScale(Sand,1-PF(data,18))),MScale(MAdd(C,MTranspose(C)),PF(data,19)));
}
struct MProjection { MMatrix Elastic;float Plastic;float VolumeCorrection; };
MProjection MProject(MMatrix Trial,float Jp,float VolumeCorrection,DATA)
{
	MSVD S=MDecompose(Trial);float2 L=MLame(Jp,data);
	float3 Strain=log(S.Stretch)+VolumeCorrection/3;float Trace=Strain.x+Strain.y+Strain.z;
	float3 Dev=Strain-Trace/3;float Norm=length(Dev),Sine=sin(PF(data,23)*0.01745329252);
	float Alpha=0.81649658093*2*Sine/(3-Sine),Shear=Norm+(3*L.y+2*L.x)/(2*L.x)*Trace*Alpha;
	if(Trace>=0) Strain=float3(0,0,0);else if(Shear>0 && Norm>1.e-8) Strain-=Dev*(Shear/Norm);
	float3 Snow=clamp(S.Stretch,float3(1-PF(data,21),1-PF(data,21),1-PF(data,21)),float3(1+PF(data,22),1+PF(data,22),1+PF(data,22)));
	float3 Stretch=exp(PF(data,18)*log(Snow)+(1-PF(data,18))*Strain);
	MProjection P;P.Plastic=clamp(Jp*max(MDet(Trial),1.e-6)/max(Stretch.x*Stretch.y*Stretch.z,1.e-6),0.4,4.0);
	P.VolumeCorrection=(1-PF(data,18))*max(0.0,VolumeCorrection+log(max(MDet(Trial),1.e-6))-log(max(Stretch.x*Stretch.y*Stretch.z,1.e-6)));
	P.Elastic=MMultiply(MMultiply(S.U,MDiagonal(Stretch)),MTranspose(S.V));return P;
}
float MW(int I,float F) { return I==0 ? 0.5*(1.5-F)*(1.5-F) : I==1 ? 0.75-(F-1)*(F-1) : 0.5*(F-0.5)*(F-0.5); }
float MDW(int I,float F) { return I==0 ? F-1.5 : I==1 ? -2*(F-1) : F-0.5; }
float3 MOrigin(DATA) { return float3(PF(data,4),PF(data,5),PF(data,6)); }
float3 MMaximum(DATA) { return float3(PF(data,8),PF(data,9),PF(data,10)); }
float3 MClosest(float3 Position,DATA)
{
	float3 Center=float3(PF(data,28),PF(data,29),PF(data,30));
	return float3(Center.x,clamp(Position.y,Center.y-PF(data,27),Center.y+PF(data,27)),Center.z);
}
float3 MContact(float3 Position,float3 Velocity,float Padding,DATA)
{
	if(PI(data,25)==0) return Velocity;
	float3 Offset=Position-MClosest(Position,data);float Distance=length(Offset);
	if(Distance>=PF(data,26)+Padding) return Velocity;
	float3 Normal=Distance>1.e-6 ? Offset/Distance : float3(1,0,0);
	float3 CapVelocity=float3(PF(data,32),PF(data,33),PF(data,34)),Relative=Velocity-CapVelocity;
	float Incoming=dot(Relative,Normal);if(Incoming>=0) return Velocity;
	float3 Tangent=Relative-Incoming*Normal;
	return CapVelocity+Tangent*max(0.0,1-PF(data,31)*(-Incoming)/max(length(Tangent),1.e-6));
}
KERNEL(SnowClear)
{
	if(tid.x<(uint)(PI(constants,1)*PI(constants,2)*PI(constants,3))) WriteUInt(b2,tid.x,0xffffffffu);
}
KERNEL(SnowPrepare)
{
	uint Id=tid.x;if(Id>=(uint)PI(constants,0)) return;uint Base=Id*28;
	WriteMatrix(b4,Id*9,MStress(ReadMatrix(b0,Base+6),ReadMatrix(b0,Base+15),ReadFloat(b0,Base+24),constants));
	float3 G=(Read3(b0,Base)-MOrigin(constants))/PF(constants,7);int3 Cell=int3(floor(G-0.5));
	uint Previous=Exchange(b2,(uint)((Cell.z*PI(constants,2)+Cell.y)*PI(constants,1)+Cell.x),Id);WriteUInt(b3,Id,Previous);
}
KERNEL(SnowGrid)
{
	uint Id=tid.x;int Width=PI(constants,1),Height=PI(constants,2),Depth=PI(constants,3),Count=PI(constants,0);
	if(Id>=(uint)(Width*Height*Depth)) return;
	int X=(int)Id%Width,Y=((int)Id/Width)%Height,Z=(int)Id/(Width*Height);
	float Dx=PF(constants,7),Inverse=1/Dx,Mass=0;float3 Origin=MOrigin(constants),Momentum=float3(0,0,0),Force=float3(0,0,0);
	for(int BZ=max(0,Z-2);BZ<=Z;++BZ) for(int BY=max(0,Y-2);BY<=Y;++BY) for(int BX=max(0,X-2);BX<=X;++BX)
	{
		int Visits=0;
		for(uint P=ReadUInt(b2,(uint)((BZ*Height+BY)*Width+BX));P<(uint)Count && Visits++<Count;P=ReadUInt(b3,P))
		{
			uint Base=P*28;float3 F=(Read3(b0,Base)-Origin)*Inverse-float3(BX,BY,BZ);int I=X-BX,J=Y-BY,K=Z-BZ;
			float WX=MW(I,F.x),WY=MW(J,F.y),WZ=MW(K,F.z),W=WX*WY*WZ;
			float3 D=(float3(I,J,K)-F)*Dx;
			float3 Gradient=PI(constants,15)==2 ? 4*Inverse*Inverse*W*D : float3(MDW(I,F.x)*WY*WZ,WX*MDW(J,F.y)*WZ,WX*WY*MDW(K,F.z))*Inverse;
			MMatrix C=MZero();if(PI(constants,15)!=0) C=ReadMatrix(b0,Base+15);
			float Contribution=W*ReadFloat(b0,Base+26);Mass+=Contribution;
			Momentum+=Contribution*(Read3(b0,Base+3)+MApply(C,D));Force-=ReadFloat(b0,Base+25)*MApply(ReadMatrix(b4,P*9),Gradient);
		}
	}
	float3 Old=float3(0,0,0),Velocity=float3(0,0,0);
	if(Mass>0)
	{
		Old=Momentum/Mass;Velocity=Old+PF(constants,11)*(Force/Mass+float3(PF(constants,12),PF(constants,13),PF(constants,14)));
		float3 Position=Origin+float3(X,Y,Z)*Dx,Maximum=MMaximum(constants);int3 Coord=int3(X,Y,Z);
		// The first node above ground also receives momentum from floor particles.
		if(Y<3 && Velocity.y<0)
		{
			Velocity.xz*=max(0.0,1+PF(constants,35)*Velocity.y/max(length(Velocity.xz),1.e-6));Velocity.y=0;
		}
		for(int A=0;A<3;++A) if((Coord[A]<2 && Velocity[A]<0) || (Position[A]>Maximum[A]-2*Dx && Velocity[A]>0)) Velocity[A]=0;
		Velocity=MContact(Position,Velocity,0.25*Dx,constants);
	}
	Write3(b1,Id*8,Velocity);Write3(b1,Id*8+3,Old);WriteFloat(b1,Id*8+6,Mass);
}
KERNEL(SnowParticles)
{
	uint Id=tid.x;if(Id>=(uint)PI(constants,0)) return;uint Base=Id*28;
	int Width=PI(constants,1),Height=PI(constants,2);float Dx=PF(constants,7),Inverse=1/Dx,Dt=PF(constants,11);
	float3 Position=Read3(b0,Base),PreviousVelocity=Read3(b0,Base+3),G=(Position-MOrigin(constants))*Inverse;
	int3 Cell=int3(floor(G-0.5));float3 F=G-float3(Cell),PIC=float3(0,0,0),Change=float3(0,0,0);
	MMatrix Gradient=MZero(),Affine=MZero();
	for(int K=0;K<3;++K) for(int J=0;J<3;++J) for(int I=0;I<3;++I)
	{
		uint Node=(uint)(((Cell.z+K)*Height+Cell.y+J)*Width+Cell.x+I)*8;
		float3 V=Read3(b1,Node),Old=Read3(b1,Node+3);
		float WX=MW(I,F.x),WY=MW(J,F.y),WZ=MW(K,F.z),W=WX*WY*WZ;float3 D=(float3(I,J,K)-F)*Dx;
		float3 DW=PI(constants,15)==2 ? 4*Inverse*Inverse*W*D : float3(MDW(I,F.x)*WY*WZ,WX*MDW(J,F.y)*WZ,WX*WY*MDW(K,F.z))*Inverse;
		PIC+=W*V;Change+=W*(V-Old);Gradient=MAdd(Gradient,MOuter(V,DW));Affine=MAdd(Affine,MScale(MOuter(W*V,D),4*Inverse*Inverse));
	}
	float3 Velocity=PI(constants,15)==0 ? (1-PF(constants,24))*PIC+PF(constants,24)*(PreviousVelocity+Change) : PIC;
	MProjection P=MProject(MMultiply(MAdd(MIdentity(),MScale(Gradient,Dt)),ReadMatrix(b0,Base+6)),ReadFloat(b0,Base+24),ReadFloat(b0,Base+27),constants);
	Position+=Dt*PIC;float3 Low=MOrigin(constants)+1.5*Dx,High=MMaximum(constants)-1.5*Dx;
	for(int A=0;A<3;++A) if(Position[A]<Low[A] || Position[A]>High[A]) Velocity[A]=0;
	Position=clamp(Position,Low,High);Velocity=MContact(Position,Velocity,0,constants);
	if(PI(constants,25)!=0)
	{
		float3 Closest=MClosest(Position,constants),Offset=Position-Closest;float Distance=length(Offset);
		if(Distance<PF(constants,26)) Position=Closest+PF(constants,26)*(Distance>1.e-6 ? Offset/Distance : float3(1,0,0));
	}
	Write3(b0,Base,clamp(Position,Low,High));Write3(b0,Base+3,Velocity);WriteMatrix(b0,Base+6,P.Elastic);WriteMatrix(b0,Base+15,Affine);WriteFloat(b0,Base+24,P.Plastic);WriteFloat(b0,Base+27,P.VolumeCorrection);
}
)MPM";
}
}
