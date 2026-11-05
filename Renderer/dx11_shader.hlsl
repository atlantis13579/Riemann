cbuffer ConstantBuffer : register(b0)
{
	matrix World;
	matrix View;
	matrix Projection;
	matrix LightView;
	matrix LightProjection;
	float4 EyePos;
	float4 LightDir;
	float4 LightColor;
	float4 MaterialColor;
	float4 OutlineColor;
	float4 RenderParams;
}

Texture2D<float> ShadowMap : register(t0);
SamplerComparisonState ShadowSampler : register(s0);

struct VS_OUTPUT
{
	float4 Pos : SV_POSITION;
	float3 PosWorld : TEXCOORD0;
	float3 Normal : TEXCOORD1;
	float4 PosLight : TEXCOORD2;
};

struct SHADOW_OUTPUT
{
	float4 Pos : SV_POSITION;
};

VS_OUTPUT MainVS(float4 Pos : POSITION, float3 Normal : NORMAL)
{
	VS_OUTPUT output = (VS_OUTPUT)0;

	float4 worldPos = mul(Pos, World);
	output.PosWorld = worldPos.xyz;

	output.Pos = mul(worldPos, View);
	output.Pos = mul(output.Pos, Projection);

	output.PosLight = mul(worldPos, LightView);
	output.PosLight = mul(output.PosLight, LightProjection);

	output.Normal = mul(float4(Normal, 0.0f), World).xyz;
	return output;
}

SHADOW_OUTPUT ShadowVS(float4 Pos : POSITION, float3 Normal : NORMAL)
{
	SHADOW_OUTPUT output = (SHADOW_OUTPUT)0;
	float4 worldPos = mul(Pos, World);
	output.Pos = mul(worldPos, LightView);
	output.Pos = mul(output.Pos, LightProjection);
	return output;
}

VS_OUTPUT OutlineVS(float4 Pos : POSITION, float3 Normal : NORMAL)
{
	VS_OUTPUT output = (VS_OUTPUT)0;

	float3 worldNormal = normalize(mul(float4(Normal, 0.0f), World).xyz);
	float4 worldPos = mul(Pos, World);
	worldPos.xyz += worldNormal * RenderParams.x;

	output.PosWorld = worldPos.xyz;
	output.Pos = mul(worldPos, View);
	output.Pos = mul(output.Pos, Projection);
	output.PosLight = float4(0.0f, 0.0f, 0.0f, 1.0f);
	output.Normal = worldNormal;
	return output;
}

float SampleShadow(float4 lightPos)
{
	float3 projected = lightPos.xyz / lightPos.w;
	float2 uv = float2(projected.x * 0.5f + 0.5f, -projected.y * 0.5f + 0.5f);

	if (uv.x < 0.0f || uv.x > 1.0f || uv.y < 0.0f || uv.y > 1.0f || projected.z < 0.0f || projected.z > 1.0f)
	{
		return 1.0f;
	}

	const float bias = 0.0015f;
	return ShadowMap.SampleCmpLevelZero(ShadowSampler, uv, projected.z - bias);
}

#define SNOW_DDX ddx
#define SNOW_DDY ddy
#define SNOW_FWIDTH fwidth
float SnowHash(float3 P)
{
	P=P*0.1031-floor(P*0.1031);P+=dot(P,P.yzx+33.33);
	float V=(P.x+P.y)*P.z;return V-floor(V);
}
float SnowNoise(float3 P)
{
	float3 I=floor(P),F=P-I;F=F*F*(3-2*F);
	float A=SnowHash(I),B=SnowHash(I+float3(1,0,0)),C=SnowHash(I+float3(0,1,0)),D=SnowHash(I+float3(1,1,0));
	float E=SnowHash(I+float3(0,0,1)),G=SnowHash(I+float3(1,0,1)),H=SnowHash(I+float3(0,1,1)),J=SnowHash(I+float3(1,1,1));
	return ((A+(B-A)*F.x)*(1-F.y)+(C+(D-C)*F.x)*F.y)*(1-F.z)
		+((E+(G-E)*F.x)*(1-F.y)+(H+(J-H)*F.x)*F.y)*F.z;
}
float3 ShadeSnowSand(float3 P,float3 N,float3 L,float3 V,float3 Light,float Shadow,float Ambient,float Snow)
{
	float Macro=SnowNoise(P*3.3),Grain=SnowNoise(P*72);
	float Filter=1/(1+dot(SNOW_FWIDTH(P)*72,SNOW_FWIDTH(P)*72));
	float Height=(Macro-0.5)*(0.006+0.016*Snow)+(Grain-0.5)*Filter*(0.002+0.001*Snow);
	float3 DX=SNOW_DDX(P),DY=SNOW_DDY(P),R1=cross(DY,N),R2=cross(N,DX);
	float Det=dot(DX,R1);
	float3 Gradient=(SNOW_DDX(Height)*R1+SNOW_DDY(Height)*R2)*(Det<0 ? -1.0 : 1.0)/max(abs(Det),1.e-8);
	N=normalize(N-Gradient);
	float3 H=normalize(L+V);
	float NL=max(dot(N,L),0.0),NV=max(dot(N,V),0.001),NH=max(dot(N,H),0.0),VH=max(dot(V,H),0.0);
	float Roughness=0.86-0.32*Snow,Alpha=Roughness*Roughness,A2=Alpha*Alpha;
	float Den=NH*NH*(A2-1)+1,D=A2/(3.14159265*Den*Den);
	float K=(Roughness+1)*(Roughness+1)/8,G1=NV/(NV*(1-K)+K),G2=NL/(NL*(1-K)+K);
	float Fresnel=0.04+0.96*pow(1-VH,5.0),Spec=D*G1*G2*Fresnel/(4*NV*max(NL,0.001));
	float Texture=0.80+0.30*Macro+0.35*(Grain-0.5)*Filter;
	float3 Sand=float3(0.64,0.46,0.25)*Texture,SnowColor=float3(0.78,0.85,0.94)*(0.86+0.18*Macro);
	float3 Base=Sand*(1-Snow)+SnowColor*Snow;
	float Sky=0.5+0.5*N.y;
	float3 AmbientLight=(float3(0.25,0.23,0.20)*(1-Snow)+float3(0.15,0.22,0.32)*Snow)*(0.6+0.4*Sky)*(0.8+Ambient);
	float3 Color=Base*(AmbientLight+Light*NL*Shadow*0.78)+Light*Spec*NL*Shadow;
	// Broad forward scattering and filtered crystal glints give snow a porous surface.
	float Wrap=clamp((dot(N,L)+0.35)/1.35,0.0,1.0);
	Color+=Snow*float3(0.07,0.11,0.16)*Wrap*(0.3+0.7*Shadow);
	float Glint=pow(clamp((Grain-0.7)*3.34,0.0,1.0),12.0)*pow(NH,90.0)*Filter;
	Color+=Light*Glint*(0.12+0.65*Snow)*Shadow;
	return pow(clamp(Color,float3(0,0,0),float3(1,1,1)),float3(0.4545,0.4545,0.4545));
}

float4 MainPS(VS_OUTPUT input) : SV_Target
{
	if (RenderParams.y > 0.5f)
	{
		return MaterialColor;
	}

	float3 normal = normalize(input.Normal);
	float3 lightToScene = normalize(LightDir.xyz);
	float3 toLight = -lightToScene;
	float3 toEye = normalize(EyePos.xyz - input.PosWorld);
	if (dot(normal, toEye) < 0.0f)
	{
		normal = -normal;
	}
	float3 halfVector = normalize(toLight + toEye);

	float ndotl = saturate(dot(normal, toLight));
	float specular = pow(saturate(dot(normal, halfVector)), 64.0f);
	float shadow = SampleShadow(input.PosLight);

	if (RenderParams.z > 0.5) return float4(ShadeSnowSand(input.PosWorld,normal,toLight,toEye,LightColor.rgb,shadow,LightDir.w,RenderParams.w),1);

	float3 baseColor = MaterialColor.rgb;
	float ambient = LightDir.w;
	float3 diffuse = baseColor * LightColor.rgb * ndotl * shadow;
	float3 spec = LightColor.rgb * specular * shadow * 0.35f;
	float3 color = baseColor * ambient + diffuse + spec;

	return float4(color, MaterialColor.a);
}

float4 OutlinePS(VS_OUTPUT input) : SV_Target
{
	return OutlineColor;
}

cbuffer VolumeParameters : register(b1)
{
	float4 VolumeEyeNear;
	float4 VolumeForwardFar;
	float4 VolumeRight;
	float4 VolumeUp;
	float4 VolumeMinDensity;
	float4 VolumeMaxThreshold;
	float4 VolumeCompressed;
	float4 VolumeRarefied;
	float4 VolumeLight;
	float4 VolumeControl;
	float4 VolumeSmoke;
};

Texture3D<float> DensityVolume : register(t1);
Texture2D<float> SceneDepth : register(t2);
SamplerState VolumeSampler : register(s1);

struct VOLUME_OUTPUT
{
	float4 Position : SV_POSITION;
	float2 UV : TEXCOORD0;
};

VOLUME_OUTPUT VolumeVS(uint Vertex : SV_VertexID)
{
	VOLUME_OUTPUT Output;
	Output.UV = float2((Vertex << 1) & 2, Vertex & 2);
	Output.Position = float4(Output.UV.x * 2 - 1, 1 - Output.UV.y * 2, 0, 1);
	return Output;
}

float VolumeDeviation(float3 Position)
{
	float3 UV = (Position - VolumeMinDensity.xyz) / (VolumeMaxThreshold.xyz - VolumeMinDensity.xyz);
	float Density = DensityVolume.SampleLevel(VolumeSampler, UV, 0);
	return VolumeSmoke.x > 0 ? Density : Density * VolumeMinDensity.w - 1;
}

float VolumeOpacityDensity(float Deviation)
{
	// A smooth dead band keeps ambient air invisible without an isosurface pop.
	float Magnitude = abs(Deviation);
	if (VolumeSmoke.x > 0) return max(0, Deviation);
	return Magnitude * smoothstep(VolumeMaxThreshold.w * 0.25, VolumeMaxThreshold.w, Magnitude);
}

float4 VolumePS(VOLUME_OUTPUT Input) : SV_Target
{
	// Orange is compression, blue is rarefaction. Trilinear density sampling,
	// a smooth ambient dead band, Beer-Lambert extinction, front-to-back
	// premultiplied accumulation, short light integration, and early termination
	// avoid hard density shells. A capped sample count stretches the step over
	// the full ray interval instead of dropping its far end.
	float3 Ray = normalize(VolumeForwardFar.xyz + (2 * Input.UV.x - 1) * VolumeRight.xyz
		+ (1 - 2 * Input.UV.y) * VolumeUp.xyz);
	float3 SafeRay = float3(Ray.x >= 0 ? max(Ray.x, 1e-7) : min(Ray.x, -1e-7),
		Ray.y >= 0 ? max(Ray.y, 1e-7) : min(Ray.y, -1e-7),
		Ray.z >= 0 ? max(Ray.z, 1e-7) : min(Ray.z, -1e-7));
	float3 A = (VolumeMinDensity.xyz - VolumeEyeNear.xyz) / SafeRay;
	float3 B = (VolumeMaxThreshold.xyz - VolumeEyeNear.xyz) / SafeRay;
	float3 Entry = min(A, B), Exit = max(A, B);
	float CosForward = max(dot(Ray, VolumeForwardFar.xyz), 1e-6);
	float Begin = max(max(Entry.x, max(Entry.y, Entry.z)), VolumeEyeNear.w / CosForward);
	float End = min(Exit.x, min(Exit.y, Exit.z));
	float Depth = SceneDepth.Load(int3(int2(Input.Position.xy), 0));
	float SceneDistance = VolumeEyeNear.w * VolumeForwardFar.w
		/ (VolumeForwardFar.w - Depth * (VolumeForwardFar.w - VolumeEyeNear.w)) / CosForward;
	End = min(End, SceneDistance);
	if (End <= Begin) return 0;

	// Integrate the entire interval even if the safety sample cap is reached.
	int Steps = min(256, max(1, (int)ceil((End - Begin) / VolumeControl.x)));
	float Step = (End - Begin) / Steps;
	float4 Result = 0;
	float3 Extent = VolumeMaxThreshold.xyz - VolumeMinDensity.xyz;
	[loop]
	for (int Index = 0; Index < Steps; ++Index)
	{
		float3 Position = VolumeEyeNear.xyz + Ray * (Begin + (Index + 0.5) * Step);
		float Deviation = VolumeDeviation(Position);
		float Sigma = VolumeOpacityDensity(Deviation) * VolumeControl.y;
		if (Sigma < 1e-5) continue;
		float4 Tint = Deviation >= 0 ? VolumeCompressed : VolumeRarefied;
		float Alpha = 1 - exp(-Sigma * Step * Tint.a);

		// Increasing light intervals resolve the nearby wisps and the broad
		// shadow through the plume without procedural density in the renderer.
		float OpticalDepth = 0;
		float LightDistance = 0;
		int LightSteps = VolumeSmoke.x > 0 ? 8 : 3;
		[loop]
		for (int LightStep = 0; LightStep < LightSteps; ++LightStep)
		{
			float LightInterval = VolumeSmoke.x > 0 ? VolumeSmoke.y * pow(1.45, LightStep) : VolumeControl.x * 2;
			LightDistance += LightInterval;
			float3 Probe = Position - VolumeLight.xyz * LightDistance;
			float3 UV = (Probe - VolumeMinDensity.xyz) / Extent;
			if (all(UV >= 0) && all(UV <= 1))
				OpticalDepth += VolumeOpacityDensity(VolumeDeviation(Probe)) * LightInterval * VolumeControl.y;
		}
		float Lighting = saturate(VolumeLight.w + 0.8 * exp(-OpticalDepth));
		Result.rgb += (1 - Result.a) * Alpha * Tint.rgb * Lighting;
		Result.a += (1 - Result.a) * Alpha;
		if (Result.a >= 0.995) break;
	}
	return Result;
}
