#include <metal_stdlib>
using namespace metal;

struct MetalVertex
{
	packed_float3 position;
	packed_float3 normal;
};

struct Uniforms
{
	float4x4 world;
	float4x4 view;
	float4x4 projection;
	float4x4 lightView;
	float4x4 lightProjection;
	float4 eyePos;
	float4 lightDir;
	float4 lightColor;
	float4 materialColor;
	float4 outlineColor;
	float4 renderParams;
};

struct VSOut
{
	float4 position [[position]];
	float3 positionWorld;
	float3 normal;
	float4 positionLight;
};

struct ShadowOut
{
	float4 position [[position]];
};

float4 MulRow(float4 value, float4x4 matrixValue)
{
	return float4(
		dot(value, matrixValue[0]),
		dot(value, matrixValue[1]),
		dot(value, matrixValue[2]),
		dot(value, matrixValue[3]));
}

vertex VSOut RiemannMainVS(
	uint vertexId [[vertex_id]],
	const device MetalVertex* vertices [[buffer(0)]],
	constant Uniforms& uniforms [[buffer(1)]])
{
	MetalVertex vertex = vertices[vertexId];

	float4 localPosition = float4(float3(vertex.position), 1.0);
	float4 worldPosition = MulRow(localPosition, uniforms.world);

	VSOut out;
	out.positionWorld = worldPosition.xyz;
	out.position = MulRow(MulRow(worldPosition, uniforms.view), uniforms.projection);
	out.positionLight = MulRow(MulRow(worldPosition, uniforms.lightView), uniforms.lightProjection);
	out.normal = normalize(MulRow(float4(float3(vertex.normal), 0.0), uniforms.world).xyz);
	return out;
}

vertex ShadowOut RiemannShadowVS(
	uint vertexId [[vertex_id]],
	const device MetalVertex* vertices [[buffer(0)]],
	constant Uniforms& uniforms [[buffer(1)]])
{
	MetalVertex vertex = vertices[vertexId];
	float4 localPosition = float4(float3(vertex.position), 1.0);
	float4 worldPosition = MulRow(localPosition, uniforms.world);

	ShadowOut out;
	out.position = MulRow(MulRow(worldPosition, uniforms.lightView), uniforms.lightProjection);
	return out;
}

vertex VSOut RiemannOutlineVS(
	uint vertexId [[vertex_id]],
	const device MetalVertex* vertices [[buffer(0)]],
	constant Uniforms& uniforms [[buffer(1)]])
{
	MetalVertex vertex = vertices[vertexId];

	float3 worldNormal = normalize(MulRow(float4(float3(vertex.normal), 0.0), uniforms.world).xyz);
	float4 localPosition = float4(float3(vertex.position), 1.0);
	float4 worldPosition = MulRow(localPosition, uniforms.world);
	worldPosition.xyz += worldNormal * uniforms.renderParams.x;

	VSOut out;
	out.positionWorld = worldPosition.xyz;
	out.position = MulRow(MulRow(worldPosition, uniforms.view), uniforms.projection);
	out.positionLight = float4(0.0, 0.0, 0.0, 1.0);
	out.normal = worldNormal;
	return out;
}

float SampleShadow(float4 lightPosition, depth2d<float> shadowMap, sampler shadowSampler)
{
	if (fabs(lightPosition.w) < 1.0e-6)
	{
		return 1.0;
	}

	float3 projected = lightPosition.xyz / lightPosition.w;
	float2 uv = float2(projected.x * 0.5 + 0.5, 0.5 - projected.y * 0.5);
	if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0 || projected.z < 0.0 || projected.z > 1.0)
	{
		return 1.0;
	}

	const float bias = 0.0015;
	return shadowMap.sample_compare(shadowSampler, uv, projected.z - bias);
}

#define SNOW_DDX dfdx
#define SNOW_DDY dfdy
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

fragment float4 RiemannMainFS(
	VSOut in [[stage_in]],
	constant Uniforms& uniforms [[buffer(1)]],
	depth2d<float> shadowMap [[texture(0)]],
	sampler shadowSampler [[sampler(0)]])
{
	if (uniforms.renderParams.y > 0.5)
	{
		return uniforms.materialColor;
	}

	float3 normal = normalize(in.normal);
	float3 lightToScene = normalize(uniforms.lightDir.xyz);
	float3 toLight = -lightToScene;
	float3 toEye = normalize(uniforms.eyePos.xyz - in.positionWorld);
	if (dot(normal, toEye) < 0.0)
	{
		normal = -normal;
	}
	float3 halfVector = normalize(toLight + toEye);

	float ndotl = saturate(dot(normal, toLight));
	float specular = pow(saturate(dot(normal, halfVector)), 64.0);
	float shadow = SampleShadow(in.positionLight, shadowMap, shadowSampler);

	if (uniforms.renderParams.z > 0.5) return float4(ShadeSnowSand(in.positionWorld,normal,toLight,toEye,uniforms.lightColor.rgb,shadow,uniforms.lightDir.w,uniforms.renderParams.w),1);

	float3 baseColor = uniforms.materialColor.rgb;
	float ambient = uniforms.lightDir.w;
	float3 diffuse = baseColor * uniforms.lightColor.rgb * ndotl * shadow;
	float3 spec = uniforms.lightColor.rgb * specular * shadow * 0.35;
	return float4(baseColor * ambient + diffuse + spec, uniforms.materialColor.a);
}

fragment float4 RiemannOutlineFS(
	VSOut in [[stage_in]],
	constant Uniforms& uniforms [[buffer(1)]])
{
	return uniforms.outlineColor;
}

struct ImguiVertex
{
	packed_float2 position;
	packed_float2 texCoord;
	uint color;
};

struct ImguiUniforms
{
	float2 screenSize;
	float2 padding;
};

struct ImguiVSOut
{
	float4 position [[position]];
	float2 texCoord;
	float4 color;
};

vertex ImguiVSOut RiemannImguiVS(
	uint vertexId [[vertex_id]],
	const device ImguiVertex* vertices [[buffer(0)]],
	constant ImguiUniforms& uniforms [[buffer(1)]])
{
	ImguiVertex vertex = vertices[vertexId];
	float2 p = vertex.position / uniforms.screenSize;

	ImguiVSOut out;
	out.position = float4(p.x * 2.0 - 1.0, 1.0 - p.y * 2.0, 0.0, 1.0);
	out.texCoord = vertex.texCoord;
	out.color = float4(
		float(vertex.color & 255u) / 255.0,
		float((vertex.color >> 8u) & 255u) / 255.0,
		float((vertex.color >> 16u) & 255u) / 255.0,
		float((vertex.color >> 24u) & 255u) / 255.0);
	return out;
}

fragment float4 RiemannImguiFS(
	ImguiVSOut in [[stage_in]],
	texture2d<float> colorTexture [[texture(0)]],
	sampler colorSampler [[sampler(0)]])
{
	return in.color * colorTexture.sample(colorSampler, in.texCoord);
}
