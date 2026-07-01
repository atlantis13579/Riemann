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
