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

fragment float4 RiemannMainFS(
	VSOut in [[stage_in]],
	constant Uniforms& uniforms [[buffer(1)]],
	depth2d<float> shadowMap [[texture(0)]],
	sampler shadowSampler [[sampler(0)]])
{
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
