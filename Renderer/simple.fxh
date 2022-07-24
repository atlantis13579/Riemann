//--------------------------------------------------------------------------------------
// File: Tutorial04.fx
//
// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License (MIT).
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// Constant Buffer Variables
//--------------------------------------------------------------------------------------
cbuffer ConstantBuffer : register( b0 )
{
	matrix World;
	matrix View;
	matrix Projection;
    float4 EyePos;
}

//--------------------------------------------------------------------------------------
struct VS_OUTPUT
{
    float4 Pos : SV_POSITION;
    float3 PosWorld : COLOR0;
    float3 Diffuse : COLOR1;
    float3 Normal : NORMAL;
};

//--------------------------------------------------------------------------------------
// Vertex Shader
//--------------------------------------------------------------------------------------
VS_OUTPUT VS(float4 Pos : POSITION, float3 Normal : NORMAL)
{
    VS_OUTPUT output = (VS_OUTPUT)0;
    output.Pos = mul( Pos, World );
    output.Pos = mul( output.Pos, View );
    output.Pos = mul( output.Pos, Projection );
    output.PosWorld = mul(Pos, World).xyz;
    output.Diffuse = float3(0.5 + 0.5 * Normal.x, 0.5 + 0.5 * Normal.y, 0.5 + 0.5 * Normal.z);
    output.Normal = mul(Normal, World);
    return output;
}


//--------------------------------------------------------------------------------------
// Pixel Shader
//--------------------------------------------------------------------------------------
float4 PS(VS_OUTPUT input) : SV_Target
{
    float3 normal = normalize(input.Normal);
    float3 eyeVec = normalize(EyePos.xyz - input.PosWorld);
    float cosTh = clamp(dot(normal, eyeVec), 0, 1);
    float3 Color = 0.7 * input.Diffuse + 0.5 * input.Diffuse * cosTh + input.Diffuse * pow(cosTh, 100);
    return float4(Color.x, Color.y, Color.z, 1.0);
}
