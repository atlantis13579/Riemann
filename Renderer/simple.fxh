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
    float3 Color : COLOR1;
    float3 Normal : COLOR2;
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
    output.Color = Normal ;
    output.Normal = mul(Normal, World);
    return output;
}


//--------------------------------------------------------------------------------------
// Pixel Shader
//--------------------------------------------------------------------------------------
float4 PS(VS_OUTPUT input) : SV_Target
{
    float3 eyeVec = normalize(EyePos.xyz - input.PosWorld);
    float3 lightVec = normalize(float3(10.0, 10.0, 12.0) - input.PosWorld);
    float3 halfVec = 0.5 * (eyeVec + lightVec);
    float cosTh = clamp(dot(input.Normal.xyz, halfVec), 0, 1);
    float cosTi = clamp(dot(input.Normal.xyz, lightVec), 0, 1);
    float3 SurfaceDiffuse = float3(0.5 + 0.5 * input.Color.x, 0.5 + 0.5 * input.Color.y, 0.5 + 0.5 * input.Color.z);
    float3 Lo = (SurfaceDiffuse + float3(0.3, 0.3, 0.3) * pow(cosTh, 8)) * float3(2.0, 2.0, 2.0) * cosTi;
    return float4(Lo.x, Lo.y, Lo.z, 1.0);
}
