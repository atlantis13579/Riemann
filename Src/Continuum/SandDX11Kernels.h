// Native DX11 sand kernels.
R"SHADER(

RWByteAddressBuffer b0 : register(u0), b1 : register(u1), b2 : register(u2), b3 : register(u3);

RWByteAddressBuffer b4 : register(u4), b5 : register(u5), b6 : register(u6), b7 : register(u7);

cbuffer ParametersBuffer : register(b0)
{
    uint4 constants[16];
};

groupshared float shared0[256];

float ReadFloat(RWByteAddressBuffer b, uint i)
{
    return asfloat(b.Load(i * 4));
}

uint ReadUInt(RWByteAddressBuffer b, uint i)
{
    return b.Load(i * 4);
}

void WriteFloat(RWByteAddressBuffer b, uint i, float v)
{
    b.Store(i * 4, asuint(v));
}

void WriteUInt(RWByteAddressBuffer b, uint i, uint v)
{
    b.Store(i * 4, v);
}

uint Exchange(RWByteAddressBuffer b, uint i, uint v)
{
    uint old;
    b.InterlockedExchange(i * 4, v, old);
    return old;
}

float PF(uint4 data[16], uint i)
{
    return asfloat(data[i / 4][i % 4]);
}

int PI(uint4 data[16], uint i)
{
    return (int)data[i / 4][i % 4];
}

int SandX(int k)
{
    return k == 0 || k == 4 || k == 6 ? -1 : (k == 1 || k == 5 || k == 7 ? 1 : 0);
}

int SandZ(int k)
{
    return k == 2 || k == 4 || k == 7 ? -1 : (k == 3 || k == 5 || k == 6 ? 1 : 0);
}

[numthreads(256, 1, 1)] void SandFlux(uint3 tid : SV_DispatchThreadID, uint3 lid : SV_GroupThreadID,
                                      uint3 gid : SV_GroupID)

{
    int i = (int)tid.x, n = PI(constants, 0);
    if (i >= n * n)
        return;

    int x = i % n, z = i / n;
    float h = ReadFloat(b0, i), sum = 0;
    float flow[8];

    for (int k = 0; k < 8; ++k)

    {

        int nx = x + SandX(k), nz = z + SandZ(k), j = nz * n + nx;
        float f = 0;

        if (nx >= 0 && nx < n && nz >= 0 && nz < n)

            if (ReadFloat(b0, j) < ReadFloat(b2, j) - 1.e-6)

                f = max(0.0, h - ReadFloat(b0, j) - PF(constants, 1) * (k < 4 ? 1.0 : 1.41421356237)) *
                    PF(constants, 2);

        flow[k] = f;
        sum += f;
    }

    float scale = sum > h ? h / sum : 1.0;

    for (int k = 0; k < 8; ++k)
        WriteFloat(b1, i * 8 + k, flow[k] * scale);
}

    [numthreads(256, 1, 1)] void SandGather(uint3 tid : SV_DispatchThreadID, uint3 lid : SV_GroupThreadID,
                                            uint3 gid : SV_GroupID)

{

    int i = (int)tid.x, n = PI(constants, 0);
    if (i >= n * n)
        return;

    int x = i % n, z = i / n;
    float h = ReadFloat(b0, i);

    for (int k = 0; k < 8; ++k)

    {

        h -= ReadFloat(b1, i * 8 + k);
        int nx = x + SandX(k), nz = z + SandZ(k);

        if (nx >= 0 && nx < n && nz >= 0 && nz < n)
            h += ReadFloat(b1, (nz * n + nx) * 8 + (k ^ 1));
    }

    WriteFloat(b3, i, max(0.0, h));
}
)SHADER"
