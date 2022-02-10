#pragma once

#include <stdint.h>

// https://en.wikipedia.org/wiki/Half-precision_floating-point_format

union Float32
{
	float Val;
	struct
	{
		uint32_t Frac : 23;
		uint32_t Exp : 8;
		uint32_t Sign : 1;
	} IEEE;
};

union Float16
{
	uint16_t Bits;
	struct
	{
		uint16_t Frac : 10;
		uint16_t Exp : 5;
		uint16_t Sign : 1;
	} IEEE;

	static Float16 FromFloat32(float src)
	{
		Float32 f;
		f.Val = src;

		Float16	dst;

		dst.IEEE.Sign = f.IEEE.Sign;

		if (!f.IEEE.Exp)
		{
			dst.IEEE.Frac = 0;
			dst.IEEE.Exp = 0;
		}
		else if (f.IEEE.Exp == 0xff)
		{
			// NaN or INF
			dst.IEEE.Frac = (f.IEEE.Frac != 0) ? 1 : 0;
			dst.IEEE.Exp = 31;
		}
		else
		{
			// regular number
			int new_exp = f.IEEE.Exp - 127;

			if (new_exp < -24)
			{ // this maps to 0
				dst.IEEE.Frac = 0;
				dst.IEEE.Exp = 0;
			}

			else if (new_exp < -14)
			{
				// this maps to a denorm
				dst.IEEE.Exp = 0;
				unsigned int exp_val = (unsigned int)(-14 - new_exp);  // 2^-exp_val
				switch (exp_val)
				{
				case 0:
					dst.IEEE.Frac = 0;
					break;
				case 1: dst.IEEE.Frac = 512 + (f.IEEE.Frac >> 14); break;
				case 2: dst.IEEE.Frac = 256 + (f.IEEE.Frac >> 15); break;
				case 3: dst.IEEE.Frac = 128 + (f.IEEE.Frac >> 16); break;
				case 4: dst.IEEE.Frac = 64 + (f.IEEE.Frac >> 17); break;
				case 5: dst.IEEE.Frac = 32 + (f.IEEE.Frac >> 18); break;
				case 6: dst.IEEE.Frac = 16 + (f.IEEE.Frac >> 19); break;
				case 7: dst.IEEE.Frac = 8 + (f.IEEE.Frac >> 20); break;
				case 8: dst.IEEE.Frac = 4 + (f.IEEE.Frac >> 21); break;
				case 9: dst.IEEE.Frac = 2 + (f.IEEE.Frac >> 22); break;
				case 10: dst.IEEE.Frac = 1; break;
				}
			}
			else if (new_exp > 15)
			{ // map this value to infinity
				dst.IEEE.Frac = 0;
				dst.IEEE.Exp = 31;
			}
			else
			{
				dst.IEEE.Exp = new_exp + 15;
				dst.IEEE.Frac = (f.IEEE.Frac >> 13);
			}
		}

		return dst;
	}

	void FromFloat(float src)
	{
		*this = Float16::FromFloat32(src);
	}

	float ToFloat() const
	{
		Float32 dst;
		dst.IEEE.Sign = IEEE.Sign;

		if (!IEEE.Exp)
		{
			if (!IEEE.Frac)
			{
				dst.IEEE.Frac = 0;
				dst.IEEE.Exp = 0;
			}
			else
			{
				const float half_denorm = (1.0f / 16384.0f);
				float mantissa = ((float)(IEEE.Frac)) / 1024.0f;
				float sgn = (IEEE.Sign) ? -1.0f : 1.0f;
				dst.Val = sgn * mantissa * half_denorm;
			}
		}
		else if (31 == IEEE.Exp)
		{
			dst.IEEE.Exp = 0xff;
			dst.IEEE.Frac = (IEEE.Frac != 0) ? 1 : 0;
		}
		else
		{
			dst.IEEE.Exp = IEEE.Exp + 112;
			dst.IEEE.Frac = (IEEE.Frac << 13);
		}
		return dst.Val;
	}
};