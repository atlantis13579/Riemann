
#pragma once

// https://github.com/miloyip/svpng

#include <stdio.h>
#include <float.h>
#include <vector>

class LibPNG
{
private:
    static void svpng(FILE *fp, unsigned int w, unsigned int h, const unsigned char* img, int alpha) {
        static const unsigned t[] = { 0, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        /* CRC32 Table */    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c };
        unsigned int a = 1, b = 0, c, p = w * (alpha ? 4 : 3) + 1, x, y, i;   /* ADLER-a, ADLER-b, CRC, pitch */
    #define SVPNG_PUT(u) fputc(u, fp)
    #define SVPNG_U8A(ua, l) for (i = 0; i < l; i++) SVPNG_PUT((ua)[i]);
    #define SVPNG_U32(u) do { SVPNG_PUT((u) >> 24); SVPNG_PUT(((u) >> 16) & 255); SVPNG_PUT(((u) >> 8) & 255); SVPNG_PUT((u) & 255); } while(0)
    #define SVPNG_U8C(u) do { SVPNG_PUT(u); c ^= (u); c = (c >> 4) ^ t[c & 15]; c = (c >> 4) ^ t[c & 15]; } while(0)
    #define SVPNG_U8AC(ua, l) for (i = 0; i < l; i++) SVPNG_U8C((ua)[i])
    #define SVPNG_U16LC(u) do { SVPNG_U8C((u) & 255); SVPNG_U8C(((u) >> 8) & 255); } while(0)
    #define SVPNG_U32C(u) do { SVPNG_U8C((u) >> 24); SVPNG_U8C(((u) >> 16) & 255); SVPNG_U8C(((u) >> 8) & 255); SVPNG_U8C((u) & 255); } while(0)
    #define SVPNG_U8ADLER(u) do { SVPNG_U8C(u); a = (a + (u)) % 65521; b = (b + a) % 65521; } while(0)
    #define SVPNG_BEGIN(s, l) do { SVPNG_U32(l); c = ~0U; SVPNG_U8AC(s, 4); } while(0)
    #define SVPNG_END() SVPNG_U32(~c)
        SVPNG_U8A("\x89PNG\r\n\32\n", 8);           /* Magic */
        SVPNG_BEGIN("IHDR", 13);                    /* IHDR chunk { */
        SVPNG_U32C(w); SVPNG_U32C(h);               /*   Width & Height (8 bytes) */
        SVPNG_U8C(8); SVPNG_U8C(alpha ? 6 : 2);     /*   Depth=8, Color=True color with/without alpha (2 bytes) */
        SVPNG_U8AC("\0\0\0", 3);                    /*   Compression=Deflate, Filter=No, Interlace=No (3 bytes) */
        SVPNG_END();                                /* } */
        SVPNG_BEGIN("IDAT", 2 + h * (5 + p) + 4);   /* IDAT chunk { */
        SVPNG_U8AC("\x78\1", 2);                    /*   Deflate block begin (2 bytes) */
        for (y = 0; y < h; y++) {                   /*   Each horizontal line makes a block for simplicity */
            SVPNG_U8C(y == h - 1);                  /*   1 for the last block, 0 for others (1 byte) */
            SVPNG_U16LC(p); SVPNG_U16LC(~p);        /*   Size of block in little endian and its 1's complement (4 bytes) */
            SVPNG_U8ADLER(0);                       /*   No filter prefix (1 byte) */
            for (x = 0; x < p - 1; x++, img++)
                SVPNG_U8ADLER(*img);                /*   Image pixel data */
        }
        SVPNG_U32C((b << 16) | a);                  /*   Deflate block end with adler (4 bytes) */
        SVPNG_END();                                /* } */
        SVPNG_BEGIN("IEND", 0); SVPNG_END();        /* IEND chunk {} */
    }
    
    inline static int  toInt(float x)
    {
        return int(x * 255.0f + 0.5f);
    }
    
public:
    static void WritePNG(const char* filename, float *ptr, int w, int h)
    {
        FILE* fp = fopen(filename, "wb");
        if (fp == nullptr)
            return;
        float fmin = FLT_MAX;
        float fmax = -FLT_MAX;
        for (int i = 0; i < w * h; ++i)
        {
            if (ptr[i] > fmax)
                fmax = ptr[i];
            if (ptr[i] < fmin)
                fmin = ptr[i];
        }
        std::vector<unsigned char> rgb;
        rgb.resize(w * h * 3);
        for (int i = 0; i < w * h; ++i)
        {
            unsigned char *p = &rgb[3*i];
            float x = (ptr[i] - fmin) / (fmax - fmin);
            p[0] = p[1] = p[2] = toInt(x);
        }
        svpng(fp, w, h, &rgb[0], 0);
        fclose(fp);
    }
    
    static void WritePNG(const char* filename, float *ptr, int w, int h, float fmin, float fmax)
    {
        FILE* fp = fopen(filename, "wb");
        if (fp == nullptr)
            return;
        std::vector<unsigned char> rgb;
        rgb.resize(w * h * 3);
        for (int i = 0; i < w * h; ++i)
        {
            unsigned char *p = &rgb[3*i];
            float x = p[i];
            if (x >= fmax)
                x = 1.0f;
            else if (x <= fmin)
                x = 0.0f;
            else
                x = (p[i] - fmin) / (fmax - fmin);
            p[0] = p[1] = p[2] = toInt(x);
        }
        svpng(fp, w, h, &rgb[0], 0);
        fclose(fp);
    }    
};
