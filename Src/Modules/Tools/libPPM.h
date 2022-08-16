
#pragma once

#include <float.h>

class LibPPM
{
public:
    static void WritePPM(const char* filename, float* p, int w, int h)
    {
        FILE* f = fopen(filename, "w");
        if (f)
        {
            float fmin = FLT_MAX;
            float fmax = -FLT_MAX;
            for (int i = 0; i < w * h; ++i)
            {
                if (p[i] > fmax)
                    fmax = p[i];
                if (p[i] < fmin)
                    fmin = p[i];
            }
            fprintf(f, "P3\n%d %d\n%d\n", w, h, 255);
            for (int i = 0; i < w * h; ++i)
            {
                float x = (p[i] - fmin) / (fmax - fmin);
                fprintf(f, "%d %d %d ", toInt(x), toInt(x), toInt(x));
            }
            fclose(f);
        }
    }
    
    static void WritePPM(const char* filename, float* p, int w, int h, float fmin, float fmax)
    {
        FILE* f = fopen(filename, "w");
        if (f)
        {
            fprintf(f, "P3\n%d %d\n%d\n", w, h, 255);
            for (int i = 0; i < w * h; ++i)
            {
                float x = p[i];
                if (x >= fmax)
                    x = 1.0f;
                else if (x <= fmin)
                    x = 0.0f;
                else
                    x = (p[i] - fmin) / (fmax - fmin);
                fprintf(f, "%d %d %d ", toInt(x), toInt(x), toInt(x));
            }
            fclose(f);
        }
    }

private:
    inline static int  toInt(float x)
    {
        return int(x * 255.0f + 0.5f);
    }
};
