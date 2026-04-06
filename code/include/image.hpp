#pragma once

#include <cassert>
#include <vecmath.h>

// Simple image class
class Image {

public:

    Image(int w, int h);
    ~Image();

    int Width() const;

    int Height() const;

    const Vector3f &GetPixel(int x, int y) const;

    void SetAllPixels(const Vector3f &color);

    void SetPixel(int x, int y, const Vector3f &color);

    static Image *LoadPPM(const char *filename);

    void SavePPM(const char *filename) const;

    static Image *LoadTGA(const char *filename);

    void SaveTGA(const char *filename) const;

    int SaveBMP(const char *filename);

    void SaveImage(const char *filename);

private:

    int width;
    int height;
    Vector3f *data;

};
