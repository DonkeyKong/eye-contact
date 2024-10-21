#include "Color.hpp"

#include <cmath>

#include <algorithm>
#include <random>
#include <stdexcept>

# define Y_OFFSET   16
# define UV_OFFSET 128
# define YUV2RGB_11  298
# define YUV2RGB_12   -1
# define YUV2RGB_13  409
# define YUV2RGB_22 -100
# define YUV2RGB_23 -210
# define YUV2RGB_32  519
# define YUV2RGB_33    0

// RGB24
struct XYZ3f
{
  float X = 0;
  float Y = 0;
  float Z = 0;
};

static void xyzToRgb(const XYZ3f &xyz, RGB24 &rgb)
{
  double x = xyz.X / 100.0;
  double y = xyz.Y / 100.0;
  double z = xyz.Z / 100.0;

  double r = x * 3.2404542 + y * -1.5371385 + z * -0.4985314;
  double g = x * -0.9692660 + y * 1.8760108 + z * 0.0415560;
  double b = x * 0.0556434 + y * -0.2040259 + z * 1.0572252;

  r = ((r > 0.0031308) ? (1.055 * pow(r, 1 / 2.4) - 0.055) : (12.92 * r)) * 255.0;
  g = ((g > 0.0031308) ? (1.055 * pow(g, 1 / 2.4) - 0.055) : (12.92 * g)) * 255.0;
  b = ((b > 0.0031308) ? (1.055 * pow(b, 1 / 2.4) - 0.055) : (12.92 * b)) * 255.0;

  rgb.R = r;
  rgb.G = g;
  rgb.B = b;
}

static void labToRgb(const Lab3f &lab, RGB24 &rgb)
{
  double y = (lab.L + 16.0) / 116.0;
  double x = lab.a / 500.0 + y;
  double z = y - lab.b / 200.0;

  double x3 = std::pow(x, 3);
  double y3 = std::pow(y, 3);
  double z3 = std::pow(z, 3);

  x = ((x3 > 0.008856) ? x3 : ((x - 16.0 / 116.0) / 7.787)) * 95.047;
  y = ((y3 > 0.008856) ? y3 : ((y - 16.0 / 116.0) / 7.787)) * 100.0;
  z = ((z3 > 0.008856) ? z3 : ((z - 16.0 / 116.0) / 7.787)) * 108.883;

  xyzToRgb({(float)x, (float)y, (float)z}, rgb);
}

RGB24::RGB24(uint8_t r, uint8_t g, uint8_t b) : R(r), G(g), B(b) {}

RGB24::RGB24(const HSV3f& color)
{
  float r, g, b;
  int range = (int)std::floor(color.H / 60.0f);
  float c = color.V * color.S;
  float x = c * (1 - std::abs(fmod(color.H / 60.0f, 2.0f) - 1));
  float m = color.V - c;

  switch (range)
  {
  case 0:
    r = (c + m);
    g = (x + m);
    b = m;
    break;
  case 1:
    r = (x + m);
    g = (c + m);
    b = m;
    break;
  case 2:
    r = m;
    g = (c + m);
    b = (x + m);
    break;
  case 3:
    r = m;
    g = (x + m);
    b = (c + m);
    break;
  case 4:
    r = (x + m);
    g = m;
    b = (c + m);
    break;
  default: // case 5:
    r = (c + m);
    g = m;
    b = (x + m);
    break;
  }

  R = (uint8_t)std::clamp((r * 255.0f), 0.0f, 255.0f);
  G = (uint8_t)std::clamp((g * 255.0f), 0.0f, 255.0f);
  B = (uint8_t)std::clamp((b * 255.0f), 0.0f, 255.0f);
}

RGB24::RGB24(const Gray8& c) : R(c.I), G(c.I), B(c.I) {}

RGB24::RGB24(const Lab3f& color)
{
  labToRgb(color, *this);
}

RGB24::RGB24(const YUV24& c)
{
  int y = YUV2RGB_11*((int)c.Y - Y_OFFSET);
  int u = (int)c.U - UV_OFFSET;
  int v = (int)c.V - UV_OFFSET;

  int uv_r=YUV2RGB_12*u+YUV2RGB_13*v;
  int uv_g=YUV2RGB_22*u+YUV2RGB_23*v;
  int uv_b=YUV2RGB_32*u+YUV2RGB_33*v;

  R = std::clamp((y + uv_r) >> 8, 0, 255); // r
  G = std::clamp((y + uv_g) >> 8, 0, 255); // g
  B = std::clamp((y + uv_b) >> 8, 0, 255); // b 
}

// RGBA32
RGBA32::RGBA32(uint8_t r, uint8_t g, uint8_t b, uint8_t a) : RGB24(r,g,b), A(a) {}

RGBA32::RGBA32(const RGB24& color) : RGB24(color), A(255) {}

RGBA32::RGBA32(const HSV3f& color) : RGB24(color), A(255) {}

RGBA32::RGBA32(const Gray8& color) : RGB24(color), A(255) {}

RGBA32::RGBA32(const Lab3f& color) : RGB24(color), A(255) {}

// Gray8
Gray8::Gray8(uint8_t i) : I(i) {}

Gray8::Gray8(const RGB24& color)
{
  I = (uint8_t)(0.299f * (float)color.R + 0.587f * (float)color.G + 0.114f * (float)color.B);
}

Gray8::Gray8(const Lab3f& color)
{
  *this = RGB24(color);
}

Gray8::Gray8(const HSV3f& color)
{
  I = color.V;
}

Gray8::Gray8(const YUV24& color)
{
  I = color.Y;
}

// HSV3f
HSV3f::HSV3f(float h, float s, float v) : H(h), S(s), V(v) {}

HSV3f::HSV3f(const RGB24& color)
{
  float r = std::clamp(color.R / 255.0f, 0.0f, 1.0f);
  float g = std::clamp(color.G / 255.0f, 0.0f, 1.0f);
  float b = std::clamp(color.B / 255.0f, 0.0f, 1.0f);

  float min = std::min(r, std::min(g, b));
  float max = std::max(r, std::max(g, b));
  float delta = max - min;

  V = max;
  S = (max > 1e-3) ? (delta / max) : 0;

  if (delta == 0)
  {
    H = 0;
  }
  else
  {
    if (r == max)
      H = (g - b) / delta;
    else if (g == max)
      H = 2 + (b - r) / delta;
    else if (b == max)
      H = 4 + (r - g) / delta;

    H *= 60;
    H = fmod(H + 360, 360);
  }
}

HSV3f::HSV3f(const Gray8& color) : H(0), S(0), V(color.I) {}

HSV3f::HSV3f(const Lab3f& color)
{
  *this = RGB24(color);
}

// Lab3f
static void rgbToXyz(const RGB24 &rgb, XYZ3f &xyz)
{
  double r = (float)rgb.R / 255.0;
  double g = (float)rgb.G / 255.0;
  double b = (float)rgb.B / 255.0;

  r = ((r > 0.04045) ? pow((r + 0.055) / 1.055, 2.4) : (r / 12.92)) * 100.0;
  g = ((g > 0.04045) ? pow((g + 0.055) / 1.055, 2.4) : (g / 12.92)) * 100.0;
  b = ((b > 0.04045) ? pow((b + 0.055) / 1.055, 2.4) : (b / 12.92)) * 100.0;

  xyz.X = r * 0.4124564 + g * 0.3575761 + b * 0.1804375;
  xyz.Y = r * 0.2126729 + g * 0.7151522 + b * 0.0721750;
  xyz.Z = r * 0.0193339 + g * 0.1191920 + b * 0.9503041;
}

static void rgbToLab(const RGB24 &rgb, Lab3f &lab)
{
  XYZ3f xyz;
  rgbToXyz(rgb, xyz);

  double x = xyz.X / 95.047;
  double y = xyz.Y / 100.00;
  double z = xyz.Z / 108.883;

  x = (x > 0.008856) ? cbrt(x) : (7.787 * x + 16.0 / 116.0);
  y = (y > 0.008856) ? cbrt(y) : (7.787 * y + 16.0 / 116.0);
  z = (z > 0.008856) ? cbrt(z) : (7.787 * z + 16.0 / 116.0);

  lab.L = (116.0 * y) - 16;
  lab.a = 500 * (x - y);
  lab.b = 200 * (y - z);
}

Lab3f::Lab3f(float l, float a, float b) : L(l), a(a), b(b) {}

Lab3f::Lab3f(const RGB24& color)
{
  rgbToLab(color, *this);
}

Lab3f::Lab3f(const HSV3f& color)
{
  RGB24 intermediate = color;
  rgbToLab(intermediate, *this);
}

Lab3f::Lab3f(const Gray8& color)
{
  RGB24 intermediate = color;
  rgbToLab(intermediate, *this);
}

float Lab3f::deltaE(const Lab3f& other) const
{
  return sqrtf(powf(L-other.L, 2) + powf(a-other.a, 2) + powf(b-other.b, 2));
}

YUV24::YUV24(uint8_t y, uint8_t u, uint8_t v) : Y(y), U(u), V(v) {}