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

// RGBColor
struct XYZColor
{
  float X = 0;
  float Y = 0;
  float Z = 0;
};

static void xyzToRgb(const XYZColor &xyz, RGBColor &rgb)
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

static void labToRgb(const LabColor &lab, RGBColor &rgb)
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

RGBColor::RGBColor(uint8_t r, uint8_t g, uint8_t b) : R(r), G(g), B(b) {}

RGBColor::RGBColor(const HSVColor& color)
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

RGBColor::RGBColor(const Grayscale& c) : R(c.I), G(c.I), B(c.I) {}

RGBColor::RGBColor(const LabColor& color)
{
  labToRgb(color, *this);
}

void RGBColor::setFromYuv(int y, int u, int v)
{
  int uv_r, uv_g, uv_b;
  u -= UV_OFFSET;
  v -= UV_OFFSET;

  uv_r=YUV2RGB_12*u+YUV2RGB_13*v;
  uv_g=YUV2RGB_22*u+YUV2RGB_23*v;
  uv_b=YUV2RGB_32*u+YUV2RGB_33*v;

  y=YUV2RGB_11*(y - Y_OFFSET);
  R = std::clamp((y + uv_r) >> 8, 0, 255); // r
  G = std::clamp((y + uv_g) >> 8, 0, 255); // g
  B = std::clamp((y + uv_b) >> 8, 0, 255); // b 
}

// RGBAColor
RGBAColor::RGBAColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a) : RGBColor(r,g,b), A(a) {}

RGBAColor::RGBAColor(const RGBColor& color) : RGBColor(color), A(255) {}

RGBAColor::RGBAColor(const HSVColor& color) : RGBColor(color), A(255) {}

RGBAColor::RGBAColor(const Grayscale& color) : RGBColor(color), A(255) {}

RGBAColor::RGBAColor(const LabColor& color) : RGBColor(color), A(255) {}

// Grayscale
Grayscale::Grayscale(uint8_t i) : I(i) {}

Grayscale::Grayscale(const RGBColor& color)
{
  I = (uint8_t)(0.299f * (float)color.R + 0.587f * (float)color.G + 0.114f * (float)color.B);
}

Grayscale::Grayscale(const LabColor& color)
{
  *this = RGBColor(color);
}

Grayscale::Grayscale(const HSVColor& color)
{
  I = color.V;
}

// HSVColor
HSVColor::HSVColor(float h, float s, float v) : H(h), S(s), V(v) {}

HSVColor::HSVColor(const RGBColor& color)
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

HSVColor::HSVColor(const Grayscale& color) : H(0), S(0), V(color.I) {}

HSVColor::HSVColor(const LabColor& color)
{
  *this = RGBColor(color);
}

// LabColor
static void rgbToXyz(const RGBColor &rgb, XYZColor &xyz)
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

static void rgbToLab(const RGBColor &rgb, LabColor &lab)
{
  XYZColor xyz;
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

LabColor::LabColor(float l, float a, float b) : L(l), a(a), b(b) {}

LabColor::LabColor(const RGBColor& color)
{
  rgbToLab(color, *this);
}

LabColor::LabColor(const HSVColor& color)
{
  RGBColor intermediate = color;
  rgbToLab(intermediate, *this);
}

LabColor::LabColor(const Grayscale& color)
{
  RGBColor intermediate = color;
  rgbToLab(intermediate, *this);
}

float LabColor::deltaE(const LabColor& other) const
{
  return sqrtf(powf(L-other.L, 2) + powf(a-other.a, 2) + powf(b-other.b, 2));
}