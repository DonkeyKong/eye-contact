#include "Color.hpp"

#include <cmath>

#include <algorithm>
#include <random>
#include <stdexcept>

template <typename T>
static T remap(T value, T inMin, T inMax, T outMin, T outMax)
{
  double t = (double)(value - inMin)/(double)(inMax-inMin);
  double out = t * (double)(outMax-outMin) + (double)(outMin);
  return std::clamp((T)std::clamp(out, (double)outMin, (double)outMax), outMin, outMax);
}

RGBAColor::RGBAColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a) : R(r), G(g), B(b), A(a)
{
}

RGBAColor::RGBAColor(const RGBColor& color) : R(color.R), G(color.G), B(color.B), A(255)
{
}

RGBColor::RGBColor(const RGBAColor& color) : R(color.R), G(color.G), B(color.B)
{
}

RGBColor::RGBColor(uint8_t r, uint8_t g, uint8_t b) : R(r), G(g), B(b)
{
}

RGBAColor HSVColor::toRGB()
{
  float r, g, b;
  int range = (int)std::floor(H / 60.0f);
  float c = V * S;
  float x = c * (1 - std::abs(fmod(H / 60.0f, 2.0f) - 1));
  float m = V - c;

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

  return {
      (uint8_t)std::clamp((r * 255.0f), 0.0f, 255.0f),
      (uint8_t)std::clamp((g * 255.0f), 0.0f, 255.0f),
      (uint8_t)std::clamp((b * 255.0f), 0.0f, 255.0f),
      (uint8_t)std::clamp((A * 255.0f), 0.0f, 255.0f)};
}

HSVColor RGBAColor::toHSV() const
{
  HSVColor hsv;

  float r = std::clamp(R / 255.0f, 0.0f, 1.0f);
  float g = std::clamp(G / 255.0f, 0.0f, 1.0f);
  float b = std::clamp(B / 255.0f, 0.0f, 1.0f);
  hsv.A = std::clamp(A / 255.0f, 0.0f, 1.0f);

  float min = std::min(r, std::min(g, b));
  float max = std::max(r, std::max(g, b));
  float delta = max - min;

  hsv.V = max;
  hsv.S = (max > 1e-3) ? (delta / max) : 0;

  if (delta == 0)
  {
    hsv.H = 0;
  }
  else
  {
    if (r == max)
      hsv.H = (g - b) / delta;
    else if (g == max)
      hsv.H = 2 + (b - r) / delta;
    else if (b == max)
      hsv.H = 4 + (r - g) / delta;

    hsv.H *= 60;
    hsv.H = fmod(hsv.H + 360, 360);
  }
  return hsv;
}

uint8_t RGBAColor::getDarkestChannel() const
{
  return std::min({R, G, B});
}

uint8_t RGBAColor::getBrightestChannel() const
{
  return std::max({R, G, B});
}


uint8_t RGBAColor::getGrayValue() const
{
  return (uint8_t)(0.299f * (float)R + 0.587f * (float)G + 0.114f * (float)B);
}

# define Y_OFFSET   16
# define UV_OFFSET 128
# define YUV2RGB_11  298
# define YUV2RGB_12   -1
# define YUV2RGB_13  409
# define YUV2RGB_22 -100
# define YUV2RGB_23 -210
# define YUV2RGB_32  519
# define YUV2RGB_33    0

void yuvToRgb(int y, int u, int v, uint8_t& r, uint8_t& g, uint8_t& b)
{
  int uv_r, uv_g, uv_b;
  u -= UV_OFFSET;
  v -= UV_OFFSET;

  uv_r=YUV2RGB_12*u+YUV2RGB_13*v;
  uv_g=YUV2RGB_22*u+YUV2RGB_23*v;
  uv_b=YUV2RGB_32*u+YUV2RGB_33*v;

  y=YUV2RGB_11*(y - Y_OFFSET);
  r = std::clamp((y + uv_r) >> 8, 0, 255); // r
  g = std::clamp((y + uv_g) >> 8, 0, 255); // g
  b = std::clamp((y + uv_b) >> 8, 0, 255); // b 
}

void RGBAColor::setFromYuv(int y, int u, int v)
{
  yuvToRgb(y, u, v, R, G, B);
  A = 255; // YUV colors are all opaque
}

void RGBColor::setFromYuv(int y, int u, int v)
{
  yuvToRgb(y, u, v, R, G, B);
}

void rgbToXyz(const RGBAColor &rgb, XYZColor &xyz)
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

void xyzToRgb(const XYZColor &xyz, RGBAColor &rgb)
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
  rgb.A = 255;
}

static void rgbToLab(const RGBAColor &rgb, LabColor &lab)
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

static void labToRgb(const LabColor &lab, RGBAColor &rgb)
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

LabColor RGBAColor::toLab() const
{
  LabColor lab;
  rgbToLab(*this, lab);
  return lab;
}

RGBAColor LabColor::toRgba() const
{
  RGBAColor rgb;
  labToRgb(*this, rgb);
  return rgb;
}

float LabColor::deltaE(const LabColor& other) const
{
  return sqrtf(powf(L-other.L, 2) + powf(a-other.a, 2) + powf(b-other.b, 2));
}