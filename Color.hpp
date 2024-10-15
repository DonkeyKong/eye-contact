#pragma once

#include <stdint.h>
#include <unordered_map>
#include <vector>

struct RGBColor;
struct RGBAColor;
struct Grayscale;
struct HSVColor;
struct LabColor;

#pragma pack(push, 1)
struct RGBColor
{
  uint8_t R = 0;
  uint8_t G = 0;
  uint8_t B = 0;

  RGBColor() = default;
  RGBColor(const RGBColor& color) = default;
  RGBColor(uint8_t r, uint8_t g, uint8_t b);
  RGBColor(const HSVColor& color);
  RGBColor(const Grayscale& color);
  RGBColor(const LabColor& color);

  void setFromYuv(int y, int u, int v);
};

struct RGBAColor : public RGBColor
{
  uint8_t A = 255;

  RGBAColor() = default;
  RGBAColor(const RGBAColor& color) = default;
  RGBAColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a);
  RGBAColor(const RGBColor& color);
  RGBAColor(const HSVColor& color);
  RGBAColor(const Grayscale& color);
  RGBAColor(const LabColor& color);
};

struct Grayscale
{
  uint8_t I = 0;

  Grayscale() = default;
  Grayscale(const Grayscale& color) = default;
  Grayscale(uint8_t i);
  Grayscale(const RGBColor& color);
  Grayscale(const LabColor& color);
  Grayscale(const HSVColor& color);

  void setFromYuv(int y, int u, int v);
};

struct HSVColor
{
  float H = 0.0f;
  float S = 0.0f;
  float V = 0.0f;

  HSVColor() = default;
  HSVColor(const HSVColor& color) = default;
  HSVColor(float h, float s, float v);
  HSVColor(const RGBColor& color);
  HSVColor(const Grayscale& color);
  HSVColor(const LabColor& color);
};

struct LabColor
{
  float L = 0;
  float a = 0;
  float b = 0;

  LabColor() = default;
  LabColor(const LabColor& color) = default;
  LabColor(float l, float a, float b);
  LabColor(const RGBColor& color);
  LabColor(const HSVColor& color);
  LabColor(const Grayscale& color);

  float deltaE(const LabColor& other) const;

  LabColor operator+ (const LabColor& c)  const
  {
    return {L + c.L, a + c.a, b + c.b};
  }

  LabColor operator* (const LabColor& c)  const
  {
    return {L * c.L, a * c.a, b * c.b};
  }

  void operator+= (const LabColor& c)
  {
    L += c.L;
    a += c.a;
    b += c.b;
  }

  LabColor operator- (const LabColor& c) const
  {
    return {L - c.L, a - c.a, b - c.b};
  }

  LabColor operator* (float c) const
  {
    return {c*L, c*a, c*b};
  }

  friend LabColor operator*(float c, const LabColor& rhs)
  {
    return {c*rhs.L, c*rhs.a, c*rhs.b};
  }
};
#pragma pack(pop)
