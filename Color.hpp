#pragma once

#include <stdint.h>
#include <unordered_map>
#include <vector>

struct RGBAColor;
struct RGBColor;
struct HSVColor;
struct LabColor;

#pragma pack(push, 1)
struct HSVColor
{
  float H = 0.0f;
  float S = 0.0f;
  float V = 0.0f;
  float A = 1.0f;

  RGBAColor toRGB();
};

struct RGBColor
{
  uint8_t R = 0;
  uint8_t G = 0;
  uint8_t B = 0;

  void setFromYuv(int y, int u, int v);

  RGBColor() {}
  RGBColor(const RGBAColor& color);
  RGBColor(uint8_t r, uint8_t g, uint8_t b);
};

struct RGBAColor
{
  uint8_t R = 0;
  uint8_t G = 0;
  uint8_t B = 0;
  uint8_t A = 255;

  HSVColor toHSV() const;
  LabColor toLab() const;
  uint8_t getBrightestChannel() const;
  uint8_t getDarkestChannel() const;
  uint8_t getGrayValue() const;

  void setFromYuv(int y, int u, int v);

  RGBAColor() {}
  RGBAColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a);
  RGBAColor(const RGBColor& color);
};

struct XYZColor
{
  float X = 0;
  float Y = 0;
  float Z = 0;
};

struct LabColor
{
  float L = 0;
  float a = 0;
  float b = 0;

  RGBAColor toRgba() const;

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

  float deltaE(const LabColor& other) const;
};
#pragma pack(pop)
