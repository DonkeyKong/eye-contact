#pragma once

#include <stdint.h>
#include <unordered_map>
#include <vector>

struct RGB24;
struct RGBA32;
struct Gray8;
struct HSV3f;
struct Lab3f;
struct YUV24;

#pragma pack(push, 1)
struct RGB24
{
  uint8_t R = 0;
  uint8_t G = 0;
  uint8_t B = 0;

  RGB24() = default;
  RGB24(const RGB24& color) = default;
  RGB24(uint8_t r, uint8_t g, uint8_t b);
  RGB24(const HSV3f& color);
  RGB24(const Gray8& color);
  RGB24(const Lab3f& color);
  RGB24(const YUV24& color);
};

struct RGBA32 : public RGB24
{
  uint8_t A = 255;

  RGBA32() = default;
  RGBA32(const RGBA32& color) = default;
  RGBA32(uint8_t r, uint8_t g, uint8_t b, uint8_t a);
  RGBA32(const RGB24& color);
  RGBA32(const HSV3f& color);
  RGBA32(const Gray8& color);
  RGBA32(const Lab3f& color);
};

struct Gray8
{
  uint8_t I = 0;

  Gray8() = default;
  Gray8(const Gray8& color) = default;
  Gray8(uint8_t i);
  Gray8(const RGB24& color);
  Gray8(const Lab3f& color);
  Gray8(const HSV3f& color);
  Gray8(const YUV24& color);
};

struct HSV3f
{
  float H = 0.0f;
  float S = 0.0f;
  float V = 0.0f;

  HSV3f() = default;
  HSV3f(const HSV3f& color) = default;
  HSV3f(float h, float s, float v);
  HSV3f(const RGB24& color);
  HSV3f(const Gray8& color);
  HSV3f(const Lab3f& color);
};

struct Lab3f
{
  float L = 0;
  float a = 0;
  float b = 0;

  Lab3f() = default;
  Lab3f(const Lab3f& color) = default;
  Lab3f(float l, float a, float b);
  Lab3f(const RGB24& color);
  Lab3f(const HSV3f& color);
  Lab3f(const Gray8& color);

  float deltaE(const Lab3f& other) const;

  Lab3f operator+ (const Lab3f& c)  const
  {
    return {L + c.L, a + c.a, b + c.b};
  }

  Lab3f operator* (const Lab3f& c)  const
  {
    return {L * c.L, a * c.a, b * c.b};
  }

  void operator+= (const Lab3f& c)
  {
    L += c.L;
    a += c.a;
    b += c.b;
  }

  Lab3f operator- (const Lab3f& c) const
  {
    return {L - c.L, a - c.a, b - c.b};
  }

  Lab3f operator* (float c) const
  {
    return {c*L, c*a, c*b};
  }

  friend Lab3f operator*(float c, const Lab3f& rhs)
  {
    return {c*rhs.L, c*rhs.a, c*rhs.b};
  }
};

struct YUV24
{
  uint8_t Y = 0;
  uint8_t U = 0;
  uint8_t V = 0;

  YUV24() = default;
  YUV24(const YUV24& color) = default;
  YUV24(uint8_t y, uint8_t u, uint8_t v);
};

template <float VMin = 0.0f, float VMax = 1.0f>
struct RGB3f
{
  static constexpr float VRange = VMax - VMin;

  float R = 0;
  float G = 0;
  float B = 0;

  RGB3f() = default;
  RGB3f(const RGB3f& color) = default;
  RGB3f(float r, float g, float b) : R(r), G(g), B(b) {}
  RGB3f(const RGB24& color)
  {
    R = (float)color.R / (255.0f * VRange) + VMin;
    G = (float)color.G / (255.0f * VRange) + VMin;
    B = (float)color.B / (255.0f * VRange) + VMin;
  }
};

#pragma pack(pop)
