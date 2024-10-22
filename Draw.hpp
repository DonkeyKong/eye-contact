#pragma once

#include "Image.hpp"
#include <algorithm>

template <typename Color>
void DrawLine(Image<Color>& img, float x1, float y1, float x2, float y2, float strokeWeight, Color color)
{
  int minX = std::clamp(std::min((int)(x1-strokeWeight), (int)(x2-strokeWeight)), 0, img.width());
  int minY = std::clamp(std::min((int)(y1-strokeWeight), (int)(y2-strokeWeight)), 0, img.height());
  int maxX = std::clamp(std::max((int)std::ceil(x1+strokeWeight), (int)std::ceil(x2+strokeWeight)), 0, img.width());
  int maxY = std::clamp(std::max((int)std::ceil(y1+strokeWeight), (int)std::ceil(y2+strokeWeight)), 0, img.height());

  float a = y2-y1;
  float b = x2-x1;
  float c = x2*y1-y2*x1;
  float d = std::sqrt(a*a + b*b);
  float strokeHalf = strokeWeight / 2.0f;
  Color* px = img.pixel();
  for (int y = minY; y < maxY; ++y)
  {
    for (int x = minX; x < maxX; ++x)
    {
      float dist = std::abs(a*(float)x - b*(float)y + c) / d;
      if (dist < strokeHalf)
      {
        px[x+y*img.width()] = color;
      }
    }
  }
}

template <typename Color>
void DrawRect(Image<Color>& img, float x, float y, float width, float height, float strokeWeight, Color color)
{
  DrawLine(img, x, y, x + width, y, strokeWeight, color);
  DrawLine(img, x + width, y, x + width, y + height, strokeWeight, color);
  DrawLine(img, x + width, y + height, x, y + height, strokeWeight, color);
  DrawLine(img, x, y + height, x, y, strokeWeight, color);
}