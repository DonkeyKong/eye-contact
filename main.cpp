#include <iostream>
#include <thread>
#include <chrono>

#include <fmt/format.h>

#include "Color.hpp"
#include "Image.hpp"
#include "ImageIO.hpp"
#include "V4LCamera.hpp"

using namespace V4L2;

template <typename Color>
Image<Color> imageFromFrame(const Frame& f)
{
  Image<Color> img(f.width, f.height);

  auto imgPtr = img.pixel();
  
  for (int y=0; y < f.height; y++)
  {
    uint8_t* yuyvPtr = f.data.start + f.strideInBytes * y;
    for (int x=0; x < f.width; x+=2)
    {
      imgPtr->setFromYuv(yuyvPtr[0], yuyvPtr[1], yuyvPtr[3]);
      ++imgPtr;
      imgPtr->setFromYuv(yuyvPtr[2], yuyvPtr[1], yuyvPtr[3]);
      ++imgPtr;
      yuyvPtr += 4;
    }
  }
  return img;
}

int main(int argc, char *argv[])
{
    std::cout << "Preparing to make eye contact...\n";

    std::cout << "Setting up cam 0...\n";
    Camera cam0("/dev/video0", 2);
    std::cout << "Cam 0 ok!\n";

    std::cout << "Setting up cam 1...\n";
    Camera cam1("/dev/video2", 2);
    std::cout << "Cam 1 ok!\n";

    while (true)
    {
      auto frame0 = cam0.getFrame();
      auto frame1 = cam1.getFrame();

      ImageIO::SaveToFile("cam0.jpg", imageFromFrame<RGBColor>(frame0));
      ImageIO::SaveToFile("cam1.jpg", imageFromFrame<RGBColor>(frame1));

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Done\n";
    return 0;
}
