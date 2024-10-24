#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

#include <fmt/format.h>

#include "Color.hpp"
#include "Image.hpp"
#include "Draw.hpp"
#include "ImageIO.hpp"
#include "V4LCamera.hpp"
#include "RemoteEyes.hpp"
#include "FaceDetector.hpp"
#include "FPSCounter.hpp"

using namespace V4L2;
using namespace std::chrono_literals;

template <typename Color>
void imageFromFrame(Image<Color>& dest, const Frame& f)
{
  // Resize the dest image
  if (dest.height() != f.height || dest.width() != f.width)
  {
    dest = Image<Color>(f.width, f.height);
  }

  auto imgPtr = dest.pixel();
  
  for (int y=0; y < f.height; y++)
  {
    uint8_t* yuyvPtr = f.data.start + f.strideInBytes * y;
    for (int x=0; x < f.width; x+=2)
    {
      if constexpr(std::is_same_v<Color, Gray8>)
      {
        imgPtr[0].I = yuyvPtr[0];
        imgPtr[1].I = yuyvPtr[2];
      }
      else
      {
        imgPtr[0] = YUV24{yuyvPtr[0], yuyvPtr[1], yuyvPtr[3]};
        imgPtr[1] = YUV24{yuyvPtr[2], yuyvPtr[1], yuyvPtr[3]};
      }
      imgPtr += 2;
      yuyvPtr += 4;
    }
  }
}

int main(int argc, char *argv[])
{
    std::cout << "Preparing to make eye contact...\n";

    std::cout << "Setting up camera...\n";
    Camera cam("/dev/video0", 1280, 720, 3);
    Image<RGB24> image0, image1;
    std::cout << "Capturing at " << cam.width() << " x " << cam.height() << "\n";

    FaceDetector detector(FaceDetector::BackModel);
    RemoteEyes eyes;
    FPSCounter fpsCounter;
    
    while (true)
    {
      std::thread captureThread([&]()
      { 
        imageFromFrame(image0, cam.getFrame());
      });

      if (image1.dataSizeBytes() > 0)
      {
        auto results = detector.Detect(image1);
        if (results.size() > 0)
        {
          eyes.look((results[0].leftEyeX - 0.5f) * -180.0f,
                    (results[0].leftEyeY - 0.5f) * -140.0f);
        }
      }
      captureThread.join();
      std::swap(image0, image1);
      fpsCounter.frame();
      std::cout << "FPS: " << fpsCounter.fps() << "\r" << std::flush;
    }

    return 0;
}
