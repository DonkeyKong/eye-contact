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
#include "FunctionTimer.hpp"

using namespace V4L2;
using namespace std::chrono_literals;

template <typename Color>
void imageFromFrame(Image<Color>& dest, const Frame& f)
{
  PROFILE_FUNCTION;

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

template <typename Color>
void imageFromFrameHalfRes(Image<Color>& dest, const Frame& f)
{
  PROFILE_FUNCTION;

  // Resize the dest image
  if (dest.height() != f.height/2 || dest.width() != f.width/2)
  {
    dest = Image<Color>(f.width/2, f.height/2);
  }

  auto imgPtr = dest.pixel();
  
  for (int y=0; y < f.height; y+=2)
  {
    uint8_t* yuyvPtr = f.data.start + f.strideInBytes * y;
    for (int x=0; x < f.width; x+=2)
    {
      if constexpr(std::is_same_v<Color, Gray8>)
      {
        imgPtr[0].I = yuyvPtr[0];
      }
      else
      {
        imgPtr[0] = YUV24{yuyvPtr[0], yuyvPtr[1], yuyvPtr[3]};
      }
      imgPtr += 1;
      yuyvPtr += 4;
    }
  }
}

int main(int argc, char *argv[])
{
    std::cout << "Preparing to make eye contact...\n";

    std::cout << "Setting up camera...\n";
    Camera cam("/dev/video0", 320, 240, 3);
    Image<RGB24> rawImage, scaledImage, detectImage;
    std::cout << "Capturing at " << cam.width() << " x " << cam.height() << "\n";

    FaceDetector detector(FaceDetector::FrontModel);
    RemoteEyes eyes;
    FPSCounter fpsCounter;
    
    while (true)
    {
      std::thread captureThread([&]()
      { 
        imageFromFrame(rawImage, cam.getFrame());
        rawImage.scale(scaledImage, detector.inputImageWidth(), detector.inputImageHeight(), 
          {scaleMode : ScaleMode::Fit, interpolationMode : InterpolationMode::Bilinear});
      });

      if (detectImage.dataSizeBytes() > 0)
      {
        auto results = detector.detect(detectImage);
        if (results.size() > 0)
        {
          eyes.look((results[0].leftEyeX - 0.5f) * -140.0f,
                    (results[0].leftEyeY - 0.5f) * -100.0f);
        }

        #ifdef DEBUG_SAVE_IMAGE
        static std::chrono::steady_clock::time_point nextSave;
        if (std::chrono::steady_clock::now() > nextSave)
        {
          nextSave = std::chrono::steady_clock::now() + 200ms;
          
          for (int i = 0; i < results.size(); ++i)
          {
            // std::cout << "Result x:" << results[i].xmin << " y:" << results[i].ymin 
            //           << " w:" << results[i].width << " h: " << results[i].height << "\n" << std::flush;
            results[i].scaleResults(detectImage.width(), detectImage.height());
            DrawRect(detectImage, results[i].xmin, results[i].ymin, results[i].width, results[i].height, 2.0f, {255, 0, 0});
            DrawPoint(detectImage, results[i].leftEyeX, results[i].leftEyeY, 2.0f, {255, 255, 0});
            DrawPoint(detectImage, results[i].rightEyeX, results[i].rightEyeY, 2.0f, {0, 255, 255});
          }
          ImageIO::SaveToFile("small.jpg", detectImage);
        }
        #endif
      }
      captureThread.join();
      std::swap(scaledImage, detectImage);
      
      #ifdef DEBUG_SHOW_FPS
      fpsCounter.frame();
      std::cout << "FPS: " << fpsCounter.fps() << "\r" << std::flush;
      #endif
    }

    return 0;
}
