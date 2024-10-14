cmake_minimum_required(VERSION 3.16)

project("EyeContact")
set(CMAKE_CXX_STANDARD 20)

# Create all the dependecy targets
add_subdirectory("deps")

add_executable( ${PROJECT_NAME}
  main.cpp
  V4LCamera.cpp
  Color.cpp
  ImageIO.cpp
)

target_link_libraries(${PROJECT_NAME} broadcom_host
                                      fmt
                                      json 
                                      libjpeg
                                      libpng
                                      magic_enum
                                      image_resampler
                                      TinyEXIF)