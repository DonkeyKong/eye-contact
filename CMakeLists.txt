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

target_compile_definitions( ${PROJECT_NAME} PRIVATE
  #DEBUG_SAVE_IMAGE
  #DEBUG_PROFILE_FUNCTIONS
  DEBUG_SHOW_FPS
  #DEBUG_DISABLE_SERIAL
)

target_link_libraries(${PROJECT_NAME} broadcom_host
                                      fmt
                                      json 
                                      libjpeg
                                      libpng
                                      magic_enum
                                      image_resampler
                                      TinyEXIF
                                      tensorflow-lite)

add_custom_target(copy_models ALL)

add_custom_command(TARGET copy_models POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_directory
                    ${PROJECT_SOURCE_DIR}/models
                    ${CMAKE_CURRENT_BINARY_DIR}/models)