cmake_minimum_required(VERSION 3.10)
project(Rasterizer1)

set(OpenCV_DIR D:/xxn/Include/OpenCV-MinGW-Build-OpenCV-4.5.2-x64)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories("D:/xxn/Include/Eigen3/include/eigen3")

add_executable(Rasterizer1 main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer1 ${OpenCV_LIBRARIES})
