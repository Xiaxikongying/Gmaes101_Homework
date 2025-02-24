cmake_minimum_required(VERSION 3.10)
project(Test)

set(OpenCV_DIR D:/xxn/Include/OpenCV-MinGW-Build-OpenCV-4.5.2-x64)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories("D:/xxn/Include/Eigen3/include/eigen3")

add_executable(Test main.cpp)
target_link_libraries(Test ${OpenCV_LIBRARIES})
