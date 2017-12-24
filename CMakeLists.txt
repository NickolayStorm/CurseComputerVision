cmake_minimum_required(VERSION 3.8)

project(OpenCVCurse)

find_package(OpenCV REQUIRED)
FIND_PACKAGE(Boost REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(stdc++fs UNKNOWN IMPORTED)
set_property(TARGET stdc++fs PROPERTY IMPORTED_LOCATION "/usr/local/lib/gcc/7/libstdc++fs.a")


set(SOURCE_FILES main.cpp)
add_executable(OpenCVCurse ${SOURCE_FILES})

target_link_libraries( OpenCVCurse ${OpenCV_LIBS} ${Boost_LIBRARIES} stdc++fs)
