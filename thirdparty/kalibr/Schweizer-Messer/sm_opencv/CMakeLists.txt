cmake_minimum_required(VERSION 3.15)
project(sm_opencv)

find_package(Boost 1.79 REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

add_definitions(-std=c++0x -D__STRICT_ANSI__)

include_directories(${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
        src/empty.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
        include)
