cmake_minimum_required(VERSION 3.15)
project(sm_eigen)

find_package(Boost 1.79 REQUIRED COMPONENTS system serialization)
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

include_directories(${Boost_INCLUDE_DIRS})

##############
## Building ##
##############

add_library(${PROJECT_NAME} SHARED
  src/random.cpp
)

target_link_libraries(${PROJECT_NAME}
                      ${Boost_LIBRARIES}
                      sm_common
                      sm_random)

target_include_directories(${PROJECT_NAME} PUBLIC
        include)

##################
## Installation ##
##################

