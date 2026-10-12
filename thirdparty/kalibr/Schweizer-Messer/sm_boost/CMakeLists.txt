cmake_minimum_required(VERSION 3.15)
project(sm_boost)

find_package(Boost 1.79 REQUIRED COMPONENTS system serialization thread)

include_directories(${Boost_INCLUDE_DIRS})

add_definitions(-fPIC)

##############
## Building ##
##############

add_library(${PROJECT_NAME} SHARED
  src/JobQueue.cpp
  src/portable_binary_iarchive.cpp
  src/portable_binary_oarchive.cpp
)

target_link_libraries(${PROJECT_NAME}
                      ${Boost_LIBRARIES}
                      sm_common)

target_include_directories(${PROJECT_NAME} PUBLIC
        include)

##################
## Installation ##
##################

