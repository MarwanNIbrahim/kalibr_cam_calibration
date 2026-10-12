cmake_minimum_required(VERSION 3.15)
project(sm_random)

##############
## Building ##
##############

add_definitions(-fPIC)

add_library(${PROJECT_NAME} SHARED
  src/random.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
        include)

##################
## Installation ##
##################

