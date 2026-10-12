cmake_minimum_required(VERSION 3.15)
project(sm_logging)

find_package(Boost 1.79 REQUIRED COMPONENTS system thread regex)

include_directories(${Boost_INCLUDE_DIRS})

add_definitions(-std=c++0x)

add_definitions(-fPIC)

##############
## Building ##
##############

add_library(${PROJECT_NAME}  SHARED
  src/Logger.cpp
  src/StdOutLogger.cpp
  src/LoggingEvent.cpp
  src/LoggingGlobals.cpp
  src/Formatter.cpp
  src/Tokens.cpp
)

target_link_libraries(${PROJECT_NAME}
                      ${Boost_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC
        include)

##################
## Installation ##
##################

