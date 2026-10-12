# Init CMake (require at least version 3.16)
CMAKE_MINIMUM_REQUIRED(VERSION 3.16)

PROJECT(kalibr_cam_calibration
        VERSION 1.0.0
        LANGUAGES C CXX)

OPTION(ENABLE_PYTHON "Build Python bindings" OFF)
OPTION(ENABLE_SAMPLES "Build samples for mono camera calibration using Kalibr" OFF)

# Define installation paths
INCLUDE(GNUInstallDirs)

# Git submodule automatic update support.
FIND_PACKAGE(Git QUIET)

IF (Git_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git")
    # Update submodules as needed
    OPTION(GIT_SUBMODULE "Check-out submodules during build" ON)

    IF (GIT_SUBMODULE)
        MESSAGE(STATUS "Git submodule update")

        EXECUTE_PROCESS(
                COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                RESULT_VARIABLE GIT_SUBMOD_RESULT)
        IF (NOT
                GIT_SUBMOD_RESULT
                EQUAL
                "0")
            MESSAGE(
                    WARNING
                    "${GIT_EXECUTABLE} submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please check-out submodules manually."
            )
        ENDIF ()
    ENDIF ()
ENDIF ()


# C++ standard settings.
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

SET(CMAKE_C_VISIBILITY_PRESET default)
SET(CMAKE_CXX_VISIBILITY_PRESET default)

# The kalibr repo has quite a few cmake warnings, suppress for easier readability
SET(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE BOOL "Suppress developer warnings" FORCE)

# Python builds ship the kalibr shared libraries inside the "kalibr" package
IF (ENABLE_PYTHON)
    SET(KALIBR_LIB_DESTINATION kalibr)
ELSE ()
    SET(KALIBR_LIB_DESTINATION ${CMAKE_INSTALL_LIBDIR})
ENDIF ()

ADD_SUBDIRECTORY(thirdparty)
ADD_SUBDIRECTORY(include)

IF (ENABLE_SAMPLES)
    ADD_SUBDIRECTORY(samples)
ENDIF ()


IF (ENABLE_PYTHON)
    MESSAGE(STATUS "Building Python bindings")
    ADD_SUBDIRECTORY(python)
ENDIF ()
