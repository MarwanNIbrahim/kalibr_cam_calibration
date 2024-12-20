# Init CMake (require at least version 3.16)
CMAKE_MINIMUM_REQUIRED(VERSION 3.16)

PROJECT(kalibr_cam_calibration
	VERSION 1.0.0
	LANGUAGES C CXX)

OPTION(ENABLE_PYTHON "Build Python bindings" OFF)

# Define installation paths
INCLUDE(GNUInstallDirs)

# Git submodule automatic update support.
FIND_PACKAGE(Git QUIET)

IF(Git_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git")
	# Update submodules as needed
	OPTION(GIT_SUBMODULE "Check-out submodules during build" ON)

	IF(GIT_SUBMODULE)
		MESSAGE(STATUS "Git submodule update")

		EXECUTE_PROCESS(
			COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			RESULT_VARIABLE GIT_SUBMOD_RESULT)
		IF(NOT
		   GIT_SUBMOD_RESULT
		   EQUAL
		   "0")
			MESSAGE(
				WARNING
					"${GIT_EXECUTABLE} submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please check-out submodules manually."
			)
		ENDIF()
	ENDIF()
ENDIF()


# C++ standard settings.
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

SET(CMAKE_C_VISIBILITY_PRESET default)
SET(CMAKE_CXX_VISIBILITY_PRESET default)

ADD_SUBDIRECTORY(thirdparty)
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(samples)

IF(ENABLE_PYTHON)
	ADD_SUBDIRECTORY(python)
ENDIF()
