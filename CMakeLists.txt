# Works with 3.14 and tested through 3.27
cmake_minimum_required(VERSION 3.14...3.27)

# Project name and a few useful settings. Other commands can pick up the results
project(
  ModelPredictiveControl
  VERSION 0.1
  DESCRIPTION "An example project with CMake"
  LANGUAGES CXX)

# Only do these if this is the main project, and not if it is included through add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

  # Optionally set things like CMAKE_CXX_STANDARD, CMAKE_POSITION_INDEPENDENT_CODE here

  # Let's ensure -std=c++xx instead of -std=g++xx
  set(CMAKE_CXX_EXTENSIONS OFF)

  # exports compile commands
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

  # compiler flags
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
      add_compile_options(-Wall -Wextra -pedantic)
  elseif(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "Test")
      add_compile_options(-Wall -Wextra -pedantic -O3)
  endif()

  # linker flags
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_link_options()
  elseif(CMAKE_BUILD_TYPE STREQUAL "Gprof")
    add_link_options(-pg -g3)
  endif()

  # Let's nicely support folders in IDEs
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)

  # Testing only available if this is the main app
  # Note this needs to be done in the main CMakeLists
  # since it calls enable_testing, which must be in the
  # main CMakeLists.
  include(CTest)

  # compile definition
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
      add_compile_definitions(DEBUG)
  endif()

  # Docs only available if this is the main app
  find_package(Doxygen)
  if(Doxygen_FOUND)
    add_subdirectory(docs)
  else()
    message(STATUS "Doxygen not found, not building docs")
  endif()
endif()

# FetchContent added in CMake 3.11, downloads during the configure step
# FetchContent_MakeAvailable was added in CMake 3.14; simpler usage
include(FetchContent)

# packages
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

FetchContent_Declare(
  fmtlib
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG 5.3.0)

FetchContent_MakeAvailable(fmtlib)

# The compiled library code is here
add_subdirectory(src)

# The executable code is here
add_subdirectory(apps)

# Testing only available if this is the main app
# Emergency override MODERN_CMAKE_BUILD_TESTING provided as well
if((CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME OR MODERN_CMAKE_BUILD_TESTING)
   AND BUILD_TESTING)
  add_subdirectory(tests)
endif()
