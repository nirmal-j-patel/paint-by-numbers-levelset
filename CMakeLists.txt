cmake_minimum_required(VERSION 2.8)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
project(paint-by-numbers-levelset)

#if (NOT CMAKE_BUILD_TYPE)
#    message(STATUS "No build type selected, default to Release")
#    set(CMAKE_BUILD_TYPE "Release")
#endif()

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()


set(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")

include_directories(./src)
include_directories(./contrib/MinimalPath/)
include_directories(contrib/ScribbleInteractorStyle/)
# add_subdirectory(contrib/ScribbleInteractorStyle/)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_AUTOMOC ON)

file(GLOB glob_sources
#     "src/*.h"
    "src/*.cxx"
    "contrib/MinimalPath/*.txx"
    "contrib/ScribbleInteractorStyle/*.cxx"
)

file(GLOB glob_forms
    "src/*.ui"
)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Concurrent REQUIRED)
qt5_wrap_ui(forms_src ${glob_forms})

add_executable(paint-by-numbers-levelset ${forms_src} ${glob_sources})
qt5_use_modules(paint-by-numbers-levelset Widgets Concurrent)

target_link_libraries(paint-by-numbers-levelset ${VTK_LIBRARIES} ${ITK_LIBRARIES})
