cmake_minimum_required(VERSION 3.1)

# This file replaces CMakeLists.txt at the root of a parafashion checkout.
# Install parafashion first, then copy this file over that root CMakeLists.txt.
# See README.md. Paths below are inside that checkout (lib/ is not copied here).
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/vcglib/CMakeLists.txt")
  message(FATAL_ERROR
    "lib/vcglib was not found next to this CMakeLists.txt. "
    "Install parafashion (clone --recursive), then copy this file to the parafashion root. "
    "See README.md.")
endif()

# Placed before project() so the C/C++ toolchain picks up this SDK.
set(CMAKE_OSX_SYSROOT /Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk/)

project(reconfiggarment)

add_compile_options(-w)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# libigl's own CMakeLists defaults these ON. Only the core module is needed.
option(LIBIGL_WITH_COMISO            "Use CoMiso"          OFF)
option(LIBIGL_WITH_EMBREE            "Use Embree"          OFF)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"          OFF)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"            OFF)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"           OFF)
option(LIBIGL_WITH_PNG               "Use PNG"             OFF)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"          OFF)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"        OFF)
option(LIBIGL_WITH_PREDICATES        "Use exact predicates" OFF)
option(LIBIGL_WITH_XML               "Use XML"             OFF)
option(LIBIGL_USE_STATIC_LIBRARY     "Use libigl as static library" ON)
option(LIBIGL_BUILD_TUTORIALS        "Build libigl tutorials" OFF)
option(LIBIGL_BUILD_TESTS            "Build libigl tests" OFF)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/opt/qt@5/lib/cmake/Qt5")
find_package(Qt5 COMPONENTS Widgets OpenGL Svg REQUIRED)

add_definitions(-DGLEW_STATIC)
add_subdirectory(lib/glew-cmake EXCLUDE_FROM_ALL)

if(NOT TARGET igl::core)
  add_subdirectory(lib/libigl)
endif()

set(VCG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(lib/vcglib EXCLUDE_FROM_ALL)

if(NOT TARGET nlohmann_json)
  add_subdirectory(lib/json EXCLUDE_FROM_ALL)
endif()

# Parametrization library only. garment-flattening's CMakeLists also builds
# woven_viz, reflec_param, multiple_poses, and other apps this project does not use.
file(GLOB WOVEN_PARAM_SRC ${CMAKE_CURRENT_SOURCE_DIR}/lib/garment-flattening/src/param/*.cpp)
add_library(woven_param STATIC ${WOVEN_PARAM_SRC})
target_include_directories(woven_param PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/garment-flattening/include
)
if(NOT TARGET polyclipping)
  add_subdirectory(lib/garment-flattening/lib/clipper EXCLUDE_FROM_ALL)
endif()
target_link_libraries(woven_param PUBLIC igl::core nlohmann_json polyclipping)

set(ATB_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(lib/AntTweakBar EXCLUDE_FROM_ALL)

include_directories(
  include
  lib/vcglib
  lib/xfield_tracer
  lib/AntTweakBar/TweakBar/include
  lib/Clipper
  lib/libigl/include
  lib/Directional/include
  lib/garment-flattening/include
)

# Sources for this program only. moc_glwidget.cpp stays in the parafashion tree.
set(RECONFIG_SOURCES
  apps/reconfiggarment.cpp
  src/reconfiggarment_widget.cpp
  src/experiment_settings.cpp
  src/garment_session.cpp
  src/manual_border_seam.cpp
  src/myglwidget.cpp
  src/moc_glwidget.cpp
  lib/vcglib/wrap/ply/plylib.cpp
  lib/vcglib/wrap/gui/trackball.cpp
  lib/vcglib/wrap/gui/trackmode.cpp
  lib/vcglib/wrap/qt/anttweakbarMapperNew.cpp
  lib/vcglib/wrap/qt/Outline2ToQImage.cpp
  lib/Clipper/clipper.cpp
)

add_executable(reconfiggarment ${RECONFIG_SOURCES})

target_link_libraries(reconfiggarment PRIVATE
  Qt5::Widgets
  Qt5::OpenGL
  Qt5::Svg
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY}
  vcglib
  AntTweakBar
  libglew_static
  nlohmann_json
  woven_param
)
