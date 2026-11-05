cmake_minimum_required(VERSION 3.16)
project(msim_fastglio_signalsim LANGUAGES CXX)
include(CTest)

add_library(msim_urban_geometry
  core/src/frames/geodesy.cpp
  core/src/gnss/urban.cpp)
target_include_directories(msim_urban_geometry PUBLIC core/include)
target_compile_features(msim_urban_geometry PUBLIC cxx_std_17)
set_target_properties(msim_urban_geometry PROPERTIES CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(msim_urban_geometry PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

if(BUILD_TESTING)
  foreach(name geodesy urban_reflection)
    add_executable(${name}_test tests/unit/${name}_test.cpp)
    target_link_libraries(${name}_test PRIVATE msim_urban_geometry)
    add_test(NAME ${name} COMMAND ${name}_test)
  endforeach()
endif()
