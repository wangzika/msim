cmake_minimum_required(VERSION 3.16)
project(msim VERSION 0.1.0 LANGUAGES CXX)

include(CTest)

add_library(msim_core core/src/version.cpp)
target_include_directories(msim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>)
target_compile_features(msim_core PUBLIC cxx_std_17)
set_target_properties(msim_core PROPERTIES CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(msim_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

if(BUILD_TESTING)
  add_executable(msim_core_smoke_test tests/unit/core_smoke_test.cpp)
  target_link_libraries(msim_core_smoke_test PRIVATE msim_core)
  add_test(NAME core_smoke COMMAND msim_core_smoke_test)
endif()

if(BUILD_TESTING)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME manifest_contract
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/contract/test_manifest.py)
endif()

if(BUILD_TESTING)
  add_test(NAME recorded_run_contract
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/contract/test_recorded_run.py)
endif()
