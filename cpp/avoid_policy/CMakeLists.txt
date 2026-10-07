cmake_minimum_required(VERSION 3.16)
project(avoid_policy VERSION 1.0.0 DESCRIPTION "Obstacle-avoidance policy inference" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(avoid_policy STATIC avoid_policy.cpp)
target_include_directories(avoid_policy PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # No -ffast-math: the results must stay comparable with the Python reference.
  target_compile_options(avoid_policy PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Werror)
endif()

add_executable(bench_avoid_policy bench_avoid_policy.cpp)
target_link_libraries(bench_avoid_policy PRIVATE avoid_policy)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_avoid_policy tests/test_avoid_policy.cpp)
  target_link_libraries(test_avoid_policy PRIVATE avoid_policy)
  add_test(NAME avoid_policy
           COMMAND test_avoid_policy ${CMAKE_CURRENT_SOURCE_DIR}/avoid_policy.bin
                   ${CMAKE_CURRENT_SOURCE_DIR}/tests/reference_cases.txt)
endif()
