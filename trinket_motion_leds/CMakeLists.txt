cmake_minimum_required(VERSION 2.8.4)

set(PROJECT_NAME pov)
set(TEST rebuild)
project(${PROJECT_NAME})

add_definitions(-DCOMPILE_UNIT_TESTS)

add_executable(test_linearpredictor LinearPredictor.cpp)
