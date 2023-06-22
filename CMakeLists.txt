cmake_minimum_required(VERSION 3.26)
project(thesis-project)

set(EXTERNAL_DIR "${PROJECT_SOURCE_DIR}/external")
set(SOURCE_DIR "${PROJECT_SOURCE_DIR}/src")
set(DATA_DIR "${PROJECT_SOURCE_DIR}/data")
set(MAIN_EXE_TARGET "exe")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# - - - - - - - - - - - - - - - - - - - - 

add_subdirectory(${EXTERNAL_DIR})
add_subdirectory(${SOURCE_DIR})