cmake_minimum_required(VERSION 3.26)
project(thesis-project)

set(EXTERNAL_DIR "${PROJECT_SOURCE_DIR}/external")
set(SOURCE_DIR "${PROJECT_SOURCE_DIR}/src")
set(DATA_DIR "${PROJECT_SOURCE_DIR}/data")
set(MAIN_EXE_TARGET "exe")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# - - - - - - - - - - - - - - - - - - - - 

add_subdirectory(${EXTERNAL_DIR})
add_subdirectory(${SOURCE_DIR})