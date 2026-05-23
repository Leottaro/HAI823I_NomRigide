cmake_minimum_required(VERSION 3.20.0)
project(poly34)

# Define the poly34 library
add_library(poly34
    poly34.cpp
)

# Include directories
set(POLY34_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${POLY34_INCLUDE_DIRS})
