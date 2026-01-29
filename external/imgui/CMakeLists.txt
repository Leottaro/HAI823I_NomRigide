cmake_minimum_required(VERSION 3.22.1)
project(imgui)

# Define the imgui library
add_library(imgui
    imgui.cpp
    imgui_demo.cpp
    imgui_draw.cpp
    imgui_impl_glfw.cpp
    imgui_impl_opengl3.cpp
    imgui_tables.cpp
    imgui_widgets.cpp
)

# Include directories
set(IMGUI_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${IMGUI_INCLUDE_DIRS})
