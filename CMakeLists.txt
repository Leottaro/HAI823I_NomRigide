cmake_minimum_required(VERSION 3.22.1)
project(hai823i_nomrigide)
set(APP_VERSION_MAJOR 1)
set(APP_VERSION_MINOR 0)

set(CMAKE_CXX_STANDARD 11)
set(OpenGL_GL_PREFERENCE GLVND) # asked me to set this to GLVND or LEGACY
# set(CMAKE_VERBOSE_MAKEFILE 1) # If you want verbose

# target setting
set(APP_TARGET hai823i_nomrigide)
include_directories(${PROJECT_SOURCE_DIR} .)

# my src files
add_executable(${APP_TARGET}
    src/main.cpp

    src/ShaderProgram.cpp
    src/ShaderProgram.hpp

    src/Camera.cpp
    src/Camera.hpp

    src/Mesh.cpp
    src/Mesh.hpp
)

# opengl
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})
target_link_libraries(${APP_TARGET} OpenGL)

# glew
add_subdirectory(external/glew/build/cmake)
include_directories(external/glew/include)
target_link_libraries(${APP_TARGET} GLEW)

# glfw
add_subdirectory(external/glfw)
include_directories(${GLFW_INCLUDE_DIRS})
target_link_libraries(${APP_TARGET} glfw3)

# imgui
add_subdirectory(external/imgui)
include_directories(external/imgui)
target_link_libraries(${APP_TARGET} imgui)

# eigen
add_subdirectory(external/eigen)
include_directories(external/eigen)
target_link_libraries(${APP_TARGET} eigen)
