cmake_minimum_required(VERSION 3.5)
project(hai823i_nomrigide)

set(CMAKE_CXX_STANDARD 11)
set(OpenGL_GL_PREFERENCE GLVND)

# set(CMAKE_VERBOSE_MAKEFILE 1)
set(APP_VERSION_MAJOR 1)
set(APP_VERSION_MINOR 0)

set(APP_TARGET hai823i_nomrigide)
include_directories(${PROJECT_SOURCE_DIR} .)

# src files
add_executable(${APP_TARGET}
    src/main.cpp
    src/ShaderProgram.cpp
    src/Camera.cpp
    src/Mesh.cpp
)


# opengl
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

# glew
find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})

# imgui
add_subdirectory(external/imgui)
include_directories(external/imgui)
target_link_libraries(${APP_TARGET} imgui)

# glfw
find_package(glfw3 REQUIRED)
include_directories(${GLFW_INCLUDE_DIRS})
link_libraries(${GLFW_LIBRARY_DIRS})

# eigen
add_subdirectory(external/eigen)
include_directories(external/eigen)
target_link_libraries(${APP_TARGET} eigen)

target_link_libraries(${APP_TARGET}
    glfw
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${EXTRA_LIBS}
)
