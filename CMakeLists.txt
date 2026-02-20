cmake_minimum_required(VERSION 3.22.1)
project(hai823i_nomrigide)
set(APP_VERSION_MAJOR 1)
set(APP_VERSION_MINOR 0)

set(CMAKE_CXX_STANDARD 11)
set(OpenGL_GL_PREFERENCE GLVND) # asked me to set this to GLVND or LEGACY
# set(CMAKE_VERBOSE_MAKEFILE 1) # If you want verbose

# target setting
set(APP_TARGET hai823i_nomrigide)
set(APP_TARGET_DEBUG ${APP_TARGET}_debug)
set(APP_TARGET_OPT ${APP_TARGET}_opt)
include_directories(${PROJECT_SOURCE_DIR} .)

# my src files
set(APP_SOURCES
    src/main.cpp

    src/ShaderProgram.cpp
    src/ShaderProgram.hpp

    src/Transformation.hpp

    src/Camera.cpp
    src/Camera.hpp

    src/Mesh.cpp
    src/Mesh.hpp

    src/DynamicObject.hpp
    src/DynamicObject.cpp
)

add_executable(${APP_TARGET_DEBUG} ${APP_SOURCES})
add_executable(${APP_TARGET_OPT} ${APP_SOURCES})

target_compile_options(${APP_TARGET_DEBUG} PRIVATE -O0 -g3)
target_compile_definitions(${APP_TARGET_DEBUG} PRIVATE DEBUG)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${APP_TARGET_OPT} PRIVATE
        -O3
        -Ofast
        -march=native
        -mtune=native
        -flto
        -funroll-loops
        -ffast-math
        -DNDEBUG
    )
    target_link_options(${APP_TARGET_OPT} PRIVATE -flto)
else()
    target_compile_options(${APP_TARGET_OPT} PRIVATE -O3 -DNDEBUG)
endif()

# opengl
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})
target_link_libraries(${APP_TARGET_DEBUG} OpenGL)
target_link_libraries(${APP_TARGET_OPT} OpenGL)

# glew
add_subdirectory(external/glew/build/cmake)
include_directories(external/glew/include)
target_link_libraries(${APP_TARGET_DEBUG} glew)
target_link_libraries(${APP_TARGET_OPT} glew)

# glfw
add_subdirectory(external/glfw)
include_directories(external/glfw/include)
target_link_libraries(${APP_TARGET_DEBUG} glfw)
target_link_libraries(${APP_TARGET_OPT} glfw)

# imgui
add_subdirectory(external/imgui)
include_directories(external/imgui)
target_link_libraries(${APP_TARGET_DEBUG} imgui)
target_link_libraries(${APP_TARGET_OPT} imgui)

# glm
add_subdirectory(external/glm)
include_directories(external/glm)
target_link_libraries(${APP_TARGET_DEBUG} glm)
target_link_libraries(${APP_TARGET_OPT} glm)

# eigen
add_subdirectory(external/eigen)
include_directories(external/eigen)
target_link_libraries(${APP_TARGET_DEBUG} eigen)
target_link_libraries(${APP_TARGET_OPT} eigen)
