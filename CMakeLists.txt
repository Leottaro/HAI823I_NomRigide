cmake_minimum_required(VERSION 3.20.0)
project(hai823i_nomrigide)
set(APP_VERSION_MAJOR 1)
set(APP_VERSION_MINOR 0)

set(CMAKE_CXX_STANDARD 17)
set(OpenGL_GL_PREFERENCE GLVND) # asked me to set this to GLVND or LEGACY
# set(CMAKE_VERBOSE_MAKEFILE 1) # If you want verbose
option(ENABLE_OPENMP "Enable OpenMP  when available" OFF)

# target setting
set(APP_TARGET hai823i_nomrigide)
set(APP_TARGET_DEBUG ${APP_TARGET}_debug)
set(APP_TARGET_OPT ${APP_TARGET}_opt)
include_directories(${PROJECT_SOURCE_DIR} .)

# my src files
set(APP_SOURCES
    src/main.cpp

    src/ShaderProgram.hpp
    src/ShaderProgram.cpp

    src/PositionHasher.hpp
    src/Transformation.hpp
    src/AABB.hpp

    src/Camera.hpp
    src/Camera.cpp

    src/Mesh.hpp
    src/Mesh.cpp
    src/MeshGenerator.cpp
    src/DynamicObjectGenerator.cpp
    src/DynamicObjectConstraints.cpp

    src/DynamicObject.hpp
    src/DynamicObject.cpp
    src/DynamicObjectCollision.cpp

    src/Scene.hpp
    src/Scene.cpp
)

add_executable(${APP_TARGET_DEBUG} ${APP_SOURCES})
add_executable(${APP_TARGET_OPT} ${APP_SOURCES})

if(ENABLE_OPENMP)
    if(APPLE AND EXISTS "/opt/homebrew/opt/libomp")
        list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/opt/libomp")
    endif()

    find_package(OpenMP QUIET)
    if(OpenMP_CXX_FOUND)
        target_link_libraries(${APP_TARGET_DEBUG} OpenMP::OpenMP_CXX)
        target_link_libraries(${APP_TARGET_OPT} OpenMP::OpenMP_CXX)
        target_compile_definitions(${APP_TARGET_DEBUG} PRIVATE USE_OPENMP)
        target_compile_definitions(${APP_TARGET_OPT} PRIVATE USE_OPENMP)
        message(STATUS "OpenMP enabled")
    else()
        message(STATUS "OpenMP not found, building without OpenMP")
    endif()
else()
    message(STATUS "OpenMP disabled")
endif()

target_compile_options(${APP_TARGET_DEBUG} PRIVATE -Wall -O0 -g3)
target_compile_definitions(${APP_TARGET_DEBUG} PRIVATE DEBUG)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${APP_TARGET_OPT} PRIVATE
        -Wall
        -O3
        # -Ofast
        -march=native
        -mtune=native
        # -flto
        -funroll-loops
        # -ffast-math
        -DNDEBUG
    )
    target_link_options(${APP_TARGET_OPT} PRIVATE -flto)
else()
    target_compile_options(${APP_TARGET_OPT} PRIVATE -Wall -O3 -DNDEBUG)
endif()

# opengl
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})
target_link_libraries(${APP_TARGET_DEBUG} OpenGL::GL)
target_link_libraries(${APP_TARGET_OPT} OpenGL::GL)

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
target_link_libraries(imgui PRIVATE glfw)
target_link_libraries(${APP_TARGET_DEBUG} imgui)
target_link_libraries(${APP_TARGET_OPT} imgui)

# poly34
add_subdirectory(external/poly34)
include_directories(external/poly34)
target_link_libraries(${APP_TARGET_DEBUG} poly34)
target_link_libraries(${APP_TARGET_OPT} poly34)

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
