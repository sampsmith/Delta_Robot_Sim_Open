cmake_minimum_required(VERSION 3.10)
project(delta_robot_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimization flags
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# ImGui source files
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/external/imgui)
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# Project source files
set(PROJECT_SOURCES
    src/main.cpp
    src/kinematics/DeltaRobot.cpp
    src/visualization/Renderer.cpp
)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${OPENGL_INCLUDE_DIR}
)

# Create executable
add_executable(${PROJECT_NAME} 
    ${PROJECT_SOURCES}
    ${IMGUI_SOURCES}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    OpenGL::GL
    glfw
    ${CMAKE_DL_LIBS}
)

# Set output directory
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}
)
