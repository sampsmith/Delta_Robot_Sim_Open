cmake_minimum_required(VERSION 3.15)
project(DeltaRobotSim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/external/imgui)
include_directories(${CMAKE_SOURCE_DIR}/external/glad/include)
include_directories(${CMAKE_SOURCE_DIR}/external/glm)

# GLAD sources (not needed for compatibility profile, but keeping structure)
# file(GLOB GLAD_SOURCES "external/glad/src/glad.c")

# ImGui sources
file(GLOB IMGUI_SOURCES 
    "external/imgui/*.cpp"
    "external/imgui/backends/imgui_impl_glfw.cpp"
    "external/imgui/backends/imgui_impl_opengl3.cpp"
)

# Project sources
set(SOURCES
    src/main.cpp
    src/DeltaRobot.cpp
    src/Renderer.cpp
)

# Executable
add_executable(${PROJECT_NAME} ${SOURCES} ${IMGUI_SOURCES})

# Set position independent code for compatibility
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Link libraries
target_link_libraries(${PROJECT_NAME} 
    OpenGL::GL
    glfw
)

# Include directories for target
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/external/imgui
    ${CMAKE_SOURCE_DIR}/external/imgui/backends
    ${CMAKE_SOURCE_DIR}/external/glad/include
    ${CMAKE_SOURCE_DIR}/external/glm
)

