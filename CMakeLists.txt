cmake_minimum_required(VERSION 3.15)
project(DeltaRobotSim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(OpenGL REQUIRED)

# Use FetchContent to automatically download dependencies
include(FetchContent)

# Download GLFW if not found
find_package(glfw3 QUIET)
if(NOT glfw3_FOUND)
    message(STATUS "GLFW not found, downloading via FetchContent...")
    FetchContent_Declare(
        glfw
        URL https://github.com/glfw/glfw/releases/download/3.3.8/glfw-3.3.8.zip
        URL_HASH SHA256=4d025083cc4a3dd1f91ab9b9ba4f5807193823e565a5bcf4be202669d9911ea6
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(glfw)
    set(glfw3_FOUND TRUE)
endif()

# Download ImGui if not already in external directory
if(EXISTS "${CMAKE_SOURCE_DIR}/external/imgui/imgui.h")
    message(STATUS "Using existing ImGui at external/imgui")
    set(imgui_SOURCE_DIR "${CMAKE_SOURCE_DIR}/external/imgui")
else()
    message(STATUS "Downloading ImGui via FetchContent...")
    FetchContent_Declare(
        imgui
        URL https://github.com/ocornut/imgui/archive/v1.89.9.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_GetProperties(imgui)
    if(NOT imgui_POPULATED)
        FetchContent_Populate(imgui)
        # After Populate, imgui_SOURCE_DIR is automatically set
        # Check if it's in a subdirectory (imgui-1.90.0)
        if(NOT EXISTS "${imgui_SOURCE_DIR}/imgui.h")
            file(GLOB imgui_dirs "${imgui_SOURCE_DIR}/imgui-*")
            if(imgui_dirs)
                list(GET imgui_dirs 0 imgui_subdir)
                set(imgui_SOURCE_DIR "${imgui_subdir}")
            endif()
        endif()
    endif()
endif()

# Download GLM if not already in external directory  
if(EXISTS "${CMAKE_SOURCE_DIR}/external/glm/glm/glm.hpp")
    message(STATUS "Using existing GLM at external/glm")
    set(glm_SOURCE_DIR "${CMAKE_SOURCE_DIR}/external/glm")
else()
    message(STATUS "Downloading GLM via FetchContent...")
    FetchContent_Declare(
        glm
        URL https://github.com/g-truc/glm/archive/0.9.9.8.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_GetProperties(glm)
    if(NOT glm_POPULATED)
        FetchContent_Populate(glm)
        # After Populate, glm_SOURCE_DIR is automatically set
        # Check if it's in a subdirectory (glm-0.9.9.8)
        if(NOT EXISTS "${glm_SOURCE_DIR}/glm/glm.hpp")
            file(GLOB glm_dirs "${glm_SOURCE_DIR}/glm-*")
            if(glm_dirs)
                list(GET glm_dirs 0 glm_subdir)
                set(glm_SOURCE_DIR "${glm_subdir}")
            endif()
        endif()
    endif()
endif()

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${imgui_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/external/glad/include)
include_directories(${glm_SOURCE_DIR})

# GLAD sources (not needed for compatibility profile, but keeping structure)
# file(GLOB GLAD_SOURCES "external/glad/src/glad.c")

# ImGui sources
file(GLOB IMGUI_SOURCES 
    "${imgui_SOURCE_DIR}/*.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
)

# Project sources
set(SOURCES
    src/main.cpp
    src/DeltaRobot.cpp
    src/Renderer.cpp
    src/MotorControl.cpp
    src/HardwareInterface.cpp
    src/DeltaRobotProtocol.cpp
    src/SequenceController.cpp
    src/ControlPanel.cpp
    src/RobotSetupPanel.cpp
    src/WaypointPanel.cpp
    src/SequencePanel.cpp
    src/HardwarePanel.cpp
    src/DisplayPanel.cpp
    src/UIManager.cpp
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

# Windows-specific libraries
if(WIN32)
    target_link_libraries(${PROJECT_NAME} ws2_32)
endif()

# Include directories for target
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${CMAKE_SOURCE_DIR}/external/glad/include
    ${glm_SOURCE_DIR}
)

