cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME manipulator_6_dof)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # User-defined library search paths can be added here
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # User sources can be added here
)

# Add include paths for HAL and other necessary includes
include_directories(
    ${PROJECT_SOURCE_DIR}/Core/Inc
    ${PROJECT_SOURCE_DIR}/Drivers/CMSIS/Core/Include
    ${PROJECT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32L4xx/Include
    ${PROJECT_SOURCE_DIR}/Drivers/STM32L4xx_HAL_Driver/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # User-defined symbols can be added here
)

# Link libraries (ensure you have the correct libraries for HAL)
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Link the HAL library here if needed
)
