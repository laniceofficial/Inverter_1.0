cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Inverter_1.0)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)


# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)
###############################以下为修改过的cmakelist#########################################


function(USER_ADD_DIR dir)
    file(GLOB USER_SOURCE
            ${dir}/*.h
            ${dir}/*.c
            ${dir}/*.cc
    )
    target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${USER_SOURCE})
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${dir})
endfunction()
###############################在此处添加你的文件路径
#USER_ADD_DIR(${CMAKE_SOURCE_DIR}/User/can_part)
#USER_ADD_DIR(${CMAKE_SOURCE_DIR}/User/cpp_files)
#USER_ADD_DIR(${CMAKE_SOURCE_DIR}/User/djimotor)
#USER_ADD_DIR(${CMAKE_SOURCE_DIR}/User/dwt)
#USER_ADD_DIR(${CMAKE_SOURCE_DIR}/User/filter_alg)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/User/pid)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/User/pr_ctr)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/User/collection)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/User/task)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/User/svpwm)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/User/bsp_soft_iic)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/User/oled)

#USER_ADD_DIR(${CMAKE_SOURCE_DIR}/User/oledlib)

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC
        stm32cubemx
##################################添加数学库
)

add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
        COMMENT "Building hex & bin file...\nEXCUTABLE SIZE:"
        COMMAND ${SIZE} ${PROJECT_NAME}.elf
)
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--no-warn-rwx-segments")
#先输入  cmake -B build -G "Ninja"
#再输入  ninja -C build
#即可编译
#######软浮点输出
#set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -mfloat-abi=soft")
set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-u, _printf_float")