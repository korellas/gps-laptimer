# GPS Laptimer - ESP-IDF Project
# For Waveshare ESP32-S3-Touch-LCD-3.49

cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/components
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# 버전 주입: git tag 또는 -DAPP_VERSION_FROM_GIT="v0.5.0" 사용
if(NOT APP_VERSION_FROM_GIT)
    execute_process(
        COMMAND git describe --tags --always --dirty
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE APP_VERSION_FROM_GIT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
endif()
if(APP_VERSION_FROM_GIT)
    add_compile_definitions(APP_VERSION_FROM_GIT="${APP_VERSION_FROM_GIT}")
endif()

project(gps_laptimer)
