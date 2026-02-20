# GPS Laptimer - ESP-IDF Project
# For Waveshare ESP32-S3-Touch-LCD-3.49

cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/components
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# 버전 주입: git describe를 항상 실행 (캐시 스테일 방지)
# 커맨드라인 -DAPP_VERSION_FROM_GIT="v1.2.0" 로 오버라이드 가능
execute_process(
    COMMAND git describe --tags --always --dirty
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE APP_GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT APP_VERSION_FROM_GIT)
    set(APP_VERSION_FROM_GIT "${APP_GIT_VERSION}")
endif()
if(APP_VERSION_FROM_GIT)
    add_compile_definitions(APP_VERSION_FROM_GIT="${APP_VERSION_FROM_GIT}")
endif()

project(gps_laptimer)
