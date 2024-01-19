cmake_minimum_required(VERSION 3.15)

include(proj.conf)

find_package(bouffalo_sdk REQUIRED HINTS $ENV{BL_SDK_BASE})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(BLCommon)


FIND_INCLUDE_DIR(INCLUDE_DIR src)

sdk_add_include_directories(${INCLUDE_DIR})

FIND_SOURCES_DIR(SOURCES_DIR src)

target_sources(app PRIVATE
                   ${SOURCES_DIR}
                   )


add_subdirectory(lib/MPU6050)







sdk_set_main_file(main.c)

project(MPU6050)
