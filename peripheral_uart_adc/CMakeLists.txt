# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

set(DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/boards)

find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(peripheral_uart_adc)

FILE(GLOB app_sources src/*.c src/*.h)
target_sources(app PRIVATE ${app_sources})