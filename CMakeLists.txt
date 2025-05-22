#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(GNSS_NRF91)

zephyr_library_sources(src/main.c)

# Add the component LTE
target_sources(app PRIVATE
    components/nrf91_modem/nrf91_modem.c)

target_include_directories(app
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/components/nrf91_modem)

# Add the component LTE
target_sources(app PRIVATE
    components/gnss/gnss.c)

target_include_directories(app
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/components/gnss)
