# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

# Reuse the production SDMMC wiring and divider for boot-time update reads.
list(APPEND EXTRA_DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../app/sd/sdmmc_nucleo_l476rg.overlay)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wildsonic_stm32_bootloader)

execute_process(
  COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR}/.. rev-parse --short=12 HEAD
  OUTPUT_VARIABLE WILDSONIC_GIT_COMMIT
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(NOT WILDSONIC_GIT_COMMIT)
  set(WILDSONIC_GIT_COMMIT "unknown")
endif()

target_sources(app PRIVATE src/main.c)
target_compile_definitions(app PRIVATE WILDSONIC_GIT_COMMIT=\"${WILDSONIC_GIT_COMMIT}\")
zephyr_include_directories(include)

