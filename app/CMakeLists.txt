# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

# SD card bus: native SDMMC interface default (or spi if overridden: -DSD_BUS=spi)
set(SD_BUS "sdmmc" CACHE STRING "SD card bus: spi or sdmmc")
option(FORMAT_SD_CARD "Build one-shot SD exFAT formatter instead of recorder" OFF)

string(REPLACE "/" "_" BOARD_FLAT "${BOARD}")
set(SD_OVERLAY ${CMAKE_CURRENT_SOURCE_DIR}/sd/${SD_BUS}_${BOARD_FLAT}.overlay)
if(NOT EXISTS ${SD_OVERLAY})
  # Zephyr 4.x qualified board targets append the SoC qualifier, while this
  # application keeps the shared overlay named after the board itself.
  string(REGEX REPLACE "/.*$" "" BOARD_NAME "${BOARD}")
  set(SD_OVERLAY ${CMAKE_CURRENT_SOURCE_DIR}/sd/${SD_BUS}_${BOARD_NAME}.overlay)
endif()
if(EXISTS ${SD_OVERLAY})
  list(APPEND EXTRA_DTC_OVERLAY_FILE ${SD_OVERLAY})
else()
  message(WARNING "No SD overlay ${SD_OVERLAY}; add one or define the 'SD' disk yourself")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wildsonic_stm32_recorder)

if(FORMAT_SD_CARD)
  message(WARNING "FORMAT_SD_CARD=ON: formatter firmware WILL ERASE the SD card on boot")
  target_sources(app PRIVATE src/format_sd.c)
else()
  target_sources(app PRIVATE
    src/main.c
    src/recorder.c
    src/wav.c
    src/audio_in_stm32l4.c
    src/audio_format.c
    src/psram_is66wvs.c
    src/psram_staging.c
    src/dsp.c
    src/rtc_sleep.c
    src/rtc_calendar.c
    src/scheduler.c
    src/console.c
    src/config_store.c
    src/led_indicator.c
    src/ble_comm.c
  )
  target_sources_ifdef(CONFIG_WILDSONIC_VERBOSE_LOGS app PRIVATE src/psram_benchmark.c)
  zephyr_include_directories(src)
endif()
