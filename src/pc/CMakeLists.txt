# SPDX-License-Identifier: MIT
# No external libraries needed (own iiod client + bundled pocketfft).
# Windows: cross-compile with mingw-w64, see build_win.sh
cmake_minimum_required(VERSION 3.10)
project(sgch_pc CXX)
set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Threads REQUIRED)

add_library(sgch_rx_core STATIC sgch_rx.cpp channels.cpp)
set_property(TARGET sgch_rx_core PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(sgch_rx_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(sgch_rx_core PUBLIC Threads::Threads)
if(WIN32)
  target_link_libraries(sgch_rx_core PUBLIC ws2_32)
endif()

add_executable(sgch_rx sgch_rx_main.cpp)
target_link_libraries(sgch_rx sgch_rx_core)
install(TARGETS sgch_rx DESTINATION bin)
add_executable(sgch_chantest chantest.cpp)
target_link_libraries(sgch_chantest sgch_rx_core)

find_package(SoapySDR CONFIG QUIET)
if(SoapySDR_FOUND)
  SOAPY_SDR_MODULE_UTIL(TARGET satcom4plutoSupport SOURCES soapy_satcom4pluto.cpp LIBRARIES sgch_rx_core)
  message(STATUS "SoapySDR found: building the satcom4pluto module")
else()
  message(STATUS "SoapySDR not found: only sgch_rx is built")
endif()
