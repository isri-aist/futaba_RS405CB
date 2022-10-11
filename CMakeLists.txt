cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME RS405CB)
set(PROJECT_DESCRIPTION "A Futaba RS405CB servo motor library")
set(PROJECT_URL https://github.com/isri-aist/futabe_RS450CB)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(PROJECT_USE_KEYWORD_LINK_LIBRARIES TRUE)
set(INSTALL_DOCUMENTATION OFF CACHE BOOL "")
set(INSTALL_GENERATED_HEADERS OFF CACHE BOOL "")
set(INSTALL_PKG_CONFIG_FILE OFF CACHE BOOL "")
set(CXX_DISABLE_WERROR ON)

include(cmake/base.cmake)
project(${PROJECT_NAME} LANGUAGES CXX)

set(headers
  serial_port.h
  RS405CB.h
  )

set(sources
  serial_port.cpp
  RS405CB.cpp
  )

set(target rs405cb)
add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE:include/RS405CB>)
set_target_properties(${target} PROPERTIES EXPORT_NAME ${PROJECT_NAME})

install(FILES ${headers} DESTINATION include/RS405CB)
install(TARGETS ${target} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION "${CMAKE_INSTALL_LIBDIR}")

