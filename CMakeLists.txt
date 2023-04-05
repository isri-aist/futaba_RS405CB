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

pkg_check_modules(serial REQUIRED serial IMPORTED_TARGET)
set(PACKAGE_EXTRA_MACROS "pkg_check_modules(serial REQUIRED serial IMPORTED_TARGET)")

set(headers
  RS405CB.h
  rs405cb_types.h
  )

set(sources
  RS405CB.cpp
  )

set(target rs405cb)
add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE:include> $<INSTALL_INTERFACE:include/RS405CB>)
set_target_properties(${target} PROPERTIES EXPORT_NAME ${PROJECT_NAME})
target_link_libraries(${target} PUBLIC PkgConfig::serial)

install(FILES ${headers} DESTINATION include/RS405CB)
install(TARGETS ${target} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION "${CMAKE_INSTALL_LIBDIR}")

add_executable(reboot_rs405cb reboot.cpp)
target_link_libraries(reboot_rs405cb PUBLIC rs405cb)
install(TARGETS reboot_rs405cb
  EXPORT ${TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION bin)
