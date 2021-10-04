cmake_minimum_required(VERSION 2.4)
project(futabe_RS450CB)

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

install(FILES ${headers} DESTINATION include)
install(TARGETS ${target} DESTINATION lib)

