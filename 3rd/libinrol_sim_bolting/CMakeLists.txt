cmake_minimum_required(VERSION 3.5)
project(libinrol_sim_bolting)

add_library(inrol_sim_bolting STATIC IMPORTED)
set_property(
  TARGET inrol_sim_bolting PROPERTY
  IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/lib/libinrol_sim_bolting.a)
set_property(
  TARGET inrol_sim_bolting APPEND PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR}/include)