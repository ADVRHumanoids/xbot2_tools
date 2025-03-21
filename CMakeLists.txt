cmake_minimum_required(VERSION 3.16)
project(xbot2_tools VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(cmake/ExportProject.cmake)

add_subdirectory(monitor)

export_project()
