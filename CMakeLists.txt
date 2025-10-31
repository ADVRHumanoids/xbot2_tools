cmake_minimum_required(VERSION 3.10)
project(xbot2_tools VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(cmake/ExportProject.cmake)

add_subdirectory(monitor)

export_project()
