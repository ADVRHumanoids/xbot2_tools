find_package(xbot2 REQUIRED)

## monitor

add_library(monitor SHARED src/monitor.cpp)

set_target_properties(monitor PROPERTIES OUTPUT_NAME xbot2_tools_monitor)

target_link_libraries(monitor
    PRIVATE xbot2::xbot2 xbot2::xbot2_ros_support)

target_include_directories(monitor
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/xbot2_tools/monitor>
    $<INSTALL_INTERFACE:include/xbot2_tools/monitor>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS monitor
    DESTINATION lib
    EXPORT ${PROJECT_NAME}Targets
)

## install headers
install(DIRECTORY include/xbot2_tools
    DESTINATION include
    )


## trajectory plugin

add_xbot2_plugin(trajectory_plugin src/trajectory_plugin.cpp)

install(TARGETS trajectory_plugin DESTINATION lib)


## gcomp plugin

add_xbot2_plugin(gcomp_plugin src/gcomp_plugin.cpp)

install(TARGETS gcomp_plugin DESTINATION lib)
