find_package(xbot2 REQUIRED)

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

# install headers
install(DIRECTORY include/xbot2_tools
    DESTINATION include
    )
