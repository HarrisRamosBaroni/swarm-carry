cmake_minimum_required(VERSION 3.8)
project(swarm_mocap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# OWL SDK — precompiled shared library bundled in lib/
set(OWL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(OWL_LIB        ${CMAKE_CURRENT_SOURCE_DIR}/lib/libowlsock.so)

add_executable(mocap_node src/mocap_node.cpp)

target_include_directories(mocap_node PRIVATE ${OWL_INCLUDE_DIR})
target_link_libraries(mocap_node ${OWL_LIB})
ament_target_dependencies(mocap_node rclcpp geometry_msgs std_msgs)

# Install node binary
install(TARGETS mocap_node DESTINATION lib/${PROJECT_NAME})

# Install the .so so the linker can find it at runtime
install(FILES lib/libowlsock.so
        DESTINATION lib)

# Install launch and config
install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

ament_package()
