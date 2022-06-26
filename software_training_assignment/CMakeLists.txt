cmake_minimum_required(VERSION 3.5)
project(software_training_assignment)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# setup interfaces
# set(srv_files
# 	""
# )

# set (msg_files
# 	""
# )

# rosidl_generate_interfaces(${PROJECT_NAME}
# 	${srv_files}
# 	${msg_files}
# )

# ament_export_dependencies(rosidl_default_runtime)

#include the 'include' directory
include_directories(include)

#create resource which references the libraries in the binary bin
set(node_plugins "")

#add plugins as SHARED library 

#add turtle_request_service_node as a plugin
add_library(clear_request SHARED 
	src/clear_turtles.cpp)
target_compile_definitions(clear_request
	PRIVATE "SOFTWARE_TRAINING_DLL")
ament_target_dependencies(clear_request
	"rclcpp"
	"rclcpp_components"
	"turtlesim"
	"std_srvs")
# another way of registering and setting the node as a component below
# rclcpp_components_register_nodes(clear_request PLUGIN "composition::clear_turtles" EXECUTABLE request_clear)
rclcpp_components_register_nodes(clear_request "composition::clear_turtles")
# this way we can execute the component with - ros2 component standalone software_training_assignment composition::clear_turtles
set(node_plugins "${node_plugins}compositon::clear_turtles;$<TARGET_FILE:clear_request>\n")

#tell where to put binaries
install(TARGETS
	clear_request
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib 
	RUNTIME DESTINATION bin)

ament_package()