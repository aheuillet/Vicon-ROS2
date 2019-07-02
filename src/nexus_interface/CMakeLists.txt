cmake_minimum_required(VERSION 3.5)
project(nexus_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(wxWidgets REQUIRED COMPONENTS net core base)

set(msg_files
  "msg/Position.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

include(${wxWidgets_USE_FILE})
add_executable(client src/gui.cpp src/communicator.cpp src/utils.cpp src/publisher.cpp)
target_link_libraries(client ViconDataStreamSDK_CPP)
target_link_libraries(client ${wxWidgets_LIBRARIES})


ament_target_dependencies(client "rclcpp" "rclutils")

get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

foreach(typesupport_impl ${typesupport_impls})
  rosidl_target_interfaces(client
    ${PROJECT_NAME} ${typesupport_impl}
  )
endforeach()

install(TARGETS client DESTINATION lib/${PROJECT_NAME})
install(FILES settings.cfg DESTINATION lib/${PROJECT_NAME})

ament_package()
