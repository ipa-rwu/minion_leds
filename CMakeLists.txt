cmake_minimum_required(VERSION 3.5)
project(minion_leds)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_library(WIRINGPI_LIBRARIES NAMES wiringPi REQUIRED)
find_package(Boost COMPONENTS chrono REQUIRED)


#need to be ${PROJECT_NAME} as per https://github.com/ros2/rosidl/issues/441
rosidl_generate_interfaces( ${PROJECT_NAME}   "srv/SetLeds.srv" DEPENDENCIES builtin_interfaces   )
ament_export_dependencies(rosidl_default_runtime)


include_directories(
  include
)

set(PACKAGE_NAME minion_leds)
set(LEDS_SOURCES
   src/leds_server.cpp
   src/leds.cpp
)

add_executable(minion_leds_server  ${LEDS_SOURCES})
ament_target_dependencies(minion_leds_server
  rclcpp)  
target_link_libraries (minion_leds_server ${WIRINGPI_LIBRARIES} crypt rt)
rosidl_target_interfaces(minion_leds_server  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(minion_leds_client  src/leds_client.cpp)
ament_target_dependencies(minion_leds_client
  rclcpp sensor_msgs)  
rosidl_target_interfaces(minion_leds_client  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries (minion_leds_client ${Boost_LIBRARIES})


install(TARGETS
  minion_leds_server
  minion_leds_client
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY params launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
