cmake_minimum_required(VERSION 3.8)
project(gazebo_actor_proxy_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)

include_directories(${gazebo_dev_INCLUDE_DIRS})
link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(actor_proxy_follower SHARED
  src/actor_proxy_follower.cpp
)

target_link_libraries(actor_proxy_follower
  ${gazebo_dev_LIBRARIES}
)

install(TARGETS actor_proxy_follower
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

