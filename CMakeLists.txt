cmake_minimum_required(VERSION 2.8.3)
project(agbot_ukf)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(${PROJECT_NAME}
  src/agbot_ukf_node.cpp)

cs_install()

cs_export()