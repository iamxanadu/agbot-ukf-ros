cmake_minimum_required(VERSION 2.8.3)
project(agbot_ukf)

find_package(catkin_simple REQUIRED)

catkin_simple()

add_compile_options(-g)

file(GLOB MATLAB_AUTOCODE src/matlab_autocode/*.cpp)

cs_add_executable(${PROJECT_NAME}
  src/agbot_ukf_node.cpp ${MATLAB_AUTOCODE})

cs_install()

cs_export()