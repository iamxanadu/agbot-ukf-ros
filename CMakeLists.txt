cmake_minimum_required(VERSION 2.8.3)
project(agbot_kf)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Add symbols for GDB debugging line by line
# add_compile_options(-g)

# Glob additional sources for compilation
file(GLOB MATLAB_AUTOCODE src/matlab_autocode/*.cpp)

# Add extra include directories
include_directories(include/matlab_autocode include/spkf)

# Compile executable
cs_add_executable(${PROJECT_NAME}
  src/agbot_kf_node.cpp ${MATLAB_AUTOCODE})

cs_install()

cs_export()