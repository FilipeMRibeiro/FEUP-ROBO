# CMake version needed
cmake_minimum_required(VERSION 2.8.3)

# Name of the project
project(reactive_robot)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
)

# Declare our catkin package.
catkin_package()

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files. If these are
# multiple executables, use multiple copies of this line.
add_executable(robot main.cpp)

# Specify libraries agains to link. Again, this line should be copied 
# for each distincti executable in the package.
target_link_libraries(robot ${catkin_LIBRARIES})

