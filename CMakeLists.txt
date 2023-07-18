cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 14)

project(quintic_polynomials_planner_ros)

find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)

find_package(catkin REQUIRED 
    COMPONENTS
        roscpp
        geometry_msgs
        tf
        )


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES quintic_polynomial_planner
        CATKIN_DEPENDS roscpp geometry_msgs tf 
        DEPENDS Eigen 
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
)
add_library(
  quintic_polynomial_planner
  src/quintic_polynomial.cpp
)
target_link_libraries(quintic_polynomial_planner ${catkin_LIBRARIES} ${Eigen_LIBRARIES})