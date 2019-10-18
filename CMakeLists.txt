cmake_minimum_required(VERSION 3.5)
project(ompl_rrtstar_ros)


set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O3")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        sensor_msgs
        roscpp
        rospy
        std_msgs)

find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED)

find_package(Boost COMPONENTS program_options REQUIRED)
find_package(ompl REQUIRED)

set(CMAKE_CXX_STANDARD 14)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OMPL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS})


catkin_package(
        CATKIN_DEPENDS
        roscpp
        geometry_msgs
        nav_msgs
        rospy
        std_msgs
        DEPENDS
        EIGEN3)


set(sources src/main.cpp
        src/MarkerStuff.cpp
        src/Planner.cpp
        src/ValidityChecker.cpp
        src/MyMotionValidator.cpp
        src/MyStateSampler.cpp
        src/StopWatch.cpp
        )

add_executable(${PROJECT_NAME} ${sources})

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OMPL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES})
