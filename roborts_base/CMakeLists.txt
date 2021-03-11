project(roborts_base)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
add_definitions(-w)
set(CMAKE_BUILD_TYPE Release)
find_package(Threads REQUIRED)
add_subdirectory(roborts_sdk)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        roborts_msgs
)

catkin_package(CATKIN_DEPENDS roscpp tf roborts_msgs)

add_executable(roborts_base_node
        src/roborts_base_node.cpp
        src/chassis.cpp
        src/gimbal.cpp
        src/referee_system.cpp
        )
target_link_libraries(roborts_base_node PUBLIC
        Threads::Threads
        ${catkin_LIBRARIES}
        roborts_sdk)
target_include_directories(roborts_base_node PUBLIC
        ${catkin_INCLUDE_DIRS}
        include
        roborts_sdk/include)
add_dependencies(roborts_base_node
        roborts_msgs_generate_messages
        roborts_sdk)