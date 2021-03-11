project(roborts_sdk)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-w)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

find_package(Threads REQUIRED)
find_package(Glog REQUIRED)

add_library(roborts_sdk
        SHARED
        src/execution.cpp
        src/handle.cpp
        src/protocol.cpp
        src/serial_device.cpp
        )
target_link_libraries(roborts_sdk
        PUBLIC
        Threads::Threads
        ${GLOG_LIBRARY})
target_include_directories(roborts_sdk
        PUBLIC
        include
)
add_executable(roborts_sdk_sample
        sample/sample.cpp)
target_link_libraries(roborts_sdk_sample
        PUBLIC
        roborts_sdk)
target_include_directories(roborts_sdk_sample
        PUBLIC
        include)
