cmake_minimum_required(VERSION 2.4)

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${CMAKE_CURRENT_SOURCE_DIR}/../..)
add_definitions(-g -ggdb -O0 -Wall)
add_library(puppet_serial 
    STATIC
    src/Serial.cpp 
    src/SerialLineHandler.cpp 
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../generic/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../generic/lib)
add_executable(print_out test/print_out.cpp)
target_link_libraries(print_out puppet_serial puppet_api)

set(CMAKE_CXX__FLAGS "-g")
