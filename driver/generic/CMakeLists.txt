cmake_minimum_required(VERSION 2.8)

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${CMAKE_CURRENT_SOURCE_DIR}/../..)
add_definitions(-g -ggdb -O0 -Wall)
add_library(puppet_api 
    STATIC
    src/PuppetMessage.cpp 
    src/PuppetTopology.cpp 
    src/PuppetParserRQ.cpp
    src/PuppetParserWF.cpp
)


add_executable(test_parser_wf test/testWF.cpp)
target_link_libraries(test_parser_wf puppet_api)

add_executable(test_parser_rq test/testRQ.cpp)
target_link_libraries(test_parser_rq puppet_api)



