cmake_minimum_required(VERSION 2.8)

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(IOKit_FOUND false)
set(IOKit_INCLUDE_DIR)
set(IOKit_LIBRARY)
set(Carbon_FOUND false)
set(Carbon_INCLUDE_DIR)
set(Carbon_LIBRARY)
set(AppKit_FOUND false)
set(AppKit_INCLUDE_DIR)
set(AppKit_LIBRARY)
if(APPLE) # The only platform it makes sense to check for IOKit

find_library(IOKit IOKit)
if(IOKit)
 set(IOKit_FOUND true)
 set(IOKit_INCLUDE_DIR ${IOKit})
 set(IOKit_LIBRARY ${IOKit})
endif(IOKit)

find_library(AppKit AppKit)
if(AppKit)
 set(AppKit_FOUND true)
 set(AppKit_INCLUDE_DIR ${AppKit})
 set(AppKit_LIBRARY ${AppKit})
endif(AppKit)

find_library(Carbon Carbon)
if(Carbon)
 set(Carbon_FOUND true)
 set(Carbon_INCLUDE_DIR ${Carbon})
 set(Carbon_LIBRARY ${Carbon})
endif(Carbon)

endif(APPLE)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${CMAKE_CURRENT_SOURCE_DIR}/../..)
#add_definitions(-g -ggdb -O0 -Wall)
add_definitions(-O3 -Wall)# -framework IOKit -framework Carbon -framework AppKit)
add_library(puppet_d2xx_serial 
    STATIC
    src/D2XXSerial.cpp 
    src/OpenD2XXSerial.cpp 
    src/SerialLineHandler.cpp 
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../generic/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../generic/lib)
link_directories(/opt/local/lib)
include_directories(/usr/local/include)
include_directories(/opt/local/include)

add_executable(print_out test/print_out.cpp)

# http://www.openguru.com/2009/04/cmake-detecting-platformoperating.html
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libftd2xx1.1.12)
  link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libftd2xx1.1.12/build/i386)
  # link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libftd2xx1.1.12/build/x86_64)
  # Linux specific code
  set(CLOCK_LIBS rt)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux") 

target_link_libraries(print_out puppet_d2xx_serial puppet_api ftdi ftd2xx pthread dl ${CLOCK_LIBS} ${IOKit_LIBRARY} ${AppKit_LIBRARY} ${Carbon_LIBRARY})


