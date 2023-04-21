cmake_minimum_required(VERSION 3.10)
project(boost_serial)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE TRUE)
set(CMAKE_FIND_DEBUG_MODE ON)
set(CMAKE_CXX_STANDARD 11)

find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(include)
include_directories(${Boost_INCLUDE_DIR})

# link_directories(${Boost_LIBRARY_DIR})

add_executable(main
    main.cpp
    boost_serial.cpp
)
target_link_libraries(main 
    ${Boost_LIBRARIES}
)