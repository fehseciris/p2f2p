cmake_minimum_required(VERSION 3.10)

project(p2f2p VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")

find_package(Python3 REQUIRED)

set(sources
    src/main.cpp 
    src/iol.cpp
    src/num.cpp
    src/oswrappers.cpp
    src/p2f2p.cpp
    src/util.cpp
    )

add_executable(
    ${CMAKE_PROJECT_NAME} 
    ${sources}
    )

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES AUTOMOC OFF)
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES AUTOUIC OFF)
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES AUTORCC OFF)

# Eof