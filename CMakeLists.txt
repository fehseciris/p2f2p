cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(FrenetSol)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

set(SOURCE_FILES
    main.cpp
    core.cpp
    iol.cpp
    numMethods.cpp
    oswrappers.cpp
    p2f2p.cpp
)

set(HEADER_FILES
    core.h
    iol.h
    numMethods.h
    oswrappers.h
    p2f2p.h
    util.h
)

add_executable(FrenetSol ${SOURCE_FILES} ${HEADER_FILES})

# Eof