cmake_minimum_required(VERSION 3.22.1)

set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 23)

project(sapr)

set(SOURCE_FILES src/main.cpp)
add_executable(main ${SOURCE_FILES})


