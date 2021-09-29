cmake_minimum_required(VERSION 3.10)
project(ExampleProject)

set(CMAKE_CXX_STANDARD 14)

include(/home/maxbundy/git/esp-idf/tools/cmake/project.cmake)

include_directories(src)

add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(../googletest googletest)
