cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 20)

project(sim)

find_package(OpenCV 4.7 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(. SRC_LISTS)

add_executable(sim ${SRC_LISTS})
target_link_libraries(sim ${OpenCV_LIBS})
