cmake_minimum_required(VERSION 3.14)

project(qea)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV 4.5 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(. SRC_LISTS)

add_executable(qea ${SRC_LISTS})
target_link_libraries(qea ${OpenCV_LIBS} -lwiringPi -pthread)
