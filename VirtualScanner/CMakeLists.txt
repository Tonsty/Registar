cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(VirtualScanner)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (VirtualScanner VirtualScanner.cpp boost.h)
target_link_libraries (VirtualScanner ${PCL_LIBRARIES} ${BOOST_LIBRARIES})

