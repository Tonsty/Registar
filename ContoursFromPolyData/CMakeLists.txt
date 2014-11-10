
cmake_minimum_required(VERSION 2.8)

PROJECT(ContoursFromPolyData)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ContoursFromPolyData ContoursFromPolyData.cxx ../set_color/set_color.cpp ../set_color/set_color.h)

target_link_libraries(ContoursFromPolyData ${VTK_LIBRARIES} ${PCL_LIBRARIES})

