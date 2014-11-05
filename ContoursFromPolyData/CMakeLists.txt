
cmake_minimum_required(VERSION 2.8)

PROJECT(ContoursFromPolyData)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(ContoursFromPolyData MACOSX_BUNDLE ContoursFromPolyData)

if(VTK_LIBRARIES)
  target_link_libraries(ContoursFromPolyData ${VTK_LIBRARIES})
else()
  target_link_libraries(ContoursFromPolyData vtkHybrid vtkWidgets)
endif()
