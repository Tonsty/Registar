TEMPLATE = app
CONFIG += debug_and_release warn_off
TARGET = manual_registration
CONFIG(debug,debug|release){
  unix:TARGET = $$join(TARGET,,,_debug)
  win32: TARGET = $$join(TARGET,,,_debug.exe)
  mac: TARGET = $$join(TARGET,,,_debug)
}else{
  unix:TARGET = $$join(TARGET,,,_release)
  win32: TARGET = $$join(TARGET,,,_release.exe)
  mac: TARGET = $$join(TARGET,,,_debug)
}

LIBS += -lQVTK -lboost_system \
        -L/usr/local/lib/ -lpcl_common -lpcl_io -lpcl_visualization -lpcl_segmentation \
        -lpcl_features -lpcl_surface -lpcl_io -lpcl_io_ply -lpcl_kdtree -lpcl_common \
        -lflann_cpp -lvtkViews -lvtkInfovis -lvtkWidgets -lvtkHybrid \
        -lvtkRendering -lvtkIO -lvtkImaging -lvtkGraphics -lvtkFiltering -lvtkCommon -lm -lvtksys -lvtkQtChart

INCLUDEPATH += . /usr/local/include/pcl-1.8/ /usr/include/eigen3 /usr/include/vtk-5.8/ /usr/include/ni/

# Input
FORMS += manual_registration.ui
HEADERS += manual_registration.h
SOURCES += manual_registration.cpp main.cpp
