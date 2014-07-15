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

LIBS += -Wall -Wextra -Wno-unknown-pragmas -fno-strict-aliasing -Wno-format-extra-args -Wno-sign-compare -Wno-invalid-offsetof \
  -Wno-conversion  -march=native -msse4.2 -mfpmath=sse -Wabi -pthread -fopenmp  -Wno-deprecated -O2 -g -DNDEBUG  -Wl,--as-needed \
  -rdynamic -lboost_system -lboost_filesystem -lboost_thread -lboost_date_time -lboost_iostreams -lpthread -lpthread \
  -L/usr/local/lib/ -lpcl_common -lpcl_io -lpcl_visualization -lpcl_segmentation -lpcl_features -lpcl_surface \
  -lQVTK -lQtGui -lQtCore -lpcl_io -lpcl_io_ply -lpng -lusb-1.0 -lOpenNI -lvtkCharts -lGLU -lGL -lSM -lICE -lX11 -lXext -lpcl_filters \
  -lpcl_sample_consensus -lpcl_search -lpcl_kdtree -lpcl_common \
  -lflann_cpp -lpcl_octree -lboost_system -lboost_filesystem -lboost_thread -lboost_date_time -lboost_iostreams -lpthread -lqhull -lvtkViews -lvtkInfovis -lvtkWidgets -lvtkHybrid -lvtkVolumeRendering -lvtkParallel \
  -lvtkRendering -lvtkIO -lvtkImaging -lvtkGraphics -lvtkFiltering -lm -lvtkCommon -lm -lvtksys -ldl -lvtkQtChart -lQtGui -lQtSql -lQtNetwork -lQtCore -Wl,-rpath,/usr/lib/openmpi/lib: -Wl,-rpath-link,/usr/lib/openmpi/lib 

INCLUDEPATH += . /usr/local/include/pcl-1.7/ /usr/include/eigen3 /usr/include/vtk-5.8/ /usr/include/ni/

# Input
FORMS += manual_registration.ui
HEADERS += manual_registration.h
SOURCES += manual_registration.cpp
