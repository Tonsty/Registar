TEMPLATE = app

CONFIG += debug_and_release

CONFIG(debug,debug|release){
 unix : TARGET = initial_registration_by_bundler_debug
 win32 : TARGET = initial_registration_by_bundler_debug.exe
 mac : TARGET = initial_registration_by_bundler_debug
}else{
 unix : TARGET = initial_registration_by_bundler_release
 win32 : TARGET = initial_registration_by_bundler_release.exe
 mac : TARGET = initial_registration_by_bundler_release
}

INCLUDEPATH += . /usr/local/include/pcl-1.7 /usr/include/eigen3

LIBS += -lboost_system \
        -L/usr/local/lib/ -lpcl_common -lpcl_io -lpcl_io_ply -lpcl_filters \
        -lopencv_core

DESTDIR = .
HEADERS +=
SOURCES += initial_registration_by_bundler.cpp
 
