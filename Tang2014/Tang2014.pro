VERSION = 1.0.1
TEMPLATE = app
CONFIG += debug_and_release debug_and_release_target warn_off 
DEPENDPATH += .
win32{
	CONFIG += console
	INCLUDEPATH += . "C:/Program Files/VTK/include/vtk-5.8/" "C:/Program Files/PCL/include/pcl-1.8/" "C:/Program Files/Eigen/include/eigen3/" "C:/Program Files/flann/include/" "D:/boost_1_55_0/"
	CONFIG(debug, debug|release){
		LIBS += -L"C:/Program Files/VTK/lib/vtk-5.8/" \
			QVTK_debug.lib vtkCommon_debug.lib vtkRendering_debug.lib vtkFiltering_debug.lib vtkGraphics_debug.lib \
		-L"D:/boost_1_55_0/stage/lib/" \
			boost_system-vc100-mt-gd-1_55.lib \
		-L"C:/Program Files/PCL/lib/" \
			pcl_visualization_debug.lib pcl_io_debug.lib pcl_common_debug.lib pcl_kdtree_debug.lib pcl_search_debug.lib \
			pcl_gpu_containers_debug.lib pcl_gpu_octree_debug.lib pcl_gpu_utils_debug.lib \
		-L"C:/Program Files/flann/lib/" \
			flann_cpp_s_debug.lib	
	}else{
		LIBS += -L"C:/Program Files/VTK/lib/vtk-5.8/" \
			QVTK_release.lib vtkCommon_release.lib vtkRendering_release.lib vtkFiltering_release.lib vtkGraphics_release.lib \
		-L"D:/boost_1_55_0/stage/lib/" \
			boost_system-vc100-mt-1_55.lib \
		-L"C:/Program Files/PCL/lib/" \
			pcl_visualization_release.lib pcl_io_release.lib pcl_common_release.lib pcl_kdtree_release.lib pcl_search_release.lib \
			pcl_gpu_containers_release.lib pcl_gpu_octree_release.lib pcl_gpu_utils_release.lib \
		-L"C:/Program Files/flann/lib/" \
			flann_cpp_s_release.lib		
	}
	QMAKE_CXXFLAGS += /openmp /MP
}
unix{
	INCLUDEPATH += . /usr/include/vtk-5.8/ /usr/local/include/pcl-1.8/ /usr/include/eigen3/
	LIBS += -L/usr/lib/ \
			-lQVTK -lvtkCommon -lQVTK -lvtkRendering -lvtkFiltering -lvtkGraphics \
			-lboost_system \
			-L/usr/local/lib/ \
			-lpcl_visualization -lpcl_io -lpcl_common -lpcl_kdtree -lpcl_search -lpcl_filters \
			-L/usr/lib/gcc/x85_64-linux-gnu/ \
			-lgomp
	QMAKE_CXXFLAGS += -fopenmp
	QMAKE_LFLAGS += -fopenmp 
}
HEADERS += graph.h \
			common.h \
			pairregistration.h \
			globalregistration.h \
			scan.h \
			loop.h \
			link.h \
			../Williams2001/SRoMCPS.h

SOURCES += graph.cpp \
			pairregistration.cpp \
			globalregistration.cpp \
			main.cpp \
			scan.cpp \
			loop.cpp \
			link.cpp \
			../Williams2001/SRoMCPS.cpp



 
