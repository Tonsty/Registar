TEMPLATE = app
CONFIG += debug_and_release debug_and_release_target warn_off 
DEPENDPATH += .
win32{
	CONFIG += console
	INCLUDEPATH += . "C:/Program Files/VTK/include/vtk-5.8/" "C:/Program Files/PCL/include/pcl-1.7/" "C:/Program Files/Eigen/include/eigen3/" "C:/Program Files/flann/include/" "D:/boost_1_55_0/"
	CONFIG(debug, debug|release){
		LIBS += -L"C:/Program Files/VTK/lib/vtk-5.8/" \
			QVTK-gd.lib vtkCommon-gd.lib QVTK-gd.lib vtkRendering-gd.lib vtkFiltering-gd.lib vtkGraphics-gd.lib \
		-L"D:/boost_1_55_0/stage/lib/" \
			libboost_system-vc100-mt-gd-1_55.lib \
		-L"C:/Program Files/PCL/lib/" \
			pcl_visualization_debug.lib pcl_io_debug.lib pcl_common_debug.lib pcl_kdtree_debug.lib pcl_search_debug.lib \
		-L"C:/Program Files/flann/lib/" \
			flann_cpp_s-gd.lib	
	}else{
		LIBS += -L"C:/Program Files/VTK/lib/vtk-5.8/" \
			QVTK.lib vtkCommon.lib QVTK.lib vtkRendering.lib vtkFiltering.lib vtkGraphics.lib \
		-L"D:/boost_1_55_0/stage/lib/" \
			libboost_system-vc100-mt-1_55.lib \
		-L"C:/Program Files/PCL/lib/" \
			pcl_visualization_release.lib pcl_io_release.lib pcl_common_release.lib pcl_kdtree_release.lib pcl_search_release.lib \
		-L"C:/Program Files/flann/lib/" \
			flann_cpp_s.lib		
	}
}
unix{
	INCLUDEPATH += . /usr/include/vtk-5.8/ /usr/local/include/pcl-1.8/ /usr/include/eigen3/
	LIBS += -L/usr/lib/ \
			-lQVTK -lvtkCommon -lQVTK -lvtkRendering -lvtkFiltering -lvtkGraphics \
			-lboost_system \
			-L/usr/local/lib/ \
			-lpcl_visualization -lpcl_io -lpcl_common -lpcl_kdtree -lpcl_search
}
HEADERS += SRoMCPS.h \
			../include/pclbase.h \
			../include/qtbase.h \
			../include/cloud.h \
			../include/cloudio.h \     
			../include/cloudmanager.h \
			../include/pairwiseregistration.h \
			../include/globalregistration.h \
			../include/mathutilities.h \
			../include/registrationdatamanager.h 
SOURCES += main.cpp \
			SRoMCPS.cpp \
			../src/cloud.cpp \
			../src/cloudio.cpp \
			../src/cloudmanager.cpp \
			../src/pairwiseregistration.cpp \
			../src/globalregistration.cpp \
			../src/registrationdatamanager.cpp


