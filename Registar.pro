TEMPLATE = app
TARGET= Registar
CONFIG += debug_and_release debug_and_release_target warn_off 
DEPENDPATH += .
UI_DIR = ui/
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
		-L/usr/lib/gcc/x85_64-linux-gnu/ \
			-lgomp \
		-L/usr/local/lib/ \
			-lpcl_visualization -lpcl_io -lpcl_common -lpcl_kdtree -lpcl_search \
			-lpcl_gpu_containers -lpcl_gpu_octree -lpcl_gpu_utils
	QMAKE_CXXFLAGS += -fopenmp
	QMAKE_LFLAGS += -fopenmp 
}
HEADERS += include/mainwindow.h \
			include/pclbase.h \
			include/qtbase.h \
			include/cloud.h \
			include/cloudio.h \     
			include/cloudmanager.h \
			include/cloudbrowser.h \
			include/cloudvisualizer.h \
			include/euclideanclusterextractiondialog.h \
			include/euclideanclusterextraction.h \
			include/voxelgriddialog.h \
			include/voxelgrid.h \
			include/movingleastsquaresdialog.h \
			include/movingleastsquares.h \
			pcl_bugfix/mls2.h \
			pcl_bugfix/mls2.hpp \
			include/boundaryestimationdialog.h \
			include/boundaryestimation.h \
			include/outliersremovaldialog.h \
			include/outliersremoval.h \
			include/normalfielddialog.h \
			include/pairwiseregistrationdialog.h \
			include/pairwiseregistration.h \
			pcl_bugfix/gpu_extract_clusters2.h \
			pcl_bugfix/gpu_extract_clusters2.hpp \
			diagram/diagramwindow.h \
			diagram/link.h \
			diagram/node.h \
			diagram/propertiesdialog.h \
			include/globalregistrationdialog.h \
			include/globalregistration.h \
			manual_registration/manual_registration.h \
			include/mathutilities.h \
			include/registrationdatamanager.h \
			include/pairwiseregistrationinteractor.h \
			include/virtualscandialog.h \
			pcl_bugfix/pcl_visualizer2.h
#			include/globalregistrationinteractor.h
SOURCES += src/main.cpp \
			src/mainwindow.cpp \
			src/cloud.cpp \
			src/cloudio.cpp \
			src/cloudmanager.cpp \
			src/cloudbrowser.cpp \
			src/cloudvisualizer.cpp \
			src/euclideanclusterextractiondialog.cpp \
			src/euclideanclusterextraction.cpp \
			src/voxelgriddialog.cpp \
			src/voxelgrid.cpp \
			src/movingleastsquaresdialog.cpp \
			src/movingleastsquares.cpp \
			src/boundaryestimationdialog.cpp \
			src/boundaryestimation.cpp \
			src/outliersremovaldialog.cpp \
			src/outliersremoval.cpp \
			src/normalfielddialog.cpp \
			src/pairwiseregistrationdialog.cpp \
			src/pairwiseregistration.cpp \
			diagram/diagramwindow.cpp \
			diagram/link.cpp \
			diagram/node.cpp \
			diagram/propertiesdialog.cpp \
			src/globalregistrationdialog.cpp \
			src/globalregistration.cpp \
			manual_registration/manual_registration.cpp \
			src/registrationdatamanager.cpp \
			src/pairwiseregistrationinteractor.cpp \
			src/virtualscandialog.cpp \
			pcl_bugfix/pcl_visualizer2.cpp
#			src/globalregistrationinteractor.cpp \
RESOURCES += res/Registar.qrc \ 
				diagram/resources.qrc
FORMS += ui/MainWindow.ui \
			ui/EuclideanClusterExtractionDialog.ui \
			ui/VoxelGridDialog.ui \
			ui/MovingLeastSquaresDialog.ui \
			ui/BoundaryEstimationDialog.ui \
			ui/OutliersRemovalDialog.ui \
			ui/NormalFieldDialog.ui \
			ui/PairwiseRegistrationDialog.ui \
			diagram/propertiesdialog.ui \
			ui/GlobalRegistrationDialog.ui \
			manual_registration/manual_registration.ui \
			ui/VirtualScanDialog.ui


