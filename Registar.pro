TEMPLATE = app
CONFIG += warn_off debug_and_release
CONFIG(debug, debug|release){
	TARGET = Registar_debug
	MOC_DIR += tmp/moc/debug/
	OBJECTS_DIR += tmp/obj/debug/
}else{
	TARGET = Registar_release
	MOC_DIR += tmp/moc/release/
	OBJECTS_DIR += tmp/obj/release/
}
RCC_DIR += tmp/rcc/
UI_DIR += ui/
DEPENDPATH += .
INCLUDEPATH += . /usr/include/vtk-5.8/ /usr/local/include/pcl-1.7/ /usr/include/eigen3/
LIBS += -L/usr/lib/ \
					-lQVTK \
					-lboost_system \
					-lvtkCommon -lQVTK -lvtkRendering -lvtkFiltering \
		-L/usr/lib/gcc/x85_64-linux-gnu/ \
					-lgomp \
		-L/usr/local/lib/ \
					-lpcl_visualization -lpcl_io -lpcl_common -lpcl_kdtree -lpcl_search \
					-lpcl_filters -lpcl_segmentation -lpcl_features -lpcl_gpu_containers -lpcl_gpu_octree \
					-lpcl_gpu_utils -lpcl_sample_consensus

QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS += -fopenmp


# Input
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
			ui/GlobalRegistrationDialog.ui
#output


