TEMPLATE = app
TARGET= Registar
CONFIG += debug_and_release debug_and_release_target warn_off 
DEPENDPATH += .
UI_DIR = ui/
include(Registar.pri)
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
			include/normalfield.h \
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
			include/utilities.h \
			include/registrationdatamanager.h \
			include/pairwiseregistrationinteractor.h \
			include/virtualscandialog.h \
			pcl_bugfix/pcl_visualizer2.h \
			include/depthcameradialog.h \
			include/virtualscan.h \
			set_color/set_color.h \
			include/addnoisedialog.h \
			include/transformationdialog.h \
			include/savecontentdialog.h \
			include/hausdorffdistancedialog.h \
			include/hausdorffdistance.h \
			include/colorfielddialog.h \
			include/generateoutliersdialog.h \
			include/tang2014dialog.h \
			include/tang2014.h \
			include/args_converter.h \
			include/backgroundcolordialog.h
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
			src/normalfield.cpp \
			src/pairwiseregistrationdialog.cpp \
			src/pairwiseregistration.cpp \
			diagram/diagramwindow.cpp \
			diagram/link.cpp \
			diagram/node.cpp \
			diagram/propertiesdialog.cpp \
			src/globalregistrationdialog.cpp \
			src/globalregistration.cpp \
			manual_registration/manual_registration.cpp \
			src/utilities.cpp \
			src/registrationdatamanager.cpp \
			src/pairwiseregistrationinteractor.cpp \
			src/virtualscandialog.cpp \
			pcl_bugfix/pcl_visualizer2.cpp \
			src/depthcameradialog.cpp \
			src/virtualscan.cpp \
			set_color/set_color.cpp \
			src/addnoisedialog.cpp \
			src/transformationdialog.cpp \
			src/savecontentdialog.cpp \
			src/hausdorffdistancedialog.cpp \
			src/hausdorffdistance.cpp \
			src/colorfielddialog.cpp \
			src/generateoutliersdialog.cpp \
			src/tang2014dialog.cpp \
			src/tang2014.cpp \
			src/args_converter.cpp \
			../Tang2014/pairregistration.cpp \
			../Tang2014/tang2014_globalregistration.cpp \
			../Tang2014/graph.cpp \
			../Williams2001/SRoMCPS.cpp \
			src/backgroundcolordialog.cpp
#			src/globalregistrationinteractor.cpp
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
			ui/VirtualScanDialog.ui \
			ui/DepthCameraDialog.ui \
			ui/AddNoiseDialog.ui \
			ui/TransformationDialog.ui \
			ui/SaveContentDialog.ui \
			ui/HausdorffDistanceDialog.ui \
			ui/ColorFieldDialog.ui  \
			ui/GenerateOutliersDialog.ui \
			ui/Tang2014Dialog.ui \
			ui/BackgroundColorDialog.ui


