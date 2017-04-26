win32{
	BOOST_INCLUDE_DIR = "D:/boost_1_55_0/"
	BOOST_LIB_DIR = "D:/boost_1_55_0/stage/lib/"
	BOOST_LIBRARIES_DEBUG = boost_system-vc100-mt-gd-1_55.lib
	BOOST_LIBRARIES_RELEASE = boost_system-vc100-mt-1_55.lib

	EIGEN3_INCLUDE_DIR = "C:/Program Files/Eigen3/include/eigen3/"

	VTK_INCLUDE_DIR = "C:/Program Files/VTK/include/vtk-5.8/"
	VTK_LIB_DIR = "C:/Program Files/VTK/lib/vtk-5.8/"
	VTK_LIBRARIES_DEBUG = QVTK_debug.lib vtkCommon_debug.lib vtkRendering_debug.lib \
					vtkFiltering_debug.lib vtkGraphics_debug.lib
	VTK_LIBRARIES_RELEASE = QVTK_release.lib vtkCommon_release.lib vtkRendering_release.lib \
					 vtkFiltering_release.lib vtkGraphics_release.lib	

	FLANN_INCLUDE_DIR = "C:/Program Files/flann/include/"
	FLANN_LIB_DIR = "C:/Program Files/flann/lib/"
	FLANN_LIBRARIES_DEBUG = flann_cpp_s_debug.lib	
	FLANN_LIBRARIES_RELEASE = flann_cpp_s_release.lib			

	PCL_INCLUDE_DIR = "C:/Program Files/PCL/include/pcl-1.8/"
	PCL_LIB_DIR = "C:/Program Files/PCL/lib/"
	PCL_LIBRARIES_DEBUG = pcl_visualization_debug.lib pcl_io_debug.lib pcl_common_debug.lib \
					pcl_kdtree_debug.lib pcl_search_debug.lib pcl_gpu_containers_debug.lib \
					pcl_gpu_octree_debug.lib pcl_gpu_utils_debug.lib
	PCL_LIBRARIES_RELEASE = pcl_visualization_release.lib pcl_io_release.lib pcl_common_release.lib \
					pcl_kdtree_release.lib pcl_search_release.lib pcl_gpu_containers_release.lib \
					pcl_gpu_octree_release.lib pcl_gpu_utils_release.lib

	CONFIG += console
	INCLUDEPATH += . $${VTK_INCLUDE_DIR} $${PCL_INCLUDE_DIR} $${EIGEN3_INCLUDE_DIR} $${FLANN_INCLUDE_DIR} $${BOOST_INCLUDE_DIR}
	CONFIG(debug, debug|release){
		LIBS += -L$${VTK_LIB_DIR} $${VTK_LIBRARIES_DEBUG} \
		-L$${BOOST_LIB_DIR} $${BOOST_LIBRARIES_DEBUG} \
		-L$${PCL_LIB_DIR} $${PCL_LIBRARIES_DEBUG} \
		-L$${FLANN_LIB_DIR} $${FLANN_LIBRARIES_DEBUG}		
	}else{
		LIBS += -L$${VTK_LIB_DIR} $${VTK_LIBRARIES_RELEASE} \
		-L$${BOOST_LIB_DIR} $${BOOST_LIBRARIES_RELEASE} \
		-L$${PCL_LIB_DIR} $${PCL_LIBRARIES_RELEASE} \
		-L$${FLANN_LIB_DIR} $${FLANN_LIBRARIES_RELEASE}			
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
