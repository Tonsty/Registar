#include <QtCore/QDebug>
#include <vtkRenderWindow.h>
#include <vtkDoubleArray.h>
#include <vtkPolygon.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

#include "../include/cloud.h"
#include "../include/cloudvisualizer.h"

using namespace registar;

CloudVisualizer::CloudVisualizer(QWidget *parent) : QVTKWidget(parent)
{
	createPCLVisualizer();
	connectPCLVisualizerandQVTK();

	//addOrientationMarker();
	//addAxis();

	setColorMode(CloudVisualizer::colorOriginal);
	setDrawNormal(false);
	setRegistrationMode(false);
	setDrawBoundary(false);
}

CloudVisualizer::~CloudVisualizer(){}

void CloudVisualizer::createPCLVisualizer()
{
	visualizer.reset(new Visualizer("", false));
	visualizer->setBackgroundColor(1.0, 1.0, 1.0);
	visualizer->setShowFPS(false);
}

void CloudVisualizer::connectPCLVisualizerandQVTK()
{
	SetRenderWindow(visualizer->getRenderWindow());
	visualizer->setupInteractor(GetInteractor(), GetRenderWindow());
	visualizer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
}

vtkSmartPointer<vtkPolyData> CloudVisualizer::generateVtkPolyData(CloudDataConstPtr cloudData, const Polygons& polygons)
{
	vtkSmartPointer<vtkPolyData> vtk_polydata = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
	for (int i = 0; i < cloudData->size(); i++)
	{
		PointType point = (*cloudData)[i];
		vtk_points->InsertNextPoint(point.x, point.y, point.z);
	}
	vtk_polydata->SetPoints(vtk_points);
	vtkSmartPointer<vtkUnsignedCharArray> vtk_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	vtk_colors->SetNumberOfComponents(3);
	vtk_colors->SetName("Colors");
	for (int i = 0; i < cloudData->size(); i++)
	{
		PointType point = (*cloudData)[i];
		unsigned char color[3];
		color[0] = point.r;
		color[1] = point.g;
		color[2] = point.b;
		vtk_colors->InsertNextTupleValue(color);
	}
	vtk_polydata->GetPointData()->SetScalars(vtk_colors);
	vtkSmartPointer<vtkDoubleArray> vtk_normals = vtkSmartPointer<vtkDoubleArray>::New();
	vtk_normals->SetNumberOfComponents(3);
	vtk_normals->SetNumberOfTuples(vtk_polydata->GetNumberOfPoints());
	for (int i = 0; i < cloudData->size(); i++)
	{
		PointType point = (*cloudData)[i];
		double normal[3];
		normal[0] = point.normal_x;
		normal[1] = point.normal_y;
		normal[2] = point.normal_z;
		vtk_normals->SetTuple(i, normal);
	}
	//vtk_polydata->GetPointData()->SetNormals(vtk_normals);
	vtkSmartPointer<vtkCellArray> vtk_polygons = vtkSmartPointer<vtkCellArray>::New();
	for (int i = 0; i < polygons.size(); i++)
	{
		Polygon polygon = polygons[i];
		vtkSmartPointer<vtkPolygon> vtk_polygon = vtkSmartPointer<vtkPolygon>::New();
		vtk_polygon->GetPointIds()->SetNumberOfIds(polygon.vertices.size());
		for (int j = 0; j < polygon.vertices.size(); j++)
		{
			vtk_polygon->GetPointIds()->SetId(j, polygon.vertices[j]);
		}
		vtk_polygons->InsertNextCell(vtk_polygon);
	}
	vtk_polydata->SetPolys(vtk_polygons);

	return vtk_polydata;
}

bool CloudVisualizer::addCloud(const Cloud* cloud)
{
	//CloudDataPtr cloudData = cloud->getCloudData();
	CloudDataPtr cloudData(new CloudData);
	if (registrationMode) pcl::transformPointCloudWithNormals(*cloud->getCloudData(), *cloudData, cloud->getRegistrationTransformation());
	else pcl::transformPointCloudWithNormals(*cloud->getCloudData(), *cloudData, cloud->getTransformation());
	QString cloudName = cloud->getCloudName();
	Polygons polygons = cloud->getPolygons();
	//visualizer->addPolygonMesh<PointType>(cloudData, cloud->getPolygons(), cloudName.toStdString());
	//visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cloudName.toStdString());
	//visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_FLAT, cloudName.toStdString());
	if (polygons.size() == 0) addCloud(cloudData, cloudName);
	else addShape(cloudData, polygons, cloudName + "_shape");
	if(drawNormal) addCloudNormals(cloudData, cloudName + "_normals"); 
	if (drawBoundary && cloud->getBoundaries() != NULL) addCloudBoundaries(cloudData, cloud->getBoundaries(), cloudName + "_boundaries");
	return true;
}

bool CloudVisualizer::addCloud(CloudDataConstPtr cloudData, const QString &cloudName)
{
	bool flag = true;
	//visualizer->addPointCloud(cloudData, cloudName.toStdString());
	switch(colorMode)
	{
		case colorNone:
		{
			flag = visualizer->addPointCloud<PointType>(cloudData, cloudName.toStdString());
			break;
		}
		case colorOriginal:
		{
			pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb_handler(cloudData);
			flag = visualizer->addPointCloud<PointType>(cloudData, rgb_handler, cloudName.toStdString());
			break;
		}
		case colorCustom:
		{
			// pcl::visualization::PointCloudColorHandlerCustom<PointType> red (cloudData, 255, 0, 0);
			// visualizer->addPointCloud<PointType>(cloudData, red, cloudName.toStdString());
			pcl::visualization::PointCloudColorHandlerRandom<PointType> randomRGB(cloudData);
			flag = visualizer->addPointCloud<PointType>(cloudData, randomRGB, cloudName.toStdString());	
			break;
		}
	};
	update();
	return flag;
}

bool CloudVisualizer::addCloud(CloudDataConstPtr cloudData, const QString &cloudName, const float &r, const float &g, const float &b)
{
	bool flag = true;
	//visualizer->addPointCloud(cloudData, cloudName.toStdString());
	// pcl::visualization::PointCloudColorHandlerCustom<PointType> red (cloudData, 255, 0, 0);
	// visualizer->addPointCloud<PointType>(cloudData, red, cloudName.toStdString());
	pcl::visualization::PointCloudColorHandlerCustom<PointType> rgb_handler (cloudData, r, g, b);
	flag = visualizer->addPointCloud<PointType>(cloudData, rgb_handler, cloudName.toStdString());	
	update();
	return flag;
}

bool CloudVisualizer::addShape(CloudDataConstPtr cloudData, const Polygons& polygons, const QString &shapeName)
{
	bool flag = true;
	vtkSmartPointer<vtkPolyData> vtk_polydata = generateVtkPolyData(cloudData, polygons);
	flag = visualizer->addModelFromPolyData(vtk_polydata, shapeName.toStdString());
	visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, shapeName.toStdString());
	//visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, shapeName.toStdString());
	visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, shapeName.toStdString());
	update();
	return flag;
}

bool CloudVisualizer::addCloudNormals(CloudDataConstPtr cloudData, const QString &cloudNormalsName)
{
	bool flag;
	flag = visualizer->addPointCloudNormals<PointType>(cloudData, 1, 0.02, cloudNormalsName.toStdString());
	visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, cloudNormalsName.toStdString());
	visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, cloudNormalsName.toStdString());
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
	// qDebug() << "haha";
	// pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloudCurvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	// qDebug() << "hahaha";
	// pcl::copyPointCloud(*cloudData, *cloudXYZ);
	// pcl::copyPointCloud(*cloudData, *cloudNormals);
	// pcl::copyPointCloud(*cloudData, *cloudCurvatures);
	// visualizer->addPointCloudPrincipalCurvatures(cloudXYZ, cloudNormals, cloudCurvatures, 1, 0.02, (cloudName + "_curvatures").toStdString());
	update();
	return flag;
}

bool CloudVisualizer::addCloudBoundaries(CloudDataConstPtr cloudData, BoundariesConstPtr boundaries, const QString &cloudBoundriesName)
{
	bool flag = true;
	CloudDataPtr cloudBoundaries(new CloudData);
	for (int i = 0; i < boundaries->size(); ++i)
	{
		if ((*boundaries)[i].boundary_point != 0)
		{
			PointType point = (*cloudData)[i];
			point.r = 255;
			point.g = 255;
			point.b = 0;
			cloudBoundaries->push_back(point);
		}
	}
	flag = addCloud(cloudBoundaries, cloudBoundriesName);
	update();
	return flag;
}

bool CloudVisualizer::removeCloud(const Cloud* cloud)
{
	QString cloudName = cloud->getCloudName();
	if (cloud->getPolygons().size() == 0) removeCloud(cloudName);
	else removeShape(cloudName + "_shape");
	if (drawNormal) removeCloud(cloudName + "_normals");
	if (drawBoundary) removeCloud(cloudName + "_boundaries");
	return true;
}

bool CloudVisualizer::removeCloud(const QString &cloudName)
{
	bool flag = visualizer->removePointCloud(cloudName.toStdString());
	update();
	return flag;
}

bool CloudVisualizer::removeShape(const QString &shapeName)
{
	bool flag = visualizer->removeShape(shapeName.toStdString());
	update();
	return flag;
}

bool CloudVisualizer::updateCloud(const Cloud* cloud)
{
	// CloudDataPtr cloudData = cloud->getCloudData();
	CloudDataPtr cloudData(new CloudData);
	if (registrationMode) pcl::transformPointCloudWithNormals(*cloud->getCloudData(), *cloudData, cloud->getRegistrationTransformation());
	else pcl::transformPointCloudWithNormals(*cloud->getCloudData(), *cloudData, cloud->getTransformation());
	QString cloudName = cloud->getCloudName();
	Polygons polygons = cloud->getPolygons();
	//visualizer->updatePolygonMesh<PointType>(cloudData, cloud->getPolygons(), cloudName.toStdString());
	//visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cloudName.toStdString());
	//visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_FLAT, cloudName.toStdString());
	if (polygons.size() == 0)
	{
		if(removeShape(cloudName + "_shape")) addCloud(cloudData, cloudName);
		else updateCloud(cloudData, cloudName);
	}
	else 
	{
		if(removeCloud(cloudName)) addShape(cloudData, polygons, cloudName + "_shape");
		else updateShape(cloudData, polygons, cloudName + "_shape");
	}
	
	if(drawNormal) updateCloudNormals(cloudData, cloudName + "_normals");
	else removeCloud(cloudName + "_normals");

	if (drawBoundary && cloud->getBoundaries() != NULL) updateCloudBoundaries(cloudData, cloud->getBoundaries(), cloudName + "_boundaries");
	else removeCloud(cloudName + "_boundaries");

	return true;
}

bool CloudVisualizer::updateCloud(CloudDataConstPtr cloudData, const QString &cloudName)
{
	bool flag = true;
	//visualizer->updatePointCloud(cloudData, cloudName.toStdString());
	switch(colorMode)
	{
		case colorNone:
		{
			flag = visualizer->updatePointCloud<PointType>(cloudData, cloudName.toStdString());
			break;
		}
		case colorOriginal:
		{
			pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb_handler(cloudData);
			flag = visualizer->updatePointCloud<PointType>(cloudData, rgb_handler, cloudName.toStdString());
			break;
		}
		case colorCustom:
		{
			// pcl::visualization::PointCloudColorHandlerCustom<PointType> blue(cloudData, 0, 0, 255);
			// visualizer->updatePointCloud<PointType>(cloudData, blue, cloudName.toStdString());
			pcl::visualization::PointCloudColorHandlerRandom<PointType> randomRGB(cloudData);
			flag = visualizer->updatePointCloud<PointType>(cloudData, randomRGB, cloudName.toStdString());		
			break;
		}
	};
	update();
	return flag;
}

bool CloudVisualizer::updateCloud(CloudDataConstPtr cloudData, const QString &cloudName, const float &r, const float &g, const float &b)
{
	bool flag = true;
	//visualizer->updatePointCloud(cloudData, cloudName.toStdString());
	// pcl::visualization::PointCloudColorHandlerCustom<PointType> blue(cloudData, 0, 0, 255);
	// visualizer->updatePointCloud<PointType>(cloudData, blue, cloudName.toStdString());
	pcl::visualization::PointCloudColorHandlerCustom<PointType> rgb_handler (cloudData, r, g, b);
	flag = visualizer->updatePointCloud<PointType>(cloudData, rgb_handler, cloudName.toStdString());	
	update();
	return flag;
}

bool CloudVisualizer::updateShape(CloudDataConstPtr cloudData, const Polygons &polygons, const QString &cloudName)
{
	bool flag = true;
	vtkSmartPointer<vtkPolyData> vtk_polydata = generateVtkPolyData(cloudData, polygons);
	visualizer->removeShape(cloudName.toStdString());
	flag = visualizer->addModelFromPolyData(vtk_polydata, cloudName.toStdString());
	visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cloudName.toStdString());
	//visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, cloudName.toStdString());
	visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, cloudName.toStdString());
	update();
	return flag;
}

bool CloudVisualizer::updateCloudNormals(CloudDataConstPtr cloudData, const QString &cloudNormalsName)
{
	bool flag = true;
	visualizer->removePointCloud(cloudNormalsName.toStdString());
	flag = visualizer->addPointCloudNormals<PointType>(cloudData, 1, 0.02, cloudNormalsName.toStdString());
	visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, cloudNormalsName.toStdString());
	visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, cloudNormalsName.toStdString());
	// visualizer->removePointCloud((cloudName + "_curvatures").toStdString());
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
	// pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloudCurvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	// pcl::copyPointCloud(*cloudData, *cloudXYZ);
	// pcl::copyPointCloud(*cloudData, *cloudNormals);
	// pcl::copyPointCloud(*cloudData, *cloudCurvatures);
	// visualizer->addPointCloudPrincipalCurvatures(cloudXYZ, cloudNormals, cloudCurvatures, 1, 0.02, (cloudName + "_curvatures").toStdString());
	update();
	return flag;
}

bool CloudVisualizer::updateCloudBoundaries(CloudDataConstPtr cloudData, BoundariesConstPtr boundaries, const QString &cloudBoundriesName)
{
	bool flag = true;
	CloudDataPtr cloudBoundaries(new CloudData);
	for (int i = 0; i < boundaries->size(); ++i)
	{
		if ((*boundaries)[i].boundary_point != 0)
		{
			PointType point = (*cloudData)[i];
			point.r = 255;
			point.g = 255;
			point.b = 0;
			cloudBoundaries->push_back(point);
		}		
	}
	removeCloud(cloudBoundriesName);
	flag = addCloud(cloudBoundaries, cloudBoundriesName);
	return flag;
}


void CloudVisualizer::addAxis()
{
	visualizer->addCoordinateSystem(2.0, 0);
	update();
}

void CloudVisualizer::removeAxis()
{
	visualizer->removeCoordinateSystem(0);
	update();
}

void CloudVisualizer::addOrientationMarker()
{
	visualizer->addOrientationMarkerWidgetAxes(GetInteractor());
}

void CloudVisualizer::removeOrientationMarker()
{
	visualizer->removeOrientationMarkerWidgetAxes();
}

void CloudVisualizer::resetCamera(const Cloud* cloud)
{
	// visualizer->resetCameraViewpoint(cloud->getCloudName().toStdString());
	// Eigen::Affine3f visualizer->getViewerPose();

	// CloudDataPtr cloudData = cloud->getCloudData();
	CloudDataPtr cloudData(new CloudData);
	pcl::transformPointCloudWithNormals(*cloud->getCloudData(), *cloudData, cloud->getTransformation());
	resetCamera(cloudData);
}

void CloudVisualizer::resetCamera(CloudDataConstPtr cloudData)
{
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloudData, centroid);
	visualizer->setCameraPosition(0, 0, 0, centroid(0), centroid(1), centroid(2), 0, 1, 0);
	update();
}


void CloudVisualizer::renderView(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	pcl::visualization::Camera camera;
	visualizer->getCameraParameters(camera);
	int xres, yres;
	xres = camera.window_size[0];
	yres = camera.window_size[1];
	std::cout << xres << " , " << yres << std::endl;

	visualizer->renderView(xres, yres, cloud);

	std::cout << *cloud << std::endl;
}