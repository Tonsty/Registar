#include <QtCore/QDebug>
#include <vtkRenderWindow.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

#include "../include/cloud.h"
#include "../include/cloudvisualizer.h"

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
}

void CloudVisualizer::connectPCLVisualizerandQVTK()
{
	SetRenderWindow(visualizer->getRenderWindow());
	visualizer->setupInteractor(GetInteractor(), GetRenderWindow());
	visualizer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
}

void CloudVisualizer::addCloud(Cloud* cloud)
{
	//CloudDataPtr cloudData = cloud->getCloudData();
	CloudDataPtr cloudData(new CloudData);
	if (registrationMode) pcl::transformPointCloudWithNormals(*cloud->getCloudData(), *cloudData, cloud->getRegistrationTransformation());
	else pcl::transformPointCloudWithNormals(*cloud->getCloudData(), *cloudData, cloud->getTransformation());
	QString cloudName = cloud->getCloudName();
	addCloud(cloudData, cloudName);

	if (drawBoundary && cloud->getBoundaries() != NULL)
	{
		CloudDataPtr cloudBoundaries(new CloudData);
		BoundariesPtr boundaries = cloud->getBoundaries();
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
		addCloud(cloudBoundaries, cloudName + "_boundaries");
	}
}

void CloudVisualizer::addCloud(CloudDataPtr cloudData, QString cloudName)
{
	//visualizer->addPointCloud(cloudData, cloudName.toStdString());
	switch(colorMode)
	{
		case colorNone:
		{
			visualizer->addPointCloud<PointType>(cloudData, cloudName.toStdString());
			//visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, cloudName.toStdString());
			break;
		}
		case colorOriginal:
		{
			pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb_handler(cloudData);
			visualizer->addPointCloud<PointType>(cloudData, rgb_handler, cloudName.toStdString());
			//visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, cloudName.toStdString());
			break;
		}
		case colorCustom:
		{
			// pcl::visualization::PointCloudColorHandlerCustom<PointType> red (cloudData, 255, 0, 0);
			// visualizer->addPointCloud<PointType>(cloudData, red, cloudName.toStdString());
			pcl::visualization::PointCloudColorHandlerRandom<PointType> randomRGB(cloudData);
			visualizer->addPointCloud<PointType>(cloudData, randomRGB, cloudName.toStdString());
			//visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, cloudName.toStdString());	
			break;
		}
	};

	if(drawNormal)
	{
		visualizer->addPointCloudNormals<PointType>(cloudData, 1, 0.02, (cloudName + "_normals").toStdString());
		visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, (cloudName + "_normals").toStdString());
		visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, (cloudName + "_normals").toStdString());
		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
		// qDebug() << "haha";
		// pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloudCurvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
		// qDebug() << "hahaha";
		// pcl::copyPointCloud(*cloudData, *cloudXYZ);
		// pcl::copyPointCloud(*cloudData, *cloudNormals);
		// pcl::copyPointCloud(*cloudData, *cloudCurvatures);
		// visualizer->addPointCloudPrincipalCurvatures(cloudXYZ, cloudNormals, cloudCurvatures, 1, 0.02, (cloudName + "_curvatures").toStdString());
	}

	update();
}

void CloudVisualizer::addCloud(CloudDataPtr cloudData, QString cloudName, float r, float g, float b)
{
	//visualizer->addPointCloud(cloudData, cloudName.toStdString());
	
	// pcl::visualization::PointCloudColorHandlerCustom<PointType> red (cloudData, 255, 0, 0);
	// visualizer->addPointCloud<PointType>(cloudData, red, cloudName.toStdString());
	pcl::visualization::PointCloudColorHandlerCustom<PointType> rgb_handler (cloudData, r, g, b);
	visualizer->addPointCloud<PointType>(cloudData, rgb_handler, cloudName.toStdString());
	//visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, cloudName.toStdString());	

	if(drawNormal)
	{
		visualizer->addPointCloudNormals<PointType>(cloudData, 1, 0.02, (cloudName + "_normals").toStdString());
		visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, (cloudName + "_normals").toStdString());
		visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, (cloudName + "_normals").toStdString());
		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
		// qDebug() << "haha";
		// pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloudCurvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
		// qDebug() << "hahaha";
		// pcl::copyPointCloud(*cloudData, *cloudXYZ);
		// pcl::copyPointCloud(*cloudData, *cloudNormals);
		// pcl::copyPointCloud(*cloudData, *cloudCurvatures);
		// visualizer->addPointCloudPrincipalCurvatures(cloudXYZ, cloudNormals, cloudCurvatures, 1, 0.02, (cloudName + "_curvatures").toStdString());
	}

	update();
}

void CloudVisualizer::removeCloud(Cloud* cloud)
{
	QString cloudName = cloud->getCloudName();
	removeCloud(cloudName);
	if(drawBoundary) removeCloud(cloudName + "_boundaries");
}

void CloudVisualizer::removeCloud(QString cloudName)
{
	visualizer->removePointCloud(cloudName.toStdString());
	visualizer->removePointCloud((cloudName + "_normals").toStdString());
	update();
}

void CloudVisualizer::updateCloud(Cloud* cloud)
{
	// CloudDataPtr cloudData = cloud->getCloudData();
	CloudDataPtr cloudData(new CloudData);
	if (registrationMode) pcl::transformPointCloudWithNormals(*cloud->getCloudData(), *cloudData, cloud->getRegistrationTransformation());
	else pcl::transformPointCloudWithNormals(*cloud->getCloudData(), *cloudData, cloud->getTransformation());
	QString cloudName = cloud->getCloudName();
	updateCloud(cloudData, cloudName);

	if (drawBoundary && cloud->getBoundaries() != NULL)
	{
		CloudDataPtr cloudBoundaries(new CloudData);
		BoundariesPtr boundaries = cloud->getBoundaries();
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
		removeCloud(cloudName + "_boundaries");
		addCloud(cloudBoundaries, cloudName + "_boundaries");
	}
	else
	{
		removeCloud(cloudName + "_boundaries");
	}
}

void CloudVisualizer::updateCloud(CloudDataPtr cloudData, QString cloudName)
{
	//visualizer->updatePointCloud(cloudData, cloudName.toStdString());
	switch(colorMode)
	{
		case colorNone:
		{
			visualizer->updatePointCloud<PointType>(cloudData, cloudName.toStdString());
			//visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, cloudName.toStdString());
			break;
		}
		case colorOriginal:
		{
			pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb_handler(cloudData);
			visualizer->updatePointCloud<PointType>(cloudData, rgb_handler, cloudName.toStdString());
			//visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, cloudName.toStdString());
			break;
		}
		case colorCustom:
		{
			// pcl::visualization::PointCloudColorHandlerCustom<PointType> blue(cloudData, 0, 0, 255);
			// visualizer->updatePointCloud<PointType>(cloudData, blue, cloudName.toStdString());
			pcl::visualization::PointCloudColorHandlerRandom<PointType> randomRGB(cloudData);
			visualizer->updatePointCloud<PointType>(cloudData, randomRGB, cloudName.toStdString());
			//visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, cloudName.toStdString());		
			break;
		}
	};

	if(drawNormal)
	{
		visualizer->removePointCloud((cloudName + "_normals").toStdString());
		visualizer->addPointCloudNormals<PointType>(cloudData, 1, 0.02, (cloudName + "_normals").toStdString());
		visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, (cloudName + "_normals").toStdString());
		visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, (cloudName + "_normals").toStdString());
		// visualizer->removePointCloud((cloudName + "_curvatures").toStdString());
		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
		// pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloudCurvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
		// pcl::copyPointCloud(*cloudData, *cloudXYZ);
		// pcl::copyPointCloud(*cloudData, *cloudNormals);
		// pcl::copyPointCloud(*cloudData, *cloudCurvatures);
		// visualizer->addPointCloudPrincipalCurvatures(cloudXYZ, cloudNormals, cloudCurvatures, 1, 0.02, (cloudName + "_curvatures").toStdString());
	}
	else
	{
		visualizer->removePointCloud((cloudName + "_normals").toStdString());
		//visualizer->removePointCloud((cloudName + "_curvatures").toStdString());
	}

	update();
}

void CloudVisualizer::updateCloud(CloudDataPtr cloudData, QString cloudName, float r, float g, float b)
{
	//visualizer->updatePointCloud(cloudData, cloudName.toStdString());

	// pcl::visualization::PointCloudColorHandlerCustom<PointType> blue(cloudData, 0, 0, 255);
	// visualizer->updatePointCloud<PointType>(cloudData, blue, cloudName.toStdString());
	pcl::visualization::PointCloudColorHandlerCustom<PointType> rgb_handler (cloudData, r, g, b);
	visualizer->updatePointCloud<PointType>(cloudData, rgb_handler, cloudName.toStdString());
	//visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, cloudName.toStdString());		

	if(drawNormal)
	{
		visualizer->removePointCloud((cloudName + "_normals").toStdString());
		visualizer->addPointCloudNormals<PointType>(cloudData, 1, 0.02, (cloudName + "_normals").toStdString());
		visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, (cloudName + "_normals").toStdString());
		visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, (cloudName + "_normals").toStdString());
		// visualizer->removePointCloud((cloudName + "_curvatures").toStdString());
		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
		// pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloudCurvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
		// pcl::copyPointCloud(*cloudData, *cloudXYZ);
		// pcl::copyPointCloud(*cloudData, *cloudNormals);
		// pcl::copyPointCloud(*cloudData, *cloudCurvatures);
		// visualizer->addPointCloudPrincipalCurvatures(cloudXYZ, cloudNormals, cloudCurvatures, 1, 0.02, (cloudName + "_curvatures").toStdString());
	}
	else
	{
		visualizer->removePointCloud((cloudName + "_normals").toStdString());
		//visualizer->removePointCloud((cloudName + "_curvatures").toStdString());
	}

	update();
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

void CloudVisualizer::resetCamera(Cloud* cloud)
{
	// visualizer->resetCameraViewpoint(cloud->getCloudName().toStdString());
	// Eigen::Affine3f visualizer->getViewerPose();

	// CloudDataPtr cloudData = cloud->getCloudData();
	CloudDataPtr cloudData(new CloudData);
	pcl::transformPointCloudWithNormals(*cloud->getCloudData(), *cloudData, cloud->getTransformation());
	resetCamera(cloudData);
}

void CloudVisualizer::resetCamera(CloudDataPtr cloudData)
{
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloudData, centroid);
	visualizer->setCameraPosition(0, 0, 0, centroid(0), centroid(1), centroid(2), 0, 1, 0);
	update();
}


