#ifndef CLOUDVISUALIZER_H
#define CLOUDVISUALIZER_H

#include <QVTKWidget.h>

#ifndef Q_MOC_RUN
#include <pcl/visualization/pcl_visualizer.h>
#include "../pcl_bugfix/pcl_visualizer2.h"
#include "pclbase.h"
#endif

namespace registar
{
	class Cloud;

	class CloudVisualizer : public QVTKWidget
	{
		Q_OBJECT

	public:
		typedef pcl::visualization::PCLVisualizer2 Visualizer;
		typedef Visualizer::Ptr VisualizerPtr;
		typedef Visualizer::ConstPtr VisualizerConstPtr;

		enum ColorMode
		{
			colorNone, colorOriginal, colorCustom
		};

		CloudVisualizer(QWidget *parent = 0);
		virtual ~CloudVisualizer();

		bool addCloud(CloudDataConstPtr cloudData, const QString &cloudName);
		bool addCloud(CloudDataConstPtr cloudData, const QString &cloudName, const float &r, const float &g, const float &b);
		bool addShape(CloudDataConstPtr cloudData, const Polygons &polygons, const QString &shapeName);
		bool addCloudNormals(CloudDataConstPtr cloudData, const QString &cloudNormalsName);
		bool addCloudBoundaries(CloudDataConstPtr cloudData, BoundariesConstPtr boundaries, const QString &cloudBoundriesName);
		bool addCloud(const Cloud* cloud);

		bool removeCloud(const QString &cloudName);
		bool removeShape(const QString &shapeName);
		bool removeCloud(const Cloud* cloud);

		bool updateCloud(CloudDataConstPtr cloudData, const QString &cloudName);
		bool updateCloud(CloudDataConstPtr cloudData, const QString &cloudName, const float &r, const float &g, const float &b);
		bool updateShape(CloudDataConstPtr cloudData, const Polygons &polygons, const QString &shapeName);
		bool updateCloudNormals(CloudDataConstPtr cloudData, const QString &cloudNormalsName);
		bool updateCloudBoundaries(CloudDataConstPtr cloudData, BoundariesConstPtr boundaries, const QString &cloudBoundriesName);
		bool updateCloud(const Cloud* cloud);

		void resetCamera(CloudDataConstPtr cloudData);
		void resetCamera(const Cloud* cloud);

		void addAxis();
		void removeAxis();
		void addOrientationMarker();
		void removeOrientationMarker();

		void renderView(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

		inline void setColorMode(ColorMode colorMode)
		{
			this->colorMode = colorMode;
		}

		inline ColorMode getColorMode()
		{
			return colorMode;
		}

		inline void setDrawNormal(bool drawNormal)
		{
			this->drawNormal = drawNormal;
		}

		inline bool getDrawNormal()
		{
			return drawNormal;
		}

		inline void setRegistrationMode(bool registrationMode)
		{
			this->registrationMode = registrationMode;
		}

		inline bool getRegistrationMode()
		{
			return registrationMode;
		}

		inline void setDrawBoundary(bool drawBoundary)
		{
			this->drawBoundary = drawBoundary;
		}

		inline bool getDrawBoundary()
		{
			return drawBoundary;
		}


	protected:
		VisualizerPtr visualizer;

	private:
		void createPCLVisualizer();
		void connectPCLVisualizerandQVTK();

		vtkSmartPointer<vtkPolyData> generateVtkPolyData(CloudDataConstPtr cloudData, const Polygons& polygons);

		ColorMode colorMode;
		bool drawNormal;

		bool registrationMode;
		bool drawBoundary;
	};
}

#endif


