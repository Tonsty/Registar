#ifndef CLOUDVISUALIZER_H
#define CLOUDVISUALIZER_H

#include <QVTKWidget.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pclbase.h"

class Cloud;

class CloudVisualizer : public QVTKWidget
{
	Q_OBJECT

public:
	typedef pcl::visualization::PCLVisualizer Visualizer;
	typedef Visualizer::Ptr VisualizerPtr;
	typedef Visualizer::ConstPtr VisualizerConstPtr;

	enum ColorMode
	{
		colorNone, colorOriginal, colorCustom
	};

	CloudVisualizer(QWidget *parent = 0);
	virtual ~CloudVisualizer();

	void addCloud(CloudDataPtr cloudData, QString cloudName);
	void addCloud(CloudDataPtr cloudData, QString cloudName, float r, float g, float b);
	void addCloud(Cloud* cloud);
	void removeCloud(QString cloudName);	
	void removeCloud(Cloud* cloud);
	void updateCloud(CloudDataPtr cloudData, QString cloudName);
	void updateCloud(CloudDataPtr cloudData, QString cloudName, float r, float g, float b);
	void updateCloud(Cloud* cloud);
	void resetCamera(CloudDataPtr cloudData);
	void resetCamera(Cloud* cloud);
	
	void addAxis();
	void removeAxis();
	void addOrientationMarker();
	void removeOrientationMarker();

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

	ColorMode colorMode;
	bool drawNormal;

	bool registrationMode;
	bool drawBoundary;
};

#endif


