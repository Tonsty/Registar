#include <QtGui/QtGui>
#include <boost/shared_ptr.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

#include "../include/qtbase.h"
#include "../include/cloudmanager.h"
#include "../include/cloudvisualizer.h"
#include "../include/cloudio.h"
#include "../include/euclideanclusterextractiondialog.h"
#include "../include/euclideanclusterextraction.h"
#include "../include/voxelgriddialog.h"
#include "../include/voxelgrid.h"
#include "../include/movingleastsquaresdialog.h"
#include "../include/movingleastsquares.h"
#include "../include/boundaryestimationdialog.h"
#include "../include/boundaryestimation.h"
#include "../include/outliersremovaldialog.h"
#include "../include/outliersremoval.h"
#include "../include/normalfielddialog.h"
#include "../include/virtualscandialog.h"
#include "../include/depthcameradialog.h"
#include "../include/virtualscan.h"
#include "../set_color/set_color.h"
#include "../include/addnoisedialog.h"
#include "../include/transformationdialog.h"
#include "../include/savecontentdialog.h"
#include "../include/hausdorffdistancedialog.h"
#include "../include/colorfielddialog.h"
#include "../include/generateoutliersdialog.h"

#include "../include/pairwiseregistrationdialog.h"
#include "../include/pairwiseregistration.h"
#include "../include/pairwiseregistrationinteractor.h"
#include "../include/registrationdatamanager.h"

#include "../diagram/diagramwindow.h"
#include "../include/globalregistrationdialog.h"
#include "../include/globalregistration.h"

#include "../manual_registration/manual_registration.h"
#include "../include/utilities.h"

#include "../include/mainwindow.h"

using namespace registar;

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent)
{
	setupUi(this);

	registerMetaType();

	cloudManager = new CloudManager(this);
	cloudVisualizer = new CloudVisualizer(this);
	cloudVisualizer->setColorMode(CloudVisualizer::colorCustom);
	pairwiseRegistrationManager = new PairwiseRegistrationManager(this);
	registrationDataManager = new RegistrationDataManager(this);

	cycleRegistrationManager = new CycleRegistrationManager(this);

	euclideanClusterExtractionDialog = 0;
	voxelGridDialog = 0;
	movingLeastSquaresDialog = 0;
	boundaryEstimationDialog = 0;
	outliersRemovalDialog = 0;
	normalFieldDialog = 0;
	colorFieldDialog = 0;
	virtualScanDialog = 0;
	depthCameraDialog = 0;
	pairwiseRegistrationDialog = 0;
	addNoiseDialog = 0;
	transformationDialog = 0;
	saveContentDialog = 0;
	hausdorffDistanceDialog = 0;
	generateOutliersDialog = 0;

	diagramWindow = 0;
	globalRegistrationDialog = 0;

	manualRegistration = 0;

	setCentralWidget(cloudVisualizer);

	cloudBrowserDockWidget->setVisible(false);
	pointBrowserDockWidget->setVisible(false);

	connect(aboutQtAction, SIGNAL(triggered()), qApp, SLOT(aboutQt()));

	QActionGroup *colorActionGroup = new QActionGroup(this);
	colorActionGroup->addAction(colorNoneAction);
	colorActionGroup->addAction(colorOriginalAction);
	colorActionGroup->addAction(colorCustomAction);
}

MainWindow::~MainWindow(){} 

void MainWindow::registerMetaType()
{
	qRegisterMetaType<CloudDataPtr>("CloudDataPtr");
	qRegisterMetaType<CloudDataConstPtr>("CloudDataConstPtr");

	qRegisterMetaType< QList<bool> >("QList<bool>");
	qRegisterMetaType<Eigen::Matrix4f>("Eigen::Matrix4f");
}

void MainWindow::changeEvent(QEvent *event)
{
	QWidget::changeEvent(event);
	switch(event->type())
	{
	case QEvent::LanguageChange: 
		retranslateUi(this);
		break;
	default:
		break;
	}
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	if (okToContinue())
	{
		writeSettings();
		event->accept();
	} 
	else 
	{
		event->ignore();
	}
}

bool MainWindow::okToContinue()
{
	int r = QMessageBox::warning(this, tr("Exit Registar"),
						tr("Do you really want to exit?"),
						QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
	switch(r)
	{
	case QMessageBox::Yes:
		return true;
	case QMessageBox::No:
	case QMessageBox::Cancel:
		return false;
	}
	return true;
}

void MainWindow::readSettings()
{

}

void MainWindow::writeSettings()
{

}

QString MainWindow::strippedName(const QString &fullFileName)
{
	return QFileInfo(fullFileName).fileName();
}

void MainWindow::on_aboutAction_triggered()
{
	QMessageBox::about(this, tr("About Registar"),
	 tr("<h2>Registar 1.1</h2>"
	 	"<p>Copyright &copy; 2014 Software Inc."
	 	"<p>Registar is a small application used for "
	 	"registering and aligning multiview point clouds."));
}

void MainWindow::on_openAction_triggered()
{
	QStringList fileNameList = QFileDialog::getOpenFileNames(this, tr("Open File"), ".", tr("PLY file (*.ply);;VTK file (*.vtk)"));

	QStringList::Iterator it = fileNameList.begin();
	while (it != fileNameList.end())
	{
		if (!(*it).isEmpty()) 
		{
			QString fileName = (*it);
			PolygonMeshPtr polygonMesh(new PolygonMesh);
			CloudDataPtr cloudData(new CloudData);
			Eigen::Matrix4f transformation;
			BoundariesPtr boundaries(new Boundaries);
			QApplication::setOverrideCursor(Qt::WaitCursor);
			if ( CloudIO::importPLYPolygonMesh(fileName, polygonMesh) )
			{
				pcl::fromPCLPointCloud2(polygonMesh->cloud, *cloudData);
			}
			else if ( !CloudIO::importPLYCloudData(fileName, cloudData) ) 
			{
				it++;
				continue;
			}
			CloudIO::importTransformation(fileName, transformation);
			CloudIO::importBoundaries(fileName, boundaries);
			QApplication::restoreOverrideCursor();
			QApplication::beep();
			Cloud* cloud = cloudManager->addCloud(cloudData, polygonMesh->polygons, Cloud::fromIO, fileName, transformation);
			cloud->setBoundaries(boundaries);
			cloudBrowser->addCloud(cloud);
			cloudVisualizer->addCloud(cloud);
			cloudVisualizer->resetCamera(cloud);
		}
		it++;
	} 
}


bool MainWindow::on_saveAction_triggered()
{
	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();

	if (cloudNameList.size() == 0) return true;
	
	if (!saveContentDialog) saveContentDialog = new SaveContentDialog();
	int flag = saveContentDialog->exec();
	if (flag == QDialog::Rejected) return true;
	
	QStringList::Iterator it = cloudNameList.begin();
	while (it != cloudNameList.end())
	{
		QString cloudName = (*it);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		CloudDataConstPtr cloudData = cloud->getCloudData();
		Eigen::Matrix4f transformation = cloud->getTransformation();
		BoundariesConstPtr boundaries = cloud->getBoundaries();
		QString fileName = cloud->getFileName();
		if (fileName.isEmpty())
		{
			QString newFileName = QFileDialog::getSaveFileName(this, tr("Save %1 as").arg(cloudName), ".", tr("PLY file (*.ply)\nVTK file (*.vtk)"));
			if (newFileName.isEmpty())
			{
				qDebug() << cloudName << "cancel saveAs!";
				it++;
				continue;
			}
			fileName = newFileName;
		}
		QApplication::setOverrideCursor(Qt::WaitCursor);
		if( saveContentDialog->savePointCheckBox->isChecked() ) 
		{
			if (fileName.mid( fileName.size()-4, 4) == ".ply")
			{
				if (cloud->getPolygons().size() > 0)
				{
					PolygonMeshPtr polygonMesh(new PolygonMesh);
					toPolygonMesh(*cloudData, cloud->getPolygons(), *polygonMesh);
					CloudIO::exportPLYPolygonMesh(fileName, polygonMesh);
				}
				else CloudIO::exportPLYCloudData(fileName, cloudData);
			}
			else if (fileName.mid( fileName.size()-4, 4) == ".vtk")
			{
				if (cloud->getPolygons().size() > 0)
				{
					PolygonMeshPtr polygonMesh(new PolygonMesh);
					toPolygonMesh(*cloudData, cloud->getPolygons(), *polygonMesh);
					CloudIO::exportVTKPolygonMesh(fileName, polygonMesh);
				}
				else CloudIO::exportVTKCloudData(fileName, cloudData);
			}
		}
		if( saveContentDialog->saveTransformationCheckBox->isChecked() ) CloudIO::exportTransformation(fileName, transformation);
		if( saveContentDialog->saveBoundaryCheckBox->isChecked() ) CloudIO::exportBoundaries(fileName, boundaries);
		QApplication::restoreOverrideCursor();
		QApplication::beep();
		qDebug() << cloudName << " saved!";
		it++;
	}

	return true;
}

bool MainWindow::on_saveAsAction_triggered()
{	
	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();

	if (cloudNameList.size() == 0) return true;

	if (!saveContentDialog) saveContentDialog = new SaveContentDialog();
	int flag = saveContentDialog->exec();
	if (flag == QDialog::Rejected) return true;

	QStringList::Iterator it = cloudNameList.begin();
	while (it != cloudNameList.end())
	{
		QString cloudName = (*it);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		CloudDataConstPtr cloudData = cloud->getCloudData();
		Eigen::Matrix4f transformation = cloud->getTransformation();
		BoundariesConstPtr boundaries = cloud->getBoundaries();
		QString fileName = cloud->getFileName();
		QString newFileName = QFileDialog::getSaveFileName(this, tr("Save %1 - %2 as").arg(cloudName).arg(strippedName(fileName)), ".", tr("PLY file (*.ply)\nVTK file (*.vtk)"));
		if (newFileName.isEmpty())
		{
			qDebug() << cloudName << "cancel saveAs!";
			it++;
			continue;
		}
		fileName = newFileName;
		QApplication::setOverrideCursor(Qt::WaitCursor);
		if( saveContentDialog->savePointCheckBox->isChecked() ) 
		{
			if (fileName.mid( fileName.size()-4, 4) == ".ply")
			{
				if (cloud->getPolygons().size() > 0)
				{
					PolygonMeshPtr polygonMesh(new PolygonMesh);
					toPolygonMesh(*cloudData, cloud->getPolygons(), *polygonMesh);
					CloudIO::exportPLYPolygonMesh(fileName, polygonMesh);
				}
				else CloudIO::exportPLYCloudData(fileName, cloudData);
			}
			else if (fileName.mid( fileName.size()-4, 4) == ".vtk")
			{
				if (cloud->getPolygons().size() > 0)
				{
					PolygonMeshPtr polygonMesh(new PolygonMesh);
					toPolygonMesh(*cloudData, cloud->getPolygons(), *polygonMesh);
					CloudIO::exportVTKPolygonMesh(fileName, polygonMesh);
				}
				else CloudIO::exportVTKCloudData(fileName, cloudData);
			}
		}
		if( saveContentDialog->saveTransformationCheckBox->isChecked() ) CloudIO::exportTransformation(newFileName, transformation);
		if( saveContentDialog->saveBoundaryCheckBox->isChecked() ) CloudIO::exportBoundaries(newFileName, boundaries);
		QApplication::restoreOverrideCursor();
		QApplication::beep();
		qDebug() << cloudName << " saved as " << newFileName;
		it++;
	}

	return true;
}

void MainWindow::on_cloudBrowser_cloudVisibleStateChanged(QStringList cloudNameList, bool isVisible)
{
	if (isVisible)
	{
		for (int i = 0; i < cloudNameList.size(); i++) 
		{
			qDebug() << cloudNameList[i];
			cloudVisualizer->addCloud(cloudManager->getCloud(cloudNameList[i]));
		}
	}
	else
	{
		for (int i = 0; i < cloudNameList.size(); i++) 
		{
			qDebug() << cloudNameList[i];
			cloudVisualizer->removeCloud(cloudManager->getCloud(cloudNameList[i]));
		}
	}
}


void MainWindow::on_euclideanClusterExtractionAction_triggered()
{
	if(!euclideanClusterExtractionDialog)
	{
		euclideanClusterExtractionDialog = new EuclideanClusterExtractionDialog(this);
		connect(euclideanClusterExtractionDialog, SIGNAL(sendParameters(QVariantMap)), 
			this, SLOT(on_euclideanClusterExtractionDialog_sendParameters(QVariantMap)));
	}
	euclideanClusterExtractionDialog->show();
	euclideanClusterExtractionDialog->raise();
	euclideanClusterExtractionDialog->activateWindow();
}

void MainWindow::on_voxelGridAction_triggered()
{
	if (!voxelGridDialog)
	{
		voxelGridDialog = new VoxelGridDialog(this);
		connect(voxelGridDialog, SIGNAL(sendParameters(QVariantMap)), 
			this, SLOT(on_voxelGridDialog_sendParameters(QVariantMap)));
	}
	voxelGridDialog->show();
	voxelGridDialog->raise();
	voxelGridDialog->activateWindow();
}

void MainWindow::on_movingLeastSquaresAction_triggered()
{
	if (!movingLeastSquaresDialog)
	{
		movingLeastSquaresDialog = new MovingLeastSquaresDialog(this);
		connect(movingLeastSquaresDialog, SIGNAL(sendParameters(QVariantMap)), 
			this, SLOT(on_movingLeastSquaresDialog_sendParameters(QVariantMap)));
	}
	movingLeastSquaresDialog->show();
	movingLeastSquaresDialog->raise();
	movingLeastSquaresDialog->activateWindow();
}

void MainWindow::on_boundaryEstimationAction_triggered()
{
	if (!boundaryEstimationDialog)
	{
		boundaryEstimationDialog = new BoundaryEstimationDialog(this);
		connect(boundaryEstimationDialog, SIGNAL(sendParameters(QVariantMap)),
			this, SLOT(on_boundaryEstimationDialog_sendParameters(QVariantMap)));
	}
	boundaryEstimationDialog->show();
	boundaryEstimationDialog->raise();
	boundaryEstimationDialog->activateWindow();
}

void MainWindow::on_outliersRemovalAction_triggered()
{
	if (!outliersRemovalDialog)
	{
		outliersRemovalDialog = new OutliersRemovalDialog(this);
		connect(outliersRemovalDialog, SIGNAL(sendParameters(QVariantMap)),
			this, SLOT(on_outliersRemovalDialog_sendParameters(QVariantMap)));
	}
	outliersRemovalDialog->show();
	outliersRemovalDialog->raise();
	outliersRemovalDialog->activateWindow();
}

void MainWindow::on_normalFieldAction_triggered()
{
	if (!normalFieldDialog)
	{
		normalFieldDialog = new NormalFieldDialog(this);
		connect(normalFieldDialog, SIGNAL(sendParameters(QVariantMap)),
			this, SLOT(on_normalFieldDialog_sendParameters(QVariantMap)));
	}
	normalFieldDialog->show();
	normalFieldDialog->raise();
	normalFieldDialog->activateWindow();
}

void MainWindow::on_colorFieldAction_triggered()
{
	if (!colorFieldDialog)
	{
		colorFieldDialog = new ColorFieldDialog(this);
		connect(colorFieldDialog, SIGNAL(sendParameters(QVariantMap)),
			this, SLOT(on_colorFieldDialog_sendParameters(QVariantMap)));
	}
	colorFieldDialog->show();
	colorFieldDialog->raise();
	colorFieldDialog->activateWindow();
}

void MainWindow::on_addNoiseAction_triggered()
{
	if (!addNoiseDialog)
	{
		addNoiseDialog = new AddNoiseDialog(this);
		connect(addNoiseDialog, SIGNAL(sendParameters(QVariantMap)),
			this, SLOT(on_addNoiseDialog_sendParameters(QVariantMap)));
	}
	addNoiseDialog->show();
	addNoiseDialog->raise();
	addNoiseDialog->activateWindow();
}

void MainWindow::on_transformationAction_triggered()
{
	if (!transformationDialog)
	{
		transformationDialog = new TransformationDialog(this);
		connect(transformationDialog, SIGNAL(sendParameters(QVariantMap)),
			this, SLOT(on_transformationDialog_sendParameters(QVariantMap)));
	}
	transformationDialog->show();
	transformationDialog->raise();
	transformationDialog->activateWindow();
}

void MainWindow::on_hausdorffDistanceAction_triggered()
{
	if (!hausdorffDistanceDialog)
	{
		hausdorffDistanceDialog = new HausdorffDistanceDialog(this);
		connect(hausdorffDistanceDialog, SIGNAL(sendParameters(QVariantMap)),
			this, SLOT(on_hausdorffDistanceDialog_sendParameters(QVariantMap)));
	}

	hausdorffDistanceDialog->targetComboBox->clear();
	QStringList allCloudNames = cloudManager->getAllCloudNames();
	hausdorffDistanceDialog->targetComboBox->addItems(allCloudNames);

	hausdorffDistanceDialog->show();
	hausdorffDistanceDialog->raise();
	hausdorffDistanceDialog->activateWindow();
}

void MainWindow::on_virtualScanAction_triggered()
{
	if (!virtualScanDialog)
	{
		virtualScanDialog = new VirtualScanDialog(this);
		connect(virtualScanDialog, SIGNAL(sendParameters(QVariantMap)),
			this, SLOT(on_virtualScanDialog_sendParameters(QVariantMap)));
	}

	virtualScanDialog->targetComboBox->clear();
	QStringList allCloudNames = cloudManager->getAllCloudNames();
	virtualScanDialog->targetComboBox->addItems(allCloudNames);

	virtualScanDialog->show();
	virtualScanDialog->raise();
	virtualScanDialog->activateWindow();
}

void MainWindow::on_depthCameraAction_triggered()
{
	if (!depthCameraDialog)
	{
		depthCameraDialog = new DepthCameraDialog(this);
		connect(depthCameraDialog, SIGNAL(sendParameters(QVariantMap)),
			this, SLOT(on_depthCameraDialog_sendParameters(QVariantMap)));
	}
	depthCameraDialog->show();
	depthCameraDialog->raise();
	depthCameraDialog->activateWindow();
}

void MainWindow::on_pairwiseRegistrationAction_triggered()
{
	if (!pairwiseRegistrationDialog)
	{
		pairwiseRegistrationDialog = new PairwiseRegistrationDialog(this);
		pairwiseRegistrationDialog->setWindowFlags( Qt::Dialog | Qt::WindowMaximizeButtonHint | Qt::WindowMinimizeButtonHint);
		connect(pairwiseRegistrationDialog, SIGNAL(sendParameters(QVariantMap)),
			this, SLOT(on_pairwiseRegistrationDialog_sendParameters(QVariantMap)));
	}

	pairwiseRegistrationDialog->targetComboBox->clear();
	pairwiseRegistrationDialog->sourceComboBox->clear();

	QStringList allCloudNames = cloudManager->getAllCloudNames();

	pairwiseRegistrationDialog->targetComboBox->addItems(allCloudNames);
	pairwiseRegistrationDialog->sourceComboBox->addItems(allCloudNames);

	int tabCurrentIndex = pairwiseRegistrationDialog->tabWidget->currentIndex();
	if (tabCurrentIndex != -1)
	{
		pairwiseRegistrationDialog->on_tabWidget_currentChanged(tabCurrentIndex);
	}

	pairwiseRegistrationDialog->show();
	pairwiseRegistrationDialog->raise();
	pairwiseRegistrationDialog->activateWindow();
}

void MainWindow::on_concatenationAction_triggered()
{
	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();
	
	QStringList::Iterator it = cloudNameList.begin();
	CloudDataPtr con_cloudData(new CloudData);
	BoundariesPtr con_boundaries(new Boundaries);
	while (it != cloudNameList.end())
	{
		QString cloudName = (*it);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		CloudDataPtr cloudData(new CloudData);
		pcl::transformPointCloudWithNormals(*cloud->getCloudData(), *cloudData, cloud->getTransformation());
		(*con_cloudData) += (*cloudData);
		(*con_boundaries) += (*cloud->getBoundaries());
		qDebug() << cloudName << " added!";
		it++;
	}
	
	Polygons polygons(0);
	Cloud* con_cloud = cloudManager->addCloud(con_cloudData, polygons, Cloud::fromFilter);
	con_cloud->setBoundaries(con_boundaries);
	cloudBrowser->addCloud(con_cloud);
	cloudVisualizer->addCloud(con_cloud);
}

void MainWindow::on_removeNaNAction_triggered()
{
	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();
	QList<bool> isVisibleList = cloudBrowser->getSelectedCloudIsVisible();

	QStringList::Iterator it_name = cloudNameList.begin();
	QList<bool>::Iterator it_visible = isVisibleList.begin();
	while (it_name != cloudNameList.end())
	{
		QString cloudName = (*it_name);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		CloudDataPtr cloudData(new CloudData);
		std::vector<int> nanIndicesVector;
		pcl::removeNaNFromPointCloud(*cloud->getCloudData(), *cloudData, nanIndicesVector);
		cloud->setCloudData(cloudData);
		cloudBrowser->updateCloud(cloud);
		bool isVisible = (*it_visible);
		if(isVisible)cloudVisualizer->updateCloud(cloud);
		qDebug() << cloudName << "NaN points removed!";
		it_name++;
		it_visible++;
	}	
}

void MainWindow::on_diagramAction_triggered()
{
	if (!diagramWindow)
	{
		diagramWindow = new DiagramWindow(this);
		QStringList allCloudNames = cloudManager->getAllCloudNames();
		for (int i = 0; i < allCloudNames.size(); ++i)
		{
			diagramWindow->addNode(allCloudNames.at(i));	
		}
	}

	diagramWindow->show();
	diagramWindow->raise();
	diagramWindow->activateWindow();	
}

void MainWindow::on_globalRegistrationAction_triggered()
{
	if (!globalRegistrationDialog)
	{
		globalRegistrationDialog = new GlobalRegistrationDialog(this);
		connect(globalRegistrationDialog, SIGNAL(sendParameters(QVariantMap)),
			this, SLOT(on_globalRegistrationDialog_sendParameters(QVariantMap)));
	}

	globalRegistrationDialog->show();
	globalRegistrationDialog->raise();
	globalRegistrationDialog->activateWindow();
}

void MainWindow::on_showCloudBrowserAction_toggled(bool isChecked)
{
	cloudBrowserDockWidget->setVisible(cloudBrowserDockWidget->isHidden());
}

void MainWindow::on_showPointBrowserAction_toggled(bool isChecked)
{
	pointBrowserDockWidget->setVisible(pointBrowserDockWidget->isHidden());
}

void MainWindow::on_colorNoneAction_toggled(bool isChecked)
{
	if(isChecked) qDebug() << "None color";

	QStringList cloudNameList = cloudBrowser->getVisibleCloudNames();
	cloudVisualizer->setColorMode(CloudVisualizer::colorNone);
	for(QStringList::iterator it = cloudNameList.begin(); it != cloudNameList.end(); it++)
	{
		QString cloudName = *it;
		Cloud* cloud = cloudManager->getCloud(cloudName);
		cloudVisualizer->updateCloud(cloud);
	}
}

void MainWindow::on_colorOriginalAction_toggled(bool isChecked)
{
	if(isChecked) qDebug() << "Original color";

	QStringList cloudNameList = cloudBrowser->getVisibleCloudNames();
	cloudVisualizer->setColorMode(CloudVisualizer::colorOriginal);
	for(QStringList::iterator it = cloudNameList.begin(); it != cloudNameList.end(); it++)
	{
		QString cloudName = *it;
		Cloud* cloud = cloudManager->getCloud(cloudName);
		cloudVisualizer->updateCloud(cloud);
	}
}

void MainWindow::on_colorCustomAction_toggled(bool isChecked)
{
	if(isChecked) qDebug() << "Custom color";

	cloudVisualizer->setColorMode(CloudVisualizer::colorCustom);

	QStringList cloudNameList = cloudBrowser->getVisibleCloudNames();
	for(QStringList::iterator it = cloudNameList.begin(); it != cloudNameList.end(); it++)
	{
		QString cloudName = *it;
		Cloud* cloud = cloudManager->getCloud(cloudName);
		cloudVisualizer->updateCloud(cloud);
	}
}

void MainWindow::on_drawNormalAction_toggled(bool isChecked)
{
	if(isChecked) qDebug() << "Draw normal";
	else qDebug() << "Undraw normal";
	
	cloudVisualizer->setDrawNormal(isChecked);
	
	QStringList cloudNameList = cloudBrowser->getVisibleCloudNames();
	for(QStringList::iterator it = cloudNameList.begin(); it != cloudNameList.end(); it++)
	{
		QString cloudName = *it;
		Cloud* cloud = cloudManager->getCloud(cloudName);
		cloudVisualizer->updateCloud(cloud);
	}
}

void MainWindow::on_drawAxisAction_toggled(bool isChecked)
{
	if(isChecked) qDebug() << "Draw axis";
	else qDebug() << "Undraw axis";

	QStringList cloudNameList = cloudBrowser->getVisibleCloudNames();

	pcl::PointCloud<PointType> cloud_box;
	for(QStringList::Iterator it = cloudNameList.begin(); it != cloudNameList.end(); it++)
	{
		QString cloudName = (*it);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		PointType min_pt, max_pt;
		pcl::getMinMax3D(*cloud->getCloudData(), min_pt, max_pt);
		cloud_box.push_back(min_pt);
		cloud_box.push_back(max_pt);
	}	
	PointType all_min_pt, all_max_pt;
	pcl::getMinMax3D(cloud_box, all_min_pt, all_max_pt);
	float axis_len = ( all_max_pt.getVector3fMap() - all_min_pt.getVector3fMap() ).norm() * 0.5f;
	if(isChecked)cloudVisualizer->addAxis(axis_len);
	else cloudVisualizer->removeAxis();
}

void MainWindow::on_drawOrientationMarkerAction_toggled(bool isChecked)
{
	if(isChecked) qDebug() << "Draw orientation marker";
	else qDebug() << "Undraw orientation marker";

	if(isChecked)cloudVisualizer->addOrientationMarker();
	else cloudVisualizer->removeOrientationMarker();
}

void MainWindow::on_registrationModeAction_toggled(bool isChecked)
{
	if(isChecked) qDebug() << "Registration mode";
	else qDebug() << "Close registration mode";

	cloudVisualizer->setRegistrationMode(isChecked);
	
	QStringList cloudNameList = cloudBrowser->getVisibleCloudNames();
	for(QStringList::iterator it = cloudNameList.begin(); it != cloudNameList.end(); it++)
	{
		QString cloudName = *it;
		Cloud* cloud = cloudManager->getCloud(cloudName);
		cloudVisualizer->updateCloud(cloud);
	}
}


void MainWindow::on_drawBoundaryAction_toggled(bool isChecked)
{
	if(isChecked) qDebug() << "Draw boundary";
	else qDebug() << "Undraw Boundary";

	cloudVisualizer->setDrawBoundary(isChecked);

	QStringList cloudNameList = cloudBrowser->getVisibleCloudNames();
	for(QStringList::iterator it = cloudNameList.begin(); it != cloudNameList.end(); it++)
	{
		QString cloudName = *it;
		Cloud* cloud = cloudManager->getCloud(cloudName);
		cloudVisualizer->updateCloud(cloud);
	}
}

void MainWindow::on_confirmRegistrationAction_triggered()
{
	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();
	QList<bool> isVisibleList = cloudBrowser->getSelectedCloudIsVisible();

	QStringList::Iterator it_name = cloudNameList.begin();
	QList<bool>::Iterator it_visible = isVisibleList.begin();
	while (it_name != cloudNameList.end())
	{
		QString cloudName = (*it_name);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		cloud->setTransformation(cloud->getRegistrationTransformation());
		bool isVisible = (*it_visible);
		if(isVisible)cloudVisualizer->updateCloud(cloud);
		qDebug() << cloudName << "registration transformation confirmed!";
		it_name++;
		it_visible++;
	}	
}

void MainWindow::on_applyTransformationAction_triggered()
{
	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();
	QList<bool> isVisibleList = cloudBrowser->getSelectedCloudIsVisible();

	QStringList::Iterator it_name = cloudNameList.begin();
	QList<bool>::Iterator it_visible = isVisibleList.begin();
	while (it_name != cloudNameList.end())
	{
		QString cloudName = (*it_name);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		CloudDataPtr cloudData(new CloudData);
		pcl::transformPointCloudWithNormals(*cloud->getCloudData(), *cloudData, cloud->getTransformation());
		cloud->setCloudData(cloudData);
		cloud->setTransformation(Eigen::Matrix4f::Identity());
		cloud->setRegistrationTransformation(Eigen::Matrix4f::Identity());
		bool isVisible = (*it_visible);
		if(isVisible)cloudVisualizer->updateCloud(cloud);
		qDebug() << cloudName << "transformation applied!";
		it_name++;
		it_visible++;
	}	
}

void MainWindow::on_forceRigidRegistrationAction_triggered()
{
	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();
	QList<bool> isVisibleList = cloudBrowser->getSelectedCloudIsVisible();

	QStringList::Iterator it_name = cloudNameList.begin();
	QList<bool>::Iterator it_visible = isVisibleList.begin();
	while (it_name != cloudNameList.end())
	{
		QString cloudName = (*it_name);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		//std::cout << getScaleFromTransformation(cloud->getRegistrationTransformation()) << std::endl;
		Eigen::Matrix4f transformation = cloud->getRegistrationTransformation();
		cloud->setRegistrationTransformation(toRigidTransformation(transformation));
		//std::cout << getScaleFromTransformation(cloud->getRegistrationTransformation()) << std::endl;
		bool isVisible = (*it_visible);
		if(isVisible)cloudVisualizer->updateCloud(cloud);
		qDebug() << cloudName << " is forced to rigid transformation!";
		it_name++;
		it_visible++;
	}		
}


void MainWindow::on_euclideanClusterExtractionDialog_sendParameters(QVariantMap parameters)
{
	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();
	QList<bool> isVisibleList = cloudBrowser->getSelectedCloudIsVisible();

	QStringList::Iterator it_name = cloudNameList.begin();
	QList<bool>::Iterator it_visible = isVisibleList.begin();
	while (it_name != cloudNameList.end())
	{
		QString cloudName = (*it_name);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		CloudDataConstPtr cloudData = cloud->getCloudData();
		CloudDataPtr cloudData_inliers, cloudData_outliers;
		
		QApplication::setOverrideCursor(Qt::WaitCursor);
		EuclideanClusterExtraction::filter(cloudData, parameters, cloudData_inliers, cloudData_outliers);
		QApplication::restoreOverrideCursor();
		QApplication::beep();

		if( parameters["overwrite"].value<bool>() )
		{
			cloud->setCloudData(cloudData_inliers);
			cloud->setPolygons(Polygons(0));
			cloudBrowser->updateCloud(cloud);
			bool isVisible = (*it_visible);
			if(isVisible)cloudVisualizer->updateCloud(cloud);
		}
		else
		{
			Polygons polygons(0);
			Cloud* cloudInliers = cloudManager->addCloud(cloudData_inliers, polygons, Cloud::fromFilter);
			cloudBrowser->addCloud(cloudInliers);
			cloudVisualizer->addCloud(cloudInliers);

			Cloud* cloudOutliers= cloudManager->addCloud(cloudData_outliers, polygons, Cloud::fromFilter);
			cloudBrowser->addCloud(cloudOutliers);
			cloudVisualizer->addCloud(cloudOutliers);
		}

		it_name++;
		it_visible++;
	}
}

void MainWindow::on_voxelGridDialog_sendParameters(QVariantMap parameters)
{
	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();
	QList<bool> isVisibleList = cloudBrowser->getSelectedCloudIsVisible();

	QStringList::Iterator it_name = cloudNameList.begin();
	QList<bool>::Iterator it_visible = isVisibleList.begin();
	while (it_name != cloudNameList.end())
	{
		QString cloudName = (*it_name);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		CloudDataConstPtr cloudData = cloud->getCloudData();

		CloudDataPtr cloudData_filtered;
		QApplication::setOverrideCursor(Qt::WaitCursor);
		VoxelGrid::filter(cloudData, parameters, cloudData_filtered);
		QApplication::restoreOverrideCursor();
		QApplication::beep();

		if( parameters["overwrite"].value<bool>() )
		{
			cloud->setCloudData(cloudData_filtered);
			cloud->setPolygons(Polygons(0));
			cloudBrowser->updateCloud(cloud);
			bool isVisible = (*it_visible);
			if(isVisible)cloudVisualizer->updateCloud(cloud);
		}
		else
		{
			Polygons polygons(0);
			Cloud* cloudFiltered = cloudManager->addCloud(cloudData_filtered, polygons, Cloud::fromFilter);
			cloudBrowser->addCloud(cloudFiltered);
			cloudVisualizer->addCloud(cloudFiltered);
		}	
		
		it_name++;
		it_visible++;
	}
}


void MainWindow::on_movingLeastSquaresDialog_sendParameters(QVariantMap parameters)
{
	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();
	QList<bool> isVisibleList = cloudBrowser->getSelectedCloudIsVisible();

	QStringList::Iterator it_name = cloudNameList.begin();
	QList<bool>::Iterator it_visible = isVisibleList.begin();
	while (it_name != cloudNameList.end())
	{
		QString cloudName = (*it_name);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		CloudDataConstPtr cloudData = cloud->getCloudData();

		CloudDataPtr cloudData_filtered;
		QApplication::setOverrideCursor(Qt::WaitCursor);
		MovingLeastSquares::filter(cloudData, parameters, cloudData_filtered);
		QApplication::restoreOverrideCursor();
		QApplication::beep();

		if( parameters["overwrite"].value<bool>() )
		{
			cloud->setCloudData(cloudData_filtered);
			cloud->setPolygons(Polygons(0));
			cloudBrowser->updateCloud(cloud);
			bool isVisible = (*it_visible);
			if(isVisible)cloudVisualizer->updateCloud(cloud);
		}
		else
		{
			Polygons polygons(0);
			Cloud* cloudFiltered = cloudManager->addCloud(cloudData_filtered, polygons, Cloud::fromFilter);
			cloudBrowser->addCloud(cloudFiltered);
			cloudVisualizer->addCloud(cloudFiltered);
		}	
		
		it_name++;
		it_visible++;
	}	
}

void MainWindow::on_boundaryEstimationDialog_sendParameters(QVariantMap parameters)
{
	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();
	QList<bool> isVisibleList = cloudBrowser->getSelectedCloudIsVisible();

	QStringList::Iterator it_name = cloudNameList.begin();
	QList<bool>::Iterator it_visible = isVisibleList.begin();
	while (it_name != cloudNameList.end())
	{
		QString cloudName = (*it_name);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		CloudDataConstPtr cloudData = cloud->getCloudData();
		CloudDataPtr cloudData_inliers, cloudData_outliers;
		BoundariesPtr boundaries;
		
		QApplication::setOverrideCursor(Qt::WaitCursor);
		BoundaryEstimation::filter(cloudData, parameters, boundaries, cloudData_inliers, cloudData_outliers);
		QApplication::restoreOverrideCursor();
		QApplication::beep();

		if( parameters["overwrite"].value<bool>() )
		{
			cloud->setCloudData(cloudData_outliers);
			cloud->setPolygons(Polygons(0));
			cloud->setBoundaries(BoundariesPtr());
			cloudBrowser->updateCloud(cloud);
			bool isVisible = (*it_visible);
			if(isVisible)cloudVisualizer->updateCloud(cloud);
		}
		else
		{
			// Cloud* cloudInliers = cloudManager->addCloud(cloudData_inliers, Cloud::fromFilter);
			// cloudBrowser->addCloud(cloudInliers);
			// cloudVisualizer->addCloud(cloudInliers);

			// Cloud* cloudOutliers= cloudManager->addCloud(cloudData_outliers, Cloud::fromFilter);
			// cloudBrowser->addCloud(cloudOutliers);
			// cloudVisualizer->addCloud(cloudOutliers);

			cloud->setBoundaries(boundaries);
			bool isVisible = (*it_visible);
			if(isVisible)cloudVisualizer->updateCloud(cloud);
		}

		it_name++;
		it_visible++;
	}
}


void MainWindow::on_outliersRemovalDialog_sendParameters(QVariantMap parameters)
{
	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();
	QList<bool> isVisibleList = cloudBrowser->getSelectedCloudIsVisible();

	QStringList::Iterator it_name = cloudNameList.begin();
	QList<bool>::Iterator it_visible = isVisibleList.begin();
	while (it_name != cloudNameList.end())
	{
		QString cloudName = (*it_name);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		CloudDataConstPtr cloudData = cloud->getCloudData();
		CloudDataPtr cloudData_inliers, cloudData_outliers;
		
		QApplication::setOverrideCursor(Qt::WaitCursor);
		OutliersRemoval::filter(cloudData, parameters, cloudData_inliers, cloudData_outliers);
		QApplication::restoreOverrideCursor();
		QApplication::beep();

		if( parameters["overwrite"].value<bool>() )
		{
			cloud->setCloudData(cloudData_inliers);
			cloud->setPolygons(Polygons(0));
			cloudBrowser->updateCloud(cloud);
			bool isVisible = (*it_visible);
			if(isVisible)cloudVisualizer->updateCloud(cloud);
		}
		else
		{
			Polygons polygons(0);
			Cloud* cloudInliers = cloudManager->addCloud(cloudData_inliers, polygons, Cloud::fromFilter);
			cloudBrowser->addCloud(cloudInliers);
			cloudVisualizer->addCloud(cloudInliers);
			
			Cloud* cloudOutliers= cloudManager->addCloud(cloudData_outliers, polygons, Cloud::fromFilter);
			cloudBrowser->addCloud(cloudOutliers);
			cloudVisualizer->addCloud(cloudOutliers);
		}

		it_name++;
		it_visible++;
	}
}

void MainWindow::on_normalFieldDialog_sendParameters(QVariantMap parameters)
{
	float X = parameters["X"].toFloat();
	float Y = parameters["Y"].toFloat();
	float Z = parameters["Z"].toFloat();

	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();
	QList<bool> isVisibleList = cloudBrowser->getSelectedCloudIsVisible();

	QStringList::Iterator it_name = cloudNameList.begin();
	QList<bool>::Iterator it_visible = isVisibleList.begin();
	while (it_name != cloudNameList.end())
	{
		QString cloudName = (*it_name);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		CloudDataConstPtr cloudData = cloud->getCloudData();

		CloudDataPtr cloudData_filtered;
		QApplication::setOverrideCursor(Qt::WaitCursor);

		cloudData_filtered.reset(new CloudData);
		pcl::PointXYZ view_point;
		view_point.x = X;
		view_point.y = Y;
		view_point.z = Z;
		flipPointCloudNormalsTowardsViewpoint(*cloudData, view_point, *cloudData_filtered);

		QApplication::restoreOverrideCursor();
		QApplication::beep();

		if( parameters["overwrite"].value<bool>() )
		{
			cloud->setCloudData(cloudData_filtered);
			cloudBrowser->updateCloud(cloud);
			bool isVisible = (*it_visible);
			if(isVisible)cloudVisualizer->updateCloud(cloud);
		}
		else
		{	
			Polygons polygons = cloud->getPolygons();
			Eigen::Matrix4f transformation = cloud->getTransformation();
			Cloud* cloudFiltered = cloudManager->addCloud(cloudData_filtered, polygons, Cloud::fromFilter, "", transformation);
			cloudBrowser->addCloud(cloudFiltered);
			cloudVisualizer->addCloud(cloudFiltered);
		}	
		
		it_name++;
		it_visible++;
	}	
}

void MainWindow::on_colorFieldDialog_sendParameters(QVariantMap parameters)
{
	float R = parameters["R"].toFloat();
	float G = parameters["G"].toFloat();
	float B = parameters["B"].toFloat();

	float Hmin = parameters["Hmin"].toFloat();
	float Hmax = parameters["Hmax"].toFloat();
	float S = parameters["S"].toFloat();
	float V = parameters["V"].toFloat();

	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();
	QList<bool> isVisibleList = cloudBrowser->getSelectedCloudIsVisible();

	QStringList::Iterator it_name = cloudNameList.begin();
	QList<bool>::Iterator it_visible = isVisibleList.begin();

	int tabIndex = colorFieldDialog->tabWidget->currentIndex();
	std::vector<RGB_> rgbs = generateUniformColors(cloudNameList.size(), Hmin, Hmax);
	std::vector<RGB_>::iterator it_rgb = rgbs.begin();

	while (it_name != cloudNameList.end())
	{
		QString cloudName = (*it_name);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		CloudDataConstPtr cloudData = cloud->getCloudData();

		CloudDataPtr cloudData_filtered;
		QApplication::setOverrideCursor(Qt::WaitCursor);

		cloudData_filtered.reset(new CloudData);
		pcl::copyPointCloud(*cloudData, *cloudData_filtered);
		if (tabIndex == 0) //Manual set color
		{
			setPointCloudColor(*cloudData_filtered, R, G, B);
		}
		else if (tabIndex == 1)//Uniform set color
		{
			setPointCloudColor(*cloudData_filtered, (*it_rgb).r, (*it_rgb).g, (*it_rgb).b );
		}

		QApplication::restoreOverrideCursor();
		QApplication::beep();

		if( parameters["overwrite"].value<bool>() )
		{
			cloud->setCloudData(cloudData_filtered);
			cloudBrowser->updateCloud(cloud);
			bool isVisible = (*it_visible);
			if(isVisible)cloudVisualizer->updateCloud(cloud);
		}
		else
		{	
			Polygons polygons = cloud->getPolygons();
			Eigen::Matrix4f transformation = cloud->getTransformation();
			Cloud* cloudFiltered = cloudManager->addCloud(cloudData_filtered, polygons, Cloud::fromFilter, "", transformation);
			cloudBrowser->addCloud(cloudFiltered);
			cloudVisualizer->addCloud(cloudFiltered);
		}	

		it_name++;
		it_visible++;
		it_rgb++;
	}	
}

void MainWindow::on_addNoiseDialog_sendParameters(QVariantMap parameters)
{
	float X = parameters["X"].toFloat();
	float Y = parameters["Y"].toFloat();
	float Z = parameters["Z"].toFloat();
	float noise_std = parameters["noise_std"].toFloat();
	int method = parameters["method"].toInt();

	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();
	QList<bool> isVisibleList = cloudBrowser->getSelectedCloudIsVisible();

	QStringList::Iterator it_name = cloudNameList.begin();
	QList<bool>::Iterator it_visible = isVisibleList.begin();
	while (it_name != cloudNameList.end())
	{
		QString cloudName = (*it_name);
		Cloud *cloud = cloudManager->getCloud(cloudName);
		CloudDataConstPtr cloudData = cloud->getCloudData();

		CloudDataPtr cloudData_filtered;
		QApplication::setOverrideCursor(Qt::WaitCursor);

		cloudData_filtered.reset(new CloudData);
		switch (method)
		{
			case 0:
			{
				addNoiseToPointCloud(*cloudData, noise_std, *cloudData_filtered);
				break;
			}
			case 1:
			{
				pcl::PointXYZ view_point;
				view_point.x = X;
				view_point.y = Y;
				view_point.z = Z;
				addNoiseAlongViewPointToPointCloud(*cloudData, noise_std, view_point, *cloudData_filtered);
			}
		}

		QApplication::restoreOverrideCursor();
		QApplication::beep();

		if( parameters["overwrite"].value<bool>() )
		{
			cloud->setCloudData(cloudData_filtered);
			cloudBrowser->updateCloud(cloud);
			bool isVisible = (*it_visible);
			if(isVisible)cloudVisualizer->updateCloud(cloud);
		}
		else
		{	
			Polygons polygons = cloud->getPolygons();
			Eigen::Matrix4f transformation = cloud->getTransformation();
			Cloud* cloudFiltered = cloudManager->addCloud(cloudData_filtered, polygons, Cloud::fromFilter, "", transformation);
			cloudBrowser->addCloud(cloudFiltered);
			cloudVisualizer->addCloud(cloudFiltered);
		}	

		it_name++;
		it_visible++;
	}	
}

void MainWindow::on_transformationDialog_sendParameters(QVariantMap parameters)
{
	QString command = parameters["command"].toString();

	float max_angle = parameters["max_angle"].toFloat();
	float max_distance = parameters["max_distance"].toFloat();
	bool rotate_around_centroid = parameters["rotate_around_centroid"].toBool();

	bool send_centroid_origin = parameters["send_centroid_origin"].toBool();
	bool align_pca_xyz = parameters["align_pca_xyz"].toBool();
	bool scale_unit = parameters["scale_unit"].toBool();

	bool uniform = parameters["uniform"].toBool();
	bool separate = parameters["separate"].toBool();

	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();
	QList<bool> isVisibleList = cloudBrowser->getSelectedCloudIsVisible();

	QStringList::Iterator it_name = cloudNameList.begin();
	QList<bool>::Iterator it_visible = isVisibleList.begin();
	while (it_name != cloudNameList.end())
	{
		QString cloudName = (*it_name);
		Cloud *cloud = cloudManager->getCloud(cloudName);

		QApplication::setOverrideCursor(Qt::WaitCursor);

		if ( command == "Random" )
		{
			Eigen::Matrix4f transformation = cloud->getTransformation();
			Eigen::Matrix4f randomRigidTransf = Eigen::Matrix4f::Identity();
			if (rotate_around_centroid)
			{
				Eigen::Matrix3f rotation = randomRotation(max_angle / 180 * M_PI).toRotationMatrix();
				Eigen::Vector3f translation = randomTranslation(max_distance);
				Eigen::Vector4f centroid;
				pcl::compute3DCentroid(*cloud->getCloudData(), centroid);
				centroid = transformation * centroid;
				translation += ( Eigen::Matrix3f::Identity() - rotation ) * centroid.block<3, 1>(0, 0);
				randomRigidTransf.block<3,3>(0,0) = rotation;
				randomRigidTransf.block<3,1>(0,3) = translation;
				randomRigidTransf = randomRigidTransf * transformation;
			}
			else
			{
				randomRigidTransf = randomRigidTransformation(max_angle / 180 * M_PI, max_distance) * transformation;
			}
			cloud->setRegistrationTransformation(randomRigidTransf);
		}
		else if ( command == "Normalize" )
		{
			Eigen::Matrix4f transformation = cloud->getTransformation();
			Eigen::Matrix4f normalizeTransformation = Eigen::Matrix4f::Identity();
			if (send_centroid_origin)
			{
				Eigen::Vector3f translation = Eigen::Vector3f(0,0,0);
				Eigen::Vector4f centroid;
				pcl::compute3DCentroid(*cloud->getCloudData(), centroid);
				centroid = transformation * centroid;
				translation = -centroid.block<3,1>(0, 0);
				normalizeTransformation.block<3,1>(0, 3) = translation;
				normalizeTransformation = normalizeTransformation * transformation;
			}
			if (align_pca_xyz)
			{
				Eigen::Transform<float,3, Eigen::Affine> pcaTransformation;
				pcl::getPrincipalTransformation(*cloud->getCloudData(), pcaTransformation);
				Eigen::Matrix3f rotation = pcaTransformation.rotation().inverse();
				Eigen::Vector3f translation = Eigen::Vector3f(0, 0, 0);
				Eigen::Vector4f centroid = pcaTransformation.matrix().block<4,1>(0, 3);
				centroid = transformation * centroid;
				translation = ( Eigen::Matrix3f::Identity() - rotation ) * centroid.block<3,1>(0,0);
				normalizeTransformation.block<3,3>(0,0) = rotation;
				normalizeTransformation.block<3,1>(0,3) = translation;
				normalizeTransformation = normalizeTransformation * transformation;
			}
			if (scale_unit)
			{
				PointType min_pt, max_pt;
				pcl::getMinMax3D(*cloud->getCloudData(), min_pt, max_pt);
			}
			cloud->setRegistrationTransformation(normalizeTransformation);
		}

		QApplication::restoreOverrideCursor();
		QApplication::beep();

		cloudBrowser->updateCloud(cloud);
		bool isVisible = (*it_visible);
		if(isVisible)cloudVisualizer->updateCloud(cloud);

		it_name++;
		it_visible++;
	}	
}

void MainWindow::on_generateOutliersAction_triggered()
{
	if (!generateOutliersDialog)
	{
		generateOutliersDialog = new GenerateOutliersDialog(this);
		connect(generateOutliersDialog, SIGNAL(sendParameters(QVariantMap)),
			this, SLOT(on_generateOutliersDialog_sendParameters(QVariantMap)));
		connect(generateOutliersDialog, SIGNAL(boundingBox()),
			this, SLOT(on_generateOutliersDialog_boundingBox()));
	}

	generateOutliersDialog->show();
	generateOutliersDialog->raise();
	generateOutliersDialog->activateWindow();
}

void MainWindow::on_generateOutliersDialog_sendParameters(QVariantMap parameters)
{
	QString command = parameters["command"].toString();

	float xmin = parameters["xmin"].toFloat();
	float xmax = parameters["xmax"].toFloat();
	float ymin = parameters["ymin"].toFloat();
	float ymax = parameters["ymax"].toFloat();
	float zmin = parameters["zmin"].toFloat();
	float zmax = parameters["zmax"].toFloat();

	int numofpoints = parameters["numofpoints"].toInt();

	int method = parameters["method"].toInt();
	bool uniform = parameters["uniform"].toBool();
	bool separate = parameters["separate"].toBool();

	float percent = parameters["percent"].toFloat();
	float noise_std = parameters["noise_std"].toFloat();

	bool overwrite = parameters["overwrite"].toBool();

	if( command == "Manual")
	{
		CloudDataPtr cloudData_outliers(new CloudData);
		cloudData_outliers->resize(numofpoints);
		for (int i =0; i < numofpoints; i++)
		{
			cloudData_outliers->points[i].getVector3fMap() = random3DPoint(xmin, xmax, ymin, ymax, zmin, zmax).getVector3fMap();
		}
		Polygons polygons;
		Cloud* cloudOutliers = cloudManager->addCloud(cloudData_outliers, polygons, Cloud::fromFilter, "", Eigen::Matrix4f::Identity());
		cloudBrowser->addCloud(cloudOutliers);
		cloudVisualizer->addCloud(cloudOutliers);
	}
	else if ( command == "Data-dependent")
	{
		if ( method == 0)
		{
			if (uniform)
			{
			}
			else if (separate)
			{
				QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();

				if (cloudNameList.begin() != cloudNameList.end())
				{
					for(QStringList::Iterator it = cloudNameList.begin(); it != cloudNameList.end(); it++)
					{
						QString cloudName = (*it);
						Cloud *cloud = cloudManager->getCloud(cloudName);
						PointType min_pt, max_pt;
						pcl::getMinMax3D(*cloud->getCloudData(), min_pt, max_pt);

						xmin = min_pt.x;
						ymin = min_pt.y;
						zmin = min_pt.z;

						xmax = max_pt.x;
						ymax = max_pt.y;
						zmax = max_pt.z;

						CloudDataPtr cloudData_outliers(new CloudData);
						cloudData_outliers->resize(percent * cloud->getCloudData()->size());
						for (int i =0; i < cloudData_outliers->size(); i++)
						{
							cloudData_outliers->points[i].getVector3fMap() = random3DPoint(xmin, xmax, ymin, ymax, zmin, zmax).getVector3fMap();
							cloudData_outliers->points[i].getNormalVector3fMap() = Eigen::Vector3f(0,0,1);
						}
						Polygons polygons;
						Cloud* cloudOutliers = cloudManager->addCloud(cloudData_outliers, polygons, Cloud::fromFilter, "", Eigen::Matrix4f::Identity());
						cloudBrowser->addCloud(cloudOutliers);
						cloudVisualizer->addCloud(cloudOutliers);
					}	
				}
			}
		}
		else if (method == 2)
		{

		}
	}

}

void MainWindow::on_generateOutliersDialog_boundingBox()
{
	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();

	if (cloudNameList.begin() != cloudNameList.end())
	{
		pcl::PointCloud<PointType> cloud_box;
		for(QStringList::Iterator it = cloudNameList.begin(); it != cloudNameList.end(); it++)
		{
			QString cloudName = (*it);
			Cloud *cloud = cloudManager->getCloud(cloudName);
			PointType min_pt, max_pt;
			pcl::getMinMax3D(*cloud->getCloudData(), min_pt, max_pt);
			cloud_box.push_back(min_pt);
			cloud_box.push_back(max_pt);
		}	
		PointType all_min_pt, all_max_pt;
		pcl::getMinMax3D(cloud_box, all_min_pt, all_max_pt);

		generateOutliersDialog->xMinLineEdit->setText(QString::number(all_min_pt.x));
		generateOutliersDialog->yMinLineEdit->setText(QString::number(all_min_pt.y));
		generateOutliersDialog->zMinLineEdit->setText(QString::number(all_min_pt.z));

		generateOutliersDialog->xMaxLineEdit->setText(QString::number(all_max_pt.x));
		generateOutliersDialog->yMaxLineEdit->setText(QString::number(all_max_pt.y));
		generateOutliersDialog->zMaxLineEdit->setText(QString::number(all_max_pt.z));
	}
}

void MainWindow::on_depthCameraDialog_sendParameters(QVariantMap parameters)
{
	int method = parameters["method"].toInt();
	bool camera_coordiante = parameters["camera_coordinate"].toBool();
	int xres, yres;
	xres = parameters["xres"].toInt();
	yres = parameters["yres"].toInt();
	float view_angle = parameters["view_angle"].toFloat();
	float noise_std = parameters["noise_std"].toFloat();
	int tesselation_level = parameters["tesselation_level"].toInt();
	float radius_sphere = parameters["radius_sphere"].toFloat();
	bool use_vertices = parameters["use_vertices"].toBool();

	switch(method)
	{
		case 0:
		{
			//QSize size_backup = centralWidget()->size();
			QSize size_wish(xres, yres);
			centralWidget()->resize(size_wish);

			pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
			Eigen::Matrix4f pose;
			cloudVisualizer->getVisualizer()->renderView2(xres, yres, temp, pose);

			temp->is_dense = false;
			temp->sensor_origin_ = Eigen::Vector4f(0, 0, 0, 0);
			temp->sensor_orientation_ = Eigen::Quaternionf(1, 0, 0, 0);

			CloudDataPtr cloudData(new CloudData);
			pcl::copyPointCloud(*temp, *cloudData);
			Eigen::Matrix4f transformation;
			if(camera_coordiante) transformation = Eigen::Matrix4f::Identity();
			else transformation = pose.inverse();
			Cloud* cloud = cloudManager->addCloud(cloudData, Polygons(0), Cloud::fromFilter, "", transformation);
			cloudBrowser->addCloud(cloud);
			cloudVisualizer->addCloud(cloud);

			// std::vector<int> nanIndicesVector;
			// pcl::removeNaNFromPointCloud( *cloudData, *cloudData, nanIndicesVector );
			// std::cout << *cloudData << std::endl;
			// pcl::PLYWriter writer;
			// writer.write( "temp.ply", *cloudData);	

			//centralWidget()->resize(size_backup);	
			break;		
		}

		case 1:
		{
			pcl::PointCloud<pcl::PointXYZ>::CloudVectorType clouds;
			std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
			std::vector<float> enthropies;
			cloudVisualizer->getVisualizer()->renderViewTesselatedSphere3(xres, yres, clouds, poses, enthropies, tesselation_level, view_angle, radius_sphere, use_vertices);

			std::vector<RGB_> rgbs = generateUniformColors(clouds.size(), 60, 300);

			for (int i = 0; i < clouds.size(); ++i)
			{
				clouds[i].is_dense = false;
				clouds[i].sensor_origin_ = Eigen::Vector4f(0, 0, 0, 0);
				clouds[i].sensor_orientation_ = Eigen::Quaternionf(1, 0, 0, 0);
				CloudDataPtr cloudData(new CloudData);
				pcl::copyPointCloud(clouds[i], *cloudData);
				Eigen::Matrix4f transformation;
				if (camera_coordiante) transformation = Eigen::Matrix4f::Identity();
				else transformation = poses[i].inverse();

				//Set Color to PointCloud
				setPointCloudColor(*cloudData, rgbs[i].r, rgbs[i].g, rgbs[i].b);

				Cloud* cloud = cloudManager->addCloud(cloudData, Polygons(0), Cloud::fromFilter, "", transformation);
				cloudBrowser->addCloud(cloud);
				cloudVisualizer->addCloud(cloud);

				//std::cout << "scan " << i << " : " << enthropies[i] << std::endl;

				//std::vector<int> nanIndicesVector;
				//pcl::removeNaNFromPointCloud( *cloudData, *cloudData, nanIndicesVector );
				//std::cout << *cloudData << std::endl;
				//pcl::PLYWriter writer;
				//std::stringstream ss;
				//ss << "temp_" << i << ".ply";
 				//writer.write( ss.str(), *cloudData);	
			}

			break;
		}
	}
}

void MainWindow::on_virtualScanDialog_sendParameters(QVariantMap parameters)
{
	QString cloudName_target = parameters["target"].toString();
	Cloud *cloud_target = cloudManager->getCloud(cloudName_target);

	int method = parameters["method"].toInt();
	bool camera_coordiante = parameters["camera_coordiante"].toBool();
	int nr_scans = parameters["nr_scans"].toInt();             
	int nr_points_in_scans = parameters["nr_points_in_scans"].toInt();   
	float vert_res = parameters["vert_res"].toFloat(); 
	float hor_res = parameters["hor_res"].toFloat();
	float max_dist = parameters["max_dist"].toFloat();
	float view_angle = parameters["view_angle"].toFloat();
	bool use_vertices = parameters["use_vertices"].toBool();

	switch(method)
	{
	case 0:
		{
			vtkSmartPointer<vtkPolyData> vtk_polydata = generateVTKPolyData(cloud_target->getCloudData(), cloud_target->getPolygons());
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
			Eigen::Matrix4f pose = cloudVisualizer->getVisualizer()->getViewerPose().matrix();
			pcl::visualization::Camera camera;
			cloudVisualizer->getVisualizer()->getCameraParameters(camera);
			std::cout << pose << std::endl;
			//std::cout << camera.pos[0] << "\t" << camera.pos[1] << "\t" << camera.pos[2] << std::endl;
			//std::cout << camera.focal[0] - camera.pos[0] << "\t" << camera.focal[1] - camera.pos[2] << "\t" << camera.focal[2] - camera.pos[2] << std::endl;
			//std::cout << camera.view[0] << "\t" << camera.view[1] << "\t" << camera.view[2] << std::endl;
			Eigen::Vector3d pos(camera.pos);
			Eigen::Vector3d focal(camera.focal);
			Eigen::Vector3d view(camera.view);
			std::cout << (pos - focal).normalized() << std::endl;
			std::cout << pose.block<3,3>(0,0).inverse() * Eigen::Vector3f(0, 0, 1) << std::endl;
			std::cout << (pos - focal).cross(view).cross(pos - focal).normalized() << std::endl;
			std::cout << pose.block<3,3>(0,0).inverse() * Eigen::Vector3f(0, 1, 0) << std::endl;
			//VirtualScan::scan(nr_scans, nr_points_in_scans, vert_res, hor_res, max_dist, pose, vtk_polydata, temp);
			break;
		}
	case 1:
		{
			break;
		}
	}
}

void MainWindow::on_hausdorffDistanceDialog_sendParameters(QVariantMap parameters)
{
	QString cloudName_target = parameters["target"].toString();
	Cloud *cloud_target = cloudManager->getCloud(cloudName_target);
	vtkSmartPointer<vtkPolyData> polyData_target = generateVTKPolyData(cloud_target->getCloudData(), cloud_target->getPolygons());

	QStringList cloudNameList = cloudBrowser->getSelectedCloudNames();
	QList<bool> isVisibleList = cloudBrowser->getSelectedCloudIsVisible();

	QStringList::Iterator it_name = cloudNameList.begin();
	QList<bool>::Iterator it_visible = isVisibleList.begin();
	while (it_name != cloudNameList.end())
	{
		QString cloudName = (*it_name);
		Cloud *cloud = cloudManager->getCloud(cloudName);

		QApplication::setOverrideCursor(Qt::WaitCursor);
		CloudDataPtr cloudData_filtered(new CloudData);
		hausdorffDistance(*cloud->getCloudData(), polyData_target, *cloudData_filtered);
		cloud->setCloudData(cloudData_filtered);
		QApplication::restoreOverrideCursor();
		QApplication::beep();

		cloudBrowser->updateCloud(cloud);
		bool isVisible = (*it_visible);
		if(isVisible)cloudVisualizer->updateCloud(cloud);

		it_name++;
		it_visible++;
	}


}

void MainWindow::on_pairwiseRegistrationDialog_sendParameters(QVariantMap parameters)
{
	//qDebug() << "Pairwise Registration Start!";

	QString cloudName_target = parameters["target"].toString();
	QString cloudName_source = parameters["source"].toString();

	QString prName = PairwiseRegistration::generateName(cloudName_target, cloudName_source);
	PairwiseRegistration *pairwiseRegistration = pairwiseRegistrationManager->getPairwiseRegistration(prName);

	if (parameters["command"] == "ShowResults")
	{
		if (pairwiseRegistration != NULL)
		{
			pairwiseRegistrationDialog->showResults(
				pairwiseRegistration->getTransformation(), 
				pairwiseRegistration->getRMSError(),
				pairwiseRegistration->getSquareErrors().size());
		}
	}

	if(parameters["command"] == "Initialize")
	{
		if(pairwiseRegistration == NULL)
		{
			RegistrationData *registrationData_target = registrationDataManager->getRegistrationData(cloudName_target);
			if ( registrationData_target == NULL)
			{
				Cloud *cloud_target = cloudManager->getCloud(cloudName_target);
				registrationData_target = registrationDataManager->addRegistrationData(cloud_target, cloudName_target);
			}
			RegistrationData *registrationData_source = registrationDataManager->getRegistrationData(cloudName_source);
			if ( registrationData_source == NULL)
			{
				Cloud *cloud_source = cloudManager->getCloud(cloudName_source);
				registrationData_source = registrationDataManager->addRegistrationData(cloud_source, cloudName_source);
			}
			//pairwiseRegistration = pairwiseRegistrationManager->addPairwiseRegistration(registrationData_target, registrationData_source, prName);
			pairwiseRegistration = new PairwiseRegistrationInteractor(registrationData_target, registrationData_source, prName);
			pairwiseRegistrationManager->addPairwiseRegistration(pairwiseRegistration);

			CloudVisualizer * cloudVisualizer = pairwiseRegistrationDialog->addCloudVisualizerTab(pairwiseRegistration->objectName());
			dynamic_cast<PairwiseRegistrationInteractor*>(pairwiseRegistration)->setCloudVisualizer(cloudVisualizer);

			if(cloudVisualizer) 
			{
				cloudVisualizer->addCloud(registrationData_target->cloudData, "target", 0, 0, 255);
				cloudVisualizer->getVisualizer()->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
				cloudVisualizer->resetCamera(registrationData_target->cloudData);
			}
			if(cloudVisualizer) 
			{
				cloudVisualizer->addCloud(registrationData_source->cloudData, "source", 255, 0, 0);
				cloudVisualizer->getVisualizer()->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source");
				cloudVisualizer->resetCamera(registrationData_target->cloudData);
			}

			pairwiseRegistrationDialog->showResults(
				pairwiseRegistration->getTransformation(), 
				pairwiseRegistration->getRMSError(),
				pairwiseRegistration->getSquareErrors().size());
			//pairwiseRegistration->cloudVisualizer->repaint();
		}
		else 
		{
			pairwiseRegistration->initialize();
			pairwiseRegistrationDialog->showResults(
				pairwiseRegistration->getTransformation(), 
				pairwiseRegistration->getRMSError(),
				pairwiseRegistration->getSquareErrors().size());
		}	
	}

	if(parameters["command"] == "Pre-Correspondences")
	{
		if (pairwiseRegistration == NULL)
		{
			qDebug() << "Still Uninitialized";
		}
		else 
		{
			parameters["isShowBoundaries"] = true;
			parameters["isShowCorrespondences"] = true;
			QApplication::setOverrideCursor(Qt::WaitCursor);
			pairwiseRegistration->process(parameters);
			QApplication::restoreOverrideCursor();
			QApplication::beep();
			pairwiseRegistrationDialog->showResults(
				pairwiseRegistration->getTransformation(), 
				pairwiseRegistration->getRMSError(), 
				pairwiseRegistration->getSquareErrors().size());
		}
	}

	if(parameters["command"] == "ICP")
	{
		if (pairwiseRegistration == NULL)
		{
			qDebug() << "Still Uninitialized";
		}
		else
		{
			parameters["isShowBoundaries"] = false;
			parameters["isShowCorrespondences"] = false;
			QApplication::setOverrideCursor(Qt::WaitCursor);			
			pairwiseRegistration->process(parameters);
			QApplication::restoreOverrideCursor();
			QApplication::beep();
			pairwiseRegistrationDialog->showResults(
				pairwiseRegistration->getTransformation(), 
				pairwiseRegistration->getRMSError(),
				pairwiseRegistration->getSquareErrors().size());

		}
	}

	if(parameters["command"] == "Export")
	{
		if (pairwiseRegistration == NULL)
		{
			qDebug() << "Still Uninitialized";
		}
		else 
		{
			pairwiseRegistration->process(parameters);
			Cloud *cloud_source= cloudManager->getCloud(cloudName_source);
			cloudVisualizer->updateCloud(cloud_source);
			pairwiseRegistrationDialog->showResults(
				pairwiseRegistration->getTransformation(), 
				pairwiseRegistration->getRMSError(),
				pairwiseRegistration->getSquareErrors().size());
		}
	}

	if(parameters["command"] == "Manual")
	{
		if (pairwiseRegistration == NULL)
		{
			qDebug() << "Still Uninitialized";
		}
		else 
		{
			if (manualRegistration == NULL)
			{
				manualRegistration = new ManualRegistration(this);
				connect(manualRegistration, SIGNAL(sendParameters(QVariantMap)), 
					this, SLOT(on_pairwiseRegistrationDialog_sendParameters(QVariantMap)));
			}
			manualRegistration->setWindowTitle(cloudName_source + "->" + cloudName_target);
			manualRegistration->setSrcCloud(pairwiseRegistration->getSource()->cloudData, cloudName_source);
			manualRegistration->setDstCloud(pairwiseRegistration->getTarget()->cloudData, cloudName_target);
			manualRegistration->clearSrcVis();
			manualRegistration->clearDstVis();
			manualRegistration->showSrcCloud();
			manualRegistration->showDstCloud();
			manualRegistration->show();
			manualRegistration->raise();
			manualRegistration->activateWindow();
		}
	}

	if (parameters["command"] == "ManualTransform")
	{
		if (pairwiseRegistration == NULL)
		{
			qDebug() << "Still Uninitialized";
		}
		else 
		{
			qDebug() << "OK";
			pairwiseRegistration->initializeTransformation(parameters["transformation"].value<Eigen::Matrix4f>());
			pairwiseRegistrationDialog->showResults(
				pairwiseRegistration->getTransformation(), 
				pairwiseRegistration->getRMSError(),
				pairwiseRegistration->getSquareErrors().size());
		}
	}
}

void MainWindow::on_globalRegistrationDialog_sendParameters(QVariantMap parameters)
{
	if (parameters["command"] == "InitializePair")
	{
		//qDebug() << parameters["targets"].toStringList();
		//qDebug() << parameters["sources"].toStringList();

		QStringList targets = parameters["targets"].toStringList();
		QStringList sources = parameters["sources"].toStringList();

		for (int i = 0; i < targets.size(); ++i)
		{
			QString cloudName_target = targets[i];
			QString cloudName_source = sources[i];
			QString prName = PairwiseRegistration::generateName(cloudName_target, cloudName_source);
			PairwiseRegistration *pairwiseRegistration = pairwiseRegistrationManager->getPairwiseRegistration(prName);
			if(pairwiseRegistration == NULL)
			{
				RegistrationData *registrationData_target = registrationDataManager->getRegistrationData(cloudName_target);
				if ( registrationData_target == NULL)
				{
					Cloud *cloud_target = cloudManager->getCloud(cloudName_target);
					registrationData_target = registrationDataManager->addRegistrationData(cloud_target, cloudName_target);
				}
				RegistrationData *registrationData_source = registrationDataManager->getRegistrationData(cloudName_source);
				if ( registrationData_source == NULL)
				{
					Cloud *cloud_source = cloudManager->getCloud(cloudName_source);
					registrationData_source = registrationDataManager->addRegistrationData(cloud_source, cloudName_source);
				}
				//pairwiseRegistration = pairwiseRegistrationManager->addPairwiseRegistration(registrationData_target, registrationData_source, prName);
				pairwiseRegistration = new PairwiseRegistrationInteractor(registrationData_target, registrationData_source, prName);
				pairwiseRegistrationManager->addPairwiseRegistration(pairwiseRegistration);

				CloudVisualizer * cloudVisualizer = pairwiseRegistrationDialog->addCloudVisualizerTab(pairwiseRegistration->objectName());
				dynamic_cast<PairwiseRegistrationInteractor*>(pairwiseRegistration)->setCloudVisualizer(cloudVisualizer);

				if(cloudVisualizer) 
				{
					cloudVisualizer->addCloud(registrationData_target->cloudData, "target", 0, 0, 255);
					cloudVisualizer->getVisualizer()->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
					cloudVisualizer->resetCamera(registrationData_target->cloudData);
				}
				if(cloudVisualizer) 
				{
					cloudVisualizer->addCloud(registrationData_source->cloudData, "source", 255, 0, 0);
					cloudVisualizer->getVisualizer()->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source");
					cloudVisualizer->resetCamera(registrationData_target->cloudData);
				}

				pairwiseRegistrationDialog->showResults(
					pairwiseRegistration->getTransformation(), 
					pairwiseRegistration->getRMSError(),
					pairwiseRegistration->getSquareErrors().size());
				//pairwiseRegistration->cloudVisualizer->repaint();
			}
			else
			{
				pairwiseRegistration->initialize();
				pairwiseRegistrationDialog->showResults(
					pairwiseRegistration->getTransformation(), 
					pairwiseRegistration->getRMSError(),
					pairwiseRegistration->getSquareErrors().size());
			}
		}
	}

	if (parameters["command"] == "InitializeCycle")
	{
		QStringList targets = parameters["targets"].toStringList();
		QStringList sources = parameters["sources"].toStringList();
		QList<PairwiseRegistration*> prList;

		// qDebug() << "InitializeCycle";
		// qDebug() << targets;
		// qDebug() << sources;

		for (int i = 0; i < targets.size(); ++i)
		{
			QString cloudName_target = targets[i];
			QString cloudName_source = sources[i];
			QString prName = PairwiseRegistration::generateName(cloudName_target, cloudName_source);
			PairwiseRegistration *pairwiseRegistration = pairwiseRegistrationManager->getPairwiseRegistration(prName);
			if(pairwiseRegistration == NULL)
			{
				RegistrationData *registrationData_target = registrationDataManager->getRegistrationData(cloudName_target);
				if ( registrationData_target == NULL)
				{
					Cloud *cloud_target = cloudManager->getCloud(cloudName_target);
					registrationData_target = registrationDataManager->addRegistrationData(cloud_target, cloudName_target);
				}
				RegistrationData *registrationData_source = registrationDataManager->getRegistrationData(cloudName_source);
				if ( registrationData_source == NULL)
				{
					Cloud *cloud_source = cloudManager->getCloud(cloudName_source);
					registrationData_source = registrationDataManager->addRegistrationData(cloud_source, cloudName_source);
				}
				//pairwiseRegistration = pairwiseRegistrationManager->addPairwiseRegistration(registrationData_target, registrationData_source, prName);
				pairwiseRegistration = new PairwiseRegistrationInteractor(registrationData_target, registrationData_source, prName);
				pairwiseRegistrationManager->addPairwiseRegistration(pairwiseRegistration);
				
				CloudVisualizer * cloudVisualizer = pairwiseRegistrationDialog->addCloudVisualizerTab(pairwiseRegistration->objectName());
				dynamic_cast<PairwiseRegistrationInteractor*>(pairwiseRegistration)->setCloudVisualizer(cloudVisualizer);

				if(cloudVisualizer) cloudVisualizer->addCloud(registrationData_target->cloudData, "target", 0, 0, 255);
				if(cloudVisualizer) cloudVisualizer->addCloud(registrationData_source->cloudData, "source", 255, 0, 0);

				pairwiseRegistrationDialog->showResults(
					pairwiseRegistration->getTransformation(), 
					pairwiseRegistration->getRMSError(),
					pairwiseRegistration->getSquareErrors().size());
				//pairwiseRegistration->cloudVisualizer->repaint();
			}
			else
			{
				pairwiseRegistration->initialize();
				pairwiseRegistrationDialog->showResults(
					pairwiseRegistration->getTransformation(), 
					pairwiseRegistration->getRMSError(),
					pairwiseRegistration->getSquareErrors().size());
			}
			prList.append(pairwiseRegistration);
		}

		cycleRegistrationManager->addCycleRegistration(prList);
	}

	if (parameters["command"] == "UniformRefine")
	{
		//qDebug() << "make circle consistent";

		QStringList targets = parameters["targets"].toStringList();
		QStringList sources = parameters["sources"].toStringList();
		QList<bool> freezeds = parameters["freezeds"].value< QList<bool> >();

		QString cloudNameCycle = "";
		for (int i = 0; i < targets.size()-1; ++i)
		{
			cloudNameCycle.append(targets[i]).append(",");
		}
		cloudNameCycle.append(targets.back());
		//qDebug() << cloudNameCycle;

		CycleRegistration* cycleRegistration = cycleRegistrationManager->getCycleRegistration(cloudNameCycle);
		if (cycleRegistration == NULL)
		{
			qDebug() << "Still Uninitialized";
		}
		else
		{
			cycleRegistration->uniform_refine();
		}
	}

	if (parameters["command"] == "ShowEstimation")
	{
		QStringList targets = parameters["targets"].toStringList();
		QStringList sources = parameters["sources"].toStringList();
		QList<Eigen::Matrix4f> transformationList;
		for (int i = 0; i < targets.size(); ++i)
		{
			QString cloudName_target = targets[i];
			QString cloudName_source = sources[i];
			QString prName = PairwiseRegistration::generateName(cloudName_target, cloudName_source);
			PairwiseRegistration *pairwiseRegistration = pairwiseRegistrationManager->getPairwiseRegistration(prName);
			QString reversePrName = PairwiseRegistration::generateName(cloudName_source, cloudName_target);
			PairwiseRegistration *reversePairwiseRegistration = pairwiseRegistrationManager->getPairwiseRegistration(reversePrName);
			if(pairwiseRegistration == NULL && reversePairwiseRegistration == NULL)
			{
				qDebug() << cloudName_target << "<-" << cloudName_source << " not ready yet!!!";
				return;				
			}
			if(pairwiseRegistration) transformationList.append(pairwiseRegistration->getTransformation());
			else transformationList.append(reversePairwiseRegistration->getTransformation().inverse());
		}
		Eigen::Matrix4f transformation_total = Eigen::Matrix4f::Identity();
		for (int i = 0; i < transformationList.size(); ++i) transformation_total *= transformationList[i];
		
		QString prName = PairwiseRegistration::generateName(targets.back(), sources.back());
		PairwiseRegistration *pairwiseRegistration = pairwiseRegistrationManager->getPairwiseRegistration(prName);
		QString reversePrName = PairwiseRegistration::generateName(sources.back(), targets.back());
		PairwiseRegistration *reversePairwiseRegistration = pairwiseRegistrationManager->getPairwiseRegistration(reversePrName);
		float error1 = 0.0f, error2 = 0.0f;
		int ovlNumber1 = 0, ovlNumber2 = 0;

		if(pairwiseRegistration) 
		{
			//std::cerr << (transformation_total * transformationList.back().inverse()).inverse() << std::endl;
			pairwiseRegistration->estimateRMSErrorByTransformation((transformation_total * transformationList.back().inverse()).inverse(), error1, ovlNumber1);
			pairwiseRegistration->estimateVirtualRMSErrorByTransformation((transformation_total * transformationList.back().inverse()).inverse(), error2, ovlNumber2);
		}
		else
		{
			reversePairwiseRegistration->estimateRMSErrorByTransformation(transformation_total * transformationList.back().inverse(), error1, ovlNumber1);
			reversePairwiseRegistration->estimateVirtualRMSErrorByTransformation(transformation_total * transformationList.back().inverse(), error2, ovlNumber2);
		} 
		globalRegistrationDialog->showEstimation(transformation_total, error1, error2, ovlNumber1, ovlNumber2);

	}

	if(parameters["command"] == "Export")
	{
		QString relations = parameters["relations"].toString();
		QStringList relationList = relations.split("\n");

		for (int i = 0; i < relationList.size(); ++i)
		{
			if (relationList[i] != "")
			{
				QStringList cloudList = relationList[i].split("<-");

				Cloud *cloud_begin= cloudManager->getCloud(cloudList[0]);
				cloud_begin->setRegistrationTransformation(cloud_begin->getTransformation());
				cloudVisualizer->updateCloud(cloud_begin);

				Eigen::Matrix4f transformation_temp = Eigen::Matrix4f::Identity();
				for (int j = 1; j < cloudList.size(); ++j)
				{
					QString cloudName_target = cloudList[j-1];
					QString cloudName_source = cloudList[j];
					qDebug() << cloudName_target << "<-" << cloudName_source;

					QString prName = PairwiseRegistration::generateName(cloudName_target, cloudName_source);
					PairwiseRegistration *pairwiseRegistration = pairwiseRegistrationManager->getPairwiseRegistration(prName);
					QString reversePrName = PairwiseRegistration::generateName(cloudName_source, cloudName_target);
					PairwiseRegistration *reversePairwiseRegistration = pairwiseRegistrationManager->getPairwiseRegistration(reversePrName);

					if(pairwiseRegistration == NULL && reversePairwiseRegistration == NULL)
					{
						qDebug() << cloudName_target << "<-" << cloudName_source << " not ready yet!!!";
						return;				
					}
					if(pairwiseRegistration) transformation_temp *= pairwiseRegistration->getTransformation();
					else transformation_temp *= reversePairwiseRegistration->getTransformation().inverse();

					Cloud *cloud_source = cloudManager->getCloud(cloudName_source);
					cloud_source->setRegistrationTransformation( transformation_temp * cloud_source->getTransformation());
					cloudVisualizer->updateCloud(cloud_source);
				}
			}

		}
	}
}


