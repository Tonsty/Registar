#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>

#include "../build/ui/ui_MainWindow.h"

class CloudManager;
class CloudVisualizer;
class EuclideanClusterExtractionDialog;
class VoxelGridDialog;
class MovingLeastSquaresDialog;
class BoundaryEstimationDialog;
class OutliersRemovalDialog;
class NormalFieldDialog;
class PairwiseRegistrationDialog;
class PairwiseRegistrationManager;

class DiagramWindow;
class GlobalRegistrationDialog;
class CycleRegistrationManager;

class ManualRegistration;


class MainWindow : public QMainWindow, public Ui_MainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = 0);
	virtual ~MainWindow();
	
protected:
	void changeEvent(QEvent *event);
	void closeEvent(QCloseEvent *event);

private:
	void registerMetaType();
	void readSettings();
	void writeSettings();
	bool okToContinue();
	QString strippedName(const QString &fullFileName);

	CloudManager *cloudManager;
	CloudVisualizer *cloudVisualizer;
	EuclideanClusterExtractionDialog *euclideanClusterExtractionDialog;
	VoxelGridDialog *voxelGridDialog;
	MovingLeastSquaresDialog *movingLeastSquaresDialog;
	BoundaryEstimationDialog *boundaryEstimationDialog;
	OutliersRemovalDialog *outliersRemovalDialog;
	NormalFieldDialog *normalFieldDialog;
	PairwiseRegistrationDialog *pairwiseRegistrationDialog;
	PairwiseRegistrationManager *pairwiseRegistrationManager;

	DiagramWindow *diagramWindow;
	GlobalRegistrationDialog *globalRegistrationDialog;
	CycleRegistrationManager *cycleRegistrationManager;

	ManualRegistration *manualRegistration;

private slots:
	void on_aboutAction_triggered();
	void on_openAction_triggered();
	bool on_saveAction_triggered();
	bool on_saveAsAction_triggered();

	void on_showCloudBrowserAction_toggled(bool isChecked);
	void on_showPointBrowserAction_toggled(bool isChecked);

	void on_cloudBrowser_cloudVisibleStateChanged(QString cloudName, bool isVisible);
	
	void on_colorNoneAction_toggled(bool isChecked);
	void on_colorOriginalAction_toggled(bool isChecked);
	void on_colorCustomAction_toggled(bool isChecked);
	void on_drawNormalAction_toggled(bool isChecked);
	void on_drawAxisAction_toggled(bool isChecked);
	void on_drawOrientationMarkerAction_toggled(bool isChecked);
	void on_registrationModeAction_toggled(bool isChecked);
	void on_drawBoundaryAction_toggled(bool isChecked);

	void on_euclideanClusterExtractionAction_triggered();
	void on_voxelGridAction_triggered();
	void on_movingLeastSquaresAction_triggered();
	void on_boundaryEstimationAction_triggered();
	void on_outliersRemovalAction_triggered();

	void on_pairwiseRegistrationAction_triggered();

	void on_diagramAction_triggered();
	void on_globalRegistrationAction_triggered();
	void on_confirmRegistrationAction_triggered();
	void on_forceRigidRegistrationAction_triggered();

	void on_euclideanClusterExtractionDialog_sendParameters(QVariantMap parameters);
	void on_voxelGridDialog_sendParameters(QVariantMap parameters);
	void on_movingLeastSquaresDialog_sendParameters(QVariantMap parameters);
	void on_boundaryEstimationDialog_sendParameters(QVariantMap parameters);
	void on_outliersRemovalDialog_sendParameters(QVariantMap parameters);

	void on_normalFieldAction_triggered();
	void on_normalFieldDialog_sendParameters(QVariantMap parameters);

	void on_concatenationAction_triggered();

	void on_pairwiseRegistrationDialog_sendParameters(QVariantMap parameters);
	void on_globalRegistrationDialog_sendParameters(QVariantMap parameters);
};

#endif 
