#ifndef PAIRWISEREGISTRATIONDIALOG_H
#define PAIRWISEREGISTRATIONDIALOG_H

#include <QtGui/QDialog>
#include <Eigen/Eigen>

#include "pclbase.h"
#include "ui_PairwiseRegistrationDialog.h"

class CloudVisualizer;

class PairwiseRegistrationDialog : public QDialog, public Ui_PairwiseRegistrationDialog
{
	Q_OBJECT

public:
	PairwiseRegistrationDialog(QWidget *parent = 0);
	virtual ~PairwiseRegistrationDialog();

	CloudVisualizer* addCloudVisualizerTab(QString targetBySource);
	void showResults(Eigen::Matrix4f transformation, float rmsError, int corrNumber);

public slots:
	void on_tabWidget_currentChanged(int index);
	void on_prePushButton_clicked();

signals:
	void sendParameters(QVariantMap parameters);

private slots:
	void on_initializePushButton_clicked();
	//void on_prePushButton_clicked();
	void on_icpPushButton_clicked();
	void on_exportPushButton_clicked();
	void on_manualPushButton_clicked();

	void on_targetComboBox_currentIndexChanged(const QString &cloudName_target);
	void on_sourceComboBox_currentIndexChanged(const QString &cloudName_source);
};

#endif
