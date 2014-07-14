#include <QtCore/QDebug>

#include "../include/euclideanclusterextractiondialog.h"

EuclideanClusterExtractionDialog::EuclideanClusterExtractionDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

EuclideanClusterExtractionDialog::~EuclideanClusterExtractionDialog(){}

void EuclideanClusterExtractionDialog::on_applyButton_clicked()
{
	float clusterTolerance = clusterToleranceLineEdit->text().toFloat();
	int minClusterSize = minClusterSizeLineEdit->text().toInt();
	int maxClusterSize = maxClusterSizeLineEdit->text().toInt();
	bool overwrite = overwriteCheckBox->isChecked();
	bool use_cpu = cpuRadioButton->isChecked();
	bool use_gpu = gpuRadioButton->isChecked();
	
	QVariantMap parameters;
	parameters["clusterTolerance"] = clusterTolerance;
	parameters["minClusterSize"] = minClusterSize;
	parameters["maxClusterSize"] = maxClusterSize;
	parameters["overwrite"] = overwrite;

	parameters["use_cpu"] = use_cpu;
	parameters["use_gpu"] = use_gpu;
	
	emit sendParameters(parameters);
}