#include <QtCore/QDebug>

#include "../include/boundaryestimationdialog.h"

BoundaryEstimationDialog::BoundaryEstimationDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

BoundaryEstimationDialog::~BoundaryEstimationDialog(){}

void BoundaryEstimationDialog::on_applyButton_clicked()
{
	float searchRadius = radiusLineEdit->text().toFloat();
	float angleThreshold = angleLineEdit->text().toFloat(); 
	float dilationRadius = dilationLineEdit->text().toFloat();
	bool overwrite = overwriteCheckBox->isChecked();
	
	QVariantMap parameters;
	parameters["searchRadius"] = searchRadius;	
	parameters["angleThreshold"] = angleThreshold;
	parameters["dilationRadius"] = dilationRadius;
	parameters["overwrite"] = overwrite;
	
	emit sendParameters(parameters);
}