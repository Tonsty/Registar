#include <QtCore/QDebug>

#include "../include/movingleastsquaresdialog.h"

MovingLeastSquaresDialog::MovingLeastSquaresDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

MovingLeastSquaresDialog::~MovingLeastSquaresDialog(){}

void MovingLeastSquaresDialog::on_applyButton_clicked()
{
	QVariantMap parameters;
	parameters["method"] = methodComboBox->currentIndex();
	parameters["searchRadius"] = radiusLineEdit->text().toFloat();

	//Wait to develop
	parameters["distinctCloudName"] = "waitForDevelop";
	
	parameters["upsamplingRadius"] = upsamplingRadiusLineEdit->text().toFloat();
	parameters["stepSize"] = stepSizeLineEdit->text().toFloat();
	parameters["density"] = densitySpinBox->value();
	parameters["voxelSize"] = voxelSizeLineEdit->text().toFloat();
	parameters["dilationIterations"] = iterationsSpinBox->value();
	parameters["computeNormal"] = computeNormalCheckBox->isChecked();
	parameters["overwrite"] = overwriteCheckBox->isChecked();

	parameters["use_scpu"] = scpuRadioButton->isChecked();;
	parameters["use_mcpu"] = mcpuRadioButton->isChecked();;	
	parameters["use_gpu"] = gpuRadioButton->isChecked();;

	emit sendParameters(parameters);
}


