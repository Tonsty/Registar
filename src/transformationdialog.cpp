#include <QtCore/QDebug>
#include <iostream>

#include "../include/transformationdialog.h"

TransformationDialog::TransformationDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

TransformationDialog::~TransformationDialog(){}

void TransformationDialog::on_applyButton_clicked()
{
	QString command = tabWidget->tabText(tabWidget->currentIndex());

	float max_angle = maxAngleLineEdit->text().toFloat();
	float max_distance = maxDistanceLineEdit->text().toFloat();
	bool rotate_around_centroid = rotateAroundCentroidCheckBox->isChecked();

	bool send_centroid_origin = sendCentroidOriginCheckBox->isChecked();
	bool align_pca_xyz = alignPCAXYZCheckBox->isChecked();
	bool scale_unit = scaleUnitCheckBox->isChecked();

	bool uniform = uniformRadioButton->isChecked();
	bool separate = separateRadioButton->isChecked();

	QVariantMap parameters;
	parameters["command"] = command;

	parameters["max_angle"] = max_angle;
	parameters["max_distance"] = max_distance;
	parameters["rotate_around_centroid"] = rotate_around_centroid;

	parameters["send_centroid_origin"] = send_centroid_origin;
	parameters["align_pca_xyz"] = align_pca_xyz;
	parameters["scale_unit"] = scale_unit;

	parameters["uniform"] = uniform;
	parameters["separate"] = separate;

	emit sendParameters(parameters);
}


