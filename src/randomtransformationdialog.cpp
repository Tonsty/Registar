#include <QtCore/QDebug>

#include "../include/randomtransformationdialog.h"

RandomTransformationDialog::RandomTransformationDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

RandomTransformationDialog::~RandomTransformationDialog(){}

void RandomTransformationDialog::on_applyButton_clicked()
{
	float max_angle = maxAngleLineEdit->text().toFloat();
	float max_distance = maxDistanceLineEdit->text().toFloat();

	QVariantMap parameters;
	parameters["max_angle"] = max_angle;
	parameters["max_distance"] = max_distance;

	emit sendParameters(parameters);
}


