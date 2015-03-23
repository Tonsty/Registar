#include <QtCore/QDebug>

#include "../include/generateoutliersdialog.h"

GenerateOutliersDialog::GenerateOutliersDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

GenerateOutliersDialog::~GenerateOutliersDialog(){}

void GenerateOutliersDialog::on_applyButton_clicked()
{
	float xmin = xMinLineEdit->text().toFloat();
	float xmax = xMaxLineEdit->text().toFloat();
	float ymin = yMinLineEdit->text().toFloat();
	float ymax = yMaxLineEdit->text().toFloat();
	float zmin = zMinLineEdit->text().toFloat();
	float zmax = zMaxLineEdit->text().toFloat();

	int numofpoints = numOfPointsLineEdit->text().toInt();

	bool overwrite = overwriteCheckBox->isChecked();
	float noise_std = noiseSTDLineEdit->text().toFloat();
	int method = methodComboBox->currentIndex();

	QVariantMap parameters;
	parameters["xmin"] = xmin;
	parameters["xmax"] = xmax;
	parameters["ymin"] = ymin;
	parameters["ymax"] = ymax;
	parameters["zmin"] = zmin;
	parameters["zmax"] = zmax;

	parameters["numofpoints"] = numofpoints;

	parameters["overwrite"] = overwrite;
	parameters["noise_std"] = noise_std;
	parameters["method"] = method;

	emit sendParameters(parameters);
}

void GenerateOutliersDialog::on_boundingBoxButton_clicked()
{
	emit boundingBox();
}