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
	QString command = tabWidget->tabText(tabWidget->currentIndex());

	float xmin = xMinLineEdit->text().toFloat();
	float xmax = xMaxLineEdit->text().toFloat();
	float ymin = yMinLineEdit->text().toFloat();
	float ymax = yMaxLineEdit->text().toFloat();
	float zmin = zMinLineEdit->text().toFloat();
	float zmax = zMaxLineEdit->text().toFloat();

	int numofpoints = numOfPointsLineEdit->text().toInt();

	int method = methodComboBox->currentIndex();
	bool uniform = uniformRadioButton->isChecked();
	bool separate = separateRadioButton->isChecked();

	float percent = percentLineEdit->text().toFloat();
	float noise_std = noiseSTDLineEdit->text().toFloat();

	bool overwrite = overwriteCheckBox->isChecked();

	QVariantMap parameters;

	parameters["command"] = command;

	parameters["xmin"] = xmin;
	parameters["xmax"] = xmax;
	parameters["ymin"] = ymin;
	parameters["ymax"] = ymax;
	parameters["zmin"] = zmin;
	parameters["zmax"] = zmax;

	parameters["numofpoints"] = numofpoints;

	parameters["method"] = method;
	parameters["uniform"] = uniform;
	parameters["separate"] = separate;

	parameters["percent"] = percent;
	parameters["noise_std"] = noise_std;

	parameters["overwrite"] = overwrite;

	emit sendParameters(parameters);
}

void GenerateOutliersDialog::on_boundingBoxButton_clicked()
{
	emit boundingBox();
}