#include <QtCore/QDebug>

#include "../include/addnoisedialog.h"

AddNoiseDialog::AddNoiseDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

AddNoiseDialog::~AddNoiseDialog(){}

void AddNoiseDialog::on_applyButton_clicked()
{
	float X, Y, Z;
	X = XLineEdit->text().toFloat();
	Y = YLineEdit->text().toFloat();
	Z = ZLineEdit->text().toFloat();
	bool overwrite = overwriteCheckBox->isChecked();
	float noise_std = noiseSTDLineEdit->text().toFloat();
	int method = methodComboBox->currentIndex();

	QVariantMap parameters;
	parameters["X"] = X;
	parameters["Y"] = Y;
	parameters["Z"] = Z;
	parameters["overwrite"] = overwrite;
	parameters["noise_std"] = noise_std;

	emit sendParameters(parameters);
}


