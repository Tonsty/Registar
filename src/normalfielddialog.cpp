#include <QtCore/QDebug>

#include "../include/normalfielddialog.h"

NormalFieldDialog::NormalFieldDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

NormalFieldDialog::~NormalFieldDialog(){}

void NormalFieldDialog::on_applyButton_clicked()
{
	float X, Y, Z;
	X = XLineEdit->text().toFloat();
	Y = YLineEdit->text().toFloat();
	Z = ZLineEdit->text().toFloat();
	bool overwrite = overwriteCheckBox->isChecked();

	QVariantMap parameters;
	parameters["X"] = X;
	parameters["Y"] = Y;
	parameters["Z"] = Z;
	parameters["overwrite"] = overwrite;

	emit sendParameters(parameters);
}


