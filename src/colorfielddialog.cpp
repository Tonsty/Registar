#include <QtCore/QDebug>

#include "../include/colorfielddialog.h"

ColorFieldDialog::ColorFieldDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

ColorFieldDialog::~ColorFieldDialog(){}

void ColorFieldDialog::on_applyButton_clicked()
{
	float R, G, B;
	R = RLineEdit->text().toFloat();
	G = GLineEdit->text().toFloat();
	B = BLineEdit->text().toFloat();
	bool overwrite = overwriteCheckBox->isChecked();

	QVariantMap parameters;
	parameters["R"] = R;
	parameters["G"] = G;
	parameters["B"] = B;
	parameters["overwrite"] = overwrite;

	float Hmin, Hmax, S, V;
	Hmin = HminLineEdit->text().toFloat();
	Hmax = HmaxLineEdit->text().toFloat();
	S = SLineEdit->text().toFloat();
	V = VLineEdit->text().toFloat();

	parameters["Hmin"] = Hmin;
	parameters["Hmax"] = Hmax;
	parameters["S"] = S;
	parameters["V"] = V;
	emit sendParameters(parameters);
}


