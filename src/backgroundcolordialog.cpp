#include <QtCore/QDebug>

#include "../include/backgroundcolordialog.h"

BackgroundColorDialog::BackgroundColorDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

BackgroundColorDialog::~BackgroundColorDialog(){}

void BackgroundColorDialog::on_applyButton_clicked()
{
	float R, G, B;
	R = RLineEdit->text().toFloat();
	G = GLineEdit->text().toFloat();
	B = BLineEdit->text().toFloat();

	QVariantMap parameters;
	parameters["R"] = R;
	parameters["G"] = G;
	parameters["B"] = B;

	emit sendParameters(parameters);
}


