#include <QtCore/QDebug>

#include "../include/hausdorffdistancedialog.h"

HausdorffDistanceDialog::HausdorffDistanceDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

HausdorffDistanceDialog::~HausdorffDistanceDialog(){}

void HausdorffDistanceDialog::on_applyButton_clicked()
{
	QVariantMap parameters;
	parameters["target"] = targetComboBox->currentText();

	emit sendParameters(parameters);
}