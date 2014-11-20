#include <QtCore/QDebug>

#include "../include/virtualscandialog.h"

VirtualScanDialog::VirtualScanDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

VirtualScanDialog::~VirtualScanDialog(){}

void VirtualScanDialog::on_applyButton_clicked()
{
	int method = methodComboBox->currentIndex();
	int xres, yres;
	xres = xresLineEdit->text().toInt();
	yres = yresLineEdit->text().toInt();
	float view_angle = viewAngleLineEdit->text().toFloat();
	bool use_vertices = useVerticesCheckBox->isChecked();

	QVariantMap parameters;
	parameters["method"] = method;
	parameters["xres"] = xres;
	parameters["yres"] = yres;
	parameters["view_angle"] = view_angle;
	parameters["use_vertices"] = use_vertices;

	emit sendParameters(parameters);
}