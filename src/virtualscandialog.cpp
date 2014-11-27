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
	bool camera_coordinate = cameraCoorRadioButton->isChecked();
	int nr_scans = nrScansLineEdit->text().toInt();             
	int nr_points_in_scans = nrPointsInScansLineEdit->text().toInt();   
	float vert_res = nrPointsInScansLineEdit->text().toFloat(); 
	float hor_res = nrPointsInScansLineEdit->text().toFloat();
	float max_dist = nrPointsInScansLineEdit->text().toFloat();
	float view_angle = viewAngleLineEdit->text().toFloat();
	bool use_vertices = useVerticesCheckBox->isChecked();

	QVariantMap parameters;
	parameters["target"] = targetComboBox->currentText();
	parameters["method"] = method;
	parameters["camera_coordinate"] = camera_coordinate;
	parameters["nr_scans"] = nr_scans;
	parameters["nr_points_in_scans"] = nr_points_in_scans;
	parameters["max_dist"] = max_dist;
	parameters["view_angle"] = view_angle;
	parameters["use_vertices"] = use_vertices;

	emit sendParameters(parameters);
}