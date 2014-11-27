#include <QtCore/QDebug>

#include "../include/depthcameradialog.h"

DepthCameraDialog::DepthCameraDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

DepthCameraDialog::~DepthCameraDialog(){}

void DepthCameraDialog::on_applyButton_clicked()
{
	int method = methodComboBox->currentIndex();
	bool camera_coordinate = cameraCoorRadioButton->isChecked();
	int xres, yres;
	xres = xresLineEdit->text().toInt();
	yres = yresLineEdit->text().toInt();
	float view_angle = viewAngleLineEdit->text().toFloat();
	float noise_std = noiseSTDLineEdit->text().toFloat();
	int tesselation_level = tesselationLevelLineEdit->text().toInt();
	float radius_sphere = radiusSphereLineEdit->text().toFloat();
	bool use_vertices = useVerticesCheckBox->isChecked();

	QVariantMap parameters;
	parameters["method"] = method;
	parameters["camera_coordinate"] = camera_coordinate;
	parameters["xres"] = xres;
	parameters["yres"] = yres;
	parameters["view_angle"] = view_angle;
	parameters["noise_std"] = noise_std;
	parameters["tesselation_level"] = tesselation_level;
	parameters["radius_sphere"] = radius_sphere;
	parameters["use_vertices"] = use_vertices;

	emit sendParameters(parameters);
}