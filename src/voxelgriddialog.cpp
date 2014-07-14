#include <QtCore/QDebug>

#include "../include/voxelgriddialog.h"

VoxelGridDialog::VoxelGridDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

VoxelGridDialog::~VoxelGridDialog(){}

void VoxelGridDialog::on_applyButton_clicked()
{
	float leafSizeX, leafSizeY, leafSizeZ;
	leafSizeX = leafSizeXLineEdit->text().toFloat();
	leafSizeY = leafSizeYLineEdit->text().toFloat();
	leafSizeZ = leafSizeZLineEdit->text().toFloat();
	bool overwrite = overwriteCheckBox->isChecked();

	QVariantMap parameters;
	parameters["leafSizeX"] = leafSizeX;
	parameters["leafSizeY"] = leafSizeY;
	parameters["leafSizeZ"] = leafSizeZ;
	parameters["overwrite"] = overwrite;

	emit sendParameters(parameters);
}


