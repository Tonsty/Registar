#include <QtCore/QDebug>

#include "../include/outliersremovaldialog.h"

OutliersRemovalDialog::OutliersRemovalDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

OutliersRemovalDialog::~OutliersRemovalDialog(){}

void OutliersRemovalDialog::on_applyButton_clicked()
{
	QVariantMap parameters;
	parameters["method"] = methodComboBox->currentIndex();
	parameters["searchRadius"] = radiusDoubleSpinBox->value(); 
	parameters["nearestK"] = nearestKSpinBox->value();
	parameters["deviation"] = deviationDoubleSpinBox->value();
	parameters["overwrite"] = overwriteCheckBox->isChecked();

	emit sendParameters(parameters);
}


