#include <QtCore/QDebug>

#include "../include/savecontentdialog.h"

SaveContentDialog::SaveContentDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	setFixedHeight(sizeHint().height());
}

SaveContentDialog::~SaveContentDialog(){}


