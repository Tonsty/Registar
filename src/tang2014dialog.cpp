#include <QtCore/QDebug>

#include "../include/tang2014dialog.h"

Tang2014Dialog::Tang2014Dialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	//setFixedHeight(sizeHint().height());
}

Tang2014Dialog::~Tang2014Dialog(){}

void Tang2014Dialog::on_applyButton_clicked()
{
	
	QString links, loops, arguments;
	links = linksTextEdit->toPlainText();
	loops = loopsTextEdit->toPlainText();
	arguments = argumentsTextEdit->toPlainText();

	QVariantMap parameters;
	parameters["links"] = links;
	parameters["loops"] = loops;
	parameters["arguments"] = arguments;

	emit sendParameters(parameters);
}


