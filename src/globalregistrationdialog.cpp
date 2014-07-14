#include <QtCore/QDebug>
#include <QtCore/qmath.h>
#include <QtGui/QErrorMessage>

#include "../include/qtbase.h"

#include "../include/globalregistrationdialog.h"

GlobalRegistrationDialog::GlobalRegistrationDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	//setFixedSize(sizeHint());
	errorMessage = 0;
}

GlobalRegistrationDialog::~GlobalRegistrationDialog(){}

void GlobalRegistrationDialog::on_addPushButton_clicked()
{
	QListWidgetItem *item = new QListWidgetItem("(new circle, e.g. 1,2,3,4)");
	item->setFlags(item->flags()|Qt::ItemIsEditable);
	circleListWidget->addItem(item);
}

void GlobalRegistrationDialog::on_deletePushButton_clicked()
{
	QList<QListWidgetItem*> selectedItems = circleListWidget->selectedItems();
	for (int i = 0; i < selectedItems.size(); ++i)
	{
		circleListWidget->removeItemWidget(selectedItems[i]);
		delete selectedItems[i];
	}
}

void GlobalRegistrationDialog::on_circleListWidget_itemChanged(QListWidgetItem *item)
{
	//qDebug() << "item changed";
	QRegExp regExp("(([0-9]+,)+)([0-9]+)");
	// if (regExp.isValid()) qDebug() << "valid";
	// else qDebug() << "invalid";
	if (regExp.exactMatch(item->text()))
	{
		//qDebug() << "matched";
		updatePairListWidget(item->text());
		updateCycleComboBox(item->text());
		//updateStaticComboBox(item->text());
	}
	else 
	{
		//qDebug() << "unmatched";
		if (!errorMessage)	errorMessage = new QErrorMessage(this);
		errorMessage->showMessage("invalid input");
		updatePairListWidget("");
		updateCycleComboBox("");
		//updateStaticComboBox("");
	}
}

void GlobalRegistrationDialog::on_circleListWidget_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous)
{
	if(current != NULL)
	{
		updatePairListWidget(current->text());
		updateCycleComboBox(current->text());
		//updateStaticComboBox(current->text());
	}
	else 
	{
		updatePairListWidget("");
		updateCycleComboBox("");
		//updateStaticComboBox("");
	}
}

void GlobalRegistrationDialog::updatePairListWidget(QString circle)
{
	while(pairListWidget->count() != 0)
	{
		delete pairListWidget->takeItem(pairListWidget->count()-1);
	}
	QRegExp regExp("(([0-9]+,)+)([0-9]+)");
	if (regExp.exactMatch(circle))
	{
		//qDebug() << "matched";
		QStringList singlelist = circle.split(",");
		QStringList pairlist = singlelist;
		for (int i = 0; i < singlelist.size(); ++i)
		{
			pairlist[i].append("<-").append(singlelist[(i+1)%singlelist.size()]);
		}
		for (int i = 0; i < pairlist.size(); ++i)
		{
			QListWidgetItem *item = new QListWidgetItem(pairlist[i]);
			item->setCheckState(Qt::Unchecked);
			pairListWidget->addItem(item);
		}

		//pairListWidget->addItems(pairlist);
	}
}

void GlobalRegistrationDialog::on_initializePairPushButton_clicked()
{
	QVariantMap parameters;
	parameters["command"] = QString("InitializePair");

	QList<QListWidgetItem*> selectedItems = pairListWidget->selectedItems();
	QStringList targets, sources;
	for (int i = 0; i < selectedItems.size(); ++i)
	{
		QStringList target_source = selectedItems[i]->text().split("<-");
		targets << target_source[0];
		sources << target_source[1];
	}
	parameters["targets"] = targets;
	parameters["sources"] = sources;

	emit sendParameters(parameters);
}

void GlobalRegistrationDialog::on_deinitializePairPushButton_clicked()
{
	
}

void GlobalRegistrationDialog::on_initializeCyclePushButton_clicked()
{
	QVariantMap parameters;
	parameters["command"] = QString("InitializeCycle");

	QStringList targets, sources;
	for (int i = 0; i < pairListWidget->count(); ++i)
	{
		QStringList target_source = pairListWidget->item(i)->text().split("<-");
		targets << target_source[0];
		sources << target_source[1];
	}
	parameters["targets"] = targets;
	parameters["sources"] = sources;

	emit sendParameters(parameters);
}

void GlobalRegistrationDialog::on_consistentPushButton_clicked()
{
	QVariantMap parameters;
	parameters["command"] = QString("Consistent");

	int offset = cycleComboBox->currentIndex();

	QStringList targets, sources;
	QList<bool> freezeds;
	for (int i = 0; i < pairListWidget->count(); ++i)
	{
		QStringList target_source = pairListWidget->item( ( i + offset ) % pairListWidget->count() )->text().split("<-");
		targets << target_source[0];
		sources << target_source[1];
		if (pairListWidget->item( ( i + offset ) % pairListWidget->count() )->checkState() == Qt::Unchecked)
		{
			freezeds.append(false);
		}
		else freezeds.append(true);
	}
	parameters["targets"] = targets;
	parameters["sources"] = sources;
	parameters["freezeds"].setValue( freezeds );
	parameters["offset"] = offset;

	//qDebug() << targets;
	//qDebug() << sources;

	emit sendParameters(parameters);
}

void GlobalRegistrationDialog::on_estimatePushButton_clicked()
{
	QVariantMap parameters;
	parameters["command"] = QString("ShowEstimation");

	int offset = cycleComboBox->currentIndex();

	QStringList targets, sources;
	QList<bool> freezeds;
	for (int i = 0; i < pairListWidget->count(); ++i)
	{
		QStringList target_source = pairListWidget->item( ( i + offset ) % pairListWidget->count() )->text().split("<-");
		targets << target_source[0];
		sources << target_source[1];
		if (pairListWidget->item( ( i + offset ) % pairListWidget->count() )->checkState() == Qt::Unchecked)
		{
			freezeds.append(false);
		}
		else freezeds.append(true);
	}
	parameters["targets"] = targets;
	parameters["sources"] = sources;
	parameters["freezeds"].setValue( freezeds );
	parameters["offset"] = offset;

	//qDebug() << targets;
	//qDebug() << sources;

	emit sendParameters(parameters);
}

void GlobalRegistrationDialog::showEstimation(Eigen::Matrix4f transformation_total, float error1, float error2, int ovlNumber1, int ovlNumber2)
{
	Eigen::Matrix4f transformation = transformation_total;

	std::stringstream temp;
	temp.str("");
	temp << transformation;
	transformationTextEdit->setText(QString::fromStdString(temp.str()));
	Eigen::Matrix3f rotation = transformation.block(0,0,3,3);
	float c = (rotation * rotation.transpose()).diagonal().mean();
	c = qSqrt(c);
	scaleLineEdit->setText(QString::number(c));
	rotation /= c;
	temp.str("");
	temp << rotation;
	rotationTextEdit->setText(QString::fromStdString(temp.str()));
	Eigen::Vector3f translation = transformation.block(0,3,3,1);
	temp.str("");
	temp << translation;
	translationLineEdit->setText(QString::fromStdString(temp.str()));
	Eigen::AngleAxisf angleAxis(rotation);
	angleLineEdit->setText( QString::number(angleAxis.angle()) );
	temp.str("");
	temp << angleAxis.axis();
	axisLineEdit->setText(QString::fromStdString(temp.str()));

	error1LineEdit->setText(QString::number(error1));
	error2LineEdit->setText(QString::number(error2));

	ovlNumber1LineEdit->setText(QString::number(ovlNumber1));
	ovlNumber2LineEdit->setText(QString::number(ovlNumber2));
}

void GlobalRegistrationDialog::updateCycleComboBox(QString circle)
{
	while( cycleComboBox->count() !=0 )
	{
		cycleComboBox->removeItem(cycleComboBox->count()-1);
	}
	QRegExp regExp("(([0-9]+,)+)([0-9]+)");
	if (regExp.exactMatch(circle))
	{
		//qDebug() << "matched";
		QStringList singlelist = circle.split(",");
		QStringList cyclelist = singlelist;
		for (int i = 1; i < singlelist.size(); ++i)
		{
			for (int j = 0; j < singlelist.size(); ++j)
			{
				cyclelist[j].append("<-").append(singlelist[(j+i)%singlelist.size()]);
			}
		}
		cycleComboBox->addItems(cyclelist);
	}
}

void GlobalRegistrationDialog::on_sendRelationPushButton_clicked()
{
	//relationTextEdit->setText(cycleComboBox->currentText());
	relationTextEdit->setText(relationTextEdit->toPlainText() + cycleComboBox->currentText() + "\n");
}

void GlobalRegistrationDialog::on_exportPushButton_clicked()
{
	QVariantMap parameters;
	parameters["command"] = QString("Export");
	parameters["relations"] = relationTextEdit->toPlainText();
	emit sendParameters(parameters);
}


// void GlobalRegistrationDialog::updateStaticComboBox(QString circle)
// {
// 	while( staticComboBox->count() !=0 )
// 	{
// 		staticComboBox->removeItem(staticComboBox->count()-1);
// 	}
// 	QRegExp regExp("(([0-9]+,)+)([0-9]+)");
// 	if (regExp.exactMatch(circle))
// 	{
// 		//qDebug() << "matched";
// 		QStringList singlelist = circle.split(",");
// 		staticComboBox->addItems(singlelist);
// 	}

// }