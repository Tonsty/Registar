#include <QtCore/QDebug>
#include <QtCore/qmath.h>
#include <sstream>

#include "../include/cloudvisualizer.h"
#include "../include/pairwiseregistrationdialog.h"

using namespace registar;

PairwiseRegistrationDialog::PairwiseRegistrationDialog(QWidget *parent) : QDialog(parent)
{
	setupUi(this);
	//tabWidget->addTab(cloudVisualizer, "Cloud_1<-Cloud_0");
	//setFixedSize(sizeHint());
}

PairwiseRegistrationDialog::~PairwiseRegistrationDialog(){}

void PairwiseRegistrationDialog::on_initializePushButton_clicked()
{
	QVariantMap parameters;
	parameters["target"] = targetComboBox->currentText();
	parameters["source"] = sourceComboBox->currentText();
	parameters["command"] = QString("Initialize");
	emit sendParameters(parameters);
}

void PairwiseRegistrationDialog::on_prePushButton_clicked()
{
	QVariantMap parameters;
	parameters["target"] = targetComboBox->currentText();
	parameters["source"] = sourceComboBox->currentText();
	parameters["command"] = QString("Pre-Correspondences");
	parameters["method"] = methodComboBox->currentIndex();
	parameters["distanceThreshold"] = distanceDoubleSpinBox->value();
	parameters["normalAngleThreshold"] = normalDoubleSpinBox->value();
	parameters["boundaryTest"] = boundaryTestCheckBox->isChecked();
	parameters["biDirectional"] = biDirectionalCheckBox->isChecked();
	parameters["use_scpu"] = scpuRadioButton->isChecked();
	parameters["use_mcpu"] = mcpuRadioButton->isChecked();
	emit sendParameters(parameters);
}

void PairwiseRegistrationDialog::on_icpPushButton_clicked()
{
	QVariantMap parameters;
	parameters["target"] = targetComboBox->currentText();
	parameters["source"] = sourceComboBox->currentText();
	parameters["command"] = QString("ICP");
	parameters["method"] = methodComboBox->currentIndex();
	parameters["distanceThreshold"] = distanceDoubleSpinBox->value();
	parameters["normalAngleThreshold"] = normalDoubleSpinBox->value();
	parameters["boundaryTest"] = boundaryTestCheckBox->isChecked();
	parameters["biDirectional"] = biDirectionalCheckBox->isChecked();
	parameters["icpNumber"] = icpNumberSpinBox->value();
	parameters["allowScaling"] = scalingCheckBox->isChecked();
	parameters["use_scpu"] = scpuRadioButton->isChecked();
	parameters["use_mcpu"] = mcpuRadioButton->isChecked();
	emit sendParameters(parameters);
}

void PairwiseRegistrationDialog::on_exportPushButton_clicked()
{
	QVariantMap parameters;
	parameters["target"] = targetComboBox->currentText();
	parameters["source"] = sourceComboBox->currentText();
	parameters["command"] = QString("Export");
	emit sendParameters(parameters);
}

void PairwiseRegistrationDialog::on_manualPushButton_clicked()
{
	QVariantMap parameters;
	parameters["target"] = targetComboBox->currentText();
	parameters["source"] = sourceComboBox->currentText();
	parameters["command"] = QString("Manual");
	emit sendParameters(parameters);	
}

void PairwiseRegistrationDialog::on_targetComboBox_currentIndexChanged(const QString &cloudName_target)
{
	QString cloudName_source = sourceComboBox->currentText();
	QString pairName = cloudName_target + "<-" + cloudName_source;

	QWidget *current = tabWidget->currentWidget();
	if ((current != NULL) && (pairName == current->objectName())) return;

	QWidget *toBeCurrent = tabWidget->findChild<QWidget*>(pairName);
	if(toBeCurrent != NULL) tabWidget->setCurrentWidget(toBeCurrent);
}


void PairwiseRegistrationDialog::on_sourceComboBox_currentIndexChanged(const QString &cloudName_source)
{
	QString cloudName_target = targetComboBox->currentText();
	QString pairName = cloudName_target + "<-" + cloudName_source;

	QWidget *current = tabWidget->currentWidget();
	if ((current != NULL) && (pairName == current->objectName())) return;

	QWidget *toBeCurrent = tabWidget->findChild<QWidget*>(pairName);
	if(toBeCurrent != NULL) tabWidget->setCurrentWidget(toBeCurrent);
}

void PairwiseRegistrationDialog::on_tabWidget_currentChanged(int index)
{
	QString tabText = tabWidget->tabText(index);
	//qDebug() << tabText;
	QStringList list = tabText.split("<-");
	QString cloudName_target, cloudName_source; 
	cloudName_target = list.at(0);
	cloudName_source = list.at(1);
	//qDebug() << cloudName_target << " " << cloudName_source;
	if(cloudName_target != targetComboBox->currentText()) 
		targetComboBox->setCurrentIndex( targetComboBox->findText(cloudName_target) );
	if(cloudName_source != sourceComboBox->currentText())	
		sourceComboBox->setCurrentIndex( sourceComboBox->findText(cloudName_source) );

	QVariantMap parameters;
	parameters["target"] = cloudName_target;
	parameters["source"] = cloudName_source;
	parameters["command"] = QString("ShowResults");
	emit sendParameters(parameters);
}

CloudVisualizer* PairwiseRegistrationDialog::addCloudVisualizerTab(QString targetBySource)
{
	CloudVisualizer *cloudVisualizer = new CloudVisualizer(this);
	cloudVisualizer->setObjectName(targetBySource);
	int index = tabWidget->addTab(cloudVisualizer, targetBySource);
	tabWidget->setCurrentIndex(index);

	return cloudVisualizer;
}

void PairwiseRegistrationDialog::showResults(const Eigen::Matrix4f &transformation, float rmsError, int corrNumber)
{
	std::stringstream temp;
	Eigen::Matrix4f T = transformation;
	temp.str("");
	temp << T;
	TTextEdit->setText(QString::fromStdString(temp.str()));
	Eigen::Matrix3f R = T.block(0,0,3,3);
	float c = (R * R.transpose()).diagonal().mean();
	c = qSqrt(c);
	cLineEdit->setText(QString::number(c));
	R /= c;
	temp.str("");
	temp << R;
	RTextEdit->setText(QString::fromStdString(temp.str()));
	Eigen::Vector3f t = T.block(0,3,3,1);
	temp.str("");
	temp << t;
	tLineEdit->setText(QString::fromStdString(temp.str()));
	Eigen::AngleAxisf angleAxis(R);
	angleLineEdit->setText( QString::number(angleAxis.angle()) );
	temp.str("");
	temp << angleAxis.axis();
	axisLineEdit->setText(QString::fromStdString(temp.str()));

	corrNumberLineEdit->setText(QString::number(corrNumber));
	errorLineEdit->setText(QString::number(rmsError));
}

