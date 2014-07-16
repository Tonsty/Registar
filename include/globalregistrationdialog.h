#ifndef GLOBALREGISTRATIONDIALOG_H
#define GLOBALREGISTRATIONDIALOG_H

#include <QtGui/QDialog>

#include <Eigen/Eigen>

#include "../build/ui/ui_GlobalRegistrationDialog.h"

class QErrorMessage;

class GlobalRegistrationDialog : public QDialog, public Ui_GlobalRegistrationDialog
{
	Q_OBJECT

public:
	GlobalRegistrationDialog(QWidget *parent = 0);
	virtual ~GlobalRegistrationDialog();

	void showEstimation(Eigen::Matrix4f transformation_total, float error1, float error2, int ovlNumber1, int ovlNumber2);
signals:
	void sendParameters(QVariantMap parameters);

private slots:
	void on_addPushButton_clicked();
	void on_deletePushButton_clicked();
	void on_circleListWidget_itemChanged(QListWidgetItem *item);
	void on_circleListWidget_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous);

	void on_initializePairPushButton_clicked();
	void on_deinitializePairPushButton_clicked();


	void on_initializeCyclePushButton_clicked();

	void on_estimatePushButton_clicked();
	void on_consistentPushButton_clicked();
	void on_sendRelationPushButton_clicked();
	void on_exportPushButton_clicked();

private:
	QErrorMessage *errorMessage;
	void updatePairListWidget(QString circle);
	void updateCycleComboBox(QString circle);
	//void updateStaticComboBox(QString circle);
};

#endif
