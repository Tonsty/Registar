#ifndef BOUNDARYESTIMATIONDIALOG_H
#define BOUNDARYESTIMATIONDIALOG_H

#include <QtGui/QDialog>
#include "ui_BoundaryEstimationDialog.h"

class BoundaryEstimationDialog : public QDialog, public Ui_BoundaryEstimationDialog
{
	Q_OBJECT

public:
	BoundaryEstimationDialog(QWidget *parent = 0);
	virtual ~BoundaryEstimationDialog();

signals:
	void sendParameters(QVariantMap parameters);

private slots:
	void on_applyButton_clicked();
}; 

#endif