#ifndef EUCLIDEANCLUSTEREXTRACTIONDIALOG_H
#define EUCLIDEANCLUSTEREXTRACTIONDIALOG_H

#include <QtGui/QDialog>
#include "../build/ui/ui_EuclideanClusterExtractionDialog.h"

class EuclideanClusterExtractionDialog : public QDialog, public Ui_EuclideanClusterExtractionDialog
{
	Q_OBJECT

public:
	EuclideanClusterExtractionDialog(QWidget *parent = 0);
	virtual ~EuclideanClusterExtractionDialog();

signals:
	//void sendParameters(float clusterTolerance, int minClusterSize, int maxClusterSize);
	void sendParameters(QVariantMap parameters);

private slots:
	void on_applyButton_clicked();
}; 

#endif