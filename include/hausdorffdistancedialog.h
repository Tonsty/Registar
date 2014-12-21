#ifndef HAUSDORFFDISTANCEDIALOG_H
#define HAUSDORFFDISTANCEDIALOG_H

#include <QtGui/QDialog>
#include "ui_HausdorffDistanceDialog.h"

class HausdorffDistanceDialog : public QDialog, public Ui_HausdorffDistanceDialog
{
	Q_OBJECT

public:
	HausdorffDistanceDialog(QWidget *parent = 0);
	virtual ~HausdorffDistanceDialog();

signals:
	void sendParameters(QVariantMap parameters);

	private slots:
		void on_applyButton_clicked();
};

#endif