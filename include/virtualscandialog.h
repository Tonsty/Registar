#ifndef VIRTUALSCANDIALOG_H
#define VIRTUALSCANDIALOG_H

#include <QtGui/QDialog>
#include "ui_NormalFieldDialog.h"

#include <QtGui/QDialog>
#include "ui_VirtualScanDialog.h"

class VirtualScanDialog : public QDialog, public Ui_VirtualScanDialog
{
	Q_OBJECT

public:
	VirtualScanDialog(QWidget *parent = 0);
	virtual ~VirtualScanDialog();

signals:
	void sendParameters(QVariantMap parameters);

	private slots:
		void on_applyButton_clicked();
};

#endif