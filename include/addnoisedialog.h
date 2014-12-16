#ifndef ADDNOISEDIALOG_H
#define ADDNOISEDIALOG_H

#include <QtGui/QDialog>
#include "ui_AddNoiseDialog.h"

class AddNoiseDialog : public QDialog, public Ui_AddNoiseDialog
{
	Q_OBJECT

public:
	AddNoiseDialog(QWidget *parent = 0);
	virtual ~AddNoiseDialog();

signals:
	void sendParameters(QVariantMap parameters);

private slots:
	void on_applyButton_clicked();
};

#endif
