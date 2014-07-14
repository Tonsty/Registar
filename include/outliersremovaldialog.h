#ifndef OUTLIERSREMOVALDIALOG_H
#define OUTLIERSREMOVALDIALOG_H

#include <QtGui/QDialog>
#include "../ui/ui_OutliersRemovalDialog.h"

class OutliersRemovalDialog : public QDialog, public Ui_OutliersRemovalDialog
{
	Q_OBJECT

public:
	OutliersRemovalDialog(QWidget *parent = 0);
	virtual ~OutliersRemovalDialog();

signals:
	void sendParameters(QVariantMap parameters);

private slots:
	void on_applyButton_clicked();
}; 

#endif