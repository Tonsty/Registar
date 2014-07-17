#ifndef NORMALFIELDDIALOG_H
#define NORMALFIELDDIALOG_H

#include <QtGui/QDialog>
#include "ui_NormalFieldDialog.h"

class NormalFieldDialog : public QDialog, public Ui_NormalFieldDialog
{
	Q_OBJECT

public:
	NormalFieldDialog(QWidget *parent = 0);
	virtual ~NormalFieldDialog();

signals:
	void sendParameters(QVariantMap parameters);

private slots:
	void on_applyButton_clicked();
};

#endif
