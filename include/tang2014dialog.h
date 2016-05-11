#ifndef TANG2014DIALOG_H
#define TANG2014DIALOG_H

#include <QtGui/QDialog>
#include "ui_Tang2014Dialog.h"

class Tang2014Dialog : public QDialog, public Ui_Tang2014Dialog
{
	Q_OBJECT

public:
	Tang2014Dialog(QWidget *parent = 0);
	virtual ~Tang2014Dialog();

signals:
	void sendParameters(QVariantMap parameters);

private slots:
	void on_applyButton_clicked();
};

#endif
