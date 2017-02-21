#ifndef BACKGROUNDCOLORDIALOG_H
#define BACKGROUNDCOLORDIALOG_H

#include <QtGui/QDialog>
#include "ui_BackgroundColorDialog.h"

class BackgroundColorDialog : public QDialog, public Ui_BackgroundColorDialog
{
	Q_OBJECT

public:
	BackgroundColorDialog(QWidget *parent = 0);
	virtual ~BackgroundColorDialog();

signals:
	void sendParameters(QVariantMap parameters);

private slots:
	void on_applyButton_clicked();
};

#endif
