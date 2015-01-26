#ifndef COLORFIELDDIALOG_H
#define COLORFIELDDIALOG_H

#include <QtGui/QDialog>
#include "ui_ColorFieldDialog.h"

class ColorFieldDialog : public QDialog, public Ui_ColorFieldDialog
{
	Q_OBJECT

public:
	ColorFieldDialog(QWidget *parent = 0);
	virtual ~ColorFieldDialog();

signals:
	void sendParameters(QVariantMap parameters);

private slots:
	void on_applyButton_clicked();
};

#endif
