#ifndef RANDOMTRANSFORMATIONDIALOG_H
#define RANDOMTRANSFORMATIONDIALOG_H

#include <QtGui/QDialog>
#include "ui_RandomTransformationDialog.h"

class RandomTransformationDialog : public QDialog, public Ui_RandomTransformationDialog
{
	Q_OBJECT

public:
	RandomTransformationDialog(QWidget *parent = 0);
	virtual ~RandomTransformationDialog();

signals:
	void sendParameters(QVariantMap parameters);

private slots:
	void on_applyButton_clicked();
};

#endif
