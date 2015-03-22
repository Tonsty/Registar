#ifndef RANDOMTRANSFORMATIONDIALOG_H
#define RANDOMTRANSFORMATIONDIALOG_H

#include <QtGui/QDialog>
#include "ui_TransformationDialog.h"

class TransformationDialog : public QDialog, public Ui_TransformationDialog
{
	Q_OBJECT

public:
	TransformationDialog(QWidget *parent = 0);
	virtual ~TransformationDialog();

signals:
	void sendParameters(QVariantMap parameters);

private slots:
	void on_applyButton_clicked();
};

#endif
