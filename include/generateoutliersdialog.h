#ifndef GENERATEOUTLIERSDIALOG_H
#define GENERATEOUTLIERSDIALOG_H

#include <QtGui/QDialog>
#include "ui_GenerateOutliersDialog.h"

class GenerateOutliersDialog : public QDialog, public Ui_GenerateOutliersDialog
{
	Q_OBJECT

public:
	GenerateOutliersDialog(QWidget *parent = 0);
	virtual ~GenerateOutliersDialog();

signals:
	void sendParameters(QVariantMap parameters);
	void boundingBox();

	private slots:
		void on_applyButton_clicked();
		void on_boundingBoxButton_clicked();
};

#endif