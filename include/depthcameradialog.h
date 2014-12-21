#ifndef DEPTHCAMERADIALOG_H
#define DEPTHCAMERADIALOG_H

#include <QtGui/QDialog>
#include "ui_DepthCameraDialog.h"

class DepthCameraDialog : public QDialog, public Ui_DepthCameraDialog
{
	Q_OBJECT

public:
	DepthCameraDialog(QWidget *parent = 0);
	virtual ~DepthCameraDialog();

signals:
	void sendParameters(QVariantMap parameters);

	private slots:
		void on_applyButton_clicked();
};

#endif