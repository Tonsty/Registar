#ifndef VOXELGRIDDIALOG_H
#define VOXELGRIDDIALOG_H

#include <QtGui/QDialog>
#include "../build/ui/ui_VoxelGridDialog.h"

class VoxelGridDialog : public QDialog, public Ui_VoxelGridDialog
{
	Q_OBJECT

public:
	VoxelGridDialog(QWidget *parent = 0);
	virtual ~VoxelGridDialog();

signals:
	void sendParameters(QVariantMap parameters);

private slots:
	void on_applyButton_clicked();
};

#endif
