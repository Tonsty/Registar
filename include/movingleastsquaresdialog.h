#ifndef MOVINGLEASTSQUARESDIALOG_H
#define MOVINGLEASTSQUARESDIALOG_H

#include <QtGui/QDialog>
#include "../ui/ui_MovingLeastSquaresDialog.h"

class MovingLeastSquaresDialog : public QDialog, public Ui_MovingLeastSquaresDialog
{
	Q_OBJECT

public:
	MovingLeastSquaresDialog(QWidget *parent = 0);
	virtual ~MovingLeastSquaresDialog();

signals:
	void sendParameters(QVariantMap parameters);

private slots:
	void on_applyButton_clicked();
};

#endif
