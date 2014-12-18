#ifndef SAVECONTENTDIALOG_H
#define SAVECONTENTDIALOG_H

#include <QtGui/QDialog>
#include "ui_SaveContentDialog.h"

class SaveContentDialog : public QDialog, public Ui_SaveContentDialog
{
	Q_OBJECT

public:
	SaveContentDialog(QWidget *parent = 0);
	virtual ~SaveContentDialog();

};

#endif
