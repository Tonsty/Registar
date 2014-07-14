/********************************************************************************
** Form generated from reading UI file 'VoxelGridDialog.ui'
**
** Created: Sat Jul 12 15:53:40 2014
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VOXELGRIDDIALOG_H
#define UI_VOXELGRIDDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_VoxelGridDialog
{
public:
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout;
    QGridLayout *gridLayout_2;
    QLabel *leafSizeXLabel;
    QLineEdit *leafSizeXLineEdit;
    QLabel *leafSizeYLabel;
    QLineEdit *leafSizeYLineEdit;
    QLabel *leafSizeZLabel;
    QLineEdit *leafSizeZLineEdit;
    QVBoxLayout *verticalLayout;
    QCheckBox *overwriteCheckBox;
    QSpacerItem *verticalSpacer;
    QGridLayout *gridLayout;
    QPushButton *defaultButton;
    QPushButton *helpButton;
    QPushButton *closeButton;
    QPushButton *applyButton;

    void setupUi(QDialog *VoxelGridDialog)
    {
        if (VoxelGridDialog->objectName().isEmpty())
            VoxelGridDialog->setObjectName(QString::fromUtf8("VoxelGridDialog"));
        VoxelGridDialog->resize(262, 140);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/bunny.png"), QSize(), QIcon::Normal, QIcon::Off);
        VoxelGridDialog->setWindowIcon(icon);
        verticalLayout_2 = new QVBoxLayout(VoxelGridDialog);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        leafSizeXLabel = new QLabel(VoxelGridDialog);
        leafSizeXLabel->setObjectName(QString::fromUtf8("leafSizeXLabel"));

        gridLayout_2->addWidget(leafSizeXLabel, 0, 0, 1, 1);

        leafSizeXLineEdit = new QLineEdit(VoxelGridDialog);
        leafSizeXLineEdit->setObjectName(QString::fromUtf8("leafSizeXLineEdit"));

        gridLayout_2->addWidget(leafSizeXLineEdit, 0, 1, 1, 1);

        leafSizeYLabel = new QLabel(VoxelGridDialog);
        leafSizeYLabel->setObjectName(QString::fromUtf8("leafSizeYLabel"));

        gridLayout_2->addWidget(leafSizeYLabel, 1, 0, 1, 1);

        leafSizeYLineEdit = new QLineEdit(VoxelGridDialog);
        leafSizeYLineEdit->setObjectName(QString::fromUtf8("leafSizeYLineEdit"));

        gridLayout_2->addWidget(leafSizeYLineEdit, 1, 1, 1, 1);

        leafSizeZLabel = new QLabel(VoxelGridDialog);
        leafSizeZLabel->setObjectName(QString::fromUtf8("leafSizeZLabel"));

        gridLayout_2->addWidget(leafSizeZLabel, 2, 0, 1, 1);

        leafSizeZLineEdit = new QLineEdit(VoxelGridDialog);
        leafSizeZLineEdit->setObjectName(QString::fromUtf8("leafSizeZLineEdit"));

        gridLayout_2->addWidget(leafSizeZLineEdit, 2, 1, 1, 1);


        horizontalLayout->addLayout(gridLayout_2);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        overwriteCheckBox = new QCheckBox(VoxelGridDialog);
        overwriteCheckBox->setObjectName(QString::fromUtf8("overwriteCheckBox"));
        overwriteCheckBox->setChecked(true);

        verticalLayout->addWidget(overwriteCheckBox);

        verticalSpacer = new QSpacerItem(78, 50, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        horizontalLayout->addLayout(verticalLayout);


        verticalLayout_2->addLayout(horizontalLayout);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        defaultButton = new QPushButton(VoxelGridDialog);
        defaultButton->setObjectName(QString::fromUtf8("defaultButton"));

        gridLayout->addWidget(defaultButton, 0, 0, 1, 1);

        helpButton = new QPushButton(VoxelGridDialog);
        helpButton->setObjectName(QString::fromUtf8("helpButton"));

        gridLayout->addWidget(helpButton, 0, 1, 1, 1);

        closeButton = new QPushButton(VoxelGridDialog);
        closeButton->setObjectName(QString::fromUtf8("closeButton"));

        gridLayout->addWidget(closeButton, 1, 0, 1, 1);

        applyButton = new QPushButton(VoxelGridDialog);
        applyButton->setObjectName(QString::fromUtf8("applyButton"));

        gridLayout->addWidget(applyButton, 1, 1, 1, 1);


        verticalLayout_2->addLayout(gridLayout);


        retranslateUi(VoxelGridDialog);
        QObject::connect(closeButton, SIGNAL(clicked()), VoxelGridDialog, SLOT(close()));

        QMetaObject::connectSlotsByName(VoxelGridDialog);
    } // setupUi

    void retranslateUi(QDialog *VoxelGridDialog)
    {
        VoxelGridDialog->setWindowTitle(QApplication::translate("VoxelGridDialog", "Voxel Grid Dialog", 0, QApplication::UnicodeUTF8));
        leafSizeXLabel->setText(QApplication::translate("VoxelGridDialog", "LeafSizeX", 0, QApplication::UnicodeUTF8));
        leafSizeXLineEdit->setText(QApplication::translate("VoxelGridDialog", "0.01", 0, QApplication::UnicodeUTF8));
        leafSizeYLabel->setText(QApplication::translate("VoxelGridDialog", "LeafSizeY", 0, QApplication::UnicodeUTF8));
        leafSizeYLineEdit->setText(QApplication::translate("VoxelGridDialog", "0.01", 0, QApplication::UnicodeUTF8));
        leafSizeZLabel->setText(QApplication::translate("VoxelGridDialog", "LeafSizeZ", 0, QApplication::UnicodeUTF8));
        leafSizeZLineEdit->setText(QApplication::translate("VoxelGridDialog", "0.01", 0, QApplication::UnicodeUTF8));
        overwriteCheckBox->setText(QApplication::translate("VoxelGridDialog", "overwrite", 0, QApplication::UnicodeUTF8));
        defaultButton->setText(QApplication::translate("VoxelGridDialog", "Default", 0, QApplication::UnicodeUTF8));
        helpButton->setText(QApplication::translate("VoxelGridDialog", "Help", 0, QApplication::UnicodeUTF8));
        closeButton->setText(QApplication::translate("VoxelGridDialog", "Close", 0, QApplication::UnicodeUTF8));
        applyButton->setText(QApplication::translate("VoxelGridDialog", "Apply", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class VoxelGridDialog: public Ui_VoxelGridDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VOXELGRIDDIALOG_H
