/********************************************************************************
** Form generated from reading UI file 'BoundaryEstimationDialog.ui'
**
** Created: Sat Jul 12 15:53:40 2014
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BOUNDARYESTIMATIONDIALOG_H
#define UI_BOUNDARYESTIMATIONDIALOG_H

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

class Ui_BoundaryEstimationDialog
{
public:
    QVBoxLayout *verticalLayout_2;
    QGridLayout *gridLayout_2;
    QLabel *label;
    QLineEdit *radiusLineEdit;
    QLabel *label_2;
    QLineEdit *angleLineEdit;
    QLabel *label_3;
    QLineEdit *dilationLineEdit;
    QHBoxLayout *horizontalLayout;
    QGridLayout *gridLayout;
    QPushButton *defaultButton;
    QPushButton *helpButton;
    QPushButton *closeButton;
    QPushButton *applyButton;
    QVBoxLayout *verticalLayout;
    QCheckBox *overwriteCheckBox;
    QCheckBox *splitCheckBox;
    QSpacerItem *verticalSpacer;

    void setupUi(QDialog *BoundaryEstimationDialog)
    {
        if (BoundaryEstimationDialog->objectName().isEmpty())
            BoundaryEstimationDialog->setObjectName(QString::fromUtf8("BoundaryEstimationDialog"));
        BoundaryEstimationDialog->resize(292, 157);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/bunny.png"), QSize(), QIcon::Normal, QIcon::Off);
        BoundaryEstimationDialog->setWindowIcon(icon);
        verticalLayout_2 = new QVBoxLayout(BoundaryEstimationDialog);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        label = new QLabel(BoundaryEstimationDialog);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_2->addWidget(label, 0, 0, 1, 1);

        radiusLineEdit = new QLineEdit(BoundaryEstimationDialog);
        radiusLineEdit->setObjectName(QString::fromUtf8("radiusLineEdit"));

        gridLayout_2->addWidget(radiusLineEdit, 0, 1, 1, 1);

        label_2 = new QLabel(BoundaryEstimationDialog);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_2->addWidget(label_2, 1, 0, 1, 1);

        angleLineEdit = new QLineEdit(BoundaryEstimationDialog);
        angleLineEdit->setObjectName(QString::fromUtf8("angleLineEdit"));

        gridLayout_2->addWidget(angleLineEdit, 1, 1, 1, 1);

        label_3 = new QLabel(BoundaryEstimationDialog);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_2->addWidget(label_3, 2, 0, 1, 1);

        dilationLineEdit = new QLineEdit(BoundaryEstimationDialog);
        dilationLineEdit->setObjectName(QString::fromUtf8("dilationLineEdit"));

        gridLayout_2->addWidget(dilationLineEdit, 2, 1, 1, 1);


        verticalLayout_2->addLayout(gridLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        defaultButton = new QPushButton(BoundaryEstimationDialog);
        defaultButton->setObjectName(QString::fromUtf8("defaultButton"));

        gridLayout->addWidget(defaultButton, 0, 0, 1, 1);

        helpButton = new QPushButton(BoundaryEstimationDialog);
        helpButton->setObjectName(QString::fromUtf8("helpButton"));

        gridLayout->addWidget(helpButton, 0, 1, 1, 1);

        closeButton = new QPushButton(BoundaryEstimationDialog);
        closeButton->setObjectName(QString::fromUtf8("closeButton"));

        gridLayout->addWidget(closeButton, 1, 0, 1, 1);

        applyButton = new QPushButton(BoundaryEstimationDialog);
        applyButton->setObjectName(QString::fromUtf8("applyButton"));

        gridLayout->addWidget(applyButton, 1, 1, 1, 1);


        horizontalLayout->addLayout(gridLayout);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        overwriteCheckBox = new QCheckBox(BoundaryEstimationDialog);
        overwriteCheckBox->setObjectName(QString::fromUtf8("overwriteCheckBox"));
        overwriteCheckBox->setChecked(true);

        verticalLayout->addWidget(overwriteCheckBox);

        splitCheckBox = new QCheckBox(BoundaryEstimationDialog);
        splitCheckBox->setObjectName(QString::fromUtf8("splitCheckBox"));

        verticalLayout->addWidget(splitCheckBox);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        horizontalLayout->addLayout(verticalLayout);


        verticalLayout_2->addLayout(horizontalLayout);

#ifndef QT_NO_SHORTCUT
        label->setBuddy(radiusLineEdit);
        label_2->setBuddy(angleLineEdit);
#endif // QT_NO_SHORTCUT

        retranslateUi(BoundaryEstimationDialog);
        QObject::connect(closeButton, SIGNAL(clicked()), BoundaryEstimationDialog, SLOT(close()));

        QMetaObject::connectSlotsByName(BoundaryEstimationDialog);
    } // setupUi

    void retranslateUi(QDialog *BoundaryEstimationDialog)
    {
        BoundaryEstimationDialog->setWindowTitle(QApplication::translate("BoundaryEstimationDialog", "Boundary Estimation Dialog", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("BoundaryEstimationDialog", "Search &Radius", 0, QApplication::UnicodeUTF8));
        radiusLineEdit->setText(QApplication::translate("BoundaryEstimationDialog", "0.06", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("BoundaryEstimationDialog", "&Angle Threshhold", 0, QApplication::UnicodeUTF8));
        angleLineEdit->setText(QApplication::translate("BoundaryEstimationDialog", "90", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("BoundaryEstimationDialog", "Dilation Radius", 0, QApplication::UnicodeUTF8));
        dilationLineEdit->setText(QApplication::translate("BoundaryEstimationDialog", "0.03", 0, QApplication::UnicodeUTF8));
        defaultButton->setText(QApplication::translate("BoundaryEstimationDialog", "Default", 0, QApplication::UnicodeUTF8));
        helpButton->setText(QApplication::translate("BoundaryEstimationDialog", "Help", 0, QApplication::UnicodeUTF8));
        closeButton->setText(QApplication::translate("BoundaryEstimationDialog", "Close", 0, QApplication::UnicodeUTF8));
        applyButton->setText(QApplication::translate("BoundaryEstimationDialog", "Apply", 0, QApplication::UnicodeUTF8));
        overwriteCheckBox->setText(QApplication::translate("BoundaryEstimationDialog", "overwrite", 0, QApplication::UnicodeUTF8));
        splitCheckBox->setText(QApplication::translate("BoundaryEstimationDialog", "split", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class BoundaryEstimationDialog: public Ui_BoundaryEstimationDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BOUNDARYESTIMATIONDIALOG_H
