/********************************************************************************
** Form generated from reading UI file 'OutliersRemovalDialog.ui'
**
** Created: Sat Jul 12 15:53:40 2014
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_OUTLIERSREMOVALDIALOG_H
#define UI_OUTLIERSREMOVALDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_OutliersRemovalDialog
{
public:
    QVBoxLayout *verticalLayout_2;
    QGridLayout *gridLayout_2;
    QLabel *label_4;
    QComboBox *methodComboBox;
    QLabel *label_3;
    QDoubleSpinBox *radiusDoubleSpinBox;
    QLabel *label;
    QSpinBox *nearestKSpinBox;
    QLabel *label_2;
    QDoubleSpinBox *deviationDoubleSpinBox;
    QSpacerItem *horizontalSpacer;
    QHBoxLayout *horizontalLayout;
    QGridLayout *gridLayout;
    QPushButton *defaultButton;
    QPushButton *helpButton;
    QPushButton *closeButton;
    QPushButton *applyButton;
    QVBoxLayout *verticalLayout;
    QCheckBox *overwriteCheckBox;
    QSpacerItem *verticalSpacer;

    void setupUi(QDialog *OutliersRemovalDialog)
    {
        if (OutliersRemovalDialog->objectName().isEmpty())
            OutliersRemovalDialog->setObjectName(QString::fromUtf8("OutliersRemovalDialog"));
        OutliersRemovalDialog->resize(300, 178);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/bunny.png"), QSize(), QIcon::Normal, QIcon::Off);
        OutliersRemovalDialog->setWindowIcon(icon);
        verticalLayout_2 = new QVBoxLayout(OutliersRemovalDialog);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        label_4 = new QLabel(OutliersRemovalDialog);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_2->addWidget(label_4, 0, 0, 1, 1);

        methodComboBox = new QComboBox(OutliersRemovalDialog);
        methodComboBox->setObjectName(QString::fromUtf8("methodComboBox"));

        gridLayout_2->addWidget(methodComboBox, 0, 1, 1, 2);

        label_3 = new QLabel(OutliersRemovalDialog);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_2->addWidget(label_3, 1, 0, 1, 1);

        radiusDoubleSpinBox = new QDoubleSpinBox(OutliersRemovalDialog);
        radiusDoubleSpinBox->setObjectName(QString::fromUtf8("radiusDoubleSpinBox"));
        radiusDoubleSpinBox->setValue(0.02);

        gridLayout_2->addWidget(radiusDoubleSpinBox, 1, 1, 1, 2);

        label = new QLabel(OutliersRemovalDialog);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_2->addWidget(label, 2, 0, 1, 1);

        nearestKSpinBox = new QSpinBox(OutliersRemovalDialog);
        nearestKSpinBox->setObjectName(QString::fromUtf8("nearestKSpinBox"));
        nearestKSpinBox->setMaximum(9999);
        nearestKSpinBox->setValue(60);

        gridLayout_2->addWidget(nearestKSpinBox, 2, 1, 1, 1);

        label_2 = new QLabel(OutliersRemovalDialog);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_2->addWidget(label_2, 3, 0, 1, 2);

        deviationDoubleSpinBox = new QDoubleSpinBox(OutliersRemovalDialog);
        deviationDoubleSpinBox->setObjectName(QString::fromUtf8("deviationDoubleSpinBox"));
        deviationDoubleSpinBox->setValue(1);

        gridLayout_2->addWidget(deviationDoubleSpinBox, 3, 2, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer, 2, 2, 1, 1);


        verticalLayout_2->addLayout(gridLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        defaultButton = new QPushButton(OutliersRemovalDialog);
        defaultButton->setObjectName(QString::fromUtf8("defaultButton"));

        gridLayout->addWidget(defaultButton, 0, 0, 1, 1);

        helpButton = new QPushButton(OutliersRemovalDialog);
        helpButton->setObjectName(QString::fromUtf8("helpButton"));

        gridLayout->addWidget(helpButton, 0, 1, 1, 1);

        closeButton = new QPushButton(OutliersRemovalDialog);
        closeButton->setObjectName(QString::fromUtf8("closeButton"));

        gridLayout->addWidget(closeButton, 1, 0, 1, 1);

        applyButton = new QPushButton(OutliersRemovalDialog);
        applyButton->setObjectName(QString::fromUtf8("applyButton"));

        gridLayout->addWidget(applyButton, 1, 1, 1, 1);


        horizontalLayout->addLayout(gridLayout);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        overwriteCheckBox = new QCheckBox(OutliersRemovalDialog);
        overwriteCheckBox->setObjectName(QString::fromUtf8("overwriteCheckBox"));
        overwriteCheckBox->setChecked(true);

        verticalLayout->addWidget(overwriteCheckBox);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        horizontalLayout->addLayout(verticalLayout);


        verticalLayout_2->addLayout(horizontalLayout);

#ifndef QT_NO_SHORTCUT
        label_4->setBuddy(methodComboBox);
        label_3->setBuddy(radiusDoubleSpinBox);
        label->setBuddy(nearestKSpinBox);
        label_2->setBuddy(deviationDoubleSpinBox);
#endif // QT_NO_SHORTCUT

        retranslateUi(OutliersRemovalDialog);
        QObject::connect(closeButton, SIGNAL(clicked()), OutliersRemovalDialog, SLOT(close()));

        QMetaObject::connectSlotsByName(OutliersRemovalDialog);
    } // setupUi

    void retranslateUi(QDialog *OutliersRemovalDialog)
    {
        OutliersRemovalDialog->setWindowTitle(QApplication::translate("OutliersRemovalDialog", "Outliers Removal Dialog", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("OutliersRemovalDialog", "&Method", 0, QApplication::UnicodeUTF8));
        methodComboBox->clear();
        methodComboBox->insertItems(0, QStringList()
         << QApplication::translate("OutliersRemovalDialog", "RadiusOutliersRemoval", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("OutliersRemovalDialog", "StatisticalOutliersRemoval", 0, QApplication::UnicodeUTF8)
        );
        label_3->setText(QApplication::translate("OutliersRemovalDialog", "Search &Radius", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("OutliersRemovalDialog", "Nearest &K", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("OutliersRemovalDialog", "Standard &Deviation Threshold", 0, QApplication::UnicodeUTF8));
        defaultButton->setText(QApplication::translate("OutliersRemovalDialog", "Default", 0, QApplication::UnicodeUTF8));
        helpButton->setText(QApplication::translate("OutliersRemovalDialog", "Help", 0, QApplication::UnicodeUTF8));
        closeButton->setText(QApplication::translate("OutliersRemovalDialog", "Close", 0, QApplication::UnicodeUTF8));
        applyButton->setText(QApplication::translate("OutliersRemovalDialog", "Apply", 0, QApplication::UnicodeUTF8));
        overwriteCheckBox->setText(QApplication::translate("OutliersRemovalDialog", "overwrite", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class OutliersRemovalDialog: public Ui_OutliersRemovalDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_OUTLIERSREMOVALDIALOG_H
