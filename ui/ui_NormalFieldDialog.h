/********************************************************************************
** Form generated from reading UI file 'NormalFieldDialog.ui'
**
** Created: Sat Jul 12 15:53:40 2014
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_NORMALFIELDDIALOG_H
#define UI_NORMALFIELDDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
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

class Ui_NormalFieldDialog
{
public:
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QGridLayout *gridLayout;
    QLabel *label_2;
    QLineEdit *XLineEdit;
    QLabel *label_3;
    QLineEdit *YLineEdit;
    QLabel *label_4;
    QLineEdit *ZLineEdit;
    QComboBox *comboBox;
    QHBoxLayout *horizontalLayout;
    QGridLayout *gridLayout_2;
    QPushButton *defaultButton;
    QPushButton *helpButton;
    QPushButton *closeButton;
    QPushButton *applyButton;
    QVBoxLayout *verticalLayout_2;
    QCheckBox *overwriteCheckBox;
    QSpacerItem *verticalSpacer;

    void setupUi(QDialog *NormalFieldDialog)
    {
        if (NormalFieldDialog->objectName().isEmpty())
            NormalFieldDialog->setObjectName(QString::fromUtf8("NormalFieldDialog"));
        NormalFieldDialog->resize(236, 198);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/bunny.png"), QSize(), QIcon::Normal, QIcon::Off);
        NormalFieldDialog->setWindowIcon(icon);
        verticalLayout_3 = new QVBoxLayout(NormalFieldDialog);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(NormalFieldDialog);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_2 = new QLabel(NormalFieldDialog);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 0, 0, 1, 1);

        XLineEdit = new QLineEdit(NormalFieldDialog);
        XLineEdit->setObjectName(QString::fromUtf8("XLineEdit"));

        gridLayout->addWidget(XLineEdit, 0, 1, 1, 1);

        label_3 = new QLabel(NormalFieldDialog);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 1, 0, 1, 1);

        YLineEdit = new QLineEdit(NormalFieldDialog);
        YLineEdit->setObjectName(QString::fromUtf8("YLineEdit"));

        gridLayout->addWidget(YLineEdit, 1, 1, 1, 1);

        label_4 = new QLabel(NormalFieldDialog);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 2, 0, 1, 1);

        ZLineEdit = new QLineEdit(NormalFieldDialog);
        ZLineEdit->setObjectName(QString::fromUtf8("ZLineEdit"));

        gridLayout->addWidget(ZLineEdit, 2, 1, 1, 1);


        verticalLayout->addLayout(gridLayout);

        comboBox = new QComboBox(NormalFieldDialog);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));

        verticalLayout->addWidget(comboBox);


        verticalLayout_3->addLayout(verticalLayout);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        defaultButton = new QPushButton(NormalFieldDialog);
        defaultButton->setObjectName(QString::fromUtf8("defaultButton"));

        gridLayout_2->addWidget(defaultButton, 0, 0, 1, 1);

        helpButton = new QPushButton(NormalFieldDialog);
        helpButton->setObjectName(QString::fromUtf8("helpButton"));

        gridLayout_2->addWidget(helpButton, 0, 1, 1, 1);

        closeButton = new QPushButton(NormalFieldDialog);
        closeButton->setObjectName(QString::fromUtf8("closeButton"));

        gridLayout_2->addWidget(closeButton, 1, 0, 1, 1);

        applyButton = new QPushButton(NormalFieldDialog);
        applyButton->setObjectName(QString::fromUtf8("applyButton"));

        gridLayout_2->addWidget(applyButton, 1, 1, 1, 1);


        horizontalLayout->addLayout(gridLayout_2);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        overwriteCheckBox = new QCheckBox(NormalFieldDialog);
        overwriteCheckBox->setObjectName(QString::fromUtf8("overwriteCheckBox"));
        overwriteCheckBox->setChecked(true);

        verticalLayout_2->addWidget(overwriteCheckBox);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);


        horizontalLayout->addLayout(verticalLayout_2);


        verticalLayout_3->addLayout(horizontalLayout);


        retranslateUi(NormalFieldDialog);

        QMetaObject::connectSlotsByName(NormalFieldDialog);
    } // setupUi

    void retranslateUi(QDialog *NormalFieldDialog)
    {
        NormalFieldDialog->setWindowTitle(QApplication::translate("NormalFieldDialog", "Normal Field Dialog", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("NormalFieldDialog", "Oriented by View Point", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("NormalFieldDialog", "X", 0, QApplication::UnicodeUTF8));
        XLineEdit->setText(QApplication::translate("NormalFieldDialog", "0", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("NormalFieldDialog", "Y", 0, QApplication::UnicodeUTF8));
        YLineEdit->setText(QApplication::translate("NormalFieldDialog", "0", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("NormalFieldDialog", "Z", 0, QApplication::UnicodeUTF8));
        ZLineEdit->setText(QApplication::translate("NormalFieldDialog", "0", 0, QApplication::UnicodeUTF8));
        comboBox->clear();
        comboBox->insertItems(0, QStringList()
         << QApplication::translate("NormalFieldDialog", "Custom", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("NormalFieldDialog", "From Visualizer Camera", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("NormalFieldDialog", "From Scanning Camera", 0, QApplication::UnicodeUTF8)
        );
        defaultButton->setText(QApplication::translate("NormalFieldDialog", "Default", 0, QApplication::UnicodeUTF8));
        helpButton->setText(QApplication::translate("NormalFieldDialog", "Help", 0, QApplication::UnicodeUTF8));
        closeButton->setText(QApplication::translate("NormalFieldDialog", "Close", 0, QApplication::UnicodeUTF8));
        applyButton->setText(QApplication::translate("NormalFieldDialog", "Apply", 0, QApplication::UnicodeUTF8));
        overwriteCheckBox->setText(QApplication::translate("NormalFieldDialog", "overwrite", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class NormalFieldDialog: public Ui_NormalFieldDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_NORMALFIELDDIALOG_H
