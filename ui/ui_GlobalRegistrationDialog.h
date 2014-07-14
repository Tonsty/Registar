/********************************************************************************
** Form generated from reading UI file 'GlobalRegistrationDialog.ui'
**
** Created: Sat Jul 12 15:53:40 2014
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GLOBALREGISTRATIONDIALOG_H
#define UI_GLOBALREGISTRATIONDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QListWidget>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTextEdit>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_GlobalRegistrationDialog
{
public:
    QHBoxLayout *horizontalLayout_6;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QSpacerItem *horizontalSpacer;
    QListWidget *circleListWidget;
    QGridLayout *gridLayout_2;
    QPushButton *consolidatePushButton;
    QPushButton *pushButton;
    QPushButton *initializeCyclePushButton;
    QPushButton *addPushButton;
    QPushButton *deletePushButton;
    QPushButton *closePushButton;
    QPushButton *consistentPushButton;
    QPushButton *pushButton_2;
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_2;
    QSpacerItem *horizontalSpacer_3;
    QListWidget *pairListWidget;
    QGridLayout *gridLayout_3;
    QPushButton *initializePairPushButton;
    QPushButton *deinitializePairPushButton;
    QSpacerItem *verticalSpacer;
    QGridLayout *gridLayout_5;
    QLineEdit *axisLineEdit;
    QLineEdit *angleLineEdit;
    QTextEdit *transformationTextEdit;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_8;
    QSpacerItem *horizontalSpacer_7;
    QComboBox *cycleComboBox;
    QLineEdit *translationLineEdit;
    QLabel *label_6;
    QLineEdit *scaleLineEdit;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_3;
    QSpacerItem *horizontalSpacer_6;
    QLineEdit *ovlNumber1LineEdit;
    QLineEdit *error1LineEdit;
    QLabel *label_7;
    QTextEdit *rotationTextEdit;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_10;
    QLabel *label_9;
    QPushButton *selectPushButton;
    QPushButton *sendRelationPushButton;
    QPushButton *estimatePushButton;
    QLabel *label_11;
    QLineEdit *ovlNumber2LineEdit;
    QLineEdit *error2LineEdit;
    QLabel *label_13;
    QVBoxLayout *verticalLayout_6;
    QLabel *label_12;
    QTextEdit *relationTextEdit;
    QPushButton *exportPushButton;
    QSpacerItem *verticalSpacer_2;

    void setupUi(QDialog *GlobalRegistrationDialog)
    {
        if (GlobalRegistrationDialog->objectName().isEmpty())
            GlobalRegistrationDialog->setObjectName(QString::fromUtf8("GlobalRegistrationDialog"));
        GlobalRegistrationDialog->resize(849, 856);
        horizontalLayout_6 = new QHBoxLayout(GlobalRegistrationDialog);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(GlobalRegistrationDialog);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);


        verticalLayout_4->addLayout(horizontalLayout);

        circleListWidget = new QListWidget(GlobalRegistrationDialog);
        circleListWidget->setObjectName(QString::fromUtf8("circleListWidget"));

        verticalLayout_4->addWidget(circleListWidget);


        verticalLayout->addLayout(verticalLayout_4);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        consolidatePushButton = new QPushButton(GlobalRegistrationDialog);
        consolidatePushButton->setObjectName(QString::fromUtf8("consolidatePushButton"));

        gridLayout_2->addWidget(consolidatePushButton, 5, 0, 1, 1);

        pushButton = new QPushButton(GlobalRegistrationDialog);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        gridLayout_2->addWidget(pushButton, 1, 1, 1, 1);

        initializeCyclePushButton = new QPushButton(GlobalRegistrationDialog);
        initializeCyclePushButton->setObjectName(QString::fromUtf8("initializeCyclePushButton"));

        gridLayout_2->addWidget(initializeCyclePushButton, 1, 0, 1, 1);

        addPushButton = new QPushButton(GlobalRegistrationDialog);
        addPushButton->setObjectName(QString::fromUtf8("addPushButton"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/add.png"), QSize(), QIcon::Normal, QIcon::Off);
        addPushButton->setIcon(icon);

        gridLayout_2->addWidget(addPushButton, 0, 0, 1, 1);

        deletePushButton = new QPushButton(GlobalRegistrationDialog);
        deletePushButton->setObjectName(QString::fromUtf8("deletePushButton"));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/images/delete.png"), QSize(), QIcon::Normal, QIcon::Off);
        deletePushButton->setIcon(icon1);

        gridLayout_2->addWidget(deletePushButton, 0, 1, 1, 1);

        closePushButton = new QPushButton(GlobalRegistrationDialog);
        closePushButton->setObjectName(QString::fromUtf8("closePushButton"));

        gridLayout_2->addWidget(closePushButton, 5, 1, 1, 1);

        consistentPushButton = new QPushButton(GlobalRegistrationDialog);
        consistentPushButton->setObjectName(QString::fromUtf8("consistentPushButton"));

        gridLayout_2->addWidget(consistentPushButton, 2, 1, 1, 1);

        pushButton_2 = new QPushButton(GlobalRegistrationDialog);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        gridLayout_2->addWidget(pushButton_2, 2, 0, 1, 1);


        verticalLayout->addLayout(gridLayout_2);


        horizontalLayout_2->addLayout(verticalLayout);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_2 = new QLabel(GlobalRegistrationDialog);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_3->addWidget(label_2);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_3);


        verticalLayout_2->addLayout(horizontalLayout_3);

        pairListWidget = new QListWidget(GlobalRegistrationDialog);
        pairListWidget->setObjectName(QString::fromUtf8("pairListWidget"));
        pairListWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);

        verticalLayout_2->addWidget(pairListWidget);


        verticalLayout_3->addLayout(verticalLayout_2);

        gridLayout_3 = new QGridLayout();
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        initializePairPushButton = new QPushButton(GlobalRegistrationDialog);
        initializePairPushButton->setObjectName(QString::fromUtf8("initializePairPushButton"));

        gridLayout_3->addWidget(initializePairPushButton, 0, 0, 1, 1);

        deinitializePairPushButton = new QPushButton(GlobalRegistrationDialog);
        deinitializePairPushButton->setObjectName(QString::fromUtf8("deinitializePairPushButton"));

        gridLayout_3->addWidget(deinitializePairPushButton, 0, 1, 1, 1);


        verticalLayout_3->addLayout(gridLayout_3);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);


        horizontalLayout_2->addLayout(verticalLayout_3);


        verticalLayout_5->addLayout(horizontalLayout_2);

        gridLayout_5 = new QGridLayout();
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        axisLineEdit = new QLineEdit(GlobalRegistrationDialog);
        axisLineEdit->setObjectName(QString::fromUtf8("axisLineEdit"));

        gridLayout_5->addWidget(axisLineEdit, 8, 1, 1, 1);

        angleLineEdit = new QLineEdit(GlobalRegistrationDialog);
        angleLineEdit->setObjectName(QString::fromUtf8("angleLineEdit"));

        gridLayout_5->addWidget(angleLineEdit, 9, 1, 1, 1);

        transformationTextEdit = new QTextEdit(GlobalRegistrationDialog);
        transformationTextEdit->setObjectName(QString::fromUtf8("transformationTextEdit"));

        gridLayout_5->addWidget(transformationTextEdit, 7, 0, 1, 1);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_8 = new QLabel(GlobalRegistrationDialog);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        horizontalLayout_5->addWidget(label_8);

        horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_7);


        gridLayout_5->addLayout(horizontalLayout_5, 5, 0, 2, 1);

        cycleComboBox = new QComboBox(GlobalRegistrationDialog);
        cycleComboBox->setObjectName(QString::fromUtf8("cycleComboBox"));

        gridLayout_5->addWidget(cycleComboBox, 2, 0, 1, 1);

        translationLineEdit = new QLineEdit(GlobalRegistrationDialog);
        translationLineEdit->setObjectName(QString::fromUtf8("translationLineEdit"));

        gridLayout_5->addWidget(translationLineEdit, 10, 1, 1, 1);

        label_6 = new QLabel(GlobalRegistrationDialog);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout_5->addWidget(label_6, 10, 0, 1, 1);

        scaleLineEdit = new QLineEdit(GlobalRegistrationDialog);
        scaleLineEdit->setObjectName(QString::fromUtf8("scaleLineEdit"));

        gridLayout_5->addWidget(scaleLineEdit, 11, 1, 1, 1);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_3 = new QLabel(GlobalRegistrationDialog);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_4->addWidget(label_3);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_6);


        gridLayout_5->addLayout(horizontalLayout_4, 5, 1, 1, 1);

        ovlNumber1LineEdit = new QLineEdit(GlobalRegistrationDialog);
        ovlNumber1LineEdit->setObjectName(QString::fromUtf8("ovlNumber1LineEdit"));

        gridLayout_5->addWidget(ovlNumber1LineEdit, 14, 1, 1, 1);

        error1LineEdit = new QLineEdit(GlobalRegistrationDialog);
        error1LineEdit->setObjectName(QString::fromUtf8("error1LineEdit"));

        gridLayout_5->addWidget(error1LineEdit, 12, 1, 1, 1);

        label_7 = new QLabel(GlobalRegistrationDialog);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_5->addWidget(label_7, 11, 0, 1, 1);

        rotationTextEdit = new QTextEdit(GlobalRegistrationDialog);
        rotationTextEdit->setObjectName(QString::fromUtf8("rotationTextEdit"));

        gridLayout_5->addWidget(rotationTextEdit, 6, 1, 2, 1);

        label_4 = new QLabel(GlobalRegistrationDialog);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_5->addWidget(label_4, 8, 0, 1, 1);

        label_5 = new QLabel(GlobalRegistrationDialog);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout_5->addWidget(label_5, 9, 0, 1, 1);

        label_10 = new QLabel(GlobalRegistrationDialog);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout_5->addWidget(label_10, 14, 0, 1, 1);

        label_9 = new QLabel(GlobalRegistrationDialog);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout_5->addWidget(label_9, 12, 0, 1, 1);

        selectPushButton = new QPushButton(GlobalRegistrationDialog);
        selectPushButton->setObjectName(QString::fromUtf8("selectPushButton"));

        gridLayout_5->addWidget(selectPushButton, 3, 1, 1, 1);

        sendRelationPushButton = new QPushButton(GlobalRegistrationDialog);
        sendRelationPushButton->setObjectName(QString::fromUtf8("sendRelationPushButton"));

        gridLayout_5->addWidget(sendRelationPushButton, 2, 1, 1, 1);

        estimatePushButton = new QPushButton(GlobalRegistrationDialog);
        estimatePushButton->setObjectName(QString::fromUtf8("estimatePushButton"));

        gridLayout_5->addWidget(estimatePushButton, 3, 0, 1, 1);

        label_11 = new QLabel(GlobalRegistrationDialog);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout_5->addWidget(label_11, 15, 0, 1, 1);

        ovlNumber2LineEdit = new QLineEdit(GlobalRegistrationDialog);
        ovlNumber2LineEdit->setObjectName(QString::fromUtf8("ovlNumber2LineEdit"));

        gridLayout_5->addWidget(ovlNumber2LineEdit, 16, 1, 1, 1);

        error2LineEdit = new QLineEdit(GlobalRegistrationDialog);
        error2LineEdit->setObjectName(QString::fromUtf8("error2LineEdit"));

        gridLayout_5->addWidget(error2LineEdit, 15, 1, 1, 1);

        label_13 = new QLabel(GlobalRegistrationDialog);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        gridLayout_5->addWidget(label_13, 16, 0, 1, 1);


        verticalLayout_5->addLayout(gridLayout_5);


        horizontalLayout_6->addLayout(verticalLayout_5);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        label_12 = new QLabel(GlobalRegistrationDialog);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        verticalLayout_6->addWidget(label_12);

        relationTextEdit = new QTextEdit(GlobalRegistrationDialog);
        relationTextEdit->setObjectName(QString::fromUtf8("relationTextEdit"));

        verticalLayout_6->addWidget(relationTextEdit);

        exportPushButton = new QPushButton(GlobalRegistrationDialog);
        exportPushButton->setObjectName(QString::fromUtf8("exportPushButton"));

        verticalLayout_6->addWidget(exportPushButton);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_6->addItem(verticalSpacer_2);


        horizontalLayout_6->addLayout(verticalLayout_6);


        retranslateUi(GlobalRegistrationDialog);
        QObject::connect(closePushButton, SIGNAL(clicked()), GlobalRegistrationDialog, SLOT(close()));

        QMetaObject::connectSlotsByName(GlobalRegistrationDialog);
    } // setupUi

    void retranslateUi(QDialog *GlobalRegistrationDialog)
    {
        GlobalRegistrationDialog->setWindowTitle(QApplication::translate("GlobalRegistrationDialog", "Global Registration Dialog", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("GlobalRegistrationDialog", "Cycle List", 0, QApplication::UnicodeUTF8));
        consolidatePushButton->setText(QApplication::translate("GlobalRegistrationDialog", "Consolidate", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("GlobalRegistrationDialog", "Sort", 0, QApplication::UnicodeUTF8));
        initializeCyclePushButton->setText(QApplication::translate("GlobalRegistrationDialog", "Initialize", 0, QApplication::UnicodeUTF8));
        addPushButton->setText(QApplication::translate("GlobalRegistrationDialog", "Add", 0, QApplication::UnicodeUTF8));
        deletePushButton->setText(QApplication::translate("GlobalRegistrationDialog", "Delete", 0, QApplication::UnicodeUTF8));
        closePushButton->setText(QApplication::translate("GlobalRegistrationDialog", "Close", 0, QApplication::UnicodeUTF8));
        consistentPushButton->setText(QApplication::translate("GlobalRegistrationDialog", "Non-uniform Consistent", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QApplication::translate("GlobalRegistrationDialog", "Uniform Consistent", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("GlobalRegistrationDialog", "Pair List", 0, QApplication::UnicodeUTF8));
        initializePairPushButton->setText(QApplication::translate("GlobalRegistrationDialog", "Initialize", 0, QApplication::UnicodeUTF8));
        deinitializePairPushButton->setText(QApplication::translate("GlobalRegistrationDialog", "Deinitialize", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("GlobalRegistrationDialog", "Transformation: T1 * T2 * ... * Tn", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("GlobalRegistrationDialog", "Translation : R1*...*Rn-1*tn+...+R1*t2+t1 ", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("GlobalRegistrationDialog", "Rotation : R1 * R2 * ... * Rn", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("GlobalRegistrationDialog", "Scale : c1 * c2 *...* cn", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("GlobalRegistrationDialog", "Axis", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("GlobalRegistrationDialog", "Angle", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("GlobalRegistrationDialog", "Overlapping Number 1", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("GlobalRegistrationDialog", "Accumulated RMS error 1", 0, QApplication::UnicodeUTF8));
        selectPushButton->setText(QApplication::translate("GlobalRegistrationDialog", "Select Best", 0, QApplication::UnicodeUTF8));
        sendRelationPushButton->setText(QApplication::translate("GlobalRegistrationDialog", "Send To Tranformations Relationship", 0, QApplication::UnicodeUTF8));
        estimatePushButton->setText(QApplication::translate("GlobalRegistrationDialog", "Estimate", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("GlobalRegistrationDialog", "Accumulated RMS error 2", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("GlobalRegistrationDialog", "Overlapping Number 2", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("GlobalRegistrationDialog", "Transformations Relationship", 0, QApplication::UnicodeUTF8));
        exportPushButton->setText(QApplication::translate("GlobalRegistrationDialog", "Export Transformations", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class GlobalRegistrationDialog: public Ui_GlobalRegistrationDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GLOBALREGISTRATIONDIALOG_H
