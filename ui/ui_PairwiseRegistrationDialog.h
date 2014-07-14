/********************************************************************************
** Form generated from reading UI file 'PairwiseRegistrationDialog.ui'
**
** Created: Sat Jul 12 15:53:40 2014
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PAIRWISEREGISTRATIONDIALOG_H
#define UI_PAIRWISEREGISTRATIONDIALOG_H

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
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QTabWidget>
#include <QtGui/QTextEdit>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_PairwiseRegistrationDialog
{
public:
    QHBoxLayout *horizontalLayout;
    QTabWidget *tabWidget;
    QVBoxLayout *verticalLayout;
    QGridLayout *gridLayout;
    QLabel *label_3;
    QLabel *label_2;
    QLabel *label;
    QLabel *label_9;
    QSpinBox *icpNumberSpinBox;
    QComboBox *methodComboBox;
    QComboBox *targetComboBox;
    QLabel *label_4;
    QDoubleSpinBox *distanceDoubleSpinBox;
    QComboBox *sourceComboBox;
    QLabel *label_8;
    QDoubleSpinBox *normalDoubleSpinBox;
    QLabel *label_14;
    QDoubleSpinBox *mlsRadiusDoubleSpinBox;
    QCheckBox *boundaryTestCheckBox;
    QSpacerItem *verticalSpacer_3;
    QGridLayout *gridLayout_2;
    QPushButton *initializePushButton;
    QPushButton *prePushButton;
    QPushButton *icpPushButton;
    QPushButton *exportPushButton;
    QPushButton *defaultPushButton;
    QPushButton *helpPushButton;
    QPushButton *closePushButton;
    QSpacerItem *verticalSpacer_2;
    QGridLayout *gridLayout_3;
    QLabel *label_17;
    QLabel *label_11;
    QTextEdit *RTextEdit;
    QLabel *label_12;
    QLabel *label_13;
    QLineEdit *cLineEdit;
    QTextEdit *TTextEdit;
    QLabel *label_10;
    QLabel *label_15;
    QLineEdit *tLineEdit;
    QLineEdit *axisLineEdit;
    QLabel *label_16;
    QLineEdit *angleLineEdit;
    QLineEdit *errorLineEdit;
    QLabel *label_18;
    QLineEdit *corrNumberLineEdit;
    QSpacerItem *verticalSpacer;

    void setupUi(QDialog *PairwiseRegistrationDialog)
    {
        if (PairwiseRegistrationDialog->objectName().isEmpty())
            PairwiseRegistrationDialog->setObjectName(QString::fromUtf8("PairwiseRegistrationDialog"));
        PairwiseRegistrationDialog->resize(1150, 855);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/bunny.png"), QSize(), QIcon::Normal, QIcon::Off);
        PairwiseRegistrationDialog->setWindowIcon(icon);
        horizontalLayout = new QHBoxLayout(PairwiseRegistrationDialog);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        tabWidget = new QTabWidget(PairwiseRegistrationDialog);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setMinimumSize(QSize(800, 600));

        horizontalLayout->addWidget(tabWidget);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_3 = new QLabel(PairwiseRegistrationDialog);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 0, 0, 1, 1);

        label_2 = new QLabel(PairwiseRegistrationDialog);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 4, 0, 1, 1);

        label = new QLabel(PairwiseRegistrationDialog);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 3, 0, 1, 1);

        label_9 = new QLabel(PairwiseRegistrationDialog);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout->addWidget(label_9, 6, 0, 1, 1);

        icpNumberSpinBox = new QSpinBox(PairwiseRegistrationDialog);
        icpNumberSpinBox->setObjectName(QString::fromUtf8("icpNumberSpinBox"));
        icpNumberSpinBox->setMinimum(1);
        icpNumberSpinBox->setValue(1);

        gridLayout->addWidget(icpNumberSpinBox, 6, 1, 1, 1);

        methodComboBox = new QComboBox(PairwiseRegistrationDialog);
        methodComboBox->setObjectName(QString::fromUtf8("methodComboBox"));

        gridLayout->addWidget(methodComboBox, 2, 1, 1, 1);

        targetComboBox = new QComboBox(PairwiseRegistrationDialog);
        targetComboBox->setObjectName(QString::fromUtf8("targetComboBox"));

        gridLayout->addWidget(targetComboBox, 0, 1, 1, 1);

        label_4 = new QLabel(PairwiseRegistrationDialog);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 1, 0, 1, 1);

        distanceDoubleSpinBox = new QDoubleSpinBox(PairwiseRegistrationDialog);
        distanceDoubleSpinBox->setObjectName(QString::fromUtf8("distanceDoubleSpinBox"));
        distanceDoubleSpinBox->setDecimals(8);
        distanceDoubleSpinBox->setSingleStep(0.001);
        distanceDoubleSpinBox->setValue(0.12);

        gridLayout->addWidget(distanceDoubleSpinBox, 3, 1, 1, 1);

        sourceComboBox = new QComboBox(PairwiseRegistrationDialog);
        sourceComboBox->setObjectName(QString::fromUtf8("sourceComboBox"));

        gridLayout->addWidget(sourceComboBox, 1, 1, 1, 1);

        label_8 = new QLabel(PairwiseRegistrationDialog);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout->addWidget(label_8, 2, 0, 1, 1);

        normalDoubleSpinBox = new QDoubleSpinBox(PairwiseRegistrationDialog);
        normalDoubleSpinBox->setObjectName(QString::fromUtf8("normalDoubleSpinBox"));
        normalDoubleSpinBox->setMaximum(180);
        normalDoubleSpinBox->setValue(45);

        gridLayout->addWidget(normalDoubleSpinBox, 4, 1, 1, 1);

        label_14 = new QLabel(PairwiseRegistrationDialog);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        gridLayout->addWidget(label_14, 5, 0, 1, 1);

        mlsRadiusDoubleSpinBox = new QDoubleSpinBox(PairwiseRegistrationDialog);
        mlsRadiusDoubleSpinBox->setObjectName(QString::fromUtf8("mlsRadiusDoubleSpinBox"));
        mlsRadiusDoubleSpinBox->setDecimals(8);
        mlsRadiusDoubleSpinBox->setSingleStep(0.001);

        gridLayout->addWidget(mlsRadiusDoubleSpinBox, 5, 1, 1, 1);


        verticalLayout->addLayout(gridLayout);

        boundaryTestCheckBox = new QCheckBox(PairwiseRegistrationDialog);
        boundaryTestCheckBox->setObjectName(QString::fromUtf8("boundaryTestCheckBox"));

        verticalLayout->addWidget(boundaryTestCheckBox);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_3);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        initializePushButton = new QPushButton(PairwiseRegistrationDialog);
        initializePushButton->setObjectName(QString::fromUtf8("initializePushButton"));

        gridLayout_2->addWidget(initializePushButton, 0, 0, 1, 1);

        prePushButton = new QPushButton(PairwiseRegistrationDialog);
        prePushButton->setObjectName(QString::fromUtf8("prePushButton"));

        gridLayout_2->addWidget(prePushButton, 0, 1, 1, 2);

        icpPushButton = new QPushButton(PairwiseRegistrationDialog);
        icpPushButton->setObjectName(QString::fromUtf8("icpPushButton"));

        gridLayout_2->addWidget(icpPushButton, 1, 0, 1, 1);

        exportPushButton = new QPushButton(PairwiseRegistrationDialog);
        exportPushButton->setObjectName(QString::fromUtf8("exportPushButton"));

        gridLayout_2->addWidget(exportPushButton, 1, 1, 1, 2);

        defaultPushButton = new QPushButton(PairwiseRegistrationDialog);
        defaultPushButton->setObjectName(QString::fromUtf8("defaultPushButton"));

        gridLayout_2->addWidget(defaultPushButton, 2, 0, 1, 1);

        helpPushButton = new QPushButton(PairwiseRegistrationDialog);
        helpPushButton->setObjectName(QString::fromUtf8("helpPushButton"));

        gridLayout_2->addWidget(helpPushButton, 2, 1, 1, 1);

        closePushButton = new QPushButton(PairwiseRegistrationDialog);
        closePushButton->setObjectName(QString::fromUtf8("closePushButton"));

        gridLayout_2->addWidget(closePushButton, 2, 2, 1, 1);


        verticalLayout->addLayout(gridLayout_2);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);

        gridLayout_3 = new QGridLayout();
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        label_17 = new QLabel(PairwiseRegistrationDialog);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        gridLayout_3->addWidget(label_17, 6, 0, 1, 1);

        label_11 = new QLabel(PairwiseRegistrationDialog);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout_3->addWidget(label_11, 0, 1, 1, 1);

        RTextEdit = new QTextEdit(PairwiseRegistrationDialog);
        RTextEdit->setObjectName(QString::fromUtf8("RTextEdit"));

        gridLayout_3->addWidget(RTextEdit, 1, 1, 1, 1);

        label_12 = new QLabel(PairwiseRegistrationDialog);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        gridLayout_3->addWidget(label_12, 2, 0, 1, 1);

        label_13 = new QLabel(PairwiseRegistrationDialog);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        gridLayout_3->addWidget(label_13, 2, 1, 1, 1);

        cLineEdit = new QLineEdit(PairwiseRegistrationDialog);
        cLineEdit->setObjectName(QString::fromUtf8("cLineEdit"));

        gridLayout_3->addWidget(cLineEdit, 3, 1, 1, 1);

        TTextEdit = new QTextEdit(PairwiseRegistrationDialog);
        TTextEdit->setObjectName(QString::fromUtf8("TTextEdit"));

        gridLayout_3->addWidget(TTextEdit, 1, 0, 1, 1);

        label_10 = new QLabel(PairwiseRegistrationDialog);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout_3->addWidget(label_10, 0, 0, 1, 1);

        label_15 = new QLabel(PairwiseRegistrationDialog);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        gridLayout_3->addWidget(label_15, 4, 0, 1, 1);

        tLineEdit = new QLineEdit(PairwiseRegistrationDialog);
        tLineEdit->setObjectName(QString::fromUtf8("tLineEdit"));

        gridLayout_3->addWidget(tLineEdit, 3, 0, 1, 1);

        axisLineEdit = new QLineEdit(PairwiseRegistrationDialog);
        axisLineEdit->setObjectName(QString::fromUtf8("axisLineEdit"));

        gridLayout_3->addWidget(axisLineEdit, 5, 0, 1, 1);

        label_16 = new QLabel(PairwiseRegistrationDialog);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        gridLayout_3->addWidget(label_16, 4, 1, 1, 1);

        angleLineEdit = new QLineEdit(PairwiseRegistrationDialog);
        angleLineEdit->setObjectName(QString::fromUtf8("angleLineEdit"));

        gridLayout_3->addWidget(angleLineEdit, 5, 1, 1, 1);

        errorLineEdit = new QLineEdit(PairwiseRegistrationDialog);
        errorLineEdit->setObjectName(QString::fromUtf8("errorLineEdit"));

        gridLayout_3->addWidget(errorLineEdit, 7, 0, 1, 1);

        label_18 = new QLabel(PairwiseRegistrationDialog);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        gridLayout_3->addWidget(label_18, 6, 1, 1, 1);

        corrNumberLineEdit = new QLineEdit(PairwiseRegistrationDialog);
        corrNumberLineEdit->setObjectName(QString::fromUtf8("corrNumberLineEdit"));

        gridLayout_3->addWidget(corrNumberLineEdit, 7, 1, 1, 1);


        verticalLayout->addLayout(gridLayout_3);

        verticalSpacer = new QSpacerItem(17, 13, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        horizontalLayout->addLayout(verticalLayout);

#ifndef QT_NO_SHORTCUT
        label_3->setBuddy(targetComboBox);
        label_2->setBuddy(normalDoubleSpinBox);
        label->setBuddy(distanceDoubleSpinBox);
        label_4->setBuddy(sourceComboBox);
#endif // QT_NO_SHORTCUT

        retranslateUi(PairwiseRegistrationDialog);
        QObject::connect(closePushButton, SIGNAL(clicked()), PairwiseRegistrationDialog, SLOT(close()));

        tabWidget->setCurrentIndex(-1);


        QMetaObject::connectSlotsByName(PairwiseRegistrationDialog);
    } // setupUi

    void retranslateUi(QDialog *PairwiseRegistrationDialog)
    {
        PairwiseRegistrationDialog->setWindowTitle(QApplication::translate("PairwiseRegistrationDialog", "Pairwise Registration Dialog", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("PairwiseRegistrationDialog", "Target", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("PairwiseRegistrationDialog", "Normal Angle Threshold", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("PairwiseRegistrationDialog", "Distance Threshold", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("PairwiseRegistrationDialog", "ICP iteration number", 0, QApplication::UnicodeUTF8));
        methodComboBox->clear();
        methodComboBox->insertItems(0, QStringList()
         << QApplication::translate("PairwiseRegistrationDialog", "Point to Point", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("PairwiseRegistrationDialog", "Point to Plane", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("PairwiseRegistrationDialog", "Point to MLSSurface", 0, QApplication::UnicodeUTF8)
        );
        label_4->setText(QApplication::translate("PairwiseRegistrationDialog", "Source", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("PairwiseRegistrationDialog", "Method", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("PairwiseRegistrationDialog", "MLS Search Radius", 0, QApplication::UnicodeUTF8));
        boundaryTestCheckBox->setText(QApplication::translate("PairwiseRegistrationDialog", "Boundary Test", 0, QApplication::UnicodeUTF8));
        initializePushButton->setText(QApplication::translate("PairwiseRegistrationDialog", "(Re)Initialize", 0, QApplication::UnicodeUTF8));
        prePushButton->setText(QApplication::translate("PairwiseRegistrationDialog", "Pre-Correspondences", 0, QApplication::UnicodeUTF8));
        icpPushButton->setText(QApplication::translate("PairwiseRegistrationDialog", "start ICP", 0, QApplication::UnicodeUTF8));
        exportPushButton->setText(QApplication::translate("PairwiseRegistrationDialog", "Export Transformation", 0, QApplication::UnicodeUTF8));
        defaultPushButton->setText(QApplication::translate("PairwiseRegistrationDialog", "Default", 0, QApplication::UnicodeUTF8));
        helpPushButton->setText(QApplication::translate("PairwiseRegistrationDialog", "Help", 0, QApplication::UnicodeUTF8));
        closePushButton->setText(QApplication::translate("PairwiseRegistrationDialog", "Close", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("PairwiseRegistrationDialog", "RMS error", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("PairwiseRegistrationDialog", "Rotation", 0, QApplication::UnicodeUTF8));
        RTextEdit->setHtml(QApplication::translate("PairwiseRegistrationDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:12pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("PairwiseRegistrationDialog", "Translation", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("PairwiseRegistrationDialog", "Scale", 0, QApplication::UnicodeUTF8));
        cLineEdit->setText(QString());
        TTextEdit->setHtml(QApplication::translate("PairwiseRegistrationDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:12pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("PairwiseRegistrationDialog", "T", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("PairwiseRegistrationDialog", "axis", 0, QApplication::UnicodeUTF8));
        tLineEdit->setText(QString());
        label_16->setText(QApplication::translate("PairwiseRegistrationDialog", "angle", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("PairwiseRegistrationDialog", "Correspondence Number", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PairwiseRegistrationDialog: public Ui_PairwiseRegistrationDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PAIRWISEREGISTRATIONDIALOG_H
