/********************************************************************************
** Form generated from reading UI file 'EuclideanClusterExtractionDialog.ui'
**
** Created: Sat Jul 12 15:53:40 2014
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_EUCLIDEANCLUSTEREXTRACTIONDIALOG_H
#define UI_EUCLIDEANCLUSTEREXTRACTIONDIALOG_H

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
#include <QtGui/QRadioButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_EuclideanClusterExtractionDialog
{
public:
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout;
    QGridLayout *gridLayout_2;
    QLabel *clusterToleranceLabel;
    QLineEdit *clusterToleranceLineEdit;
    QLabel *minClusterSizeLabel;
    QLineEdit *minClusterSizeLineEdit;
    QLabel *maxClusterSizeLabel;
    QLineEdit *maxClusterSizeLineEdit;
    QVBoxLayout *verticalLayout;
    QCheckBox *overwriteCheckBox;
    QRadioButton *cpuRadioButton;
    QRadioButton *gpuRadioButton;
    QSpacerItem *verticalSpacer;
    QGridLayout *gridLayout;
    QPushButton *defaultButton;
    QPushButton *helpButton;
    QPushButton *closeButton;
    QPushButton *applyButton;

    void setupUi(QDialog *EuclideanClusterExtractionDialog)
    {
        if (EuclideanClusterExtractionDialog->objectName().isEmpty())
            EuclideanClusterExtractionDialog->setObjectName(QString::fromUtf8("EuclideanClusterExtractionDialog"));
        EuclideanClusterExtractionDialog->resize(377, 174);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/bunny.png"), QSize(), QIcon::Normal, QIcon::Off);
        EuclideanClusterExtractionDialog->setWindowIcon(icon);
        verticalLayout_2 = new QVBoxLayout(EuclideanClusterExtractionDialog);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        clusterToleranceLabel = new QLabel(EuclideanClusterExtractionDialog);
        clusterToleranceLabel->setObjectName(QString::fromUtf8("clusterToleranceLabel"));

        gridLayout_2->addWidget(clusterToleranceLabel, 0, 0, 1, 1);

        clusterToleranceLineEdit = new QLineEdit(EuclideanClusterExtractionDialog);
        clusterToleranceLineEdit->setObjectName(QString::fromUtf8("clusterToleranceLineEdit"));

        gridLayout_2->addWidget(clusterToleranceLineEdit, 0, 1, 1, 1);

        minClusterSizeLabel = new QLabel(EuclideanClusterExtractionDialog);
        minClusterSizeLabel->setObjectName(QString::fromUtf8("minClusterSizeLabel"));

        gridLayout_2->addWidget(minClusterSizeLabel, 1, 0, 1, 1);

        minClusterSizeLineEdit = new QLineEdit(EuclideanClusterExtractionDialog);
        minClusterSizeLineEdit->setObjectName(QString::fromUtf8("minClusterSizeLineEdit"));

        gridLayout_2->addWidget(minClusterSizeLineEdit, 1, 1, 1, 1);

        maxClusterSizeLabel = new QLabel(EuclideanClusterExtractionDialog);
        maxClusterSizeLabel->setObjectName(QString::fromUtf8("maxClusterSizeLabel"));

        gridLayout_2->addWidget(maxClusterSizeLabel, 2, 0, 1, 1);

        maxClusterSizeLineEdit = new QLineEdit(EuclideanClusterExtractionDialog);
        maxClusterSizeLineEdit->setObjectName(QString::fromUtf8("maxClusterSizeLineEdit"));

        gridLayout_2->addWidget(maxClusterSizeLineEdit, 2, 1, 1, 1);


        horizontalLayout->addLayout(gridLayout_2);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        overwriteCheckBox = new QCheckBox(EuclideanClusterExtractionDialog);
        overwriteCheckBox->setObjectName(QString::fromUtf8("overwriteCheckBox"));
        overwriteCheckBox->setChecked(true);

        verticalLayout->addWidget(overwriteCheckBox);

        cpuRadioButton = new QRadioButton(EuclideanClusterExtractionDialog);
        cpuRadioButton->setObjectName(QString::fromUtf8("cpuRadioButton"));
        cpuRadioButton->setChecked(false);

        verticalLayout->addWidget(cpuRadioButton);

        gpuRadioButton = new QRadioButton(EuclideanClusterExtractionDialog);
        gpuRadioButton->setObjectName(QString::fromUtf8("gpuRadioButton"));
        gpuRadioButton->setChecked(true);

        verticalLayout->addWidget(gpuRadioButton);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        horizontalLayout->addLayout(verticalLayout);


        verticalLayout_2->addLayout(horizontalLayout);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        defaultButton = new QPushButton(EuclideanClusterExtractionDialog);
        defaultButton->setObjectName(QString::fromUtf8("defaultButton"));

        gridLayout->addWidget(defaultButton, 0, 0, 1, 1);

        helpButton = new QPushButton(EuclideanClusterExtractionDialog);
        helpButton->setObjectName(QString::fromUtf8("helpButton"));

        gridLayout->addWidget(helpButton, 0, 1, 1, 1);

        closeButton = new QPushButton(EuclideanClusterExtractionDialog);
        closeButton->setObjectName(QString::fromUtf8("closeButton"));

        gridLayout->addWidget(closeButton, 1, 0, 1, 1);

        applyButton = new QPushButton(EuclideanClusterExtractionDialog);
        applyButton->setObjectName(QString::fromUtf8("applyButton"));

        gridLayout->addWidget(applyButton, 1, 1, 1, 1);


        verticalLayout_2->addLayout(gridLayout);


        retranslateUi(EuclideanClusterExtractionDialog);
        QObject::connect(closeButton, SIGNAL(clicked()), EuclideanClusterExtractionDialog, SLOT(close()));

        QMetaObject::connectSlotsByName(EuclideanClusterExtractionDialog);
    } // setupUi

    void retranslateUi(QDialog *EuclideanClusterExtractionDialog)
    {
        EuclideanClusterExtractionDialog->setWindowTitle(QApplication::translate("EuclideanClusterExtractionDialog", "Euclidean Cluster Extraction Dialog", 0, QApplication::UnicodeUTF8));
        clusterToleranceLabel->setText(QApplication::translate("EuclideanClusterExtractionDialog", "ClusterTolerance", 0, QApplication::UnicodeUTF8));
        clusterToleranceLineEdit->setText(QApplication::translate("EuclideanClusterExtractionDialog", "0.01", 0, QApplication::UnicodeUTF8));
        minClusterSizeLabel->setText(QApplication::translate("EuclideanClusterExtractionDialog", "MinClusterSize", 0, QApplication::UnicodeUTF8));
        minClusterSizeLineEdit->setText(QApplication::translate("EuclideanClusterExtractionDialog", "1", 0, QApplication::UnicodeUTF8));
        maxClusterSizeLabel->setText(QApplication::translate("EuclideanClusterExtractionDialog", "MaxClusterSize", 0, QApplication::UnicodeUTF8));
        maxClusterSizeLineEdit->setText(QApplication::translate("EuclideanClusterExtractionDialog", "10000000", 0, QApplication::UnicodeUTF8));
        overwriteCheckBox->setText(QApplication::translate("EuclideanClusterExtractionDialog", "overwrite", 0, QApplication::UnicodeUTF8));
        cpuRadioButton->setText(QApplication::translate("EuclideanClusterExtractionDialog", "CPU", 0, QApplication::UnicodeUTF8));
        gpuRadioButton->setText(QApplication::translate("EuclideanClusterExtractionDialog", "GPU", 0, QApplication::UnicodeUTF8));
        defaultButton->setText(QApplication::translate("EuclideanClusterExtractionDialog", "Default", 0, QApplication::UnicodeUTF8));
        helpButton->setText(QApplication::translate("EuclideanClusterExtractionDialog", "Help", 0, QApplication::UnicodeUTF8));
        closeButton->setText(QApplication::translate("EuclideanClusterExtractionDialog", "Close", 0, QApplication::UnicodeUTF8));
        applyButton->setText(QApplication::translate("EuclideanClusterExtractionDialog", "Apply", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class EuclideanClusterExtractionDialog: public Ui_EuclideanClusterExtractionDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_EUCLIDEANCLUSTEREXTRACTIONDIALOG_H
