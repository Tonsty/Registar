/********************************************************************************
** Form generated from reading UI file 'MovingLeastSquaresDialog.ui'
**
** Created: Sat Jul 12 15:53:40 2014
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MOVINGLEASTSQUARESDIALOG_H
#define UI_MOVINGLEASTSQUARESDIALOG_H

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
#include <QtGui/QRadioButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_MovingLeastSquaresDialog
{
public:
    QVBoxLayout *verticalLayout_3;
    QGridLayout *gridLayout_2;
    QLabel *label_3;
    QComboBox *methodComboBox;
    QLabel *label;
    QLineEdit *radiusLineEdit;
    QLabel *label_7;
    QComboBox *cloudComboBox;
    QSpacerItem *horizontalSpacer;
    QLabel *label_6;
    QLineEdit *upsamplingRadiusLineEdit;
    QLabel *label_2;
    QLineEdit *stepSizeLineEdit;
    QLabel *label_4;
    QSpinBox *densitySpinBox;
    QSpacerItem *horizontalSpacer_2;
    QLabel *label_5;
    QLineEdit *voxelSizeLineEdit;
    QLabel *label_8;
    QSpinBox *iterationsSpinBox;
    QSpacerItem *horizontalSpacer_3;
    QSpacerItem *horizontalSpacer_4;
    QHBoxLayout *horizontalLayout;
    QGridLayout *gridLayout;
    QPushButton *defaultButton;
    QPushButton *helpButton;
    QPushButton *closeButton;
    QPushButton *applyButton;
    QSpacerItem *verticalSpacer_2;
    QSpacerItem *verticalSpacer_3;
    QVBoxLayout *verticalLayout_2;
    QCheckBox *computeNormalCheckBox;
    QCheckBox *overwriteCheckBox;
    QSpacerItem *verticalSpacer;
    QVBoxLayout *verticalLayout;
    QRadioButton *scpuRadioButton;
    QRadioButton *mcpuRadioButton;
    QRadioButton *gpuRadioButton;

    void setupUi(QDialog *MovingLeastSquaresDialog)
    {
        if (MovingLeastSquaresDialog->objectName().isEmpty())
            MovingLeastSquaresDialog->setObjectName(QString::fromUtf8("MovingLeastSquaresDialog"));
        MovingLeastSquaresDialog->resize(524, 215);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/bunny.png"), QSize(), QIcon::Normal, QIcon::Off);
        MovingLeastSquaresDialog->setWindowIcon(icon);
        verticalLayout_3 = new QVBoxLayout(MovingLeastSquaresDialog);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        label_3 = new QLabel(MovingLeastSquaresDialog);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_2->addWidget(label_3, 0, 0, 1, 1);

        methodComboBox = new QComboBox(MovingLeastSquaresDialog);
        methodComboBox->setObjectName(QString::fromUtf8("methodComboBox"));

        gridLayout_2->addWidget(methodComboBox, 0, 1, 1, 2);

        label = new QLabel(MovingLeastSquaresDialog);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_2->addWidget(label, 0, 3, 1, 1);

        radiusLineEdit = new QLineEdit(MovingLeastSquaresDialog);
        radiusLineEdit->setObjectName(QString::fromUtf8("radiusLineEdit"));

        gridLayout_2->addWidget(radiusLineEdit, 0, 4, 1, 1);

        label_7 = new QLabel(MovingLeastSquaresDialog);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_2->addWidget(label_7, 1, 0, 1, 1);

        cloudComboBox = new QComboBox(MovingLeastSquaresDialog);
        cloudComboBox->setObjectName(QString::fromUtf8("cloudComboBox"));

        gridLayout_2->addWidget(cloudComboBox, 1, 1, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer, 1, 3, 1, 1);

        label_6 = new QLabel(MovingLeastSquaresDialog);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout_2->addWidget(label_6, 2, 0, 1, 1);

        upsamplingRadiusLineEdit = new QLineEdit(MovingLeastSquaresDialog);
        upsamplingRadiusLineEdit->setObjectName(QString::fromUtf8("upsamplingRadiusLineEdit"));

        gridLayout_2->addWidget(upsamplingRadiusLineEdit, 2, 1, 1, 1);

        label_2 = new QLabel(MovingLeastSquaresDialog);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_2->addWidget(label_2, 2, 3, 1, 1);

        stepSizeLineEdit = new QLineEdit(MovingLeastSquaresDialog);
        stepSizeLineEdit->setObjectName(QString::fromUtf8("stepSizeLineEdit"));

        gridLayout_2->addWidget(stepSizeLineEdit, 2, 4, 1, 1);

        label_4 = new QLabel(MovingLeastSquaresDialog);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_2->addWidget(label_4, 3, 0, 1, 1);

        densitySpinBox = new QSpinBox(MovingLeastSquaresDialog);
        densitySpinBox->setObjectName(QString::fromUtf8("densitySpinBox"));
        densitySpinBox->setMinimum(1);
        densitySpinBox->setMaximum(999);
        densitySpinBox->setValue(30);

        gridLayout_2->addWidget(densitySpinBox, 3, 1, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer_2, 3, 3, 1, 1);

        label_5 = new QLabel(MovingLeastSquaresDialog);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout_2->addWidget(label_5, 4, 0, 1, 1);

        voxelSizeLineEdit = new QLineEdit(MovingLeastSquaresDialog);
        voxelSizeLineEdit->setObjectName(QString::fromUtf8("voxelSizeLineEdit"));

        gridLayout_2->addWidget(voxelSizeLineEdit, 4, 1, 1, 1);

        label_8 = new QLabel(MovingLeastSquaresDialog);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout_2->addWidget(label_8, 4, 2, 1, 2);

        iterationsSpinBox = new QSpinBox(MovingLeastSquaresDialog);
        iterationsSpinBox->setObjectName(QString::fromUtf8("iterationsSpinBox"));
        iterationsSpinBox->setMinimum(0);
        iterationsSpinBox->setValue(1);

        gridLayout_2->addWidget(iterationsSpinBox, 4, 4, 1, 1);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer_3, 1, 4, 1, 1);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer_4, 3, 4, 1, 1);


        verticalLayout_3->addLayout(gridLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        defaultButton = new QPushButton(MovingLeastSquaresDialog);
        defaultButton->setObjectName(QString::fromUtf8("defaultButton"));

        gridLayout->addWidget(defaultButton, 0, 0, 1, 1);

        helpButton = new QPushButton(MovingLeastSquaresDialog);
        helpButton->setObjectName(QString::fromUtf8("helpButton"));

        gridLayout->addWidget(helpButton, 0, 1, 1, 1);

        closeButton = new QPushButton(MovingLeastSquaresDialog);
        closeButton->setObjectName(QString::fromUtf8("closeButton"));

        gridLayout->addWidget(closeButton, 1, 0, 1, 1);

        applyButton = new QPushButton(MovingLeastSquaresDialog);
        applyButton->setObjectName(QString::fromUtf8("applyButton"));

        gridLayout->addWidget(applyButton, 1, 1, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer_2, 2, 0, 1, 1);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer_3, 2, 1, 1, 1);


        horizontalLayout->addLayout(gridLayout);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        computeNormalCheckBox = new QCheckBox(MovingLeastSquaresDialog);
        computeNormalCheckBox->setObjectName(QString::fromUtf8("computeNormalCheckBox"));
        computeNormalCheckBox->setChecked(true);

        verticalLayout_2->addWidget(computeNormalCheckBox);

        overwriteCheckBox = new QCheckBox(MovingLeastSquaresDialog);
        overwriteCheckBox->setObjectName(QString::fromUtf8("overwriteCheckBox"));
        overwriteCheckBox->setChecked(true);

        verticalLayout_2->addWidget(overwriteCheckBox);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);


        horizontalLayout->addLayout(verticalLayout_2);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        scpuRadioButton = new QRadioButton(MovingLeastSquaresDialog);
        scpuRadioButton->setObjectName(QString::fromUtf8("scpuRadioButton"));

        verticalLayout->addWidget(scpuRadioButton);

        mcpuRadioButton = new QRadioButton(MovingLeastSquaresDialog);
        mcpuRadioButton->setObjectName(QString::fromUtf8("mcpuRadioButton"));
        mcpuRadioButton->setChecked(true);

        verticalLayout->addWidget(mcpuRadioButton);

        gpuRadioButton = new QRadioButton(MovingLeastSquaresDialog);
        gpuRadioButton->setObjectName(QString::fromUtf8("gpuRadioButton"));

        verticalLayout->addWidget(gpuRadioButton);


        horizontalLayout->addLayout(verticalLayout);


        verticalLayout_3->addLayout(horizontalLayout);

#ifndef QT_NO_SHORTCUT
        label_3->setBuddy(methodComboBox);
        label->setBuddy(radiusLineEdit);
        label_7->setBuddy(cloudComboBox);
        label_6->setBuddy(upsamplingRadiusLineEdit);
        label_2->setBuddy(stepSizeLineEdit);
        label_4->setBuddy(densitySpinBox);
        label_5->setBuddy(voxelSizeLineEdit);
        label_8->setBuddy(iterationsSpinBox);
#endif // QT_NO_SHORTCUT

        retranslateUi(MovingLeastSquaresDialog);
        QObject::connect(closeButton, SIGNAL(clicked()), MovingLeastSquaresDialog, SLOT(close()));

        QMetaObject::connectSlotsByName(MovingLeastSquaresDialog);
    } // setupUi

    void retranslateUi(QDialog *MovingLeastSquaresDialog)
    {
        MovingLeastSquaresDialog->setWindowTitle(QApplication::translate("MovingLeastSquaresDialog", "Moving Least Squares Dialog", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MovingLeastSquaresDialog", "Upsampling &Method", 0, QApplication::UnicodeUTF8));
        methodComboBox->clear();
        methodComboBox->insertItems(0, QStringList()
         << QApplication::translate("MovingLeastSquaresDialog", "None", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MovingLeastSquaresDialog", "Distinct Cloud", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MovingLeastSquaresDialog", "Sample Local Plane", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MovingLeastSquaresDialog", "Random Uniform Density", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MovingLeastSquaresDialog", "Voxel Grid Dilation", 0, QApplication::UnicodeUTF8)
        );
        label->setText(QApplication::translate("MovingLeastSquaresDialog", "Search &Radius", 0, QApplication::UnicodeUTF8));
        radiusLineEdit->setText(QApplication::translate("MovingLeastSquaresDialog", "0.06", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MovingLeastSquaresDialog", "Distinct &Cloud", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MovingLeastSquaresDialog", "&Upsampling Radius", 0, QApplication::UnicodeUTF8));
        upsamplingRadiusLineEdit->setText(QApplication::translate("MovingLeastSquaresDialog", "0.04", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MovingLeastSquaresDialog", "Upsampling &Step Size", 0, QApplication::UnicodeUTF8));
        stepSizeLineEdit->setText(QApplication::translate("MovingLeastSquaresDialog", "0.01", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MovingLeastSquaresDialog", "Point &Density", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MovingLeastSquaresDialog", "Dilation &Voxel Size", 0, QApplication::UnicodeUTF8));
        voxelSizeLineEdit->setText(QApplication::translate("MovingLeastSquaresDialog", "0.025", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("MovingLeastSquaresDialog", "Dilation &Iterations", 0, QApplication::UnicodeUTF8));
        defaultButton->setText(QApplication::translate("MovingLeastSquaresDialog", "Default", 0, QApplication::UnicodeUTF8));
        helpButton->setText(QApplication::translate("MovingLeastSquaresDialog", "Help", 0, QApplication::UnicodeUTF8));
        closeButton->setText(QApplication::translate("MovingLeastSquaresDialog", "Close", 0, QApplication::UnicodeUTF8));
        applyButton->setText(QApplication::translate("MovingLeastSquaresDialog", "Apply", 0, QApplication::UnicodeUTF8));
        computeNormalCheckBox->setText(QApplication::translate("MovingLeastSquaresDialog", "Compute Normal", 0, QApplication::UnicodeUTF8));
        overwriteCheckBox->setText(QApplication::translate("MovingLeastSquaresDialog", "overwrite", 0, QApplication::UnicodeUTF8));
        scpuRadioButton->setText(QApplication::translate("MovingLeastSquaresDialog", "Single CPU", 0, QApplication::UnicodeUTF8));
        mcpuRadioButton->setText(QApplication::translate("MovingLeastSquaresDialog", "Multiple CPUs", 0, QApplication::UnicodeUTF8));
        gpuRadioButton->setText(QApplication::translate("MovingLeastSquaresDialog", "GPU", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MovingLeastSquaresDialog: public Ui_MovingLeastSquaresDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MOVINGLEASTSQUARESDIALOG_H
