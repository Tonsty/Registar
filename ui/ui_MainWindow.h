/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created: Sat Jul 12 15:53:40 2014
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDockWidget>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "../include/cloudbrowser.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *openAction;
    QAction *saveAction;
    QAction *saveAsAction;
    QAction *exitAction;
    QAction *aboutAction;
    QAction *aboutQtAction;
    QAction *voxelGridAction;
    QAction *movingLeastSquaresAction;
    QAction *showCloudBrowserAction;
    QAction *showPointBrowserAction;
    QAction *colorNoneAction;
    QAction *colorOriginalAction;
    QAction *colorCustomAction;
    QAction *drawNormalAction;
    QAction *boundaryEstimationAction;
    QAction *outliersRemovalAction;
    QAction *pairwiseRegistrationAction;
    QAction *drawAxisAction;
    QAction *drawOrientationMarkerAction;
    QAction *translationAction;
    QAction *rotationAction;
    QAction *scaleAction;
    QAction *euclideanClusterExtractionAction;
    QAction *normalFieldAction;
    QAction *concatenationAction;
    QAction *diagramAction;
    QAction *globalRegistrationAction;
    QAction *registrationModeAction;
    QAction *drawBoundaryAction;
    QWidget *centralwidget;
    QMenuBar *menubar;
    QMenu *fileMenu;
    QMenu *editMenu;
    QMenu *toolsMenu;
    QMenu *globalRegistrationMenu;
    QMenu *optionsMenu;
    QMenu *helpMenu;
    QMenu *menuFilters;
    QMenu *menuTransformation;
    QMenu *menuFieldsOperation;
    QMenu *renderMenu;
    QMenu *menu_Color;
    QStatusBar *statusbar;
    QToolBar *fileToolBar;
    QDockWidget *cloudBrowserDockWidget;
    QWidget *cloudBrowserDockWidgetContents;
    QVBoxLayout *verticalLayout;
    CloudBrowser *cloudBrowser;
    QDockWidget *pointBrowserDockWidget;
    QWidget *pointBrowserDockWidgetContents;
    QToolBar *visualizerBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1068, 808);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/bunny.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindow->setWindowIcon(icon);
        openAction = new QAction(MainWindow);
        openAction->setObjectName(QString::fromUtf8("openAction"));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/images/open.png"), QSize(), QIcon::Normal, QIcon::Off);
        openAction->setIcon(icon1);
        saveAction = new QAction(MainWindow);
        saveAction->setObjectName(QString::fromUtf8("saveAction"));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/images/save.png"), QSize(), QIcon::Normal, QIcon::Off);
        saveAction->setIcon(icon2);
        saveAsAction = new QAction(MainWindow);
        saveAsAction->setObjectName(QString::fromUtf8("saveAsAction"));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/images/saveAs.png"), QSize(), QIcon::Normal, QIcon::Off);
        saveAsAction->setIcon(icon3);
        exitAction = new QAction(MainWindow);
        exitAction->setObjectName(QString::fromUtf8("exitAction"));
        aboutAction = new QAction(MainWindow);
        aboutAction->setObjectName(QString::fromUtf8("aboutAction"));
        aboutAction->setIcon(icon);
        aboutQtAction = new QAction(MainWindow);
        aboutQtAction->setObjectName(QString::fromUtf8("aboutQtAction"));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/images/qt-logo.png"), QSize(), QIcon::Normal, QIcon::Off);
        aboutQtAction->setIcon(icon4);
        voxelGridAction = new QAction(MainWindow);
        voxelGridAction->setObjectName(QString::fromUtf8("voxelGridAction"));
        movingLeastSquaresAction = new QAction(MainWindow);
        movingLeastSquaresAction->setObjectName(QString::fromUtf8("movingLeastSquaresAction"));
        showCloudBrowserAction = new QAction(MainWindow);
        showCloudBrowserAction->setObjectName(QString::fromUtf8("showCloudBrowserAction"));
        showCloudBrowserAction->setCheckable(true);
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/images/layers.png"), QSize(), QIcon::Normal, QIcon::Off);
        showCloudBrowserAction->setIcon(icon5);
        showPointBrowserAction = new QAction(MainWindow);
        showPointBrowserAction->setObjectName(QString::fromUtf8("showPointBrowserAction"));
        showPointBrowserAction->setCheckable(true);
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/images/points.png"), QSize(), QIcon::Normal, QIcon::Off);
        showPointBrowserAction->setIcon(icon6);
        colorNoneAction = new QAction(MainWindow);
        colorNoneAction->setObjectName(QString::fromUtf8("colorNoneAction"));
        colorNoneAction->setCheckable(true);
        colorOriginalAction = new QAction(MainWindow);
        colorOriginalAction->setObjectName(QString::fromUtf8("colorOriginalAction"));
        colorOriginalAction->setCheckable(true);
        colorOriginalAction->setChecked(true);
        colorCustomAction = new QAction(MainWindow);
        colorCustomAction->setObjectName(QString::fromUtf8("colorCustomAction"));
        colorCustomAction->setCheckable(true);
        drawNormalAction = new QAction(MainWindow);
        drawNormalAction->setObjectName(QString::fromUtf8("drawNormalAction"));
        drawNormalAction->setCheckable(true);
        boundaryEstimationAction = new QAction(MainWindow);
        boundaryEstimationAction->setObjectName(QString::fromUtf8("boundaryEstimationAction"));
        outliersRemovalAction = new QAction(MainWindow);
        outliersRemovalAction->setObjectName(QString::fromUtf8("outliersRemovalAction"));
        pairwiseRegistrationAction = new QAction(MainWindow);
        pairwiseRegistrationAction->setObjectName(QString::fromUtf8("pairwiseRegistrationAction"));
        drawAxisAction = new QAction(MainWindow);
        drawAxisAction->setObjectName(QString::fromUtf8("drawAxisAction"));
        drawAxisAction->setCheckable(true);
        drawOrientationMarkerAction = new QAction(MainWindow);
        drawOrientationMarkerAction->setObjectName(QString::fromUtf8("drawOrientationMarkerAction"));
        drawOrientationMarkerAction->setCheckable(true);
        translationAction = new QAction(MainWindow);
        translationAction->setObjectName(QString::fromUtf8("translationAction"));
        rotationAction = new QAction(MainWindow);
        rotationAction->setObjectName(QString::fromUtf8("rotationAction"));
        scaleAction = new QAction(MainWindow);
        scaleAction->setObjectName(QString::fromUtf8("scaleAction"));
        euclideanClusterExtractionAction = new QAction(MainWindow);
        euclideanClusterExtractionAction->setObjectName(QString::fromUtf8("euclideanClusterExtractionAction"));
        normalFieldAction = new QAction(MainWindow);
        normalFieldAction->setObjectName(QString::fromUtf8("normalFieldAction"));
        concatenationAction = new QAction(MainWindow);
        concatenationAction->setObjectName(QString::fromUtf8("concatenationAction"));
        diagramAction = new QAction(MainWindow);
        diagramAction->setObjectName(QString::fromUtf8("diagramAction"));
        globalRegistrationAction = new QAction(MainWindow);
        globalRegistrationAction->setObjectName(QString::fromUtf8("globalRegistrationAction"));
        registrationModeAction = new QAction(MainWindow);
        registrationModeAction->setObjectName(QString::fromUtf8("registrationModeAction"));
        registrationModeAction->setCheckable(true);
        drawBoundaryAction = new QAction(MainWindow);
        drawBoundaryAction->setObjectName(QString::fromUtf8("drawBoundaryAction"));
        drawBoundaryAction->setCheckable(true);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1068, 23));
        fileMenu = new QMenu(menubar);
        fileMenu->setObjectName(QString::fromUtf8("fileMenu"));
        editMenu = new QMenu(menubar);
        editMenu->setObjectName(QString::fromUtf8("editMenu"));
        toolsMenu = new QMenu(menubar);
        toolsMenu->setObjectName(QString::fromUtf8("toolsMenu"));
        globalRegistrationMenu = new QMenu(toolsMenu);
        globalRegistrationMenu->setObjectName(QString::fromUtf8("globalRegistrationMenu"));
        optionsMenu = new QMenu(menubar);
        optionsMenu->setObjectName(QString::fromUtf8("optionsMenu"));
        helpMenu = new QMenu(menubar);
        helpMenu->setObjectName(QString::fromUtf8("helpMenu"));
        menuFilters = new QMenu(menubar);
        menuFilters->setObjectName(QString::fromUtf8("menuFilters"));
        menuTransformation = new QMenu(menuFilters);
        menuTransformation->setObjectName(QString::fromUtf8("menuTransformation"));
        menuFieldsOperation = new QMenu(menuFilters);
        menuFieldsOperation->setObjectName(QString::fromUtf8("menuFieldsOperation"));
        renderMenu = new QMenu(menubar);
        renderMenu->setObjectName(QString::fromUtf8("renderMenu"));
        menu_Color = new QMenu(renderMenu);
        menu_Color->setObjectName(QString::fromUtf8("menu_Color"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);
        fileToolBar = new QToolBar(MainWindow);
        fileToolBar->setObjectName(QString::fromUtf8("fileToolBar"));
        fileToolBar->setMinimumSize(QSize(0, 0));
        MainWindow->addToolBar(Qt::TopToolBarArea, fileToolBar);
        cloudBrowserDockWidget = new QDockWidget(MainWindow);
        cloudBrowserDockWidget->setObjectName(QString::fromUtf8("cloudBrowserDockWidget"));
        cloudBrowserDockWidget->setWindowIcon(icon5);
        cloudBrowserDockWidgetContents = new QWidget();
        cloudBrowserDockWidgetContents->setObjectName(QString::fromUtf8("cloudBrowserDockWidgetContents"));
        verticalLayout = new QVBoxLayout(cloudBrowserDockWidgetContents);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        cloudBrowser = new CloudBrowser(cloudBrowserDockWidgetContents);
        cloudBrowser->setObjectName(QString::fromUtf8("cloudBrowser"));
        cloudBrowser->setSelectionMode(QAbstractItemView::ExtendedSelection);
        cloudBrowser->setColumnCount(3);

        verticalLayout->addWidget(cloudBrowser);

        cloudBrowserDockWidget->setWidget(cloudBrowserDockWidgetContents);
        MainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(2), cloudBrowserDockWidget);
        pointBrowserDockWidget = new QDockWidget(MainWindow);
        pointBrowserDockWidget->setObjectName(QString::fromUtf8("pointBrowserDockWidget"));
        pointBrowserDockWidget->setWindowIcon(icon6);
        pointBrowserDockWidgetContents = new QWidget();
        pointBrowserDockWidgetContents->setObjectName(QString::fromUtf8("pointBrowserDockWidgetContents"));
        pointBrowserDockWidget->setWidget(pointBrowserDockWidgetContents);
        MainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(2), pointBrowserDockWidget);
        visualizerBar = new QToolBar(MainWindow);
        visualizerBar->setObjectName(QString::fromUtf8("visualizerBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, visualizerBar);

        menubar->addAction(fileMenu->menuAction());
        menubar->addAction(editMenu->menuAction());
        menubar->addAction(menuFilters->menuAction());
        menubar->addAction(renderMenu->menuAction());
        menubar->addAction(toolsMenu->menuAction());
        menubar->addAction(optionsMenu->menuAction());
        menubar->addAction(helpMenu->menuAction());
        fileMenu->addAction(openAction);
        fileMenu->addAction(saveAction);
        fileMenu->addAction(saveAsAction);
        fileMenu->addSeparator();
        fileMenu->addAction(exitAction);
        toolsMenu->addAction(pairwiseRegistrationAction);
        toolsMenu->addAction(globalRegistrationMenu->menuAction());
        globalRegistrationMenu->addAction(diagramAction);
        globalRegistrationMenu->addAction(globalRegistrationAction);
        helpMenu->addAction(aboutAction);
        helpMenu->addAction(aboutQtAction);
        menuFilters->addAction(euclideanClusterExtractionAction);
        menuFilters->addAction(voxelGridAction);
        menuFilters->addAction(movingLeastSquaresAction);
        menuFilters->addAction(boundaryEstimationAction);
        menuFilters->addAction(outliersRemovalAction);
        menuFilters->addSeparator();
        menuFilters->addAction(menuTransformation->menuAction());
        menuFilters->addAction(menuFieldsOperation->menuAction());
        menuFilters->addSeparator();
        menuFilters->addAction(concatenationAction);
        menuTransformation->addAction(translationAction);
        menuTransformation->addAction(rotationAction);
        menuTransformation->addAction(scaleAction);
        menuFieldsOperation->addAction(normalFieldAction);
        renderMenu->addAction(menu_Color->menuAction());
        renderMenu->addAction(drawNormalAction);
        renderMenu->addSeparator();
        renderMenu->addAction(drawAxisAction);
        renderMenu->addAction(drawOrientationMarkerAction);
        renderMenu->addSeparator();
        renderMenu->addAction(registrationModeAction);
        renderMenu->addAction(drawBoundaryAction);
        menu_Color->addAction(colorNoneAction);
        menu_Color->addAction(colorOriginalAction);
        menu_Color->addAction(colorCustomAction);
        fileToolBar->addAction(openAction);
        fileToolBar->addAction(saveAction);
        fileToolBar->addAction(saveAsAction);
        fileToolBar->addSeparator();
        fileToolBar->addAction(showCloudBrowserAction);
        fileToolBar->addAction(showPointBrowserAction);

        retranslateUi(MainWindow);
        QObject::connect(exitAction, SIGNAL(triggered()), MainWindow, SLOT(close()));

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Registar", 0, QApplication::UnicodeUTF8));
        openAction->setText(QApplication::translate("MainWindow", "&Open", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        openAction->setStatusTip(QApplication::translate("MainWindow", "Open an PointCloud file", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        saveAction->setText(QApplication::translate("MainWindow", "&Save", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        saveAction->setStatusTip(QApplication::translate("MainWindow", "Save the PointCloud to original file", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        saveAsAction->setText(QApplication::translate("MainWindow", "Save &As", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        saveAsAction->setStatusTip(QApplication::translate("MainWindow", "Save the PointCloud to disk as new file", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        exitAction->setText(QApplication::translate("MainWindow", "&Exit", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        exitAction->setStatusTip(QApplication::translate("MainWindow", "Exit Registar", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        aboutAction->setText(QApplication::translate("MainWindow", "&About", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        aboutAction->setStatusTip(QApplication::translate("MainWindow", "Show Registar's About box", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        aboutQtAction->setText(QApplication::translate("MainWindow", "About &Qt", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        aboutQtAction->setStatusTip(QApplication::translate("MainWindow", "Show the Qt library's About box", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        voxelGridAction->setText(QApplication::translate("MainWindow", "Voxel Grid", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        voxelGridAction->setStatusTip(QApplication::translate("MainWindow", "Use voxel grid to resample PointCloud", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        movingLeastSquaresAction->setText(QApplication::translate("MainWindow", "Moving Least Squares", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        movingLeastSquaresAction->setStatusTip(QApplication::translate("MainWindow", "Use Moving Least Squares to resample PointCloud", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        showCloudBrowserAction->setText(QApplication::translate("MainWindow", "Show Cloud Browser", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        showCloudBrowserAction->setToolTip(QApplication::translate("MainWindow", "Show Cloud Browser", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        showPointBrowserAction->setText(QApplication::translate("MainWindow", "Show Point Browser", 0, QApplication::UnicodeUTF8));
        colorNoneAction->setText(QApplication::translate("MainWindow", "&None", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        colorNoneAction->setStatusTip(QApplication::translate("MainWindow", "Render with no color", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        colorOriginalAction->setText(QApplication::translate("MainWindow", "&Original", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        colorOriginalAction->setStatusTip(QApplication::translate("MainWindow", "Render with original color ", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        colorCustomAction->setText(QApplication::translate("MainWindow", "&Custom", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        colorCustomAction->setStatusTip(QApplication::translate("MainWindow", "Render foreach custom color ", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        drawNormalAction->setText(QApplication::translate("MainWindow", "Draw &Normal", 0, QApplication::UnicodeUTF8));
        boundaryEstimationAction->setText(QApplication::translate("MainWindow", "Boundary Estimation", 0, QApplication::UnicodeUTF8));
        outliersRemovalAction->setText(QApplication::translate("MainWindow", "Outliers Removal", 0, QApplication::UnicodeUTF8));
        pairwiseRegistrationAction->setText(QApplication::translate("MainWindow", "Pairwise Registration", 0, QApplication::UnicodeUTF8));
        drawAxisAction->setText(QApplication::translate("MainWindow", "Draw &Axis", 0, QApplication::UnicodeUTF8));
        drawOrientationMarkerAction->setText(QApplication::translate("MainWindow", "Draw &Orientation Marker", 0, QApplication::UnicodeUTF8));
        translationAction->setText(QApplication::translate("MainWindow", "Translation", 0, QApplication::UnicodeUTF8));
        rotationAction->setText(QApplication::translate("MainWindow", "Rotation", 0, QApplication::UnicodeUTF8));
        scaleAction->setText(QApplication::translate("MainWindow", "Scale", 0, QApplication::UnicodeUTF8));
        euclideanClusterExtractionAction->setText(QApplication::translate("MainWindow", "Euclidean Cluster Extraction", 0, QApplication::UnicodeUTF8));
        normalFieldAction->setText(QApplication::translate("MainWindow", "Normal Field", 0, QApplication::UnicodeUTF8));
        concatenationAction->setText(QApplication::translate("MainWindow", "Concatenation", 0, QApplication::UnicodeUTF8));
        diagramAction->setText(QApplication::translate("MainWindow", "Diagram", 0, QApplication::UnicodeUTF8));
        globalRegistrationAction->setText(QApplication::translate("MainWindow", "Registration", 0, QApplication::UnicodeUTF8));
        registrationModeAction->setText(QApplication::translate("MainWindow", "&Registration Mode", 0, QApplication::UnicodeUTF8));
        drawBoundaryAction->setText(QApplication::translate("MainWindow", "Draw &Boundary", 0, QApplication::UnicodeUTF8));
        fileMenu->setTitle(QApplication::translate("MainWindow", "&File", 0, QApplication::UnicodeUTF8));
        editMenu->setTitle(QApplication::translate("MainWindow", "&Edit", 0, QApplication::UnicodeUTF8));
        toolsMenu->setTitle(QApplication::translate("MainWindow", "&Tools", 0, QApplication::UnicodeUTF8));
        globalRegistrationMenu->setTitle(QApplication::translate("MainWindow", "Global Registration", 0, QApplication::UnicodeUTF8));
        optionsMenu->setTitle(QApplication::translate("MainWindow", "&Options", 0, QApplication::UnicodeUTF8));
        helpMenu->setTitle(QApplication::translate("MainWindow", "&Help", 0, QApplication::UnicodeUTF8));
        menuFilters->setTitle(QApplication::translate("MainWindow", "Fil&ters", 0, QApplication::UnicodeUTF8));
        menuTransformation->setTitle(QApplication::translate("MainWindow", "Transformation", 0, QApplication::UnicodeUTF8));
        menuFieldsOperation->setTitle(QApplication::translate("MainWindow", "FieldsOperation", 0, QApplication::UnicodeUTF8));
        renderMenu->setTitle(QApplication::translate("MainWindow", "&Render", 0, QApplication::UnicodeUTF8));
        menu_Color->setTitle(QApplication::translate("MainWindow", "&Color", 0, QApplication::UnicodeUTF8));
        fileToolBar->setWindowTitle(QApplication::translate("MainWindow", "toolBar", 0, QApplication::UnicodeUTF8));
        cloudBrowserDockWidget->setWindowTitle(QApplication::translate("MainWindow", "Cloud Browser", 0, QApplication::UnicodeUTF8));
        pointBrowserDockWidget->setWindowTitle(QApplication::translate("MainWindow", "Point Browser", 0, QApplication::UnicodeUTF8));
        visualizerBar->setWindowTitle(QApplication::translate("MainWindow", "toolBar", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
