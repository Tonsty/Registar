#include <QtGui/QTreeWidgetItem>
#include <QtCore/QFileInfo>
#include <QtCore/QtCore>
#include <QtGui/QAction>

#include "../include/cloud.h" 
#include "../include/cloudbrowser.h"

using namespace registar;

CloudBrowser::CloudBrowser(QWidget *parent) : QTreeWidget(parent)
{
	QStringList headLabels;
	headLabels << "" << "" << "";
	setHeaderLabels(headLabels);
	connect(this, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this, SLOT(on_itemChanged(QTreeWidgetItem*, int)));	
}

CloudBrowser::~CloudBrowser(){}

void CloudBrowser::addCloud(Cloud *cloud)
{
	QString cloudName = cloud->getCloudName();
	QString fileName = cloud->getFileName();
	CloudDataConstPtr cloudData = cloud->getCloudData();

	QStringList strings;
	strings << cloudName << QFileInfo(fileName).fileName() << "Visible";
	QTreeWidgetItem *treeWidgetItem = new QTreeWidgetItem(strings);
	treeWidgetItem->setCheckState(2, Qt::Checked);
	this->addTopLevelItem(treeWidgetItem);

	QStringList strings1;
	strings1 << "" << "FullFileName" << fileName;
	new QTreeWidgetItem(treeWidgetItem, strings1);
	QStringList strings2;
	strings2 << "" << "Size" << QString::number(cloudData->size());
	new QTreeWidgetItem(treeWidgetItem, strings2);
	QStringList strings3;
	strings3 << "" << "Width" << QString::number(cloudData->width);
	new QTreeWidgetItem(treeWidgetItem, strings3);
	QStringList strings4;
	strings4 << "" << "Height" << QString::number(cloudData->height);
	new QTreeWidgetItem(treeWidgetItem, strings4);

	resizeColumnToContents(0);
	resizeColumnToContents(1);
	resizeColumnToContents(2);
}

void CloudBrowser::removeCloud(Cloud* cloud)
{
	QString cloudName = cloud->getCloudName();
	QList<QTreeWidgetItem*> treeWidgetItemList = findItems(cloudName, Qt::MatchExactly, 0);
	QTreeWidgetItem* treeWidgetItem = *(treeWidgetItemList.begin());	
	takeTopLevelItem(indexOfTopLevelItem(treeWidgetItem));
}

void CloudBrowser::updateCloud(Cloud* cloud)
{
	QString cloudName = cloud->getCloudName();
	QString fileName = cloud->getFileName();
	CloudDataConstPtr cloudData = cloud->getCloudData();

	QList<QTreeWidgetItem*> treeWidgetItemList = findItems(cloudName, Qt::MatchExactly, 0);

	QTreeWidgetItem* treeWidgetItem = *(treeWidgetItemList.begin());
	treeWidgetItem->setText(1, QFileInfo(fileName).fileName());
	treeWidgetItem->child(0)->setText(2, fileName);	
	treeWidgetItem->child(1)->setText(2, QString::number(cloudData->size()));
	treeWidgetItem->child(2)->setText(2, QString::number(cloudData->width));
	treeWidgetItem->child(3)->setText(2, QString::number(cloudData->height));
}

QStringList CloudBrowser::getSelectedCloudNames()
{
	QStringList cloudNameList;

	QList<QTreeWidgetItem*> seletcedItems = selectedItems();
	QList<QTreeWidgetItem*>::Iterator it = seletcedItems.begin();
	while (it != seletcedItems.end())
	{
		if ( !(*it)->text(0).isEmpty() )
		{
			QString cloudName = (*it)->text(0);
			cloudNameList << cloudName;
		}
		it++;
	}

	return cloudNameList;
}

QList<bool> CloudBrowser::getSelectedCloudIsVisible()
{
	QList<bool> isVisibleList;

	QList<QTreeWidgetItem*> seletcedItems = selectedItems();
	QList<QTreeWidgetItem*>::Iterator it = seletcedItems.begin();
	while (it != seletcedItems.end())
	{
		if ( !(*it)->text(0).isEmpty() )
		{
			Qt::CheckState checkState = (*it)->checkState(2);
			if (checkState == Qt::Checked)
			{
				isVisibleList.push_back(true);
			}
			else if(checkState == Qt::Unchecked)
			{
				isVisibleList.push_back(false);
			}
		}
		it++;
	}

	return isVisibleList;

}

QStringList CloudBrowser::getVisibleCloudNames()
{
	QStringList cloudNameList;
	
	for (int i = 0; i < topLevelItemCount(); ++i)
	{
		QTreeWidgetItem *treeWidgetItem = topLevelItem(i);
		QString cloudName = treeWidgetItem->text(0);
		Qt::CheckState checkState = treeWidgetItem->checkState(2);
		if(checkState == Qt::Checked)
		{
			cloudNameList << cloudName;
		}
	}

	return cloudNameList;
}

void CloudBrowser::on_itemChanged(QTreeWidgetItem* treeWidgetItem, int column)
{
	if (!treeWidgetItem->text(0).isEmpty() && (column == 2))
	{
		QString cloudName = treeWidgetItem->text(0);
		Qt::CheckState checkState = treeWidgetItem->checkState(column);
		if (checkState == Qt::Checked)
		{
			emit cloudVisibleStateChanged(cloudName, true);
		}
		else if(checkState == Qt::Unchecked)
		{
			emit cloudVisibleStateChanged(cloudName, false);
		}
	}
}
