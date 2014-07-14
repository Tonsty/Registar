#ifndef CLOUDBROWSER_H
#define CLOUDBROWSER_H

#include <QtGui/QTreeWidget>

class Cloud;

class CloudBrowser : public QTreeWidget
{
	Q_OBJECT

public:
	CloudBrowser(QWidget *parent = 0 );
	virtual ~CloudBrowser();

	void addCloud(Cloud* cloud);
	void removeCloud(Cloud* cloud);
	void updateCloud(Cloud* cloud);

	QStringList getSelectedCloudNames();
	QList<bool> getSelectedCloudIsVisible();

	QStringList getVisibleCloudNames();

signals:
	void cloudVisibleStateChanged(QString cloudName, bool isVisible);

private slots:
	void on_itemChanged(QTreeWidgetItem* treeWidgetItem, int column);

};

#endif 
