#ifndef CLOUDBROWSER_H
#define CLOUDBROWSER_H

#include <QtGui/QTreeWidget>

namespace registar
{
	class Cloud;
}

class CloudBrowser : public QTreeWidget
{
	Q_OBJECT

public:
	CloudBrowser(QWidget *parent = 0 );
	virtual ~CloudBrowser();

	void addCloud(registar::Cloud* cloud);
	void removeCloud(registar::Cloud* cloud);
	void updateCloud(registar::Cloud* cloud);

	QStringList getSelectedCloudNames();
	QList<bool> getSelectedCloudIsVisible();

	QStringList getVisibleCloudNames();

signals:
	void cloudVisibleStateChanged(QStringList cloudNameList, bool isVisible);

private slots:
	void on_itemChanged(QTreeWidgetItem* treeWidgetItem, int column);

};

#endif 
