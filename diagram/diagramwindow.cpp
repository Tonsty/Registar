#include <QtGui/QtGui>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>

#include "diagramwindow.h"
#include "link.h"
#include "node.h"
#include "propertiesdialog.h"

DiagramWindow::DiagramWindow(QWidget *parent): QMainWindow(parent)
{
    scene = new QGraphicsScene(0, 0, 1200, 1000);

    view = new QGraphicsView;
    view->setScene(scene);
    view->setDragMode(QGraphicsView::RubberBandDrag);
    view->setRenderHints(QPainter::Antialiasing
                         | QPainter::TextAntialiasing);
    view->setContextMenuPolicy(Qt::ActionsContextMenu);
    setCentralWidget(view);

    minZ = 0;
    maxZ = 0;
    seqNumber = 0;

    createActions();
    createMenus();
    createToolBars();

    connect(scene, SIGNAL(selectionChanged()),
            this, SLOT(updateActions()));

    setWindowTitle(tr("Diagram"));
    updateActions();
}

void DiagramWindow::addNode()
{
    Node *node = new Node;
    QString nodeName = tr("%1").arg(seqNumber);
    node->setText(nodeName);
    nodeMap[nodeName] = node;
    nodeSet.insert( nodeName );
    setupNode(node);
}

void DiagramWindow::addNode(QString nodeName)
{
	Node *node = new Node;
	node->setText(nodeName);
    nodeMap[nodeName] = node;
	setupNode(node);
}

void DiagramWindow::openFile()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open (Ordered)OverlapInfo"), ".", tr("OverlapInfo files (*.ovl *oovl)"));
    QFileInfo fileInfo(fileName);
    // qDebug() << fileInfo.suffix() << "\n";
    if( fileInfo.suffix() == "ovl" ) loadOVLFile(fileName);
    else if (fileInfo.suffix() == "oovl") loadOOLVFile(fileName);
}

void DiagramWindow::loadOOLVFile(QString fileName)
{
    qDebug() << fileName;

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "Cannot import Ordered OverlapInfo!";
        return;
    }

    QTextStream in(&file);

    
}

void DiagramWindow::loadOVLFile(QString fileName)
{
    qDebug() << fileName;

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "Cannot import OverlapInfo!";
        return;
    }

    QTextStream in(&file);
    int lineCount = 0;
    QString currentLine;
    while( ( currentLine = in.readLine().trimmed() ) != "")
    {
        QStringList targetSourcesList = currentLine.split("<<--");

        QString target = targetSourcesList[0].trimmed();
        QString sources = targetSourcesList[1].trimmed();

        QStringList targetList = target.split(QRegExp("[()]"));
        int targetIndex = targetList[0].trimmed().toInt();
        int targetWeight = targetList[1].trimmed().toInt();

        qDebug() << "targetIndex : " << targetIndex;
        qDebug() << "targetWeight : " << targetWeight;

        nodeSet.insert( QString::number(targetIndex) );

        QStringList sourcesList = sources.split(",");
        for (int i = 0; i < sourcesList.size() && i < 2; ++i)
        {
            QString source = sourcesList[i].trimmed();
            if (source.trimmed() != "")
            {
                QStringList sourceList = source.split(QRegExp("[((%)]"));
                int sourceIndex = sourceList[0].trimmed().toInt();
                int sourceWeight = sourceList[1].trimmed().toInt();

                QString targetIndexStr = QString::number(targetIndex);
                QString sourceIndexStr = QString::number(sourceIndex);

                if (nodeEdgeMultiMap.find(targetIndexStr, sourceIndexStr) == nodeEdgeMultiMap.end() &&
                    nodeEdgeMultiMap.find(sourceIndexStr, targetIndexStr) == nodeEdgeMultiMap.end() )
                {
                    qDebug() << "sourceIndex : " << sourceIndex;
                    qDebug() << "sourceWeight : " << sourceWeight;

                    if ( sourceIndex > targetIndex)
                    {
                        nodeEdgeMultiMap.insert(QString::number(targetIndex), QString::number(sourceIndex));
                    }
                    else
                    {
                        nodeEdgeMultiMap.insert(QString::number(sourceIndex), QString::number(targetIndex));
                    }
                }
            }
        }
        
        lineCount++;
    }
    qDebug() << lineCount - 1 << "vertice(s)";

    for (QSet<QString>::const_iterator it = nodeSet.begin(); it != nodeSet.end(); ++it) addNode(*it);
    for (QMultiMap<QString, QString>::const_iterator it = nodeEdgeMultiMap.begin(); it != nodeEdgeMultiMap.end(); ++it)
    {
        addLink(it.key(), it.value());
    }
}

void DiagramWindow::exportFile()
{
    QString newfileName = QFileDialog::getSaveFileName(this, tr("Save as Links file"), ".", tr("Links file (*.links)"));
    QFile file(newfileName);
    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << "Cannot export Links!";
    }

    QTextStream out(&file);
    for (QMultiMap<QString, QString>::const_iterator it = nodeEdgeMultiMap.begin(); it != nodeEdgeMultiMap.end(); ++it)
    {
        out << it.key() << " " << it.value() << "\n";
    }

}

void DiagramWindow::loadGraph(QStringList &nodeList, QMultiMap<QString, QString> &nodeEdgeMultiMap)
{
	QList<QGraphicsItem *> items = scene->items();
	for (int i = 0; i < items.size(); ++i)
	{
		Node *node = dynamic_cast<Node*>(items[i]);
		if (node)
		{
            nodeList << node->text();
			QSet<Link *> myLinks = node->getLinks();
			for (QSet<Link *>::const_iterator iter = myLinks.constBegin(); 
				iter != myLinks.constEnd(); ++iter)
			{
				if ((*iter)->fromNode() == node)
				{
					nodeEdgeMultiMap.insert(node->text(), (*iter)->toNode()->text());
				}
				else
				{
					nodeEdgeMultiMap.insert(node->text(), (*iter)->fromNode()->text());
				}
			}
		}
	}
}

void DiagramWindow::addLink()
{
    NodePair nodes = selectedNodePair();
    if (nodes == NodePair())
        return;

    QString nodeName_first = nodes.first->text();
    QString nodeName_second = nodes.second->text();

    if (nodeEdgeMultiMap.find(nodeName_first, nodeName_second) == nodeEdgeMultiMap.end() &&
        nodeEdgeMultiMap.find(nodeName_second, nodeName_first) == nodeEdgeMultiMap.end())
    {
        if ( nodeName_first.toInt() < nodeName_second.toInt())
        {
            nodeEdgeMultiMap.insert(nodeName_first, nodeName_second);
        }
        else
        {
            nodeEdgeMultiMap.insert(nodeName_second, nodeName_first);            
        }

    }

    Link *link = new Link(nodes.first, nodes.second);
    scene->addItem(link);
}

void DiagramWindow::addLink(QString fromNodeName, QString toNodeName)
{
    NodePair nodes;
    nodes.first = nodeMap[fromNodeName];
    nodes.second = nodeMap[toNodeName];

    Link *link = new Link(nodes.first, nodes.second);
    scene->addItem(link);
}

void DiagramWindow::del()
{
    QList<QGraphicsItem *> items = scene->selectedItems();
    QMutableListIterator<QGraphicsItem *> i(items);
    while (i.hasNext()) {
        QGraphicsItem *uncertain = i.next();
        Link *link = dynamic_cast<Link *>(uncertain);
        if (link) {
            QString nodeName_from = link->fromNode()->text();
            QString nodeName_to = link->toNode()->text();

            nodeEdgeMultiMap.remove(nodeName_from, nodeName_to);
            nodeEdgeMultiMap.remove(nodeName_to, nodeName_from);

            delete link;
            i.remove();
        }
        else
        {
            Node *node = dynamic_cast<Node *>(uncertain);
            if (node)
            {
                QString nodeName = node->text();
                nodeSet.remove(nodeName);
            }           
        }
    }

    qDeleteAll(items);
}

void DiagramWindow::cut()
{
    Node *node = selectedNode();
    if (!node)
        return;

    copy();
    delete node;
}

void DiagramWindow::copy()
{
    Node *node = selectedNode();
    if (!node)
        return;

    QString str = QString("Node %1 %2 %3 %4")
                  .arg(node->textColor().name())
                  .arg(node->outlineColor().name())
                  .arg(node->backgroundColor().name())
                  .arg(node->text());
    QApplication::clipboard()->setText(str);
}

void DiagramWindow::paste()
{
    QString str = QApplication::clipboard()->text();
    QStringList parts = str.split(" ");

    if (parts.count() >= 5 && parts.first() == "Node") {
        Node *node = new Node;
        node->setText(QStringList(parts.mid(4)).join(" "));
        node->setTextColor(QColor(parts[1]));
        node->setOutlineColor(QColor(parts[2]));
        node->setBackgroundColor(QColor(parts[3]));
        setupNode(node);
    }
}

void DiagramWindow::bringToFront()
{
    ++maxZ;
    setZValue(maxZ);
}

void DiagramWindow::sendToBack()
{
    --minZ;
    setZValue(minZ);
}

void DiagramWindow::properties()
{
    Node *node = selectedNode();
    Link *link = selectedLink();

    if (node) {
        PropertiesDialog dialog(node, this);
        dialog.exec();
    } else if (link) {
        QColor color = QColorDialog::getColor(link->color(), this);
        if (color.isValid())
            link->setColor(color);
    }
}

void DiagramWindow::updateActions()
{
    bool hasSelection = !scene->selectedItems().isEmpty();
    bool isNode = (selectedNode() != 0);
    bool isNodePair = (selectedNodePair() != NodePair());

    cutAction->setEnabled(isNode);
    copyAction->setEnabled(isNode);
    addLinkAction->setEnabled(isNodePair);
    deleteAction->setEnabled(hasSelection);
    bringToFrontAction->setEnabled(isNode);
    sendToBackAction->setEnabled(isNode);
    propertiesAction->setEnabled(isNode);

    foreach (QAction *action, view->actions())
        view->removeAction(action);

    foreach (QAction *action, editMenu->actions()) {
        if (action->isEnabled())
            view->addAction(action);
    }
}

void DiagramWindow::createActions()
{
    exitAction = new QAction(tr("E&xit"), this);
    exitAction->setShortcut(tr("Ctrl+Q"));
    connect(exitAction, SIGNAL(triggered()), this, SLOT(close()));

    openFileAction = new QAction(tr("&Open"), this);
    openFileAction->setShortcut(tr("Ctrl+O"));
    connect(openFileAction, SIGNAL(triggered()), this, SLOT(openFile()));

    exportFileAction = new QAction(tr("&Export"), this);
    exportFileAction->setShortcut(tr("Ctrl+E"));
    connect(exportFileAction, SIGNAL(triggered()), this, SLOT(exportFile()));

    addNodeAction = new QAction(tr("Add &Node"), this);
    addNodeAction->setIcon(QIcon(":/images/node.png"));
    addNodeAction->setShortcut(tr("Ctrl+N"));
    connect(addNodeAction, SIGNAL(triggered()), this, SLOT(addNode()));

    addLinkAction = new QAction(tr("Add &Link"), this);
    addLinkAction->setIcon(QIcon(":/images/link.png"));
    addLinkAction->setShortcut(tr("Ctrl+L"));
    connect(addLinkAction, SIGNAL(triggered()), this, SLOT(addLink()));

    deleteAction = new QAction(tr("&Delete"), this);
    deleteAction->setIcon(QIcon(":/images/delete.png"));
    deleteAction->setShortcut(tr("Del"));
    connect(deleteAction, SIGNAL(triggered()), this, SLOT(del()));

    cutAction = new QAction(tr("Cu&t"), this);
    cutAction->setIcon(QIcon(":/images/cut.png"));
    cutAction->setShortcut(tr("Ctrl+X"));
    connect(cutAction, SIGNAL(triggered()), this, SLOT(cut()));

    copyAction = new QAction(tr("&Copy"), this);
    copyAction->setIcon(QIcon(":/images/copy.png"));
    copyAction->setShortcut(tr("Ctrl+C"));
    connect(copyAction, SIGNAL(triggered()), this, SLOT(copy()));

    pasteAction = new QAction(tr("&Paste"), this);
    pasteAction->setIcon(QIcon(":/images/paste.png"));
    pasteAction->setShortcut(tr("Ctrl+V"));
    connect(pasteAction, SIGNAL(triggered()), this, SLOT(paste()));

    bringToFrontAction = new QAction(tr("Bring to &Front"), this);
    bringToFrontAction->setIcon(QIcon(":/images/bringtofront.png"));
    connect(bringToFrontAction, SIGNAL(triggered()),
            this, SLOT(bringToFront()));

    sendToBackAction = new QAction(tr("&Send to Back"), this);
    sendToBackAction->setIcon(QIcon(":/images/sendtoback.png"));
    connect(sendToBackAction, SIGNAL(triggered()),
            this, SLOT(sendToBack()));

    propertiesAction = new QAction(tr("P&roperties..."), this);
    connect(propertiesAction, SIGNAL(triggered()),
            this, SLOT(properties()));
}

void DiagramWindow::createMenus()
{
    fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(openFileAction);
    fileMenu->addAction(exportFileAction);
    fileMenu->addAction(exitAction);

    editMenu = menuBar()->addMenu(tr("&Edit"));
    editMenu->addAction(addNodeAction);
    editMenu->addAction(addLinkAction);
    editMenu->addAction(deleteAction);
    editMenu->addSeparator();
    editMenu->addAction(cutAction);
    editMenu->addAction(copyAction);
    editMenu->addAction(pasteAction);
    editMenu->addSeparator();
    editMenu->addAction(bringToFrontAction);
    editMenu->addAction(sendToBackAction);
    editMenu->addSeparator();
    editMenu->addAction(propertiesAction);
}

void DiagramWindow::createToolBars()
{
    editToolBar = addToolBar(tr("Edit"));
    editToolBar->addAction(addNodeAction);
    editToolBar->addAction(addLinkAction);
    editToolBar->addAction(deleteAction);
    editToolBar->addSeparator();
    editToolBar->addAction(cutAction);
    editToolBar->addAction(copyAction);
    editToolBar->addAction(pasteAction);
    editToolBar->addSeparator();
    editToolBar->addAction(bringToFrontAction);
    editToolBar->addAction(sendToBackAction);
}

void DiagramWindow::setZValue(int z)
{
    Node *node = selectedNode();
    if (node)
        node->setZValue(z);
}

void DiagramWindow::setupNode(Node *node)
{
    node->setPos(QPoint(80 + (150 * (seqNumber % 9)),
                        80 + (100 * ((seqNumber / 9) % 8))));
    scene->addItem(node);
    ++seqNumber;

    scene->clearSelection();
    node->setSelected(true);
    bringToFront();
}

Node *DiagramWindow::selectedNode() const
{
    QList<QGraphicsItem *> items = scene->selectedItems();
    if (items.count() == 1) {
        return dynamic_cast<Node *>(items.first());
    } else {
        return 0;
    }
}

Link *DiagramWindow::selectedLink() const
{
    QList<QGraphicsItem *> items = scene->selectedItems();
    if (items.count() == 1) {
        return dynamic_cast<Link *>(items.first());
    } else {
        return 0;
    }
}

DiagramWindow::NodePair DiagramWindow::selectedNodePair() const
{
    QList<QGraphicsItem *> items = scene->selectedItems();
    if (items.count() == 2) {
        Node *first = dynamic_cast<Node *>(items.first());
        Node *second = dynamic_cast<Node *>(items.last());
        if (first && second)
            return NodePair(first, second);
    }
    return NodePair();
}
