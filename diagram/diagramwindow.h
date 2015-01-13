#ifndef DIAGRAMWINDOW_H
#define DIAGRAMWINDOW_H

#include <QtGui/QMainWindow>
#include <QtCore/QPair>
#include <QtCore/QSet>
#include <QtCore/QMultiMap>

class QAction;
class QGraphicsItem;
class QGraphicsScene;
class QGraphicsView;
class Link;
class Node;

class DiagramWindow : public QMainWindow
{
    Q_OBJECT

public:
    DiagramWindow(QWidget *parent = 0);

    void addNode(QString nodeName);
    void addLink(QString fromNodeName, QString toNodeName, QColor color = Qt::darkRed);
    void loadGraph(QStringList &nodeList, QMultiMap<QString, QString> &nodeEdgeMultiMap);
    void loadOVLFile(QString fileName);
    void loadOOLVFile(QString fileName);
    void initialMaxSpanningTree();

private slots:
    void addNode();
    void addLink();
    void addLink2();
    void del();
    void cut();
    void copy();
    void paste();
    void bringToFront();
    void sendToBack();
    void properties();
    void updateActions();
    void openFile();
    void exportFile();

private:
    typedef QPair<Node *, Node *> NodePair;

    void createActions();
    void createMenus();
    void createToolBars();
    void setZValue(int z);
    void setupNode(Node *node);
    Node *selectedNode() const;
    Link *selectedLink() const;
    NodePair selectedNodePair() const;

    QMenu *fileMenu;
    QMenu *editMenu;
    QToolBar *editToolBar;
    QAction *openFileAction;
    QAction *exportFileAction;
    QAction *exitAction;
    QAction *addNodeAction;
    QAction *addLinkAction;
    QAction *addLinkAction2;
    QAction *deleteAction;
    QAction *cutAction;
    QAction *copyAction;
    QAction *pasteAction;
    QAction *bringToFrontAction;
    QAction *sendToBackAction;
    QAction *propertiesAction;

    QGraphicsScene *scene;
    QGraphicsView *view;

    int minZ;
    int maxZ;
    int seqNumber;

    QMap<QString, Node*> nodeMap;

    QSet<QString> nodeSet;
    QMultiMap<QString, QString> nodeEdgeMultiMap;

    std::vector< std::pair< std::pair<int, int>, int > > edgeWeightVec;
    unsigned int current_index;
};

#endif
