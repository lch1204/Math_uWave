#include "graph.h"
#include "ui_graph.h"
#include <qt5/QtWidgets/qwidget.h>


Graph::Graph(QWidget *parent)
    : QWidget(parent)
{
    setupUi(this);
    GraphX->setAxis("X, м");
    GraphY->setAxis("Y, м");
    GraphZ->setAxis("Z, м");
}

Graph::~Graph()
{
    delete ui;
}

void Graph::setX(double X, double ekfX, double t_sim)
{
    GraphX->setYT(X,ekfX,t_sim);
}

void Graph::setY(double Y, double ekfY, double t_sim)
{
    GraphY->setYT(Y,ekfY,t_sim);
}

void Graph::setZ(double Z,double ekfZ, double t_sim)
{
    GraphZ->setYT(Z,ekfZ,t_sim);
}
