#ifndef GRAPH_H
#define GRAPH_H

#include <QWidget>
#include "ui_graph.h"

class Graph : public QWidget , private Ui::Graph
{
    Q_OBJECT

public:
    explicit Graph(QWidget *parent = nullptr);
    ~Graph();

private:
    Ui::Graph *ui;
public slots:
    void setX(double X, double ekfX, double t_sim);
    void setY(double Y, double ekfY, double t_sim);
    void setZ(double Z, double ekfZ, double t_sim);
};

#endif // GRAPH_H
