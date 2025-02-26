/********************************************************************************
** Form generated from reading UI file 'graph.ui'
**
** Created by: Qt User Interface Compiler version 5.15.13
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GRAPH_H
#define UI_GRAPH_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <setvariable_t.h>

QT_BEGIN_NAMESPACE

class Ui_Graph
{
public:
    QVBoxLayout *verticalLayout;
    QTabWidget *tabWidget;
    QWidget *X;
    QVBoxLayout *verticalLayout_2;
    SetVariable_t *GraphX;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_3;
    SetVariable_t *GraphY;
    QWidget *tab_3;
    QVBoxLayout *verticalLayout_4;
    SetVariable_t *GraphZ;

    void setupUi(QWidget *Graph)
    {
        if (Graph->objectName().isEmpty())
            Graph->setObjectName(QString::fromUtf8("Graph"));
        Graph->resize(825, 659);
        verticalLayout = new QVBoxLayout(Graph);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        tabWidget = new QTabWidget(Graph);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        X = new QWidget();
        X->setObjectName(QString::fromUtf8("X"));
        verticalLayout_2 = new QVBoxLayout(X);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        GraphX = new SetVariable_t(X);
        GraphX->setObjectName(QString::fromUtf8("GraphX"));

        verticalLayout_2->addWidget(GraphX);

        tabWidget->addTab(X, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        verticalLayout_3 = new QVBoxLayout(tab_2);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        GraphY = new SetVariable_t(tab_2);
        GraphY->setObjectName(QString::fromUtf8("GraphY"));

        verticalLayout_3->addWidget(GraphY);

        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        verticalLayout_4 = new QVBoxLayout(tab_3);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        GraphZ = new SetVariable_t(tab_3);
        GraphZ->setObjectName(QString::fromUtf8("GraphZ"));

        verticalLayout_4->addWidget(GraphZ);

        tabWidget->addTab(tab_3, QString());

        verticalLayout->addWidget(tabWidget);


        retranslateUi(Graph);

        tabWidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(Graph);
    } // setupUi

    void retranslateUi(QWidget *Graph)
    {
        Graph->setWindowTitle(QCoreApplication::translate("Graph", "Form", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(X), QCoreApplication::translate("Graph", "Tab 1", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("Graph", "Tab 2", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QCoreApplication::translate("Graph", "Page", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Graph: public Ui_Graph {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GRAPH_H
