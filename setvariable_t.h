#ifndef SETVARIABLE_T_H
#define SETVARIABLE_T_H

#include <QWidget>
#include "ui_setvariable_t.h"
#include <QtCharts>
#include <QHBoxLayout>


class SetVariable_t : public QWidget, private Ui::SetVariable_t
{
    Q_OBJECT

public:
    explicit SetVariable_t(QWidget *parent = nullptr);
    ~SetVariable_t();
    QHBoxLayout *hlay;
    QChartView * chartView;
    QChart *chart;
    QLineSeries *real;
    QLineSeries *ekf;
    QValueAxis *xAxis;
    QValueAxis *yAxis;
    QTimer *timer;

private:
    Ui::SetVariable_t *ui;
public slots:
    void setYT(double realT, double ekfT, double T);
    void setAxis(const QString &titleY);
};

#endif // SETVARIABLE_T_H
