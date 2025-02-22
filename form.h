#ifndef FORM_H
#define FORM_H

#include <QWidget>
#include "ui_form.h"
#include <QtCharts>
#include <QHBoxLayout>

namespace Ui {
class Form;
}

class Form : public QWidget
{
    Q_OBJECT

public:
    explicit Form(QWidget *parent = nullptr);
    ~Form();


private:
    Ui::Form *ui;
    QHBoxLayout *hLayout;
    QChart *chart = nullptr;
    QSplineSeries *trajectoryAUVreal = nullptr;
    QSplineSeries *trajectoryAUVekf = nullptr;
    QScatterSeries *auvPositionReal = nullptr;
    QScatterSeries *auvPositionEkf = nullptr;
    QScatterSeries *beaconPositionReal = nullptr;
    QLineSeries *circleSeries = nullptr;
    QChartView * chartView;
    QValueAxis *xAxis = nullptr;
    QValueAxis *yAxis = nullptr;
    QGraphicsPolygonItem * velocityArrowHead = nullptr;
    QLineSeries * velocityVectorSeries = nullptr;
    QGraphicsPolygonItem * velocityArrowHeadReal = nullptr;
    QLineSeries * velocityVectorSeriesReal = nullptr;

    double x_auv_real = 0;
    double y_auv_real = 0;
    double x_auv_ekf = 0;
    double y_auv_ekf = 0;

public slots:
    void setXY_auv_real(double x, double y);
    void setXY_auv_ekf(double x, double y);
    void setCircle(double r);
    void setVelocityVector_ekf(double vx, double vy);
    void setVelocityVector_real(double vx, double vy);
};

#endif // FORM_H
