#include "form.h"
#include "ui_form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Form)
{
    ui->setupUi(this);
    chart = new QChart();
    chartView = new QChartView(this);
    hLayout = new QHBoxLayout();
    hLayout->addWidget(chartView);
    setLayout(hLayout);
    chartView->setChart(chart);

    chartView->setChart(chart);
    xAxis = new QValueAxis(chart);
    yAxis = new QValueAxis(chart);

    trajectoryAUVreal = new QSplineSeries(chart);
    auvPositionReal = new QScatterSeries(chart);

    trajectoryAUVekf = new QSplineSeries(chart);
    auvPositionEkf = new QScatterSeries(chart);

    beaconPositionReal = new QScatterSeries(chart);

    circleSeries = new QLineSeries(chart);  // Инициализация новой серии для окружности

    // circleSeries->setMarkerShape(QScatterSeries::MarkerShapeCircle);
    circleSeries->setBrush(QBrush(Qt::blue));  // Цвет окружности

    chart->addSeries(trajectoryAUVreal);
    chart->addSeries(auvPositionReal);
    chart->addSeries(trajectoryAUVekf);
    chart->addSeries(auvPositionEkf);
    chart->addSeries(beaconPositionReal);
    chart->addSeries(circleSeries);

    xAxis->setRange(-100,100);
    xAxis->setTickCount(10);
    xAxis->setTitleText("X, m");


    yAxis->setRange(-100,100);
    yAxis->setTickCount(10);
    yAxis->setTitleText("Y, m");

    chart->addAxis(xAxis, Qt::AlignLeft);
    chart->addAxis(yAxis, Qt::AlignBottom);

    trajectoryAUVreal->attachAxis(xAxis);
    trajectoryAUVreal->attachAxis(yAxis);
    auvPositionReal->attachAxis(xAxis);
    auvPositionReal->attachAxis(yAxis);
    trajectoryAUVekf->attachAxis(xAxis);
    trajectoryAUVekf->attachAxis(yAxis);
    auvPositionEkf->attachAxis(xAxis);
    auvPositionEkf->attachAxis(yAxis);
    beaconPositionReal->attachAxis(xAxis);
    beaconPositionReal->attachAxis(yAxis);
    circleSeries->attachAxis(xAxis);
    circleSeries->attachAxis(yAxis);

    chart->layout()->setContentsMargins(0,0,0,0);
    chartView->setInteractive(true);
    chartView->setDragMode(QGraphicsView::ScrollHandDrag);
    chartView->setMinimumSize(600,600);
    chartView->setRubberBand(QChartView::RectangleRubberBand);

    beaconPositionReal->append(10,10);
}

Form::~Form()
{
    delete ui;
}

void Form::setXY_auv_real(double x, double y)
{
    trajectoryAUVreal->append(x,y);
    auvPositionReal->clear();
    auvPositionReal->append(x,y);
}

void Form::setXY_auv_ekf(double x, double y)
{
    trajectoryAUVekf->append(x,y);
    auvPositionEkf->clear();
    auvPositionEkf->append(x,y);
}

void Form::setCircle(double r)
{
    circleSeries->clear();  // Удаление старой окружности

    // Отрисовка новой окружности
    const int numPoints = 100;  // Количество точек для аппроксимации окружности
    const double centerX = 10.0;
    const double centerY = 10.0;

    for (int i = 0; i < numPoints; ++i) {
        double angle = 2 * M_PI * i / numPoints;
        double x = centerX + r * cos(angle);
        double y = centerY + r * sin(angle);
        circleSeries->append(x, y);
    }
    double x = centerX + r * cos(0);
    double y = centerY + r * sin(0);
    circleSeries->append(x, y);
}
