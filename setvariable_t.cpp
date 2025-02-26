#include "setvariable_t.h"
#include "ui_setvariable_t.h"


SetVariable_t::SetVariable_t(QWidget *parent)
    : QWidget(parent)
{
    ui->setupUi(this);
    chart = new QChart();
    chartView = new QChartView(chart);
    real = new QLineSeries;
    ekf = new QLineSeries;
    xAxis = new QValueAxis();
    yAxis = new QValueAxis();

    chart->addSeries(real);
    chart->addSeries(ekf);
    chart->createDefaultAxes();

    //установка максимального и минимального значения по оси
    xAxis->setRange(0,10);
    //установка количества градаций на оси (по умолчанию =5)
    // xAxis->setTickCount(11);
    xAxis->setTitleText("T, c");


    yAxis->setRange(0,10);
    // yAxis->setTickCount(11);
    yAxis->setTitleText("dt, c");


    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->chart()->setAxisX(xAxis, ekf);
    chartView->chart()->setAxisY(yAxis, ekf);
    chartView->chart()->setAxisX(xAxis, real);
    chartView->chart()->setAxisY(yAxis, real);
    chartView->setInteractive(true);
    chartView->setDragMode(QGraphicsView::ScrollHandDrag);
    chartView->setRubberBand(QChartView::RectangleRubberBand);

    hlay = new QHBoxLayout();
    hlay->addWidget(chartView);
    setLayout(hlay);
}

SetVariable_t::~SetVariable_t()
{
    delete ui;
}

void SetVariable_t::setYT(double realT, double ekfT, double T)
{
    real->append(T, realT);

    ekf->append(T, ekfT);
}

void SetVariable_t::setAxis(const QString &titleY)
{
    yAxis->setTitleText(titleY);
}
