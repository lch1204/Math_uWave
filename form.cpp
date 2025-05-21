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
    velocityVectorSeries = new QLineSeries(chart);  // Серия для вектора скорости
    velocityVectorSeriesReal = new QLineSeries(chart);  // Серия для вектора скорости

    circleSeries = new QLineSeries(chart);  // Инициализация новой серии для окружности

    // circleSeries->setMarkerShape(QScatterSeries::MarkerShapeCircle);
    circleSeries->setBrush(QBrush(Qt::blue));  // Цвет окружности
    velocityVectorSeries->setPen(QPen(Qt::black, 2));  // Настройка стиля линии вектора скорости

    chart->addSeries(trajectoryAUVreal);
    chart->addSeries(auvPositionReal);
    chart->addSeries(trajectoryAUVekf);
    chart->addSeries(auvPositionEkf);
    chart->addSeries(beaconPositionReal);
    chart->addSeries(circleSeries);
    chart->addSeries(velocityVectorSeries);  // Добавляем серию для вектора скорости
    chart->addSeries(velocityVectorSeriesReal);  // Добавляем серию для вектора скорости

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
    velocityVectorSeries->attachAxis(xAxis);  // Привязка серии вектора скорости к осям
    velocityVectorSeries->attachAxis(yAxis);
    velocityVectorSeriesReal->attachAxis(xAxis);  // Привязка серии вектора скорости к осям
    velocityVectorSeriesReal->attachAxis(yAxis);

    chart->layout()->setContentsMargins(0,0,0,0);
    chartView->setInteractive(true);
    chartView->setDragMode(QGraphicsView::ScrollHandDrag);
    chartView->setMinimumSize(600,600);
    chartView->setRubberBand(QChartView::RectangleRubberBand);

    beaconPositionReal->append(5,5);//здесь на графике отображаю
    // Инициализация графических элементов для наконечника стрелки
    velocityArrowHead = new QGraphicsPolygonItem();
    chartView->scene()->addItem(velocityArrowHead);
    velocityArrowHeadReal = new QGraphicsPolygonItem();
    chartView->scene()->addItem(velocityArrowHeadReal);
}

Form::~Form()
{
    delete ui;
}

void Form::setXY_auv_real(double x, double y)
{
    x_auv_real = x;
    y_auv_real = y;
    trajectoryAUVreal->append(x,y);
    auvPositionReal->clear();
    auvPositionReal->append(x,y);
}

void Form::setXY_auv_ekf(double x, double y)
{
    x_auv_ekf = x;
    y_auv_ekf = y;
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

void Form::setVelocityVector_ekf(double vx, double vy) {
    const double centerX = x_auv_ekf;  // Центр окружности
    const double centerY = y_auv_ekf;

    // Масштабирующий коэффициент для длины стрелки
    const double scale = 1.1;  // Можно настроить в зависимости от желаемой длины

    // Вычисляем конечную точку стрелки
    double endX = centerX + vx * scale;
    double endY = centerY + vy * scale;

    // Очищаем старую стрелку
    velocityVectorSeries->clear();
    velocityArrowHead->setPolygon(QPolygonF());  // Очищаем наконечник

    // Рисуем линию вектора скорости
    velocityVectorSeries->append(centerX, centerY);
    velocityVectorSeries->append(endX, endY);

    // Рисуем наконечник стрелки
    const double arrowSize = 5.0;  // Размер наконечника
    QLineF line(centerX, centerY, endX, endY);
    double angle = std::atan2(-line.dy(), line.dx());  // Угол наклона линии

    // Создаем полигон для наконечника стрелки
    QPolygonF arrowHead;
    arrowHead.append(QPointF(0, 0));
    arrowHead.append(QPointF(-arrowSize, -arrowSize / 2));
    arrowHead.append(QPointF(-arrowSize, arrowSize / 2));

    // Трансформируем полигон в соответствии с углом наклона линии
    QTransform transform;
    transform.translate(endX, endY);
    transform.rotateRadians(-angle);
    velocityArrowHead->setPolygon(transform.map(arrowHead));
    velocityArrowHead->setBrush(QBrush(Qt::black));  // Цвет наконечника
}

void Form::setVelocityVector_real(double vx, double vy)
{
    const double centerX = x_auv_real;  // Центр окружности
    const double centerY = y_auv_real;

    // Масштабирующий коэффициент для длины стрелки
    const double scale = 1.1;  // Можно настроить в зависимости от желаемой длины

    // Вычисляем конечную точку стрелки
    double endX = centerX + vx * scale;
    double endY = centerY + vy * scale;

    // Очищаем старую стрелку
    velocityVectorSeriesReal->clear();
    velocityArrowHeadReal->setPolygon(QPolygonF());  // Очищаем наконечник

    // Рисуем линию вектора скорости
    velocityVectorSeriesReal->append(centerX, centerY);
    velocityVectorSeriesReal->append(endX, endY);

    // Рисуем наконечник стрелки
    const double arrowSize = 5.0;  // Размер наконечника
    QLineF line(centerX, centerY, endX, endY);
    double angle = std::atan2(-line.dy(), line.dx());  // Угол наклона линии

    // Создаем полигон для наконечника стрелки
    QPolygonF arrowHead;
    arrowHead.append(QPointF(0, 0));
    arrowHead.append(QPointF(-arrowSize, -arrowSize / 2));
    arrowHead.append(QPointF(-arrowSize, arrowSize / 2));

    // Трансформируем полигон в соответствии с углом наклона линии
    QTransform transform;
    transform.translate(endX, endY);
    transform.rotateRadians(-angle);
    velocityArrowHeadReal->setPolygon(transform.map(arrowHead));
    velocityArrowHeadReal->setBrush(QBrush(Qt::black));  // Цвет наконечника
}
