#include <QApplication>
#include "MathAUV/su_rov.h"
#include "form.h"
#include "graph.h"
#include <QObject>

double X[2000][2];

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    SU_ROV su;
    Form form;
    Graph g;
    form.show();
    g.show();

    QObject::connect(&su, &SU_ROV::updateCoromAUVReal,        &form, &Form::setXY_auv_real);
    QObject::connect(&su, &SU_ROV::updateCoromAUVEkf,         &form, &Form::setXY_auv_ekf);
    QObject::connect(&su, &SU_ROV::updateCircle,              &form, &Form::setCircle);
    QObject::connect(&su, &SU_ROV::updateVelocityVector_ekf,  &form, &Form::setVelocityVector_ekf);
    QObject::connect(&su, &SU_ROV::updateVelocityVector_real, &form, &Form::setVelocityVector_real);
    QObject::connect(&su, &SU_ROV::updateX, &g, &Graph::setX);
    QObject::connect(&su, &SU_ROV::updateY, &g, &Graph::setY);
    QObject::connect(&su, &SU_ROV::updateZ, &g, &Graph::setZ);
    return a.exec();
}
