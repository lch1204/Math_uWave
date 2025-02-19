#include <QApplication>
#include "MathAUV/su_rov.h"
#include "form.h"
#include <QObject>

double X[2000][2];

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    SU_ROV su;
    Form form;
    form.show();

    QObject::connect(&su, &SU_ROV::updateCoromAUVReal, &form, &Form::setXY_auv_real);
    QObject::connect(&su, &SU_ROV::updateCoromAUVEkf, &form, &Form::setXY_auv_ekf);
    QObject::connect(&su, &SU_ROV::updateCircle, &form, &Form::setCircle);
    return a.exec();
}
