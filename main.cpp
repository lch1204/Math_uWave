#include <QCoreApplication>
#include "MathAUV/su_rov.h"

double X[2000][2];

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    SU_ROV su;
    int idx = 1;
    // ROV_Model rov( K[100 * idx + 5], K[100 * idx + 6], K[100 * idx + 7]);

    return a.exec();
}
