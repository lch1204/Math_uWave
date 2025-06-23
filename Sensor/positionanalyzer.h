#ifndef POSITIONANALYZER_H
#define POSITIONANALYZER_H

#include <QVector>
#include <QDebug>
#include <cmath>

class PositionAnalyzer {
public:
    void update(double x_actual, double y_actual, double x_est, double y_est) {
        double dx = x_actual - x_est;
        double dy = y_actual - y_est;
        double error = std::sqrt(dx*dx + dy*dy);

        x_errors.append(dx);
        y_errors.append(dy);
        total_errors.append(error);

        qDebug() << "x_global:" << x_actual << "y_global:" << y_actual;
        qDebug() << "state[0]:" << x_est << "state[1]:" << y_est;

        printStatistics();
    }

private:
    QVector<double> x_errors;
    QVector<double> y_errors;
    QVector<double> total_errors;

    void printStatistics() {
        qDebug() << "----- Error Statistics -----";

        printStat("X", x_errors);
        printStat("Y", y_errors);
        printStat("Total", total_errors);

        qDebug() << "----------------------------";
    }

    void printStat(const QString& label, const QVector<double>& errors) {
        if (errors.isEmpty()) return;

        double minError = *std::min_element(errors.begin(), errors.end());
        double maxError = *std::max_element(errors.begin(), errors.end());

        double sum = 0.0;
        for (double e : errors)
            sum += e;

        double mean = sum / errors.size();

        double variance = 0.0;
        for (double e : errors)
            variance += (e - mean) * (e - mean);
        variance /= errors.size();

        double rmse = std::sqrt(std::accumulate(errors.begin(), errors.end(), 0.0,
                                [](double a, double b) { return a + b * b; }) / errors.size());

        qDebug() << label << "Error: min =" << minError
                 << "max =" << maxError
                 << "variance =" << variance
                 << "RMSE =" << rmse;
    }
};

#endif // POSITIONANALYZER_H
