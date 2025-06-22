#ifndef LOGREADER_H
#define LOGREADER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <QDebug>
#include "../Sensor/SensorIMU.h"
extern double X[2000][2];
extern QVector<double> K;

constexpr double EARTH_RADIUS = 6378137.0; // радиус Земли в метрах (WGS84)
constexpr double DEG_TO_RAD = M_PI / 180.0;

constexpr double lat0 = 5552.330783537318;
constexpr double long0 = 3800.0909083273323;

//// Структура для хранения результатов измерений IMU
//struct IMUOutput {
//    Eigen::Vector3d orientation;        // roll,pitch,yaw (рад)
//    Eigen::Vector3d angularRates;       // рад/с
//    Eigen::Vector3d accelerationGlobal; // м/с² в глобальном frame
//    Eigen::Vector3d speedGlobal;        // м/с в глобальном frame
//    Eigen::Vector3d coordinateGlobal;   // м в глобальном frame

//    // --- добавляем для локальной (body) системы ---
//    Eigen::Vector3d accelerationLocal;  // м/с² в body frame
//    Eigen::Vector3d speedLocal;         // м/с в body frame
//    Eigen::Vector3d coordinateLocal;    // м в body frame
//};

class LogReader {
public:
    LogReader(const std::string& filePath) : filePath(filePath)
    {
        if (!parseCSV()) {
            std::cerr << "Failed to parse CSV file." << std::endl;
            return;
        }

        if (entries.empty()) {
            std::cerr << "No entries found." << std::endl;
            return;
        }
    }

    IMUOutput imu;
    double depth = 0;
    double distance = 0;
    int counterACK = 0;
    Eigen::Vector2d coord_auv;
    // Запуск чтения лога с обработкой данных
    void run() {
        // Обработка записей с учетом времени
        if (entries.empty()) {
            std::cerr << "No entries found." << std::endl;
            return;
        }
        qDebug() << "run";
            // Конвертация первого времени
            startTime = convertTimeToMs(entries[i].timeRPI);
            long long delay = 0;
            while (delay<50) {
                i++;
                long long entryTime = convertTimeToMs(entries[i].timeRPI);
                delay = entryTime - startTime;
            }
            qDebug() << "iii =="<<i;
            const auto& entry = entries[i];

            // Заполнение структур данных
            imu = createIMUOutput(entry);
            depth = entry.puwv7_Depth_m;
            if (entry.counterACK != counterACK)
            {distance = entry.puwv3_propTime * 1500.0; // Преобразование в метры
             counterACK =   entry.counterACK ;
            }
            else distance = -10;
            qDebug() << "entry.counterACK" << entry.counterACK;
            coord_auv = convertToMercator(entry.gll_lat, entry.gll_long, lat0, long0);

            // Здесь можно использовать полученные данные
            // imu, depth, distance
    }

    // Переводит координаты из формата DDMM.MMMMMM в десятичные градусы
    double dmmToDegrees(double dmm) {
        int degrees = static_cast<int>(dmm) / 100;
        double minutes = dmm - degrees * 100;
        return degrees + minutes / 60.0;
    }

    // Переводит широту и долготу в метры относительно начальной точки
    Eigen::Vector2d convertToMercator(double lat_dmm, double lon_dmm, double lat0_dmm, double lon0_dmm) {
        // Конвертируем в десятичные градусы
        double lat = dmmToDegrees(lat_dmm) * DEG_TO_RAD;
        double lon = dmmToDegrees(lon_dmm) * DEG_TO_RAD;
        double lat0 = dmmToDegrees(lat0_dmm) * DEG_TO_RAD;
        double lon0 = dmmToDegrees(lon0_dmm) * DEG_TO_RAD;

        // Проекция Меркатора
        double x = EARTH_RADIUS * (lon - lon0) * cos(lat0);
        double y = EARTH_RADIUS * (lat - lat0);

        return Eigen::Vector2d(x, y);
    }

private:
    struct LogEntry {
        std::string timeRPI;
        double gll_lat = 0.0;   // Широта в формате DDMM.MMMMMM
        double gll_long = 0.0;  // Долгота в формате DDMM.MMMMMM
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        double X_accel = 0.0;
        double Y_accel = 0.0;
        double Z_accel = 0.0;
        double X_rate = 0.0;
        double Y_rate = 0.0;
        double Z_rate = 0.0;
        double X_magn = 0.0;
        double Y_magn = 0.0;
        double Z_magn = 0.0;
        double puwv3_propTime = 0.0;
        double puwv7_Depth_m = 0.0;
        int counterACK = 0;
    };

    std::string filePath;
    std::vector<LogEntry> entries;
    long long startTime = 0;
    long long accumulatedTime = 0;
    long i = 0;
    //для альтернативного метода расчета угла курса
       double A[3][3];  //матрица перехода
       double I[3];   //Ix, Iy, Iz

    // Парсинг CSV файла
    bool parseCSV() {
        std::ifstream file(filePath);
        if (!file.is_open()) return false;

        std::string line;
        std::getline(file, line); // Пропуск заголовка
        std::getline(file, line); // Пропуск заголовка

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> fields;

            while (std::getline(ss, cell, ',')) {
                fields.push_back(cell);
            }

//            if (fields.size() < 81) continue; // Пропуск неполных строк

            LogEntry entry;
            entry.timeRPI = fields[0];
            entry.gll_lat = parseDouble(fields[6]);   // gll.lat (индекс 6)
            entry.gll_long = parseDouble(fields[8]);  // gll.long (индекс 8)
            entry.yaw     = parseDouble(fields[42]); // Индексы полей
            entry.pitch   = parseDouble(fields[43]);
            entry.roll    = parseDouble(fields[44]);
            entry.X_accel = parseDouble(fields[45]);
            entry.Y_accel = parseDouble(fields[46]);
            entry.Z_accel = parseDouble(fields[47]);
            entry.X_rate  = parseDouble(fields[48]);
            entry.Y_rate  = parseDouble(fields[49]);
            entry.Z_rate  = parseDouble(fields[50]);
            entry.X_magn  = parseDouble(fields[51]);
            entry.Y_magn  = parseDouble(fields[52]);
            entry.Z_magn  = parseDouble(fields[53]);
            entry.puwv3_propTime = parseDouble(fields[60]); // puwv3.propTime
            entry.puwv7_Depth_m  = parseDouble(fields[70]); // puwv7.Depth_m
            entry.counterACK = parseInt(fields[78]);

            entries.push_back(entry);
        }
        qDebug() << "return true entries.size" << entries.size();
        return true;
    }

    // Конвертация времени в миллисекунды
    long long convertTimeToMs(const std::string& timeStr) {
        int hours, minutes, seconds, milliseconds;
        char colon;
        std::stringstream ss(timeStr);
        ss >> hours >> colon >> minutes >> colon >> seconds >> colon >> milliseconds;
        return (hours * 3600LL + minutes * 60LL + seconds) * 1000LL + milliseconds;
    }

    // Создание структуры IMUOutput из записи
    IMUOutput createIMUOutput(const LogEntry& entry) {
        constexpr double deg2rad = M_PI / 180.0;
        constexpr double g = 9.8; // Ускорение свободного падения

        IMUOutput imu;
        double K_70 = -0.60299999;
        double X_170 = entry.X_magn + K_70; //Mx с учетом коррекции
        double K_71 = 0.751;
        double X_171 = entry.Y_magn + K_71; //My с учетом коррекции
        double K_72 =0;
        double X_172 = entry.Z_magn + sin(0.5*entry.roll/57.3)*K_72; //Mz с учетом коррекции

        double teta = entry.pitch*M_PI/180; double gamma = entry.roll*M_PI/180;
        A[0][0] = cos(teta); A[0][1] = sin(teta)*sin(gamma); A[0][2] = -sin(teta)*cos(gamma);
        A[1][0] = 0; A[1][1] = cos(gamma); A[1][2] = sin(gamma);
        A[2][0] = sin(teta); A[2][1] = -sin(gamma)*cos(teta); A[2][2] = cos(teta)*cos(gamma);

        double X_300 = I[0] = A[0][0]*X_170 + A[0][1]*X_171 + A[0][2]*X_172;
        double X_400 = I[1] = A[1][0]*X_170 + A[1][1]*X_171 + A[1][2]*X_172;
        double X_500 = I[2] = A[2][0]*X_170 + A[2][1]*X_171 + A[2][2]*X_172;

        double X_178 = atan2(-I[1],-I[0])*57.3; //почему-то здесь ранее стоял Х75, что странно

        double X_79 = -1/cos(entry.pitch/57.3)*(-entry.Z_rate*cos(entry.roll/57.3)-entry.Y_rate*sin(entry.roll/57.3));

//        if (!flagYawInit) {
//           flagYawInit = true;
//           X[91][0] = X[91][1]= X[178][0] + K[178];
//           drewYaw = X[69][0];
//        }
        //   integrate(X[69][0],X[91][0],X[91][1],dt); //интегрируем показание Z_rate для нахождения текущего угла курса
        static double X_91[2];
        integrate(X_79,X_91[0],X_91[1],0.05); //интегрируем показание Z_rate для нахождения текущего угла курса
        qDebug() << "X_91[0]"<<X_91[0];

        // Ориентация (перевод из градусов в радианы)
        imu.orientation = Eigen::Vector3d(
            entry.roll * deg2rad,
            entry.pitch * deg2rad,
            (X_91[0]) * deg2rad
        );

        // Угловые скорости (перевод из градусов в радианы)
        imu.angularRates = Eigen::Vector3d(
            entry.X_rate * deg2rad,
            entry.Y_rate * deg2rad,
            entry.Z_rate * deg2rad
        );

        // Ускорения в локальной системе (перевод g в м/с²)
        imu.accelerationLocal = Eigen::Vector3d(
            entry.X_accel,
            entry.Y_accel,
            entry.Z_accel
        );

        // Инициализация остальных полей нулями
        imu.accelerationGlobal = Eigen::Vector3d::Zero();
        imu.speedGlobal = Eigen::Vector3d::Zero();
        imu.coordinateGlobal = Eigen::Vector3d::Zero();
        imu.speedLocal = Eigen::Vector3d::Zero();
        imu.coordinateLocal = Eigen::Vector3d::Zero();

        return imu;
    }
    void integrate(double &input, double &output, double &prevOutput, double dt) {
        output = prevOutput + dt*input;
        prevOutput = output;
    }

    // Парсинг строки в double
    double parseDouble(const std::string& s) {
        if (s.empty()) return 0.0;
        try {
            return std::stod(s);
        } catch (...) {
            return 0.0;
        }
    }
    int parseInt(const std::string& s) {
        if (s.empty()) return 0;
        try {
            return std::stod(s);
        } catch (...) {
            return 0;
        }
    }
};

//int main() {
//    LogReader reader("logData-hydro-24-06-23-10-07-42.csv");
//    reader.run();
//    return 0;
//}
#endif // LOGREADER_H
