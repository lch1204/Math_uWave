#ifndef SU_ROV_H
#define SU_ROV_H

#include <QObject>
#include <QVector>
#include "qkx_coeffs.h"
#include "kx_protocol.h"
#include <QTimer>
#include "../Navigation/NavigationSystem.h"
#include "sensormove.h"
#include "SensorIMU.h"
#include "SensorPressure.h"


#define ANPA_MOD_CNT 24

extern double X[2000][2];
extern QVector<double> K;

struct InitData {
    double m;
    double Fa;
    double Farx[3];
    double cv1[4];
    double cv2[4];
    double cw1[4];
    double cw2[4];
    double lambda[6][6];
    double Ta[6][8];
    double J[3];
    double kd;
    double h[4]; //радиус-вектор координат центра водоизмещения НПА в связанной СК, [1]-x [2]-y [3]-z
    double Td;
    double depth_limit;
    double max_depth;
}; //struct InitData

class SU_ROV : public QObject {
    Q_OBJECT
public:
    explicit SU_ROV(QObject *parent = 0);
    virtual ~SU_ROV();
signals:

public slots:
private:
    void start();

public:
    void model(const float Upl,const float Upp,const float Usl,const float Usp, const float Uzl, const float Uzp);
    void runge(const float Upl,const float Upp,const float Usl,const float Usp, const float Uzl, const float Uzp,const float Ttimer,const float dt=0.01);

    double a[ANPA_MOD_CNT];
    double da[ANPA_MOD_CNT];
    //константы
    double m;
    double Fa;
    double Farx[3];
    double g;
    double G;
    double cv1[4];
    double cv2[4];
    double cw1[4];
    double cw2[4];
    double lambda[7][7];
    double Ta[7][9];
    double C[7][7];
    double Vt[7];
    double Wv[7]; //вектор силы моментов, вызванных внешними возмущениями
    double J[4];
    double kd;
    double h[4]; //радиус-вектор координат центра водоизмещения НПА в связанной СК [1]-x [2]-y [3]-z
    double Td;
    double depth_limit;
    double max_depth;
    //переменные
    double sumX, sumZ;
    double cur_depth, Wx, Wy, Wz;
    double Psi_g, Gamma_g, Tetta_g;

    double Psi_gi, W_Psi_g, W_Gamma_g, W_Tetta_g;
    int N;
    double deltaSx, deltaSz;

    double Ppl, Ppp, Psl, Psp, Pzl, Pzp;
    double Ppl_x, Ppp_x, Psl_x, Psp_x, Pzl_x, Pzp_x;
    double Ppl_y, Ppp_y, Psl_y, Psp_y, Pzl_y, Pzp_y;
    double Ppl_z, Ppp_z, Psl_z, Psp_z, Pzl_z, Pzp_z;
    double Upl, Upp, Usl, Usp, Uzl, Uzp; //напряжения движителей

    double FloatageX, FloatageY, FloatageZ, Fdx, Fdy, Fdz, Fgx, Fgy, Fgz, Fcx, Fcy, Fcz;
    double Mdx, Mdy, Mdz, Mgx, Mgy, Mgz, Mcx, Mcy, Mcz;
    double Mpl_x, Mpp_x, Msl_x, Msp_x, Mzl_x, Mzp_x;
    double Mpl_y, Mpp_y, Msl_y, Msp_y, Mzl_y, Mzp_y;
    double Mpl_z, Mpp_z, Msl_z, Msp_z, Mzl_z, Mzp_z;
    double Max,May,Maz; // моменты от силы Архимеда

    double x_global, y_global, z_global;
    double vx_local,  vy_local, vz_local;  //lineinye skorosti SPA v svyazannyh osyah
    double vx_global, vy_global, vz_global;

    double ekf_x, ekf_y, ekf_z;       // Координаты от EKF
    double ekf_vx, ekf_vy, ekf_vz;    // Скорости от EKF
    double nav_error;                 // Ошибка навигации

public:
    void BFS_DRK(double Upsi, double Uteta, double Ugamma, double Ux, double Uy, double Uz);

    void resetModel();
    void tick(const float Ttimer);
    float Fx,Fy,Fz; //total forces for XYZ-axis
    float Mx,My,Mz; //total moments for XYZ-axis
    void integrate(double &input, double &output, double &prevOutput, double dt);
    void updateNavigation(double dt);



protected:
    Qkx_coeffs * K_protocol=nullptr;
    x_protocol * X_protocol=nullptr;
    QTimer timer;
    // Параметры управления
    double m_targetDepth = 0.0;      ///< Целевая глубина
    double m_targetX = 0.0;          ///< Целевая X-координата
    double m_targetY = 0.0;          ///< Целевая Y-координата
    double m_circleRadius = 5.0;     ///< Радиус окружности
    double m_circleCenterX = 0.0;    ///< X-центр окружности
    double m_circleCenterY = 0.0;    ///< Y-центр окружности

    NavigationSystem *navSystem = nullptr;
    SensorMove * sen_uWave = nullptr; //подключение гидроакустики
    PressureSensor * senPressure = nullptr; //подключение датчика давления
    IMUSensor * senIMU = nullptr; //подключение бсо

    /**
 * @brief Прогнозирование состояния на основе модели движения
 * @param dt Шаг времени (сек)
 */
    void prediction(double dt) {
        // Получаем данные IMU
        IMUOutput imu_data = senIMU->getOutput();

        // Прогнозируем положение на основе линейных скоростей из IMU
        state_(0) += imu_data.speed(0) * dt; // x += Vx * dt
        state_(1) += imu_data.speed(1) * dt; // y += Vy * dt
        state_(2) += imu_data.speed(2) * dt; // z += Vz * dt

        // Прогнозируем скорость (если нужно)
        state_(3) = imu_data.speed(0); // Vx
        state_(4) = imu_data.speed(1); // Vy
        state_(5) = imu_data.speed(2); // Vz

        // Обновление ковариации
        covariance_ = F_ * covariance_ * F_.transpose() + Q_;
    }

    /**
 * @brief Коррекция состояния по измерению расстояния до маяка
 * @param measured_distance Измеренное расстояние (м)
 */
    void correction(double measured_distance) {
        if (first_measurement_) {
            // Инициализация состояния по первому измерению
            state_(0) = x_global;
            state_(1) = y_global;
            state_(2) = z_global;
            first_measurement_ = false;
            qDebug() << "Инициализация EKF: x=" << state_(0) << " y=" << state_(1) << " z=" << state_(2);
        }
        // Координаты АНПА из EKF
        double x_auv = state_(0);
        double y_auv = state_(1);
        double z_auv = state_(2);

        // Координаты маяка
        double x_beacon = beacon_position_(0);
        double y_beacon = beacon_position_(1);
        double z_beacon = beacon_position_(2);

        // Расстояние между АНПА и маяком
        double dx = x_auv - x_beacon;
        double dy = y_auv - y_beacon;
        double dz = z_auv - z_beacon;
        double predicted_distance = sqrt(dx*dx + dy*dy + dz*dz);

        // Проверка на минимальное расстояние
        if (predicted_distance < 1e-3) {
            qDebug() << "Ошибка: АНПА и маяк в одной точке!";
            return;
        }

        // Матрица Якоби H (производные расстояния по координатам)
        H_(0, 0) = dx / predicted_distance;
        H_(0, 1) = dy / predicted_distance;
        H_(0, 2) = dz / predicted_distance;

        // Обновление по Калману
        Eigen::VectorXd z(1);
        z << measured_distance;
        Eigen::VectorXd y = z - Eigen::VectorXd::Constant(1, predicted_distance);
        Eigen::MatrixXd S = H_ * covariance_ * H_.transpose() + R_;
        Eigen::MatrixXd K = covariance_ * H_.transpose() * S.inverse();

        // Коррекция состояния и ковариации
        state_ = state_ + K * y;
        covariance_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_) * covariance_;

        qDebug() << "Измеренное расстояние:" << measured_distance;
        qDebug() << "Предсказанное расстояние:" << predicted_distance;
        qDebug() << "Матрица H:" << H_(0, 0) << "H_(0, 1)" << H_(0, 1) <<"H_(0, 2)" << H_(0, 2);
        qDebug() << "Коэффициент Калмана K:" << K(0) << "K(1)" << K(1) << "K(2)" << K(2);
    }

private:
    bool first_measurement_; // Флаг первого измерения
    // Состояние EKF: [x, y, z, vx, vy, vz]
    Eigen::VectorXd state_;
    // Ковариационная матрица
    Eigen::MatrixXd covariance_;
    // Матрица процесса (модель движения)
    Eigen::MatrixXd F_;
    // Матрица измерений
    Eigen::MatrixXd H_;
    // Шум процесса
    Eigen::MatrixXd Q_;
    // Шум измерений
    Eigen::MatrixXd R_;
    // Позиция маяка (глобальные координаты)
    Eigen::Vector3d beacon_position_;
    // Время последнего обновления
    double last_update_time_;
};

#endif // SU_ROV_H
