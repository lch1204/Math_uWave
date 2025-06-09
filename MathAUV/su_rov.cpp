#include "su_rov.h"
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <QApplication>



SU_ROV::SU_ROV(QObject *parent) : QObject(parent)
{
    protocol = new ControlSystem::PC_Protocol(ConfigFile,"agent");
    qDebug() << "---start exchange";
    protocol->startExchange();

    connect(&timerExchange, &QTimer::timeout,[this](){
        this->tickExchange();
    });
    timerExchange.start(100);
}

void SU_ROV::tickExchange()
{
    readData();
    sendData();
}

void SU_ROV::model(const float Upl,const float Upp,const float Usl,const float Usp, const float Uzl, const float Uzp) {
    int limit1, limit2;
    double G;

    //модули упоров движителей
    Ppl = a[7];  // передний левый(1)
    Ppp = a[8];  // передний правый(2)
    Psl = a[9];  // стредний левый(3)
    Psp = a[10];  // средний правый(4)
    Pzl = a[11];  // задний левый(5)
    Pzp = a[12];  // задний правый(6)

    //проекции упоров движителей на продольную ось апарата X
    Ppl_x = Ppl*Ta[1][1];
    Ppp_x = Ppp*Ta[1][2];
    Psl_x = Psl*Ta[1][3];
    Psp_x = Psp*Ta[1][4];
    Pzl_x = Pzl*Ta[1][5];
    Pzp_x = Pzp*Ta[1][6];

    //проекции упоров движителей на продольную ось апарата Y
    Ppl_y = Ppl*Ta[2][1];
    Ppp_y = Ppp*Ta[2][2];
    Psl_y = Psl*Ta[2][3];
    Psp_y = Psp*Ta[2][4];
    Pzl_y = Pzl*Ta[2][5];
    Pzp_y = Pzp*Ta[2][6];

    //проекции упоров движителей на продольную ось апарата Z
    Ppl_z = Ppl*Ta[3][1];
    Ppp_z = Ppp*Ta[3][2];
    Psl_z = Psl*Ta[3][3];
    Psp_z = Psp*Ta[3][4];
    Pzl_z = Pzl*Ta[3][5];
    Pzp_z = Pzp*Ta[3][6];

    //момент создаваемый движетельным комплексом вокруг оси X
    Mpl_x = Ppl*Ta[4][1];
    Mpp_x = Ppp*Ta[4][2];
    Msl_x = Psl*Ta[4][3];
    Msp_x = Psp*Ta[4][4];
    Mzl_x = Pzl*Ta[4][5];
    Mzp_x = Pzp*Ta[4][6];

    //момент создаваемый движетельным комплексом вокруг оси Y
    Mpl_y = Ppl*Ta[5][1];
    Mpp_y = Ppp*Ta[5][2];
    Msl_y = Psl*Ta[5][3];
    Msp_y = Psp*Ta[5][4];
    Mzl_y = Pzl*Ta[5][5];
    Mzp_y = Pzp*Ta[5][6];

    //момент создаваемый движетельным комплексом вокруг оси Z
    Mpl_z = Ppl*Ta[6][1];
    Mpp_z = Ppp*Ta[6][2];
    Msl_z = Psl*Ta[6][3];
    Msp_z = Psp*Ta[6][4];
    Mzl_z = Pzl*Ta[6][5];
    Mzp_z = Pzp*Ta[6][6];

    double g = 9.81;
    G = m*g; //вес аппарата
    Fa = 100;
    Farx[0] = 0; Farx[1] = 0; Farx[2] = -Fa;

    //обнуление вертикальной положительной скорости аппарата при достижении поверхности
    limit1 = limit2 = 0;
    if (a[17] >= max_depth) {
        a[17] = max_depth;
        if (a[3] <= 0) {
            a[3] = 0;
            limit1 = 1;
        }
    };

    //обнуление вертикальной положительной скорости аппарата при достижении дна
    if (a[17] <= 0)
    {
        a[17] = 0;
        if (a[3] >= 0)
        {
            a[3] = 0;
            limit2 = 1;
        }
    };

    Fdx = Ppl_x + Ppp_x + Psl_x + Psp_x + Pzl_x + Pzp_x; // вектор сил и моментов, создаваемых движительным комплексом
    Fgx = -cv1[1] * a[1] * fabs(a[1]) - cv2[1] * a[1]; //произведение D1*Vx
    FloatageX = -sin(a[5]) * (G + Farx[2]);
    Fcx = C[1][1]*a[1] + C[1][2]*a[2]+C[1][3]*a[3]+C[1][4]*a[18]+C[1][5]*a[19] + C[1][6]*a[20];
    FloatageX = 0; //обнуление плавучести
    da[1] = (1/(m + lambda[1][1])) * (Fdx + Fgx + Fcx + FloatageX + Wv[1]); //vx'

    Fdy = Ppl_y + Ppp_y + Psl_y + Psp_y + Pzl_y + Pzp_y; // вектор сил и моментов, создаваемых движительным комплексом
    Fgy = -cv1[2] * a[2] * fabs(a[2]) - cv2[2] * a[2]; //произведение D1*Vy
    FloatageY = cos(a[5]) * sin(a[4]) * (G + Farx[2]);
    Fcy = C[2][1]*a[1] + C[2][2]*a[2]+C[2][3]*a[3]+C[2][4]*a[18]+C[2][5]*a[19] + C[2][6]*a[20];
    FloatageY = 0; //обнуление плавучести
    da[2] = (1/(m + lambda[2][2])) * (Fdy + Fgy + Fcy + FloatageY + Wv[2]); //vy'

    Fdz = Ppl_z + Ppp_z + Psl_z + Psp_z + Pzl_z + Pzp_z; // вектор сил и моментов, создаваемых движительным комплексом
    Fgz = -cv1[3] * a[3] * fabs(a[3]) - cv2[3] * a[3]; //произведение D1*Vz
    FloatageZ = cos(a[4]) * cos(a[5]) * (G + Farx[2]);
    Fcz = C[3][1]*a[1] + C[3][2]*a[2]+C[3][3]*a[3]+C[3][4]*a[18]+C[3][5]*a[19] + C[3][6]*a[20];
    FloatageZ = 0; //обнуление плавучести
    da[3] = (1/(m + lambda[3][3])) * (Fdz + Fgz + Fcz + FloatageZ + Wv[3]); //vz'

    da[4] = a[18] + (1/cos(a[5]) * ((a[19]) * sin(a[4]) * sin(a[5])  + sin(a[5]) * cos(a[4]) * a[20])) + Vt[4];  //производная крена

    da[5] = a[19] * cos(a[4]) - sin(a[4]) * a[20] + Vt[5];  //производная дифферента

    da[6] = (1/cos(a[5])) * (a[19] * sin(a[4]) + cos(a[4]) * (a[20])) + Vt[6]; //производная крена
    // Из матмодели имеем
    //K_двi - усредненный коэффициент усиления i-го движителя; T_двi=J_i/K_v1i  – наибольшее значение постоянной времени i-го ВМА
    da[7] = (1/Td) * (kd * (double)Upl - Ppl);  // передний нижний правый(1)simulation_time_
    da[8] = (1/Td) * (kd * (double)Upp - Ppp);  // передний нижний левый(2)
    da[9] = (1/Td) * (kd * (double)Usl - Psl);  // задний нижний левый(3)
    da[10] = (1/Td) * (kd * (double)Usp - Psp); //задний нижний правый(4)а
    da[11] = (1/Td) * (kd * (double)Uzl - Pzl); // передний верхний правый(5)
    da[12] = (1/Td) * (kd * (double)Uzp - Pzp); // передний верхний левый(6)

    double alfa[4][4]; //матрица перевода из связанной СК в глобальную СК
    alfa[1][1] = cos(a[5])*cos(a[6]);
    alfa[2][1] = sin(a[6])*cos(a[5]);
    alfa[3][1] = -sin(a[5]);
    alfa[1][2] = cos(a[6])*sin(a[5])*sin(a[4])-cos(a[4])*sin(a[6]);
    alfa[2][2] = cos(a[6])*cos(a[4])+sin(a[4])*sin(a[5])*sin(a[6]);
    alfa[3][2] = sin(a[4])*cos(a[5]);
    alfa[1][3] = sin(a[6])*sin(a[4])+cos(a[6])*cos(a[4])*sin(a[5]);
    alfa[2][3] = sin(a[5])*sin(a[6])*cos(a[4])-cos(a[6])*sin(a[4]);
    alfa[3][3] = cos(a[5])*cos(a[4]);

    da[15] = alfa[1][1] * a[1] + alfa[1][2] * a[2] + alfa[1][3] * a[3] + Vt[1];
    //dx_global

    da[16] = alfa[2][1] * a[1] + alfa[2][2] * a[2] + alfa[2][3] * a[3] + Vt[2];
    //dy_global

    da[17] = alfa[3][1] * a[1] + alfa[3][2] * a[2] + alfa[3][3] * a[3] + Vt[3];
    //dz_global

    double Fax = -sin(a[5])*Fa;
    double Fay = sin(a[4])*cos(a[5])*Fa;
    double Faz = cos(a[5])*cos(a[4])*Fa;

    Mdx = Mpl_x + Mpp_x + Msl_x + Msp_x + Mzl_x + Mzp_x;
    Mgx = -cw1[1] * a[18] * fabs(a[18]) - cw2[1] * a[18];
    Max = -h[2]*Faz + h[3]*Fay;
    //Max = 0; //обнуление момента от силы архимеды
    Mcx = C[4][1]*a[1] + C[4][2]*a[2]+C[4][3]*a[3]+C[4][4]*a[18]+C[4][5]*a[19] + C[4][6]*a[20];
    da[18] = (1/(J[1] + lambda[4][4])) * (Mdx + Mcx + Mgx + Max + Wv[4]);

    Mdy = Mpl_y + Mpp_y + Msl_y + Msp_y + Mzl_y + Mzp_y;
    Mgy = -cw1[2] * a[19] * fabs(a[19]) - cw2[2] * a[19];
    May = -Faz*h[1] + Fax*h[3];
    //May = 0; //обнуление момента от силы архимеды
    Mcy = C[5][1]*a[1] + C[5][2]*a[2]+C[5][3]*a[3]+C[5][4]*a[18]+C[5][5]*a[19] + C[5][6]*a[20];
    da[19] = (1/(J[2] + lambda[5][5])) * (Mdy + Mcy + Mgy + May + Wv[5]);

    Mdz = Mpl_z + Mpp_z + Msl_z + Msp_z + Mzl_z + Mzp_z;
    Mgz = -cw1[3] * a[20] * fabs(a[20]) - cw2[3] * a[20];
    Maz = -h[1]*Fay + h[2]*Fax;
    //Maz = 0; //обнуление момента от силы архимеды
    Mcz = C[6][1]*a[1] + C[6][2]*a[2]+C[6][3]*a[3]+C[6][4]*a[18]+C[6][5]*a[19] + C[6][6]*a[20];
    da[20] = (1/(J[3] + lambda[6][6])) * (Mdz + Mcz + Mgz + Maz + Wv[6]);

    da[21] = a[1];
    da[22] = a[2];
    da[23] = a[3];
}

void SU_ROV::resetModel(){
    for (int i=0;i<ANPA_MOD_CNT;i++) {a[i] = 0.0f; da[i]=0.0f;}   //f на конце означает число с плавающей точкой
    for (int i=0; i<7;i++){
        Wv[i]=0;
        Vt[i]=0;
        h[i]=0;
    }
}

void SU_ROV::tick(const float Ttimer)
{
    simulation_time_ += Ttimer; // Ttimer = 0.01
    double usilenie = 50000;
    BFS_DRK(10,0,0,1*usilenie,0,0);


    runge(X[50][0], X[60][0], X[70][0], X[80][0], X[90][0], X[100][0],Ttimer,Ttimer);
    sen_uWave->run_simulation_with_MathAUV(x_global, y_global, z_global, Ttimer);
    Eigen::Vector3d true_angular_rates(Wx, Wy, Wz);
    Eigen::Vector3d true_linear_vel(vx_local, vy_local, vz_local);
    senIMU->update(Gamma_g, Tetta_g,Psi_g,true_angular_rates,true_linear_vel);
    senPressure->update(z_global,Ttimer);


    // 1. Получение данных с датчиков
    auto imu_data = senIMU->getOutput();
    double depth = senPressure->getOutput();
    double distance = sen_uWave->getOutput();

    // 2. Прогноз фильтра
    X[151][0] = imu_data.angularRates(2);
    ekf_->predict(Ttimer, imu_data.acceleration, imu_data.orientation(2));

    // Коррекция по расстоянию до маяка
    if (distance > 0) {
        double time = simulation_time_ - last_correction_time_;
        qDebug() << "time" << time;
        (ekf_->correct(distance, time, 2));
            last_correction_time_ = simulation_time_;
    }

    // 4. Коррекция глубины по датчику давления
    ekf_->correctDepth(depth);

    X[102][0] = distance;

    // 4. Обновление интерфейса
    updateNavigationDisplay();
    Eigen::VectorXd state = ekf_->getState();
    senIMU->updateCoordinate(state[0], state[1], state[2]);
}

void SU_ROV::constructor()
{

    X[124][1]  = ekf_x = a[15] = protocol->rec_data.data.sensor.positions[0][0];
    X[125][1]  = ekf_y = a[16] = protocol->rec_data.data.sensor.positions[0][1];
    X[126][1]  = ekf_z = a[17] = protocol->rec_data.data.sensor.positions[0][2];
    da[15] = 0;
    da[16] = 0;
    da[17] = 0;

    double xb = protocol->rec_data.data.sensor.positions[1][0];
    double yb = protocol->rec_data.data.sensor.positions[1][1];
    double zb = protocol->rec_data.data.sensor.positions[1][2];
    beacon_position_ << xb, yb,zb; // Пример координат маяка
    qDebug() << "xb, yb,zb" << xb <<"yb" << yb << "zb"<< zb;
    last_update_time_ = 0.0;
    ekf_ = new NavigationEKF(beacon_position_);
    ekf_->setState(ekf_x,ekf_y,ekf_z, 0);

    // Инициализация EKF
    state_.resize(6);
    state_.setZero(); // Начальное состояние: [x=0, y=0, z=0, vx=0, vy=0, vz=0]

    double dt = 0.01;
    first_measurement_ = true; // Инициализация флага


    sen_uWave = new SensorMove();
    sen_uWave->readConfig("config.json");
    sen_uWave->addPositionAUV(a[15],a[16],a[17]);
    sen_uWave->addPositionModem(xb,yb,zb);

    senIMU = new IMUSensor(dt);
    senPressure = new PressureSensor();

    Upl = Upp = Usl = Usp = Uzl = Uzp = 0;

    X_protocol = new x_protocol("kx_pult.conf", "x",X);
    K_protocol = new Qkx_coeffs("kx_pult.conf","k");

    X[1][0]=32;
    connect(&timer, &QTimer::timeout,[this, dt](){
        X[2][0]=K[32];
        this->tick(dt);
    });
    timer.start(100);

    m = 8.2;
    cv1[1] = 7.4; cv1[2] = 5.9; cv1[3] = 10.0;
    cv2[1] = 0.7; cv2[2] = 0.5; cv2[3] = 0.9;
    cw1[1] = 1; cw1[2] = 1.4; cw1[3] = 0.018;
    cw2[1] = 0.1; cw2[2] = 0.14; cw2[3] = 0.0018;
    //Vt[1] = 0.2; Vt[2] = 0.2; Vt[3] = 0.2; Vt[4] = 0; Vt[5] = 0; Vt[6] = 0; // скорость течения
    //Wv[1] = 0; Wv[2] = 0; Wv[3] = 0; Wv[4] = 0; Wv[5] = 0; Wv[6] = 0; //внешние возмущения, лин. скорости([1]-[3], угловые скорости - [4]-[6])
    //h[1]= ; h[2]= ; h[3]= ; // радиус-вектор координат центра водоизмещения
    lambda[1][1] = 0.66; lambda[2][2] = 3.768; lambda[3][3] = 3.768;
    lambda[4][4] = 0; lambda[5][5] = 0.0185; lambda[6][6] = 0.0185;
    Ta[1][1] = 0; Ta[1][2] = 0; Ta[1][3] = 1; Ta[1][4] = 1; Ta[1][5] = 0; Ta[1][6] = 0;
    Ta[2][1] = 0.766; Ta[2][2] = -0.766; Ta[2][3] = 0; Ta[2][4] = 0; Ta[2][5] = 0.766; Ta[2][6] = -0.766;
    Ta[3][1] = 0.64; Ta[3][2] = 0.64; Ta[3][3] = 0; Ta[3][4] = 0; Ta[3][5] = 0.64; Ta[3][6] = 0.64;
    Ta[4][1] = -136.2*0.001; Ta[4][2] = 136.2*0.001; Ta[4][3] = 0; Ta[4][4] = 0; Ta[4][5] = -136.3*0.001; Ta[4][6] = 136.3*0.001;
    Ta[5][1] = -136.6*0.001; Ta[5][2] = -136.6*0.001; Ta[5][3] = 0; Ta[5][4] = 0; Ta[5][5] = 136.6*0.001; Ta[5][6] = 136.6*0.001;
    Ta[6][1] = 162.8*0.001; Ta[6][2] = -162.8*0.001; Ta[6][3] = 99*0.001; Ta[6][4] = -99*0.001; Ta[6][5] = -162.8*0.001; Ta[6][6] = 162.8*0.001;
    //матрица сил и моментов инерции (проверить вторую матрицу, пока я вбила из мат модели, но кажется там я ошиблась она же не симметрична относительно оси, что странно)
    C[1][1] = 0; C[1][2] = (m+lambda[2][2])*a[20]; C[1][3] = -(m + lambda[3][3])*a[19]; C[1][4] = 0; C[1][5] = 0; C[1][6] = 0;
    C[2][1] = -(m + lambda[1][1])*a[20]; C[2][2] = 0; C[2][3] = (m + lambda[3][3])*a[18]; C[2][4] = 0; C[2][5] = 0; C[2][6] = 0;
    C[3][1] = (m + lambda[1][1])*a[19]; C[3][2] = -(m+lambda[2][2])*a[18]; C[3][3] = 0; C[3][4] = 0; C[3][5] = 0; C[3][6] = 0;
    C[4][1] = 0; C[4][2] = 0; C[4][3] = 0; C[4][4] = 0; C[4][5] = -(J[3]+lambda[6][6])*a[20]; C[4][6] = (J[2]+lambda[5][5])*a[19];
    C[5][1] = 0; C[5][2] = 0; C[5][3] = 0; C[5][4] = (J[3]+lambda[6][6])*a[20]; C[5][5] = 0; C[5][6] = -(J[1]+lambda[4][4])*a[18];
    C[6][1] = 0; C[6][2] = 0; C[6][3] = 0; C[6][4] = -(J[2]+lambda[5][5])*a[19]; C[6][5] = (J[1]+lambda[4][4])*a[18]; C[6][6] = 0;
    J[1] = 0.02871; J[2] = 0.1045; J[3] = 0.1045; //моменты инерции вдоль соотв-х осей
    kd = 2; //коэффициент усиления движителей
    Td = 0.15; //постоянная времени движителей
    depth_limit=50;
    max_depth=50;
    protocol->send_data.payload.start = true;
}

void SU_ROV::readData()
{
    if (protocol->rec_data.data.startAlgorithm)
    {
        if (!startt) {
            startt = true;
            constructor();
        }
    }
    else
    {
        startt = false;
    }
}

void SU_ROV::sendData()
{

}

void SU_ROV::updateNavigationDisplay() {
    static double rr =0;
    Eigen::VectorXd state = ekf_->getState();
    emit updateCoromAUVEkf(state[0], state[1]);
    emit updateCoromAUVReal(x_global, y_global);
    qDebug() << "x_global" << x_global << "; y_global" << y_global<< "; z_global" << z_global;
    if (X[102][0]>0)
    {
        emit updateCircle(X[102][0]);
        rr = X[102][0];

    }
    protocol->send_data.payload.map.circle_radius = rr;
    qDebug() << "X[102][0]" << X[102][0];
    qDebug() << "rr" << rr;
    emit updateVelocityVector_ekf(state[3], state[4]);
    emit updateVelocityVector_real(vx_global,vy_global);
    emit updateX(x_global, state[0], simulation_time_);
    emit updateY(y_global, state[1], simulation_time_);
    emit updateZ(z_global, state[2], simulation_time_);
    if (startt)
    {
//        if (X[102][0]>0) protocol->send_data.payload.map.circle_radius = X[102][0];
        protocol->send_data.payload.map.ekf_vx = state[3];
        protocol->send_data.payload.map.ekf_vy = state[4];
        protocol->send_data.payload.map.ekf_x = state[0];
        protocol->send_data.payload.map.ekf_y = state[1];
        protocol->send_data.payload.map.real_vx = vx_global;
        protocol->send_data.payload.map.real_vy = vy_global;
        protocol->send_data.payload.map.real_x = x_global;
//        protocol->send_data.payload.map.real_y = y_global;
        protocol->send_data.payload.graph.ekf_x = state[0];
//        protocol->send_data.payload.graph.ekf_y = state[1];
        protocol->send_data.payload.graph.ekf_z = state[2];
        protocol->send_data.payload.map.real_y = y_global;
        protocol->send_data.payload.graph.ekf_y = state[1];
        protocol->send_data.payload.graph.x = x_global;
        protocol->send_data.payload.graph.y = y_global;
        protocol->send_data.payload.graph.z = z_global;
        protocol->send_data.payload.graph.timestamp =simulation_time_;
    }
}

bool SU_ROV::check_measurement_validity(double distance, double dt) {
    double innovation = distance - predicted_distance;
    double innovation_cov = (H_ * covariance_ * H_.transpose() + R_)(0,0);

    // Проверка по хи-квадрат с 95% доверительным интервалом
    return (innovation * innovation) < 5.991 * innovation_cov; // 5.991 для 2 степеней свободы
}

SU_ROV::~SU_ROV(){
}

void SU_ROV::runge(const float Upl,const float Upp,const float Usl,const float Usp, const float Uzl, const float Uzp, const float Ttimer, const float dt) {
    const double Kc = 180/M_PI;// перевод в градусы
    double a1[24], y[24];
    int i;
    const double H1 = dt;
    const int n = ANPA_MOD_CNT;
    model(Upl, Upp, Usl, Usp, Uzl, Uzp);
    for (i = 1; i < n; i++) {
        a1[i] = a[i];
        y[i] = da[i];
        a[i] = a1[i] + 0.5 * H1 * da[i];
    }

    model(Upl, Upp, Usl, Usp, Uzl, Uzp);
    for (i = 1; i < n; i++)
    {
        y[i] = y[i]+ 2 * da[i];
        a[i] = a1[i] + 0.5 * H1 * da[i];
    }

    model(Upl, Upp, Usl, Usp, Uzl, Uzp);
    for (i = 1; i < n; i++) {
        y[i] = y[i] + 2 * da[i];
        a[i] = a1[i] + H1 * da[i];
    }

    model(Upl, Upp, Usl, Usp, Uzl, Uzp);
    for (i = 1; i < n; i++) {
        a[i] = a1[i] + (H1 / 6) * (y[i] + da[i]);
    }

    //данные в СУ ( с преобразованием координат)

    x_global = a[15]; //координата х в глобальной СК
    y_global = a[16];  //координата у в глобальной СК
    z_global = a[17]; //отстояние от дна относительно реперной точки, расположенной на дне
    cur_depth = max_depth + z_global;  //текущая глубина
    Wx = a[18] * Kc; //угловые скорости в связанной СК в градусах/секунду
    Wy = a[19] * Kc;
    Wz = a[20] * Kc;

    vx_local = a[1]; vy_local = a[2]; vz_local = a[3];  //линейные скорости в связанной СК
    vx_global = da[15]; vy_global = da[16]; vz_global = da[17];  // линейные скорости в глобальной СК

    Gamma_g = a[4] * Kc; // угол крена (градусы)
    Tetta_g = a[5] * Kc; // угол дифферента
    Psi_g = a[6] * Kc; // угол курса

    W_Gamma_g = da[4] * Kc; // производная угла крена
    W_Tetta_g = da[5] * Kc; // производная угла диффеннента
    W_Psi_g = da[6] * Kc; // производная угла курса

    X[10][0]=Wx;
    X[11][0]=Wy;
    X[12][0]=Wz;

    X[13][0]=vx_local;
    X[14][0]=vy_local;
    X[15][0]=vz_local;

    X[16][0]=vx_global;
    X[17][0]=vy_global;
    X[18][0]=vz_global;

    X[19][0]=Gamma_g;
    X[20][0]=Tetta_g;
    X[21][0]=Psi_g;

    X[22][0]=x_global;
    X[23][0]=y_global;
    X[24][0]=z_global;

    X[25][0]=Ppl;
    X[26][0]=Ppp;
    X[27][0]=Psl;
    X[28][0]=Psp;
    X[29][0]=Pzl;
    X[30][0]=Pzp;

}

void SU_ROV::BFS_DRK(double Upsi, double Uteta, double Ugamma, double Ux, double Uy, double Uz)
{
    X[50][0] = (K[20]*Ux + K[21]*Uy + K[22]*Uz + K[23]*Ugamma + K[24]*Uteta + K[25]*Upsi)*K[26];//U1
    X[60][0] = (K[30]*Ux + K[31]*Uy + K[32]*Uz + K[33]*Ugamma + K[34]*Uteta + K[35]*Upsi)*K[36];//U2
    X[70][0] = (K[40]*Ux + K[41]*Uy + K[42]*Uz + K[43]*Ugamma + K[44]*Uteta + K[45]*Upsi)*K[46];//U3
    X[80][0] = (K[50]*Ux + K[51]*Uy + K[52]*Uz + K[53]*Ugamma + K[54]*Uteta + K[55]*Upsi)*K[56];//U4
    X[90][0] = (K[60]*Ux + K[61]*Uy + K[62]*Uz + K[63]*Ugamma + K[64]*Uteta + K[65]*Upsi)*K[66];//U5
    X[100][0] =(K[70]*Ux + K[71]*Uy + K[72]*Uz + K[73]*Ugamma + K[74]*Uteta + K[75]*Upsi)*K[76];//U6
}

Eigen::Matrix3d SU_ROV::eulerRotationMatrix(const Eigen::Vector3d& eulerAnglesDegrees) {
    // Преобразование углов из градусов в радианы
    double psi = eulerAnglesDegrees[2] * M_PI / 180.0; // Рыскание (Z)
    double theta = eulerAnglesDegrees[1] * M_PI / 180.0; // Тангаж (Y)
    double gamma = eulerAnglesDegrees[0] * M_PI / 180.0; // Крен (X)

    // Матрица поворота вокруг оси Z (рыскание)
    Eigen::Matrix3d Rz;
    Rz << cos(psi), -sin(psi), 0,
        sin(psi),  cos(psi), 0,
        0,         0,        1;

    // Матрица поворота вокруг оси Y (тангаж)
    Eigen::Matrix3d Ry;
    Ry << cos(theta), 0, sin(theta),
        0,          1, 0,
        -sin(theta), 0, cos(theta);

    // Матрица поворота вокруг оси X (крен)
    Eigen::Matrix3d Rx;
    Rx << 1, 0,          0,
        0, cos(gamma), -sin(gamma),
        0, sin(gamma),  cos(gamma);

    // Комбинированная матрица поворота: Rz * Ry * Rx (Z-Y-X)
    Eigen::Matrix3d rotationMatrix = Rz * Ry * Rx;
    return rotationMatrix;
}
