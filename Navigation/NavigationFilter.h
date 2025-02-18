#ifndef NAVIGATIONFILTER_H
#define NAVIGATIONFILTER_H
#pragma once
#include <Eigen/Dense>

class NavigationFilter {
public:
    NavigationFilter() {
        initFilter();
    }

    void initFilter() {
        x.setZero();
        P.setIdentity();
        P *= 0.1;
    }

    void predict(double dt) {
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;

        x.segment<3>(0) += x.segment<3>(3) * dt;
        P = F * P * F.transpose() + Q;
    }

    void update(const Eigen::Vector3d& measurement) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
        H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

        Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        x += K * (measurement - H * x);
        P = (Eigen::MatrixXd::Identity(6,6) - K*H) * P;
    }

    Eigen::VectorXd getState() const { return x; }

private:
    Eigen::VectorXd x{6};
    Eigen::MatrixXd P{6,6};
    Eigen::MatrixXd Q{Eigen::MatrixXd::Identity(6,6)*0.1};
    Eigen::MatrixXd R{Eigen::Matrix3d::Identity()*0.1};
};

#endif // NAVIGATIONFILTER_H
