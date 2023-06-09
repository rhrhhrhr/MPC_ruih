//
// Created by 16058 on 2023/6/2.
//

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(Matrix &x_hat_ini, Matrix &P_hat_ini, Matrix &A, Matrix &B, Matrix &C, Matrix &Q, Matrix &R) {
    this->x_hat = x_hat_ini;
    this->P_hat = P_hat_ini;
    this->A = A;
    this->B = B;
    this->C = C;
    this->Q = Q;
    this->R = R;

    this->I = Matrix(x_hat_ini.getRow());
    this->A_T = A.Trans();
    this->C_T = C.Trans();
}

KalmanFilter::KalmanFilter(const KalmanFilter &filter) = default;

KalmanFilter::~KalmanFilter() = default;

Matrix &KalmanFilter::operator()(Matrix &u, Matrix &y) {
    // Prediction
    // 预测
    x_hat_pre = A * x_hat + B * u;
    P_hat_pre = A * P_hat * A_T + Q;

    // Update
    // 更新
    K = P_hat_pre * C_T * (C * P_hat_pre * C_T + R).Inv();
    x_hat = x_hat_pre + K * (y - C * x_hat_pre);
    P_hat = (I - K * C) * P_hat_pre;

    return x_hat;
}

KalmanFilter &KalmanFilter::operator=(const KalmanFilter &filter) = default;
