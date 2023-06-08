//
// Created by 16058 on 2023/6/2.
//

#include "KalmanFilter.h"

Matrix KalmanFilter::Eye(uint32_t n) {
    MatDataType_t eye_arr[n * n];

    for(int i = 0; i < n * n; i++) {
        if(i / n == i % n) {
            eye_arr[i] = 1;
        } else {
            eye_arr[i] = 0;
        }
    }

    Matrix eye = Matrix(n, n, eye_arr);

    return eye;
}

KalmanFilter::KalmanFilter(Matrix &x_ini, Matrix &P_ini, Matrix &A, Matrix &B, Matrix &C, Matrix &Q, Matrix &R) {
    this->x = x_ini;
    this->P = P_ini;
    this->A = A;
    this->B = B;
    this->C = C;
    this->Q = Q;
    this->R = R;

    this->I = Eye(x_ini.getRow());
    this->A_T = A.Trans();
    this->C_T = C.Trans();
}

Matrix& KalmanFilter::operator()(Matrix &u, Matrix &y) {
    x_pre = A * x + B * u;
    P_pre = A * P * A_T + Q;

    K = P_pre * C_T * (C * P_pre * C_T + R).Inv();
    x = x_pre + K * (y - C * x_pre);
    P = (I - K * C) * P_pre;

    return x;
}
