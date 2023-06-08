//
// Created by 16058 on 2023/6/2.
//

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "Matrix.h"

class KalmanFilter {
private:
    Matrix x, P, A, B, C, Q, R;
    Matrix K, x_pre, P_pre;
    Matrix I, A_T, C_T;

    static Matrix Eye(uint32_t);

public:
    KalmanFilter(Matrix &, Matrix &, Matrix &, Matrix &, Matrix &, Matrix &, Matrix &);

    Matrix& operator()(Matrix &, Matrix &);
};

#endif //KALMAN_FILTER_H
