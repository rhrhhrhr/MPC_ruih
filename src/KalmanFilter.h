//
// Created by 16058 on 2023/6/2.
//

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "Matrix.h"

/**
 *  Kalman filter class / 卡尔曼滤波器类
 */
class KalmanFilter {
private:
    /**
     * Posteriori state estimate x_hat and posteriori estimate covariance P_hat / 后验状态估计x_hat和后验协方差矩阵P_hat
     */
    Matrix x_hat, P_hat;
    /**
     * State-transition model A, control-input model B and Observation model / 状态转移矩阵A，控制输入矩阵B和观测矩阵C
     */
    Matrix A, B, C;
    /**
     * Covariance of the process noise Q and covariance of the observation noise R / 过程噪声协方差矩阵Q和观测噪声协方差矩阵R
     */
    Matrix Q, R;
    /**
     * Kalman gain / 卡尔曼增益
     */
    Matrix K;
    /**
     * Priori state estimate x_hat_pre and priori estimate covariance P_hat_pre / 先验状态估计x_hat_pre和先验协方差矩阵P_hat_pre
     */
    Matrix x_hat_pre, P_hat_pre;
    /**
     * Identity matrix, the transposition of A, the transposition of C / 单位矩阵，A、C的转置
     */
    Matrix I, A_T, C_T;

public:
    /**
     * KalmanFilter class constructor / KalmanFilter类的构造函数
     * @param x_hat_ini - Initial value of the state estimate / 状态估计的初值
     * @param P_hat_ini - Initial value of the estimate covariance / 协方差矩阵的初值
     * @param A - State-transition model / 状态转移矩阵
     * @param B - Control-input model / 控制输入矩阵
     * @param C - Observation model / 观测矩阵
     * @param Q - Covariance of the process noise / 过程噪声协方差矩阵
     * @param R - Covariance of the observation noise / 观测噪声协方差矩阵
     */
    KalmanFilter(Matrix &x_hat_ini, Matrix &P_hat_ini, Matrix &A, Matrix &B, Matrix &C, Matrix &Q, Matrix &R);
    /**
     * KalmanFilter class copy constructor / KalmanFilter类拷贝构造函数
     * @param filter - Another Kalman filter / 被拷贝的对象
     */
    KalmanFilter(const KalmanFilter &filter);
    /**
     * KalmanFilter class destructor / KalmanFilter类的析构函数
     */
    ~KalmanFilter();
    /**
     * Using Kalman filter through operator() / 通过()调用卡尔曼滤波器
     * @param u - Control-input / 控制输入
     * @param y - System-output / 系统输出
     */
    Matrix &operator()(Matrix &u, Matrix &y);
    /**
     * KalmanFilter class assignment operator / KalmanFilter类赋值运算符
     */
    KalmanFilter &operator=(const KalmanFilter &filter);
};

#endif //KALMAN_FILTER_H
