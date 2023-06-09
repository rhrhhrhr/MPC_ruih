//
// Created by 16058 on 2023/4/10.
//

#ifndef MPC_H
#define MPC_H

#include "KalmanFilter.h"

/**
 *  MPCController class / MPC控制器类
 */
class MPCController {
private:
    /**
     * Lipschitz constant required in GPAD algorithm / GPAD算法中需要的李普希茨常数
     */
    MatDataType_t L_phi;
    /**
     * Cost function error epsilon_V and constraint error epsilon_g of the solver / 求解器的代价函数误差epsilon_V和
     * 约束误差epsilon_g
     */
    MatDataType_t epsilon_V, epsilon_g;
    /**
     * Maximum iteration steps of the solver / 求解器最大迭代步数
     */
    uint32_t max_iter;
    /**
     * MPC prediction horizon / MPC预测时域
     */
    uint32_t N;
    /**
     * State-transition model A and control-input model B / 状态转移矩阵A和控制输入矩阵B
     */
    Matrix A, B;
    /**
     * State penalty matrix Q, input penalty matrix R and terminal penalty matrix QN / 状态惩罚矩阵Q、输入惩罚矩阵R和
     * 终端惩罚矩阵QN
     */
    Matrix Q, R, QN;
    /**
     * State and input constraints F * x + G * u \<= c / 状态和输入约束F * x + G * u \<= c
     */
    Matrix F, G, c;
    /**
     * Terminal constraints FN * xN \<= cN / 终端约束FN * xN \<= cN
     */
    Matrix FN, cN;
    /**
     * The matrix array required to solve optimization problems, please refer to the article An Accelerated Dual
     * Gradient-Projection Algorithm for Embedded Linear Model Predictive Control by Panagiotis Patrinos and Alberto
     * Bemporad for details / 求解优化问题时需要的矩阵数组，详情请参考文章An Accelerated Dual Gradient-Projection Algorithm
     * for Embedded Linear Model Predictive Control，作者为Panagiotis Patrinos and Alberto Bemporad
     */
    Matrix *K, *D, *M, *L, *C;
    /**
     * Temporary variables required for solving optimization problems / 求解优化问题时需要的临时变量
     */
    Matrix *y, *y_p, *w, *x, *u, *x_bar, *u_bar;

    /**
     * Initialize matrix arrays K, D, M, L, C / 矩阵数组K, D, M, L, C初始化
     */
    void FactorIni();
    /**
     * Solving Dual Problems / 求解对偶问题
     * @param state - Real time state of the system / 系统实时状态
     * @param y_ - Dual Variables in MPC Optimization Problems / MPC优化问题对偶变量
     * @param x_ - Predicting State Sequences for MPC Optimization Problems / MPC优化问题预测状态序列
     * @param u_ - Predicting Input Sequences for MPC Optimization Problems / MPC优化问题预测输入序列
     */
    void SolveDual(Matrix &state, Matrix *y_, Matrix *x_, Matrix *u_);
    /**
     * State and input constraints function g(xk, uk) = F * xk + G * uk - c / 状态输入约束函数g(xk, uk) = F * xk + G * uk
     * - c
     * @param xk - State prediction value at time k after the current time / 当前时刻后的k时刻的状态预测值
     * @param uk - Input predicted value at time k after the current time / 当前时刻后的k时刻的输入预测值
     */
    Matrix g(Matrix &xk, Matrix &uk);
    /**
     * Terminal constraints function gN(xN) = FN * xN - cN / 终端约束函数gN(xN) = FN * xN - cN
     * @param xN - State prediction value at time N after the current time / 当前时刻后的N时刻的状态预测值
     */
    Matrix gN(Matrix &xN);
    /**
     * Find the maximum value in g(X, U), including g(xk, uk) and gN(xN) / 求取g(X, U)中的最大元素(包括g(xk, uk)和gN(xN))
     * @param x_ - Predicting State Sequences for MPC Optimization Problems / MPC优化问题预测状态序列
     * @param u_ - Predicting Input Sequences for MPC Optimization Problems / MPC优化问题预测输入序列
     */
    MatDataType_t gMax(Matrix *x_, Matrix *u_);
    /**
     * Cost function V(X, U) / 代价函数V(X, U)
     * @param x_ - Predicting State Sequences for MPC Optimization Problems / MPC优化问题预测状态序列
     * @param u_ - Predicting Input Sequences for MPC Optimization Problems / MPC优化问题预测输入序列
     */
    MatDataType_t V(Matrix *x_, Matrix *u_);
    /**
     * Dual function Psi(y, state) / 对偶函数Psi(y, state)
     * @param state - Real time state of the system / 系统实时状态
     * @param y_ - Dual Variables in MPC Optimization Problems / MPC优化问题对偶变量
     */
    MatDataType_t Psi(Matrix &state, Matrix *y_);
    /**
     * Determine if all elements of variable w are nonnegative. The definition of w refers to Nesterov's accelerated
     * Gradient descent, please refer to the article An Accelerated Dual Gradient-Projection Algorithm for Embedded
     * Linear Model Predictive Control by Panagiotis Patrinos and Alberto Bemporad for details / 判断变量w是否所有元素非负。
     * w的定义涉及Nesterov’s加速梯度下降法，详情请参考文章An Accelerated Dual Gradient-Projection Algorithm for Embedded Linear
     * Model Predictive Control，作者为Panagiotis Patrinos and Alberto Bemporad
     */
    bool wNonNeg();
    /**
     * Determine whether the solver should stop / 判断求解器是否应该停止
     * @param state - Real time state of the system / 系统实时状态
     */
    bool Stop(Matrix &state);

public:
    /**
     * MPCController class constructor / MPCController类的构造函数
     * @param L_phi                 - Lipschitz constant required by the solver (which needs to be calculated in
     *                                advance) / 初始参数分别为求解器所需的李普希茨常数(需要单独求)
     * @param epsilon_V, epsilon_g  - Cost function error epsilon_V and constraint error epsilon_g of the solver / 求解
     *                                器的代价函数误差epsilon_V和约束误差epsilon_g
     * @param max_iter              - Maximum iteration steps of the solver / 求解器最大迭代步数
     * @param A, B                  - State-transition model A and control-input model B / 状态转移矩阵A和控制输入矩阵B
     * @param Q, R, QN              - State penalty matrix Q, input penalty matrix R and terminal penalty matrix QN /
     *                                状态惩罚矩阵Q、输入惩罚矩阵R和终端惩罚矩阵QN
     * @param F, G, c               - State and input constraints F * x + G * u \<= c / 状态和输入约束F * x + G * u \<= c
     * @param FN, cN                - Terminal constraints FN * xN \<= cN / 终端约束FN * xN \<= cN
     */
    MPCController(MatDataType_t L_phi,
                  MatDataType_t epsilon_V, MatDataType_t epsilon_g,
                  uint32_t max_iter,
                  uint32_t N,
                  Matrix &A, Matrix &B,
                  Matrix &Q, Matrix &R, Matrix &QN,
                  Matrix &F, Matrix &G, Matrix &c,
                  Matrix &FN, Matrix &cN);
    /**
     * MPCController class copy constructor / MPCController类拷贝构造函数
     * @param mpc - Another MPC controller / 被拷贝的对象
     */
    MPCController(const MPCController &mpc);
    /**
     * MPCController class destructor / MPCController类的析构函数
     */
    ~MPCController();
    /**
     * Using MPC controller through operator() / 通过()调用MPC控制器
     * @param state - Real time state of the system / 系统实时状态
     */
    Matrix operator()(Matrix &state);
    /**
     * MPCController class assignment operator / MPCController类赋值运算符
     */
    MPCController &operator=(const MPCController &);
};

#endif //MPC_H
