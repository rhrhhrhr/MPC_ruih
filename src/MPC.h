//
// Created by 16058 on 2023/4/10.
//

#ifndef MPC_H
#define MPC_H

#include "Matrix.h"
#include "MPCConfig.h"

class MPC {
private:
    // MPC控制器所需的参数和矩阵
    // Parameters and matrices required for MPC controllers
    MatDataType_t L_phi, epsilon_V, epsilon_g;
    uint32_t max_iter, N;
    Matrix A, B, Q, R, QN, F, G, c, FN, cN;
    Matrix * K, * D, * M, * L, * C;
    Matrix * y, * y_p, * w, * x, * u, * x_bar, * u_bar, * x_bar_p, * u_bar_p;

    void FactorIni();  // 矩阵数组K, D, M, L, C初始化 Initialize matrix arrays K, D, M, L, C
    void SolveDual(Matrix &, Matrix *, Matrix *, Matrix *);  // 求解对偶问题 Solving Dual Problems

    // 判断求解器是否应该停止
    // Determine whether the solver should stop
    Matrix g(Matrix &, Matrix &);
    Matrix gN(Matrix &);
    MatDataType_t V(Matrix *, Matrix *);
    MatDataType_t Psi(Matrix *, Matrix &);
    MatDataType_t gMax(Matrix *, Matrix *);
    bool wNonNeg();
    bool Stop(Matrix &);

public:
    // MPC初始化
    // MPC initialization
    MPC(MatDataType_t, MatDataType_t, MatDataType_t, uint32_t , uint32_t , Matrix &, Matrix &, Matrix &, Matrix &, Matrix &, Matrix &, Matrix &,
        Matrix &, Matrix &, Matrix &);
    ~MPC();

    // 优化求解器主体
    // The main part of optimization solver
    Matrix Solver(Matrix &);
};

#endif //MPC_H
