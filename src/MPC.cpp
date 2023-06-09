//
// Created by 16058 on 2023/4/10.
//

#include "MPC.h"

MPCController::MPCController(MatDataType_t L_phi, MatDataType_t epsilon_V, MatDataType_t epsilon_g, uint32_t max_iter,
         uint32_t N, Matrix &A, Matrix &B, Matrix &Q, Matrix &R, Matrix &QN,
         Matrix &F, Matrix &G, Matrix &c, Matrix &FN, Matrix &cN)
{
    this->L_phi = L_phi;
    this->epsilon_V = epsilon_V;
    this->epsilon_g = epsilon_g;
    this->max_iter = max_iter;
    this->N = N;

    this->A = A;
    this->B = B;
    this->Q = Q;
    this->R = R;
    this->QN = QN;
    this->F = F;
    this->G = G;
    this->c = c;
    this->FN = FN;
    this->cN = cN;

    K = new Matrix[N];
    D = new Matrix[N];
    M = new Matrix[N];
    L = new Matrix[N];
    C = new Matrix[N];

    y = new Matrix[N + 1];
    y_p = new Matrix[N + 1];
    w = new Matrix[N + 1];
    x = new Matrix[N + 1];
    u = new Matrix[N];
    x_bar = new Matrix[N + 1];
    u_bar = new Matrix[N];

    FactorIni();
}

MPCController::MPCController(const MPCController &mpc) {
    L_phi = mpc.L_phi;
    epsilon_V = mpc.epsilon_V;
    epsilon_g = mpc.epsilon_g;
    max_iter = mpc.max_iter;
    N = mpc.N;

    A = mpc.A;
    B = mpc.B;
    Q = mpc.Q;
    R = mpc.R;
    QN = mpc.QN;
    F = mpc.F;
    G = mpc.G;
    c = mpc.c;
    FN = mpc.FN;
    cN = mpc.cN;

    K = new Matrix[mpc.N];
    D = new Matrix[mpc.N];
    M = new Matrix[mpc.N];
    L = new Matrix[mpc.N];
    C = new Matrix[mpc.N];

    for(int i = 0; i < mpc.N; i++) {
        K[i] = mpc.K[i];
        D[i] = mpc.D[i];
        M[i] = mpc.M[i];
        L[i] = mpc.L[i];
        C[i] = mpc.C[i];
    }

    y = new Matrix[mpc.N + 1];
    y_p = new Matrix[mpc.N + 1];
    w = new Matrix[mpc.N + 1];
    x = new Matrix[mpc.N + 1];
    u = new Matrix[mpc.N];
    x_bar = new Matrix[mpc.N + 1];
    u_bar = new Matrix[mpc.N];
}

// 释放内存
// free memory
MPCController::~MPCController() {
    delete[] K;
    delete[] D;
    delete[] M;
    delete[] L;
    delete[] C;
    delete[] y;
    delete[] y_p;
    delete[] w;
    delete[] x;
    delete[] u;
    delete[] x_bar;
    delete[] u_bar;

    K = nullptr;
    D = nullptr;
    M = nullptr;
    L = nullptr;
    C = nullptr;
    y = nullptr;
    y_p = nullptr;
    w = nullptr;
    x = nullptr;
    u = nullptr;
    x_bar = nullptr;
    u_bar = nullptr;
}

void MPCController::FactorIni() {
    Matrix P[N + 1];
    Matrix R_bar[N];
    Matrix S_bar[N];

    Matrix A_T, B_T, G_T;
    A_T = A.Trans();
    B_T = B.Trans();
    G_T = G.Trans();

    P[N] = QN;

    for (int k = (int)(N - 1); k >= 0; k--)
    {
        R_bar[k] = R + B_T * P[k + 1] * B;
        S_bar[k] = B_T * P[k + 1] * A;

        P[k] = Q + A_T * P[k + 1] * A - S_bar[k].Trans() * R_bar[k].Inv() * S_bar[k];
    }

    for (int k = 0; k < N; k++)
    {
        Matrix R_bar_Inv = R_bar[k].Inv();

        K[k] = R_bar_Inv * S_bar[k] * (-1);
        D[k] = R_bar_Inv * G_T * (-1);
        M[k] = R_bar_Inv * B_T * (-1);
        L[k] = (A + B * K[k]).Trans();
        C[k] = (F + G * K[k]).Trans();
    }
}

// 求解使对偶问题最小的x, u, 用到了里卡提递归
// Solve x, u to minimize the dual problem using Riccati recursion
void MPCController::SolveDual(Matrix &state, Matrix *y_, Matrix *x_, Matrix *u_) {
    Matrix e[N];

    e[N - 1] = FN.Trans() * y_[N];

    for(int k = (int)(N - 2); k >= 0; k--)
    {
        e[k] = L[k + 1] * e[k + 1] + C[k + 1] * y_[k + 1];
    }

    x_[0] = state;

    for(int k = 0; k < N; k++)
    {
        u_[k] = K[k] * x_[k] + D[k] * y_[k] + M[k] * e[k];
        x_[k + 1] = A * x_[k] + B * u_[k];
    }
}

Matrix MPCController::g(Matrix &xk, Matrix &uk) {

    Matrix g_k = F * xk + G * uk - c;

    return g_k;
}

Matrix MPCController::gN(Matrix &xN) {
    Matrix g_N = FN * xN - cN;

    return g_N;
}

MatDataType_t MPCController::gMax(Matrix *x_, Matrix *u_) {

    MatDataType_t temp;
    MatDataType_t res = gN(x_[N]).MaxVal();

    for(int k = 0; k < N; k++)
    {
        temp = g(x_[k], u_[k]).MaxVal();
        res = res > temp ? res : temp;
    }

    return res;
}

MatDataType_t MPCController::V(Matrix *x_, Matrix *u_) {
    MatDataType_t res;
    Matrix res_mat(1, 1);

    for(int k = 0; k < N; k++)
    {
        res_mat = res_mat + (x_[k].Trans() * Q * x_[k] + u_[k].Trans() * R * u_[k]) * 0.5;
    }

    res_mat = res_mat + x_[N].Trans() * QN * x_[N] * 0.5;

    res = res_mat(0, 0);

    return res;
}

MatDataType_t MPCController::Psi(Matrix &state, Matrix *y_) {
    MatDataType_t res;
    Matrix res_mat(1, 1);

    Matrix x_[N + 1];
    Matrix u_[N];

    SolveDual(state, y_, x_, u_);

    for(int k = 0; k < N; k++)
    {
        res_mat = res_mat + y_[k].Trans() * g(x_[k], u_[k]);
    }

    res_mat = res_mat + y_[N].Trans() * gN(x_[N]);

    res = res_mat(0, 0) + V(x_, u_);

    return res;
}

bool MPCController::wNonNeg() {
    bool res = true;

    for(int i = 0; i < N + 1; i++)
    {
        res = res & (w[i] >= 0);
        if(!res) break;
    }

    return res;
}

bool MPCController::Stop(Matrix &state) {
    bool res1, res2;
    MatDataType_t temp1, temp2, temp3, max;
    Matrix matTemp;

    for(int k = 0; k < N; k++)
    {
        matTemp = matTemp + w[k].Trans() * g(x[k], u[k]);
    }

    matTemp = matTemp + w[N].Trans() * gN(x[N]);

    temp1 = -matTemp(0, 0);
    temp2 = V(x, u);
    temp3 = Psi(state, y);

    max = temp3 > 1 ? temp3 : 1;

    res1 = (temp1 <= epsilon_V) | (temp1 <= (epsilon_V / (1 + epsilon_V)) * temp2); // 条件1
    res2 = (temp2 - temp3) <= (epsilon_V * max); // 条件2

    return res1 | res2;
}

// GPAD算法求解MPC优化问题
// Using GPAD algorithm to solve MPC optimization problems
Matrix MPCController::operator()(Matrix &state) {

    int iter = 0;
    MatDataType_t theta = 1;
    MatDataType_t theta_p = 1;
    bool doloop = true;

    // 将y, y_p等初始化为全零向量
    for(int i = 0; i < N; i++)
    {
        y[i] = Matrix(F.getRow(), 1);
        y_p[i] = Matrix(F.getRow(), 1);
    }

    y[N] = Matrix(FN.getRow(), 1);
    y_p[N] = Matrix(FN.getRow(), 1);

    // Nesterov’s加速梯度投影法解决线性MPC问题的对偶问题
    // Using Nesterov’s Accelerated Gradient Descent method to solve the dual problem of MPC
    while (iter < max_iter && doloop)
    {
        // 利用上一步的y求解这一步的w
        for(int i = 0; i < N + 1; i++)
        {
            w[i] = y[i] + (y[i] - y_p[i]) * (theta * ((1 / theta_p) - 1));
        }

        // 求解对偶问题的优化，从而可以利用梯度投影求下一时刻的
        SolveDual(state, w, x, u);

        //求解z_bar用于判断何时停止
        x_bar[N] = x_bar[N] * (1 - theta) + x[N] * theta;

        for(int i = 0; i < N; i++)
        {
            x_bar[i] = x_bar[i] * (1 - theta) + x[i] * theta;
            u_bar[i] = u_bar[i] * (1 - theta) + u[i] * theta;
        }

        //令y_p = y
        for(int i = 0; i < N + 1; i++)
        {
            y_p[i] = y[i];
        }

        //求解下一时刻的y
        y[N] = w[N] + gN(x[N]) * (1 / L_phi);
        y[N] = y[N].NonNegProj();

        for(int i = 0; i < N; i++)
        {
            y[i] = w[i] + g(x[i], u[i]) * (1 / L_phi);
            y[i] = y[i].NonNegProj();
        }

        theta_p = theta;
        //求解下一时刻的theta
        theta = (sqrtf(powf(theta, 4) + 4 * powf(theta, 2)) - powf(theta, 2)) / 2;

        //判断是否应该停止
        if(gMax(x_bar, u_bar) <= epsilon_g) {
            doloop = false;
        }

        else if(gMax(x, u) <= epsilon_g)
        {
            if(wNonNeg()) {
                if(Stop(state)) {
                    doloop = false;
                }
            }
        }

        iter++;
    }

    return u[0];
}

MPCController &MPCController::operator=(const MPCController &mpc) {
    if(this != &mpc) {
        L_phi = mpc.L_phi;
        epsilon_V = mpc.epsilon_V;
        epsilon_g = mpc.epsilon_g;
        max_iter = mpc.max_iter;
        N = mpc.N;

        A = mpc.A;
        B = mpc.B;
        Q = mpc.Q;
        R = mpc.R;
        QN = mpc.QN;
        F = mpc.F;
        G = mpc.G;
        c = mpc.c;
        FN = mpc.FN;
        cN = mpc.cN;

        delete[] K;
        delete[] D;
        delete[] M;
        delete[] L;
        delete[] C;
        delete[] y;
        delete[] y_p;
        delete[] w;
        delete[] x;
        delete[] u;
        delete[] x_bar;
        delete[] u_bar;

        K = new Matrix[mpc.N];
        D = new Matrix[mpc.N];
        M = new Matrix[mpc.N];
        L = new Matrix[mpc.N];
        C = new Matrix[mpc.N];

        for(int i = 0; i < mpc.N; i++) {
            K[i] = mpc.K[i];
            D[i] = mpc.D[i];
            M[i] = mpc.M[i];
            L[i] = mpc.L[i];
            C[i] = mpc.C[i];
        }

        y = new Matrix[mpc.N + 1];
        y_p = new Matrix[mpc.N + 1];
        w = new Matrix[mpc.N + 1];
        x = new Matrix[mpc.N + 1];
        u = new Matrix[mpc.N];
        x_bar = new Matrix[mpc.N + 1];
        u_bar = new Matrix[mpc.N];
    }

    return *this;
}
