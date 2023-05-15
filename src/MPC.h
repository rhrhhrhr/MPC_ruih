//
// Created by 16058 on 2023/4/10.
//

#ifndef MPC_H
#define MPC_H

#include "Matrix.h"

class MPC {
private:
    float L_phi, epsilon_V, epsilon_g;
    uint32_t max_iter, N;
    Matrix A, B, Q, R, QN, F, G, c, FN, cN;
    Matrix * K, * D, * M, * L, * C;
    Matrix * y, * y_p, * w, * x, * u, * x_bar, * u_bar, * x_bar_p, * u_bar_p;

    void FactorIni();
    void SolveDual(Matrix &, Matrix *, Matrix *, Matrix *);
    Matrix g(Matrix &, Matrix &);
    Matrix gN(Matrix &);
    float V(Matrix *, Matrix *);
    float Psi(Matrix *, Matrix &);
    float gMax(Matrix *, Matrix *);
    bool wNonNeg();
    bool Stop(Matrix &);

public:
    MPC(float, float, float, uint32_t , uint32_t , Matrix &, Matrix &, Matrix &, Matrix &, Matrix &, Matrix &, Matrix &,
        Matrix &, Matrix &, Matrix &);
    ~MPC();
    Matrix Solver(Matrix &);
};

#endif //MPC_H
