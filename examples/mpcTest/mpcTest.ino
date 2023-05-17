#include "MPC.h"

void setup() {
// write your initialization code here
    Serial.begin(115200);

    // 初始化MPC相关的矩阵
    // Initialize the matrix required for MPC
    MatDataType_t A_arr[4] = {2, 1, 0, 2 };
    MatDataType_t B_arr[4] = {1, 0, 0, 1 };
    MatDataType_t Q_arr[4] = {1, 0, 0, 3 };
    MatDataType_t R_arr[4] = {1, 0, 0, 1 };
    MatDataType_t QN_arr[4] = {4.16703866, 1.75655333, 1.75655333, 7.45580065};
    MatDataType_t F_arr[12] = {0.2, 0, -0.2, 0};
    MatDataType_t G_arr[12] = {0, 0, 0, 0, 1, 0, -1, 0, 0, 1, 0, -1 };
    MatDataType_t c_arr[6] = {1, 1, 1, 1, 1, 1};
    MatDataType_t FN_arr[8] = {1.58351933, 0.87827667, 0.086517, 1.78876199, -1.58351933, -0.87827667, -0.086517, -1.78876199};
    MatDataType_t cN_arr[4] = {1, 1, 1, 1};

    Matrix A = Matrix(2, 2);
    Matrix B = Matrix(2, 2);
    Matrix Q = Matrix(2, 2);
    Matrix R = Matrix(2, 2);
    Matrix QN = Matrix(2, 2);
    Matrix F = Matrix(6, 2);
    Matrix G = Matrix(6, 2);
    Matrix c = Matrix(6, 1);
    Matrix FN = Matrix(4, 2);
    Matrix cN = Matrix(4, 1);

    A = A_arr;
    B = B_arr;
    Q = Q_arr;
    R = R_arr;
    QN = QN_arr;
    F = F_arr;
    G = G_arr;
    c = c_arr;
    FN = FN_arr;
    cN = cN_arr;

    MatDataType_t state_arr[2] = {1.5, -0.75};
    Matrix state(2, 1);
    state = state_arr;

    // 初始化MPC类
    // Initialize MPC class
    MPC mpc = MPC(9.90287, 0.001, 0.001, 1000, 5, A, B, Q, R, QN, F, G, c, FN, cN);

    // 用于测试求解时间
    // Test the time needed for solving the problem
    uint32_t begin, end;
    Matrix input;

    begin = micros();
    input = mpc.Solver(state);
    end = micros();

    input.Print();
    Serial.println(end - begin);
}

void loop() {
// write your code here
}