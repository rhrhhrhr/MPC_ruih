#include <MPC.h>

void setup() {
// write your initialization code here
    Serial.begin(115200);

    // 初始化MPC相关的矩阵
    // Initialize the matrix required for MPC
    MatDataType_t L_phi = 9.90287;
    MatDataType_t e_V = 0.001;
    MatDataType_t e_g = 0.001;
    uint32_t max_iter = 1000;
    uint32_t N = 5;
    
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

    Matrix A = Matrix(2, 2, A_arr);
    Matrix B = Matrix(2, 2, B_arr);
    Matrix Q = Matrix(2, 2, Q_arr);
    Matrix R = Matrix(2, 2, R_arr);
    Matrix QN = Matrix(2, 2, QN_arr);
    Matrix F = Matrix(6, 2, F_arr);
    Matrix G = Matrix(6, 2, G_arr);
    Matrix c = Matrix(6, 1, c_arr);
    Matrix FN = Matrix(4, 2, FN_arr);
    Matrix cN = Matrix(4, 1, cN_arr);
    
    // 初始化MPC类
    // Initialize MPC class
    MPC mpc = MPC(L_phi, e_V, e_g, max_iter, N, A, B, Q, R, QN, F, G, c, FN, cN);

    MatDataType_t state_arr[2] = {1.5, -0.75};
    Matrix state(2, 1, state_arr);

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
