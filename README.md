# MPC_ruih
Arduino library for linear MPC controller.
## References
This algorithm refers to the article [*`An Accelerated Dual Gradient-Projection Algorithm for Embedded Linear Model Predictive Control`*](https://ieeexplore.ieee.org/document/6426458) by Panagiotis Patrinos and Alberto Bemporad. You can read the [article](https://ieeexplore.ieee.org/document/6426458) for more details.
## Usage
Please see [matrixTest](https://github.com/rhrhhrhr/MPC_ruih/blob/main/examples/matrixTest/matrixTest.ino) if you want to know how to use the Matrix class and see [mpcTest](https://github.com/rhrhhrhr/MPC_ruih/blob/main/examples/mpcTest/mpcTest.ino) for MPC class.
### Parameters of the Matrix class
```cpp
uint32_t row, column;             // row and column of matrix 矩阵的行和列
MatDataType_t data[MAXSIZE] = {};      // data of matrix 矩阵的数据
```
The data of the matrix is stored in a fixed length array `data[MAXSIZE]`. You could modify the `MAXSIZE` in [MPCConfig.h](https://github.com/rhrhhrhr/MPC_ruih/blob/main/src/MPCConfig.h).
### Initialize a matrix
You can initialize a matrix in four ways: without parameters, with row and column, by an array or by another matrix.
#### Without parameters
This operation will generate an all zero matrix with one row and one column<br><br>
**code:**
```cpp
Matrix mat;
mat.Print();
```
**result:**
```cpp
0
```
#### With row and column
This operation will generate an all zero matrix with the corresponding row and column<br><br>
**code:**
```cpp
Matrix mat = Matrix(2, 2);
mat.Print();
```
**result:**
```cpp
0 0 
0 0 
```
#### By an array
This operation will push the data in array into matrix.<br><br>
**code:**
```cpp
MatDataType_t arr[4] = {1, 2, 3, 4};
Matrix mat = Matrix(2, 2);
mat = arr;
mat.Print();
```
**result:**
```cpp
1 2 
3 4 
```
Note that this method should be used after declaring the matrix's row and column, or the assignment won't success. The same data with different row and column will represent different matrices.<br><br>
**code:**
```cpp
MatDataType_t arr[4] = {1, 2, 3, 4};
Matrix mat1 = Matrix(2, 2);
Matrix mat2 = Matrix(4, 1);
mat1 = arr;
mat2 = arr;
mat1.Print();
mat2.Print();
```
**result:**
```cpp
1 2 
3 4 

1 
2 
3 
4 
```
#### By another matrix
This operation will copy the row, column and data of another matrix.<br><br>
**code:**
```cpp
MatDataType_t arr[4] = {1, 2, 3, 4};
Matrix mat1 = Matrix(2, 2);
Matrix mat2 = Matrix(4, 1);
mat1.Print();
mat2.Print();

mat1 = arr;
mat2 = mat1;
mat1.Print;
mat2.Print
```
**result:**
```cpp
0 0 
0 0 

0 
0 
0 
0 

1 2 
3 4 

1 2 
3 4 
```
### Parameters of the MPC class
```cpp
MatDataType_t L_phi, epsilon_V, epsilon_g;
uint32_t max_iter, N;
Matrix A, B, Q, R, QN, F, G, c, FN, cN;
```
The `L_phi` here is the Lipschitz constant for the gradient of the dual problem of the MPC optimizition problem dual. It is used for updating the dual varible y according to the Nesterov’s Accelerated Gradient Descent method. It is better to calculate it in advance. The corresponding code about how to calculate it in python has been uploaded in repo [MPC_ruih_MPCSetup](https://github.com/rhrhhrhr/MPC_ruih_MPCSetup). For more details you can refer to the article [*An Accelerated Dual Gradient-Projection Algorithm for Embedded Linear Model Predictive Control*](https://ieeexplore.ieee.org/document/6426458).<br><br>
The `epsilon_V` and the `epsilon_g` here are the tolerances of the error between optimal cost and real cost and the violation of constraints respectively. Note that the tolerances not only describe the absolute error, but also describe the relative error sometimes. This depends on the specific situation.<br><br>
The `max_iter` is the maximum number of the solving step.<br><br>
The `N` is the prediction horizon of the MPC controller.<br><br>
Matrix `A`, `B` describe the system's state space equation $$x_{k+1} = Ax_k + Bu_k$$
Matrix `Q`, `R`, `QN` describe the cost function of the MPC controller $$V(X, U) = \sum\limits_{k=0}^{N-1}(x_k^TQx_k + u_k^TRu_k) + x_N^TQ_Nx_N$$
Matrix `F`, `G`, `c` describe the state and input constraints $$Fx_k + Gu_k \le c$$
Matrix `FN`, `cN` describe the terminal constraints $$F_Nx_N \le c_N$$
### Initialize a MPC controller
You can use the python package LinearMPCFactor in repo [MPC_ruih_MPCSetup](https://github.com/rhrhhrhr/MPC_ruih_MPCSetup) to generate the setup code for a MPC controller as below:<br><br>
**code:**
```python
import numpy as np
import LinearMPCFactor as lMf

if __name__ == '__main__':
    A = np.array([[2, 1], [0, 2]])  # state space equation A 状态空间方程中的A
    B = np.array([[1, 0], [0, 1]])  # state space equation B 状态空间方程中的B
    Q = np.array([[1, 0], [0, 3]])  # cost function Q, which determines the convergence rate of the state 代价函数中的Q，决定了状态的收敛速度
    R = np.array([[1, 0], [0, 1]])  # cost function R, which determines the convergence rate of the input 代价函数中的R，决定了输入的收敛速度

    A_x = np.array([[1, 0], [-1, 0]])  # state constraints A_x @ x_k <= b_x 状态约束 A_x @ x_k <= b_x
    b_x = np.array([5, 5])

    A_u = np.array([[1, 0], [-1, 0], [0, 1], [0, -1]])  # input constraints A_u @ u_k <= b_u 输入约束 A_u @ u_k <= b_u
    b_u = np.array([1, 1, 1, 1])

    mpc = lMf.LinearMPCFactor(A, B, Q, R, 5, A_x, b_x, A_u, b_u)

    # mpc.decPlace = 4  # This is the number of decimal places reserved for matrix data, which defaults to 6 decimal places 这是矩阵数据保留的小数位数，默认保留小数点后6位

    mpc.PrintCppCode(0.001, 0.001, 1000)
```
**result:**
```cpp
MatDataType_t A_arr[4] = {2, 1, 0, 2};
MatDataType_t B_arr[4] = {1, 0, 0, 1};
MatDataType_t Q_arr[4] = {1, 0, 0, 3};
MatDataType_t R_arr[4] = {1, 0, 0, 1};
MatDataType_t QN_arr[4] = {4.167039, 1.756553, 1.756553, 7.455801};
MatDataType_t F_arr[12] = {1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
MatDataType_t G_arr[12] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, -1.0};
MatDataType_t c_arr[6] = {5, 5, 1, 1, 1, 1};
MatDataType_t FN_arr[8] = {1.583519, 0.878277, -1.583519, -0.878277, 0.086517, 1.788762, -0.086517, -1.788762};
MatDataType_t cN_arr[4] = {1.0, 1.0, 1.0, 1.0};

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

MatDataType_t L_phi = 9.90287;

MPC mpc = MPC(L_phi, 0.001, 0.001, 1000, 5, A, B, Q, R, QN, F, G, c, FN, cN);
```
## Note
Considering the real-time requirements of the algorithm, the data of the matrix is stored in a fixed length array rather than vector container. If larger matrix operations are needed, you can modify the `MAXSIZE` in the [MPCConfig.h](https://github.com/rhrhhrhr/MPC_ruih/blob/main/src/MPCConfig.h) file, which is the maximum number of matrix elements. In addition, you can also determine whether the data type of the matrix is float or double through the `SINGLE_PRECISION` and `DOUBLE_PRECISION` in the [MPCConfig.h](https://github.com/rhrhhrhr/MPC_ruih/blob/main/src/MPCConfig.h) file.
## Licence
MIT
## Author
Rui Huang
