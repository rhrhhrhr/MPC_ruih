# MPC_ruih
Arduino library for linear MPC controller.
## References
This algorithm refers to the article [*`An Accelerated Dual Gradient-Projection Algorithm for Embedded Linear Model Predictive Control`*](https://ieeexplore.ieee.org/document/6426458) by Panagiotis Patrinos and Alberto Bemporad. You can read the [article](https://ieeexplore.ieee.org/document/6426458) for more details.
## Usage
Please see [mpcTest](https://github.com/rhrhhrhr/MPC_ruih/blob/main/examples/mpcTest/mpcTest.ino) if you want to know how to use the MPC class and see [matrixTest](https://github.com/rhrhhrhr/MPC_ruih/blob/main/examples/matrixTest/matrixTest.ino) for Matrix class.
### Parameters of the MPC class
```cpp
MatDataType_t L_phi, epsilon_V, epsilon_g;
uint32_t max_iter, N;
Matrix A, B, Q, R, QN, F, G, c, FN, cN;
```
The `L_phi` here is the Lipschitz constant for the gradient of the dual problem of the MPC optimizition problem dual. It is used for updating the dual varible y according to the Nesterov’s Accelerated Gradient Descent method. It is better to calculate it in advance. The corresponding code about how to calculate it in python will be uploaded later. For more details you can refer to the article [*An Accelerated Dual Gradient-Projection Algorithm for Embedded Linear Model Predictive Control*](https://ieeexplore.ieee.org/document/6426458).<br><br>
The `epsilon_V` and the `epsilon_g` here are the tolerances of the error between optimal cost and real cost and the violation of constraints respectively. Note that the tolerances not only describe the absolute error, but also describe the relative error sometimes. This depends on the specific situation.<br><br>
The `max_iter` is the maximum number of the solving step.<br><br>
The `N` is the prediction horizon of the MPC controller.<br><br>
Matrix `A`, `B` describe the system's state space equation $$x_{k+1} = Ax_k + Bu_k$$<br><br>
Matrix `Q`, `R`, `QN` describe the cost function of the MPC controller $$V(X, U) = \sum\limits_{k=0}^{N-1}(x_k^TQx_k + u_k^TRu_k) + x_N^TQ_Nx_N$$<br><br>
Matrix `F`, `G`, `c` describe the state and input constraints $$Fx_k + Gu_k \le c$$<br><br>
Matrix `FN`, `cN` describe the terminal constraints $$F_Nx_N \le c_N$$
### Initialize a matrix
You can initialize a matrix in four ways, without parameters, with row and column, by an array or by another matrix.
#### Without parameters
This operation will generate an all zero matrix with one row and one column<br>
code:
```cpp
Matrix mat;
mat.Print();
```
result:
```cpp
0
```
#### With row and column
This operation will generate an all zero matrix with the corresponding row and column<br>
code:
```cpp
Matrix mat = Matrix(2, 2);
mat.Print();
```
result:
```cpp
0 0 
0 0 
```
#### By an array
This operation will push the data in array into matrix.<br>
code:
```cpp
MatDataType_t arr[4] = {1, 2, 3, 4};
Matrix mat = Matrix(2, 2);
mat = arr;
mat.Print();
```
result:
```cpp
1 2 
3 4 
```
Note that this method should be used after declare the matrix's row and column, or the assignment won't success. The same data with different row and column will represent different matrices.<br>
code:
```cpp
MatDataType_t arr[4] = {1, 2, 3, 4};
Matrix mat1 = Matrix(2, 2);
Matrix mat2 = Matrix(4, 1);
mat1 = arr;
mat2 = arr;
mat1.Print();
mat2.Print();
```
result:
```cpp
1 2 
3 4 

1 
2 
3 
4 
```
#### By another matrix
This operation will copy the row, column and data of another matrix.<br>
code:
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
result:
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
## Note
Considering the real-time requirements of the algorithm, the data of the matrix is stored in a fixed length array rather than vector container. If larger matrix operations are needed, you can modify the `MAXSIZE` in the [MPCConfig.h](https://github.com/rhrhhrhr/MPC_ruih/blob/main/src/MPCConfig.h) file, which is the maximum number of matrix elements. In addition, you can also determine whether the data type of the matrix is float or double through the `SINGLE_PRECISION` and `DOUBLE_PRECISION` in the [MPCConfig.h](https://github.com/rhrhhrhr/MPC_ruih/blob/main/src/MPCConfig.h) file.
## Licence
MIT
## Author
Rui Huang
