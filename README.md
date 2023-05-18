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
Matrix * K, * D, * M, * L, * C;
Matrix * y, * y_p, * w, * x, * u, * x_bar, * u_bar, * x_bar_p, * u_bar_p;
```
The `L_phi` here is the Lipschitz constant for the gradient of the dual problem of the MPC optimizition problem dual. It is used for updating the dual varible y according to the Nesterov’s Accelerated Gradient Descent method. It is better to calculate it in advance. The corresponding code about how to calculate it in python will be uploaded later. For more details you can refer to the article [*An Accelerated Dual Gradient-Projection Algorithm for Embedded Linear Model Predictive Control*](https://ieeexplore.ieee.org/document/6426458).<br><br>
The `epsilon_V` and the `epsilon_g` here are the tolerances of the error between optimal cost and real cost and the violation of constraints respectively. Note that the tolerances not only describe the absolute error, but also describe the relative error sometimes. This depends on the specific situation.<br><br>
Matrix A, B describe the system's state space equation $$x_{k+1} = Ax_k + Bu_k$$<br><br>
Matrix Q, R, QN describe the cost function of the MPC controller $$V(X, U) = \sum\limits_{k=0}^{N-1}(x_k^TQx_k + u_k^TRu_k) + x_N^TQ_Nx_N$$
## Note
Considering the real-time requirements of the algorithm, the data of the matrix is stored in a fixed length array rather than vector container. If larger matrix operations are needed, you can modify the `MAXSIZE` in the [MPCConfig.h](https://github.com/rhrhhrhr/MPC_ruih/blob/main/src/MPCConfig.h) file, which is the maximum number of matrix elements. In addition, you can also determine whether the data type of the matrix is float or double through the `SINGLE_PRECISION` and `DOUBLE_PRECISION` in the [MPCConfig.h](https://github.com/rhrhhrhr/MPC_ruih/blob/main/src/MPCConfig.h) file.
## Licence
MIT
## Author
Rui Huang
