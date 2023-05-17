# MPC_ruih
Arduino library for linear MPC controller.
## References
This algorithm refers to the article *An Accelerated Dual Gradient-Projection Algorithm for Embedded Linear Model Predictive Control* by by Panagiotis Patrinos and Alberto Bemporad.
## Usage
Please see [mpcTest](https://github.com/rhrhhrhr/MPC_ruih/blob/main/examples/mpcTest/mpcTest.ino) if you want to know how to use the MPC class and see [matrixTest](https://github.com/rhrhhrhr/MPC_ruih/blob/main/examples/matrixTest/matrixTest.ino) for Matrix class.
## Note
Considering the real-time requirements of the algorithm, the data of the matrix is stored in a fixed length array rather than vector container. If larger matrix operations are needed, you can modify the `MAXSIZE` in the [MPCConfig.h](https://github.com/rhrhhrhr/MPC_ruih/blob/main/src/MPCConfig.h) file, which is the maximum number of matrix elements. In addition, In addition, you can also determine whether the data type of the matrix is float or double through the macro definitions `SINGLE_PRECISION` and `DOUBLE_PRECISION` in the [MPCConfig.h](https://github.com/rhrhhrhr/MPC_ruih/blob/main/src/MPCConfig.h) file.
## Licence
MIT
## Author
Rui Huang
