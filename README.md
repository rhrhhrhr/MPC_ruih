# MPC_ruih

This algorithm refers to the article *An Accelerated Dual Gradient-Projection Algorithm for Embedded Linear Model Predictive Control* by by Panagiotis Patrinos and Alberto Bemporad.

Considering the real-time requirements of the algorithm, the data of the matrix is stored in a fixed length array. If larger matrix operations are needed, MAXSIZE can be modified.
