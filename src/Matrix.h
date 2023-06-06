//
// Created by 16058 on 2023/4/10.
//

#ifndef MATRIX_H
#define MATRIX_H

#include "MPCConfig.h"

// 矩阵类及相关运算
// Matrix class and related operations
class Matrix {
private:
    uint32_t row, column;             // 矩阵的行和列
    MatDataType_t data[MAXSIZE] = {};      // 矩阵的数据

    // 矩阵初等变换
    void RowExchange(uint32_t , uint32_t);
    void RowMul(uint32_t , MatDataType_t);
    void RowAdd(uint32_t , uint32_t , MatDataType_t);

public:
    // 矩阵初始化
    // Matrix initialization
    Matrix();
    Matrix(uint32_t , uint32_t);
    Matrix(uint32_t, uint32_t, MatDataType_t *);

    // 获取矩阵信息
    // Get matrix information
    void Print();

    bool operator>=(MatDataType_t);

    uint32_t getRow() const;
    uint32_t getCol() const;
    MatDataType_t& operator()(uint32_t, uint32_t);

    MatDataType_t MaxVal();
    Matrix NonNegProj();

    // 矩阵的基本运算
    // Basic Operations of Matrix
    Matrix Trans();
    Matrix Inv();

    Matrix operator+(const Matrix &);
    Matrix operator-(const Matrix &);
    Matrix operator*(const Matrix &);
    Matrix operator*(MatDataType_t);
    Matrix& operator=(const Matrix &);
};

#endif //MATRIX_H
