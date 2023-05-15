//
// Created by 16058 on 2023/4/10.
//

#ifndef MATRIX_H
#define MATRIX_H

#include <Arduino.h>
#define MaxSize 15  // 因为求解优化需要大量读写操作, 使用vector容器或动态内存速度相较传统数组会很慢, 而使用了
                    // 一个大小固定的数组, 若数组大小不够, 可以修改这个值

class Matrix {
private:
    uint32_t row, column;
    float data[MaxSize] = {0};

public:
    Matrix();
    Matrix(uint32_t , uint32_t);

    uint32_t getRow() const;
    uint32_t getCol() const;

    void Print();
    float MaxVal();
    Matrix EuProj();

    Matrix Trans();
    Matrix Inv();
    void RowExchange(uint32_t , uint32_t);
    void RowMul(uint32_t , float);
    void RowAdd(uint32_t , uint32_t , float);

    Matrix operator+ (const Matrix&);
    Matrix operator- (const Matrix&);
    Matrix operator* (const Matrix&);
    Matrix operator* (float);
    Matrix& operator= (const Matrix&);
    Matrix& operator= (float*);
    float operator() (uint32_t, uint32_t);

    bool operator>= (float);
};

#endif //MATRIX_H
