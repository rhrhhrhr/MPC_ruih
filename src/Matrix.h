//
// Created by 16058 on 2023/4/10.
//

#ifndef MATRIX_H
#define MATRIX_H

#include "MPCConfig.h"

/**
 *  Matrix class / 矩阵类
 */
class Matrix {
private:
    /**
     * The number of rows and columns in a matrix / 矩阵的行数和列数
     */
    uint32_t row, column;
    /**
     * Matrix data, stored in an array with a length of MAXSIZE / 矩阵的数据，用长度为MAXSIZE的数组存储
     */
    MatDataType_t data[MAXSIZE] = {};
    /**
     * Switch the i-th and jth rows of the matrix / 将矩阵的第i行和第j行进行交换
     * @param i - i-th row / 第i行
     * @param j - j-th row / 第j列
     */
    void RowSwitch(uint32_t i, uint32_t j);
    /**
     * Each element in the i-the row can be multiplied by a non-zero constant k / 给矩阵的第i行所有元素乘非零常数k
     * @param i - i-th row / 第i行
     * @param k - A non-zero constant k / 非零常数k
     */
    void RowMul(uint32_t i, MatDataType_t k);
    /**
     * j-th row can be replaced by the sum of that row and a multiple of i-th row / 将矩阵的第i行替换为第i行加第j行乘常数k
     * @param i - i-th row / 第i行
     * @param j - j-the row / 第j行
     * @param k - constant k / 常数k
     */
    void RowAdd(uint32_t i, uint32_t j, MatDataType_t k);

public:
    /**
     * Matrix class constructor, initialize a zero matrix with one row and one column / Matrix类的构造函数，初始化一个一行一列
     * 的零矩阵
     */
    Matrix();
    /**
     * Matrix class constructor, initialize an n-dimensional identity matrix / Matrix类的构造函数，初始化一个n维的单位矩阵
     * @param n - Dimension of Identity matrix / 单位矩阵的维度
     */
    explicit Matrix(uint32_t n);
    /**
     * Matrix class constructor, initialize a zero matrix with r row and c column / Matrix类的构造函数，初始化一个r行c列的零
     * 矩阵
     * @param r - Number of rows in the matrix / 矩阵的行数
     * @param c - Number of columns in the matrix / 矩阵的列数
     */
    Matrix(uint32_t r, uint32_t c);
    /**
     * Matrix class constructor, initialize a matrix with r row and c column, And copy the values of array arr into the
     * matrix / Matrix类的构造函数，初始化一个r行c列的矩阵，并将数组arr的值拷贝进矩阵
     * @param r - Number of rows in the matrix / 矩阵的行数
     * @param c - Number of columns in the matrix / 矩阵的列数
     * @param arr - Matrix data / 矩阵的数据
     */
    Matrix(uint32_t r, uint32_t c, MatDataType_t *arr);
    /**
     * Matrix class copy constructor / Matrix类拷贝构造函数
     * @param mat - Another matrix / 被拷贝的对象
     */
    Matrix(const Matrix &mat);
    /**
     * Matrix class destructor / Matrix类的析构函数
     */
    ~Matrix();
    /**
     * Print matrix through serial port / 通过串口打印矩阵
     */
    void Print();
    /**
     * Determine whether all elements of the matrix are greater than or equal to a certain value / 判断矩阵所有元素是否都大于
     * 等于某个值
     * @param val - The value to be compared / 需要被比较的值
     */
    bool operator>=(MatDataType_t val);
    /**
     * Get the number of rows in the matrix / 获取矩阵的行数
     */
    uint32_t getRow() const;
    /**
     * Get the number of columns in the matrix / 获取矩阵的列数
     */
    uint32_t getCol() const;
    /**
     * Obtain data from the i-th row and jth column of the matrix / 获取矩阵第i行第j列的数据
     * @param i - i-th row / 第i行
     * @param j - j-th row / 第j列
     */
    MatDataType_t &operator()(uint32_t i, uint32_t j);
    /**
     * Get the maximum value among all elements of the matrix / 获取矩阵所有元素中的最大值
     */
    MatDataType_t MaxVal();
    /**
     * Project all elements of the matrix onto a nonnegative interval / 将矩阵所有元素投影到非负区间
     */
    Matrix NonNegProj();
    /**
     * Matrix transposition / 矩阵转置
     */
    Matrix Trans();
    /**
     * Matrix inversion / 矩阵求逆
     */
    Matrix Inv();
    /**
     * Matrix addition / 矩阵加法
     * @param mat - Matrix to add / 需要加的矩阵
     */
    Matrix operator+(const Matrix &mat);
    /**
     * Matrix subtraction / 矩阵减法
     * @param mat - Matrix to subtract / 需要减的矩阵
     */
    Matrix operator-(const Matrix &mat);
    /**
     * Matrix multiplication / 矩阵乘法
     * @param mat - Matrix to multiply / 需要乘的矩阵
     */
    Matrix operator*(const Matrix &mat);
    /**
     * Matrix scalar multiplication / 矩阵数乘
     * @param k - Number to multiply / 需要乘的数
     */
    Matrix operator*(MatDataType_t k);
    /**
     * Matrix class assignment operator / Matrix类赋值运算符
     */
    Matrix &operator=(const Matrix &mat);
};

#endif //MATRIX_H
