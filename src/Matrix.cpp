//
// Created by 16058 on 2023/4/10.
//

#include "Matrix.h"

// 交换矩阵的第i行和第j行
// Exchange the i-th and jth rows of the matrix
void Matrix::RowExchange(uint32_t i, uint32_t j) {
    if (i < row && j < row)
    {
        MatDataType_t temp;

        for (int k = 0; k < column; k++)
        {
            temp = data[i * column + k];
            data[i * column + k] = data[j * column + k];
            data[j * column + k] = temp;
        }
    }
}

// 给矩阵的第i行乘k
// Multiplying the i-th row of a matrix by k
void Matrix::RowMul(uint32_t i, MatDataType_t k) {
    if (i < row  && k != 0)
    {
        for (int j = 0; j < column; j++)
        {
            data[i * column + j] = k * data[i * column + j];
        }
    }
}

// 矩阵第j行加上第i行乘k
// Add the i-th row multiplied by k to the jth row of the matrix
void Matrix::RowAdd(uint32_t i, uint32_t j, MatDataType_t k) {
    if (i < row && j < row)
    {
        for (int m = 0; m < column; m++)
        {
            data[j * column + m] = data[j * column + m] + k * data[i * column + m];
        }
    }
}

// 当没有输入参数时，创建矩阵[[0]]
// When there are no input parameters, create a matrix [[0]]
Matrix::Matrix() {
    row = 1;
    column = 1;
}

// 创建一个r行c列的零矩阵
// Create a zero matrix with r rows and c columns
Matrix::Matrix(uint32_t r, uint32_t c) {
    row = r;
    column = c;
}

// 创建一个r行c列的矩阵，且用数组给它赋值
// Create a matrix with r rows and c columns, and assign values to it using an array
Matrix::Matrix(uint32_t r, uint32_t c, MatDataType_t *arr) {
    row = r;
    column = c;

    memcpy(data, arr, row * column * sizeof(MatDataType_t));
}

// 获取矩阵的行
// Get the row of the matrix
uint32_t Matrix::getRow() const {
    return row;
}

// 获取矩阵的列
// Get the column of the matrix
uint32_t Matrix::getCol() const {
    return column;
}

// 获取第i行第j列的数据
// Get the i-th row and jth column of the matrix
MatDataType_t& Matrix::operator()(uint32_t i, uint32_t j) {
    return data[i * column + j];
}

// 打印矩阵
// Print the matrix
void Matrix::Print() {
    for (int i = 0; i < row * column; i++)
    {
        Serial.printf("%f\t", data[i]);
        if ((i + 1) % column == 0)
            Serial.println();
    }
    Serial.println();
}

// 判断矩阵所有元素是否大于等于val
// Determine whether all elements of the matrix are greater than or equal to val
bool Matrix::operator>= (MatDataType_t val) {
    bool res = true;

    for(int i = 0; i < row * column; i++) {
        res = res & (data[i] >= val);
        if(!res) break;
    }

    return res;
}

// 获取所有矩阵元素中的最大值
// Get the maximum value among all matrix elements
MatDataType_t Matrix::MaxVal() {

    MatDataType_t max;

    max = data[0];

    for(int i = 1; i < row * column; i++)
    {
        max = max > data[i] ? max : data[i];
    }

    return max;
}

// 把矩阵向非负象限投影
// Project a matrix into the nonnegative orthant
Matrix Matrix::NonNegProj() {
    Matrix mat(row, column);

    for(int i = 0; i < row * column; i++)
    {
        mat.data[i] = data[i] > 0 ? data[i] : 0;
    }

    return mat;
}

// 矩阵转置
// Matrix transpose
Matrix Matrix::Trans() {
    Matrix mat(column, row);

    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < column; j++)
        {
            mat.data[j * row + i] = data[i * column + j];
        }
    }

    return mat;
}

// 矩阵求逆(高斯法)
// Matrix inversion(Gaussian elimination method)
Matrix Matrix::Inv() {
    Matrix mat_inv(row, column);
    Matrix matrixTemp(row, column);

    if (row == column)
    {
        memcpy(matrixTemp.data, data, row * column * sizeof(MatDataType_t));

        for (int i = 0; i < row; i++)
        {
            mat_inv.data[i * row + i] = 1;
        }

        for (int i = 0; i < column; i++)
        {
            int max = i;
            MatDataType_t k1, k2;

            for (int j = i + 1; j < row; j++) {
                if (abs(matrixTemp.data[j * matrixTemp.column + i]) >
                    abs(matrixTemp.data[max * matrixTemp.column + i]))
                    max = j;
            }

            if (matrixTemp.data[max * column + i] != 0)
            {
                if (max != i)
                {
                    matrixTemp.RowExchange(i, max);
                    mat_inv.RowExchange(i, max);
                }

                k1 = 1 / (matrixTemp.data[i * column + i]);
                matrixTemp.RowMul(i, k1);
                mat_inv.RowMul(i, k1);

                for (int j = 0; j < row; j++) {
                    if (j != i) {
                        k2 = -(matrixTemp.data[j * matrixTemp.column + i]);
                        matrixTemp.RowAdd(i, j, k2);
                        mat_inv.RowAdd(i, j, k2);
                    }
                }
            }
            else
            {
                Serial.println("不可逆");
                break;
            }
        }
    }

    else
    {
        Serial.println("不可逆");
    }

    return mat_inv;
}

// 矩阵加法
// Matrix addition
Matrix Matrix::operator+ (const Matrix &mat) {
    Matrix temp(this->row, this->column);

    for (int i = 0; i < this->row * this->column; i++)
    {
        temp.data[i] = this->data[i] + mat.data[i];
    }

    return temp;
}

// 矩阵减法
// Matrix subtraction
Matrix Matrix::operator- (const Matrix &mat) {
    Matrix temp(this->row, this->column);

    for (int i = 0; i < this->row * this->column; i++)
    {
        temp.data[i] = this->data[i] - mat.data[i];
    }

    return temp;
}

// 矩阵乘法
// Matrix multiplication
Matrix Matrix::operator* (const Matrix &mat) {
    Matrix temp(this->row, mat.column);
#ifdef ORDER_OF_MATMUL_IJK
    for (int i = 0; i < this->row; i++) {
        for (int j = 0; j < mat.column; j++) {
            for (int k = 0; k < this->column; k++) {
                temp.data[i * mat.column + j] +=
                        this->data[i * this->column + k] * mat.data[k * mat.column + j];
            }
        }
    }
#endif

#ifdef ORDER_OF_MATMUL_IKJ
    for (int i = 0; i < this->row; i++) {
        for (int k = 0; k < this->column; k++) {
            for (int j = 0; j < mat.column; j++) {
                temp.data[i * mat.column + j] +=
                        this->data[i * this->column + k] * mat.data[k * mat.column + j];
            }
        }
    }
#endif

#ifdef ORDER_OF_MATMUL_JIK
    for (int j = 0; j < mat.column; j++) {
        for (int i = 0; i < this->row; i++) {
            for (int k = 0; k < this->column; k++) {
                temp.data[i * mat.column + j] +=
                        this->data[i * this->column + k] * mat.data[k * mat.column + j];
            }
        }
    }
#endif

#ifdef ORDER_OF_MATMUL_JKI
    for (int j = 0; j < mat.column; j++) {
        for (int k = 0; k < this->column; k++) {
            for (int i = 0; i < this->row; i++) {
                temp.data[i * mat.column + j] +=
                        this->data[i * this->column + k] * mat.data[k * mat.column + j];
            }
        }
    }
#endif

#ifdef ORDER_OF_MATMUL_KIJ
    for (int k = 0; k < this->column; k++) {
        for (int i = 0; i < this->row; i++) {
            for (int j = 0; j < mat.column; j++) {
                temp.data[i * mat.column + j] +=
                        this->data[i * this->column + k] * mat.data[k * mat.column + j];
            }
        }
    }
#endif

#ifdef ORDER_OF_MATMUL_KJI
    for (int k = 0; k < this->column; k++) {
        for (int j = 0; j < mat.column; j++) {
            for (int i = 0; i < this->row; i++) {
                temp.data[i * mat.column + j] +=
                        this->data[i * this->column + k] * mat.data[k * mat.column + j];
            }
        }
    }
#endif

    return temp;
}

// 矩阵数乘
// Scalar multiplication
Matrix Matrix::operator* (MatDataType_t k) {
    Matrix temp(row, column);

    for(int i = 0; i < row * column; i++)
    {
        temp.data[i] = k * data[i];
    }

    return temp;
}

// 矩阵拷贝
// Matrix copy
Matrix& Matrix::operator= (const Matrix &mat) {
    if(this != &mat)
    {
        row = mat.row;
        column = mat.column;
        memcpy(data, mat.data, row * column * sizeof(MatDataType_t));
    }

    return *this;
}
