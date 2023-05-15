//
// Created by 16058 on 2023/4/10.
//

#include "Matrix.h"

// 当没有输入参数时，创建矩阵[[0]]
Matrix::Matrix() {
    row = 1;
    column = 1;
}

// 创建一个全为0的矩阵
Matrix::Matrix(uint32_t r, uint32_t c) {
    row = r;
    column = c;
}

// 获取矩阵的行
uint32_t Matrix::getRow() const {
    return row;
}

// 获取矩阵的列
uint32_t Matrix::getCol() const {
    return column;
}

// 打印矩阵
void Matrix::Print() {
    for (int i = 0; i < row * column; i++)
    {
        Serial.printf("%f\t", data[i]);
        if ((i + 1) % column == 0)
            Serial.println();
    }
    Serial.println();
}

// 获取矩阵中的最大值
float Matrix::MaxVal() {

    float max;

    max = data[0];

    for(int i = 1; i < row * column; i++)
    {
        max = max > data[i] ? max : data[i];
    }

    return max;
}

// 矩阵向正半空间投影
Matrix Matrix::EuProj() {
    Matrix mat(row, column);

    for(int i = 0; i < row * column; i++)
    {
        mat.data[i] = data[i] > 0 ? data[i] : 0;
    }

    return mat;
}

// 矩阵转置
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
Matrix Matrix::Inv() {
    Matrix mat_inv(row, column);
    Matrix matrixTemp(row, column);

    if (row == column)
    {
        memcpy(matrixTemp.data, data, row * column * sizeof(float));

        for (int i = 0; i < row; i++)
        {
            mat_inv.data[i * row + i] = 1;
        }

        for (int i = 0; i < column; i++)
        {
            int max = i;
            float k1, k2;

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

// 矩阵求逆用到的矩阵初等变换
// 交换矩阵的第i行和第j行
void Matrix::RowExchange(uint32_t i, uint32_t j) {
    if (i < row && j < row)
    {
        float temp;

        for (int k = 0; k < column; k++)
        {
            temp = data[i * column + k];
            data[i * column + k] = data[j * column + k];
            data[j * column + k] = temp;
        }
    }
}

// 给矩阵的第i行乘k
void Matrix::RowMul(uint32_t i, float k) {
    if (i < row  && k != 0)
    {
        for (int j = 0; j < column; j++)
        {
            data[i * column + j] = k * data[i * column + j];
        }
    }
}

// 矩阵第j行加上第i行乘k
void Matrix::RowAdd(uint32_t i, uint32_t j, float k) {
    if (i < row && j < row)
    {
        for (int m = 0; m < column; m++)
        {
            data[j * column + m] = data[j * column + m] + k * data[i * column + m];
        }
    }
}

// 矩阵加法
Matrix Matrix::operator+ (const Matrix& mat) {
    Matrix temp(this->row, this->column);

    for (int i = 0; i < this->row * this->column; i++)
    {
        temp.data[i] = this->data[i] + mat.data[i];
    }

    return temp;
}

// 矩阵减法
Matrix Matrix::operator- (const Matrix& mat) {
    Matrix temp(this->row, this->column);

    for (int i = 0; i < this->row * this->column; i++)
    {
        temp.data[i] = this->data[i] - mat.data[i];
    }

    return temp;
}

// 矩阵乘法
Matrix Matrix::operator* (const Matrix& mat) {
    Matrix temp(this->row, mat.column);

    for (int i = 0; i < this->row; i++) {
        for (int j = 0; j < mat.column; j++) {
            for (int k = 0; k < this->column; k++) {
                temp.data[i * mat.column + j] +=
                        this->data[i * this->column + k] * mat.data[k * mat.column + j];
            }
        }
    }

    return temp;
}

// 矩阵数乘
Matrix Matrix::operator* (float k) {
    Matrix temp(row, column);

    for(int i = 0; i < row * column; i++)
    {
        temp.data[i] = k * data[i];
    }

    return temp;
}

// 矩阵拷贝
Matrix& Matrix::operator= (const Matrix& mat) {
    if(this != &mat)
    {
        row = mat.row;
        column = mat.column;
        memcpy(data, mat.data, row * column * sizeof(float));
    }

    return *this;
}

// 数组给矩阵赋值
Matrix& Matrix::operator= (float *arr) {
    memcpy(data, arr, row * column * sizeof(float));

    return *this;
}

// 获取第i行第j列的数据
float Matrix::operator()(uint32_t i, uint32_t j) {
    float res = 0;

    if(i < row && j < column){
        res = data[i * column + j];
    }

    return res;
}

// 判断矩阵所有元素是否大于等于val
bool Matrix::operator>= (float val) {
    bool res = true;

    for(int i = 0; i < row * column; i++) {
        res = res & (data[i] >= val);
        if(!res) break;
    }

    return res;
}
