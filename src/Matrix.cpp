//
// Created by 16058 on 2023/4/10.
//

#include "Matrix.h"

void Matrix::RowSwitch(uint32_t i, uint32_t j) {
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

void Matrix::RowMul(uint32_t i, MatDataType_t k) {
    if (i < row  && k != 0)
    {
        for (int j = 0; j < column; j++)
        {
            data[i * column + j] = k * data[i * column + j];
        }
    }
}

void Matrix::RowAdd(uint32_t i, uint32_t j, MatDataType_t k) {
    if (i < row && j < row)
    {
        for (int m = 0; m < column; m++)
        {
            data[j * column + m] += k * data[i * column + m];
        }
    }
}

Matrix::Matrix() {
    row = 1;
    column = 1;
}

Matrix::Matrix(uint32_t n) {
    row = n;
    column = n;

    for(int i = 0; i < n; i++) {
        data[i * n + i] = 1;
    }
}

Matrix::Matrix(uint32_t r, uint32_t c) {
    row = r;
    column = c;
}

Matrix::Matrix(uint32_t r, uint32_t c, MatDataType_t *arr) {
    row = r;
    column = c;

    memcpy(data, arr, row * column * sizeof(MatDataType_t));
}

Matrix::Matrix(const Matrix &mat) = default;

// 析构函数
// Destructor
Matrix::~Matrix() = default;

uint32_t Matrix::getRow() const {
    return row;
}

uint32_t Matrix::getCol() const {
    return column;
}

MatDataType_t &Matrix::operator()(uint32_t i, uint32_t j) {
    return data[i * column + j];
}

void Matrix::Print() {
    for (int i = 0; i < row * column; i++)
    {
        Serial.printf("%f\t", data[i]);
        if ((i + 1) % column == 0)
            Serial.println();
    }
    Serial.println();
}

bool Matrix::operator>= (MatDataType_t val) {
    bool res = true;

    for(int i = 0; i < row * column; i++) {
        res = res & (data[i] >= val);
        if(!res) break;
    }

    return res;
}

MatDataType_t Matrix::MaxVal() {

    MatDataType_t max;

    max = data[0];

    for(int i = 1; i < row * column; i++)
    {
        max = max > data[i] ? max : data[i];
    }

    return max;
}

Matrix Matrix::NonNegProj() {
    Matrix mat(row, column);

    for(int i = 0; i < row * column; i++)
    {
        mat.data[i] = data[i] > 0 ? data[i] : 0;
    }

    return mat;
}

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

// Gaussian Elimination Method
// 高斯消元法
Matrix Matrix::Inv() {
    Matrix mat_inv(row);
    Matrix matrixTemp(row, column);

    if (row == column)
    {
        memcpy(matrixTemp.data, data, row * column * sizeof(MatDataType_t));

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
                    matrixTemp.RowSwitch(i, max);
                    mat_inv.RowSwitch(i, max);
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

Matrix Matrix::operator+(const Matrix &mat) {
    Matrix temp(this->row, this->column);

    for (int i = 0; i < this->row * this->column; i++)
    {
        temp.data[i] = this->data[i] + mat.data[i];
    }

    return temp;
}

Matrix Matrix::operator-(const Matrix &mat) {
    Matrix temp(this->row, this->column);

    for (int i = 0; i < this->row * this->column; i++)
    {
        temp.data[i] = this->data[i] - mat.data[i];
    }

    return temp;
}

// The calculation order of Matrix multiplication will affect the solution speed
// 矩阵乘法计算顺序会影响求解速度
Matrix Matrix::operator*(const Matrix &mat) {
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

Matrix Matrix::operator*(MatDataType_t k) {
    Matrix temp(row, column);

    for(int i = 0; i < row * column; i++)
    {
        temp.data[i] = k * data[i];
    }

    return temp;
}

Matrix &Matrix::operator=(const Matrix &mat) = default;
