#include "MPC.h"

void setup() {
// write your initialization code here
    Serial.begin(115200);

    MatDataType_t arr1[4] = {1, 2, 3, 4};
    MatDataType_t arr2[4] = {2, 3, 5, 3};

    Matrix mat1 = Matrix(2, 2);
    Matrix mat2 = Matrix(4, 1);
    Matrix mat3, mat4;

    mat1.Print();
    mat2.Print();
    mat3.Print();

    mat2 = mat1.Trans();
    mat2.Print();

    mat2 = mat1.Inv();
    mat3 = mat1 * mat2;
    mat3.Print();

    mat3 = mat1 + mat2;
    mat3.Print();

    mat4 = mat1 * mat2 + mat2 * mat3;
    mat4.Print();
}

void loop() {
// write your code here
}