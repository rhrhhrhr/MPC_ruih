//
// Created by 16058 on 2023/5/17.
//

#ifndef MPCCONFIG_H
#define MPCCONFIG_H

#include <Arduino.h>

// 矩阵元素个数的最大值
// Maximum number of matrix elements

#ifndef MAXSIZE
#define MAXSIZE 20
#endif

// 矩阵元素的数据类型，SINGLE_PRECISION为float，DOUBLE_PRECISION为double
// The data type of matrix elements, SINGLE_PRECISION for float, DOUBLE_PRECISION for double

#ifndef PRECISION_DOUBLE
typedef float MatDataType_t;
#else
typedef double MatDataType_t;
#endif

// 矩阵乘法的顺序，会影响矩阵计算的速度，在ESP32上测试是jki的顺序最快，和预想不符，具体原因还没有搞清楚，实际使用时可以自己试试哪个快
// Order of matrix multiplication, it will affect the speed of matrix multiplication
// Testing on ESP32 shows that the order of jki is the fastest, which is not as expected.
// The specific reason is not yet clear, so you can try which one is faster when using it yourself

//#define ORDER_OF_MATMUL_IJK
//#define ORDER_OF_MATMUL_IKJ
//#define ORDER_OF_MATMUL_JIK
#define ORDER_OF_MATMUL_JKI
//#define ORDER_OF_MATMUL_KIJ
//#define ORDER_OF_MATMUL_KJI

#endif //MPCCONFIG_H
