//
// Created by 16058 on 2023/5/17.
//

#ifndef MPCCONFIG_H
#define MPCCONFIG_H

#include <Arduino.h>

// 矩阵元素的数据类型，SINGLE_PRECISION为float，DOUBLE_PRECISION为double
// The data type of matrix elements, SINGLE_PRECISION for float, DOUBLE_PRECISION for double
#define SINGLE_PRECISION

// 矩阵元素个数的最大值
// Maximum number of matrix elements
#define MAXSIZE 20

#ifdef SINGLE_PRECISION
typedef float MatDataType_t;
#else
#ifdef DOUBLE_PRECISION
typedef double MatDataType_t;
#endif
#endif

#endif //MPCCONFIG_H
