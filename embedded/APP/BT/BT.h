#ifndef __BT_H
#define __BT_H


#include <stdio.h>
#include <stdint.h>

// 联合体，二者联动自动设置
typedef union {
    float f;
    uint8_t BT_float[4];
} float_to_hex;

// set函数
void set_float_value(float_to_hex *converter, float value);


#endif


