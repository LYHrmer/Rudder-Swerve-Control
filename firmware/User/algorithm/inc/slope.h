#ifndef SLOPE_H
#define SLOPE_H
#include "main.h"

typedef enum {
    Slope_First_REAL,
    // 其他可能的枚举值
} Enum_Slope_First;

typedef struct {
    float Out;
    Enum_Slope_First Slope_First;
    float Now_Planning;
    float Now_Real;
    float Increase_Value;
    float Decrease_Value;
    float Target;
} Slope_t;

////快速开方
//extern float invSqrt(float num);

//斜波函数初始化
void slope_init(Slope_t *slope, float increase_value, float decrease_value, Enum_Slope_First slope_first) ;
void slope_calc(Slope_t *slope);



#endif
