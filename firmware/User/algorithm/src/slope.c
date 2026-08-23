#include "slope.h"
#include "arm_math.h"
#include "user_lib.h"

////快速开方
//float invSqrt(float num)
//{
//    float halfnum = 0.5f * num;
//    float y = num;
//    long i = *(long *)&y;
//    i = 0x5f3759df - (i >> 1);
//    y = *(float *)&i;
//    y = y * (1.5f - (halfnum * y * y));
//    return y;
//}

void slope_init(Slope_t *slope, float increase_value, float decrease_value, Enum_Slope_First slope_first)
{
    slope->Out = 0.0f;
    slope->Slope_First = slope_first;
    slope->Now_Planning = 0.0f;
    slope->Now_Real = 0.0f;
    slope->Increase_Value = increase_value;
    slope->Decrease_Value = decrease_value;
    slope->Target = 0.0f;
}

void slope_calc(Slope_t *slope)
{
    if (slope->Now_Planning > 0.0f) {
        if (slope->Target > slope->Now_Planning) {
            // 正值加速
            if (fabs(slope->Now_Planning - slope->Target) > slope->Increase_Value) {
                slope->Out += slope->Increase_Value;
            } else {
                slope->Out = slope->Target;
            }
        } else if (slope->Target < slope->Now_Planning) {
            // 正值减速
            if (fabs(slope->Now_Planning - slope->Target) > slope->Decrease_Value) {
                slope->Out -= slope->Decrease_Value;
            } else {
                slope->Out = slope->Target;
            }
        }
    } else if (slope->Now_Planning < 0.0f) {
        if (slope->Target < slope->Now_Planning) {
            // 负值加速
            if (fabs(slope->Now_Planning - slope->Target) > slope->Increase_Value) {
                slope->Out -= slope->Increase_Value;
            } else {
                slope->Out = slope->Target;
            }
        } else if (slope->Target > slope->Now_Planning) {
            // 负值减速
            if (fabs(slope->Now_Planning - slope->Target) > slope->Decrease_Value) {
                slope->Out += slope->Decrease_Value;
            } else {
                slope->Out = slope->Target;
            }
        }
    } else {
        if (slope->Target > slope->Now_Planning) {
            // 0值正加速
            if (fabs(slope->Now_Planning - slope->Target) > slope->Increase_Value) {
                slope->Out += slope->Increase_Value;
            } else {
                slope->Out = slope->Target;
            }
        } else if (slope->Target < slope->Now_Planning) {
            // 0值负加速
            if (fabs(slope->Now_Planning - slope->Target) > slope->Increase_Value) {
                slope->Out -= slope->Increase_Value;
            } else {
                slope->Out = slope->Target;
            }
        }
    }

    // 善后工作
    slope->Now_Planning = slope->Out;
}
