/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : func.h
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-21 15:15:15
LastEditors  :
LastEditTime :
Description  : 常用函数
Others       :
Log          :
*******************************************************************************/

#pragma once

#include "../namespaces.h"

MATH_BEGIN_NAMESPACE

#include <string>

#ifndef SET_BIT
#define SET_BIT(x, y) ((x) |= (1 << (y)))
#endif

#ifndef GET_BIT
#define GET_BIT(x, y) ((x) >> (y)&1)
#endif

#ifndef CLEAR_BIT
#define CLEAR_BIT(x, y) ((x) &= ~(1 << (y)))
#endif

#ifndef REVERSE_BIT
#define REVERSE_BIT(x, y) ((x) ^= (1 << (y)))
#endif

/**
 * @brief   数值中1的数量
 * @param   [unsigned int] val - 无符号数
 * @return  int - 比特位为1的数量
 */
static inline int CountBitsSet(unsigned int val)
{
    int bits = 0;
    while (val) {
        val &= val - 1;
        bits++;
    }
    return bits;
}

/**
 * @brief   字符串转数值 (未实现, 参考std::stoul)
 * @param   [const std::string] str - 字符串
 * @return  unsigned int - 数值结果
 */
// static inline unsigned int BinaryToValue(const std::string str);

/**
 * @brief   数值转字符串
 * @param   [T] str - 十进制数值
 * @return  std::string - 字符串 (如：0b00001111)
 */
template <typename T>
static inline std::string DecimalToBinary(T val) {
    static const size_t length = 8 * sizeof(T);
    std::string numb;
    numb.resize(length);
    for (size_t i = 0; i < length; ++i) {
        const auto bit_at_index_i = static_cast<char>((val >> i) & 1);
        numb[length - 1 - i] = bit_at_index_i + '0';
    }
    return "0b" + numb;
}

MATH_END_NAMESPACE
