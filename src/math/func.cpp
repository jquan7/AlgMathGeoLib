/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : func.cpp
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-21 15:15:15
LastEditors  :
LastEditTime :
Description  : 常用函数
Others       :
Log          :
*******************************************************************************/

#include "func.h"

MATH_BEGIN_NAMESPACE

template<typename T>
T QuickSelect(T* input, int p, int r, int k) {
    // assert: p <= k <= r
    int key[3] = {p, k, r};
    if (!(key[0] <= key[1] && key[1] <= key[2]))
        std::sort(key, key + 3);

    while (key[0] < key[2]) {
        int pp = key[0], rr = key[2];
        int pivot = input[key[2]];
        while (pp < rr) {
            while (input[pp] < pivot)
                pp++;
            while (input[rr] > pivot)
                rr--;
            if (input[pp] == input[rr])
                pp++;
            else if (pp < rr) {
                int tmp = input[pp];
                input[pp] = input[rr];
                input[rr] = tmp;
            }
        }
        int length = rr - key[0] + 1;
        if (length == key[1])
            return input[rr];
        else if (key[1] < length)
            key[2] = rr - 1;
        else {
            key[0] = rr + 1;
            key[1] -= length;
        }
    }
    return input[key[0]];
}

template<typename T>
T Median(T* input, int N) {
    return QuickSelect(input, 0, N - 1, N >> 1);
}


MATH_END_NAMESPACE
