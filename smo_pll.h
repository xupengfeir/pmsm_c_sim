#ifndef __SMO_PLL_H
#define __SMO_PLL_H

#include <stdio.h>
#include <math.h>

#define SATURATION_LIMIT  0.2                  // A 安培                      

struct PI_pll{
    double Kp;      // P
    double Ti;      // 积分时间
    double Ki;      //I
    double i_state; // 积分项累加值
    double i_limit; // 输出限制
};


struct SmoPll{
    double ual;     //测量电压 U
    double ube;
    double val;     // 滤波后电压，作为滑膜输入 V
    double vbe;
    double eal;     //smo输出电压 E
    double ebe;

    double ial;
    double ibe;
    double ial_es;  // 估计的当前电流
    double ibe_es;
    double ial_es_next;     // 估计的下一次电流
    double ibe_es_next;

    double A;   // exp(-R/LsTs)
    double B;   // (1-A)/R
    
    double k;   // 滑膜率系数

    double we;  // 角速度
    double theta_e; // 估计的电角度

    struct PI_pll Pipll;       // pll 的 pi控制器

};
extern struct SmoPll smopll;

void SmoPLL_init();



#endif