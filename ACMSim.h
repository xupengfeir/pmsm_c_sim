#ifndef __ACMSIM_H
#define __ACMSIM_H

#define _USE_MATH_DEFINES

#include "stdio.h"
#include <process.h>
#include <conio.h>
#include <stdlib.h>
#include "math.h"
#include "time.h"

#define SYNCHRONOUS_MACHINE 2       
#define MACHINE_TYPE SYNCHRONOUS_MACHINE        // 定义电机类型:同步电机


#if MACHINE_TYPE == SYNCHRONOUS_MACHINE
    #define NULL_D_AXIS_CURRENT_CONTROL -1
    #define CONTROL_STRATEGY NULL_D_AXIS_CURRENT_CONTROL
    #define SENSORLESS_CONTROL false
    #define NUM_OF_STATE    2       // 状态量 id,iq
#endif 

#define NUMBER_OF_SIM (200000)      // 仿真次数

#define MACHINE_TS 1.25e-4     // 电机仿真周期
#define MACHINE_TS_INVERSE 8000    
#define DOWN_FREQ_EXE   2      // 降采样  控制器
#define DOWN_FREQ_EXE_INVERSE   0.5

#define TS (MACHINE_TS * DOWN_FREQ_EXE)
#define TS_INVERSE (MACHINE_TS_INVERSE * DOWN_FREQ_EXE_INVERSE)     //4000

/* 变换矩阵 park 反park */
#define AB2D(A, B, COS, SIN)    ( (A) * COS + (B) * SIN )   // 变换后q轴分量
#define AB2Q(A, B, COS, SIN)    ( (A) *-SIN + (B) * COS )   // 变换后q轴分量
#define DQ2A(D, Q, COS, SIN)    ( (D) * COS + (Q) *-SIN )   // 变换后alpha轴分量
#define DQ2B(D, Q, COS, SIN)    ( (D) * SIN + (Q) * COS )   // 变换后beta轴分量

/* 特定常数运算的常值 减少计算时间*/
#define TRUE True
#define FALSE False
#define True (1)
#define False (0)
#define true 1
#define false 0
#define D2PI            0.15915494309189535     // 1/(2*pi)
#define TWO_PI_OVER_3   2.09439510239319549     // (2*pi)/3
#define ONE_PI_OVER_3   1.04719755119659775     // (1*pi)/3
#define SIN_2PID3       0.86602540378443871     // sin(2*pi/3)
#define SIN_N_2PID3    -0.86602540378443871     // sin(-2*pi/3)
#define SQRT_2D3        0.81649658092772603     // sqrt(2/3)

#define ONED6              0.16666666666666667

#define RAD_PER_SEC_2_RPM (60.0/(2*M_PI*ACSM.npp))
#define RPM_2_RAD_PER_SEC ((2*M_PI*ACSM.npp)/60.0)
#define M_PI_OVER_180   0.017453292519943295

#define PHASE_NUMBER    3       //三相电机


struct SychronousMachine{
    double x[NUM_OF_STATE];    // 状态量
    double rpm;     // 转速  转/min
    double rpm_cmd; // 指定的转速
    double rpm_drive_cmd;      

    double Tload;   // 负载转矩
    double Tem;     // 电磁转矩

    double Ld;      // d轴电感
    double Lq;      // q轴电感
    double Ls;
    double L0;      // 静止坐标系下的量
    double L1;      // 同上

    double R;       // 定子电阻
    double rFlux;   // 永磁体（pmsm的转子）磁链

    double Bc;      // 电机阻尼系数
    double Js;      // 转动惯量
    double npp;     // 极对数
     
    double Ts;     // 电机仿真周期

    double id;      // d轴电流
    double iq;      // q轴电流
    double ial;     // alpha轴电流
    double ibe;     // beta轴电流

    double ud;      // d轴电压
    double uq;      // q轴电压
    double ual;
    double ube;
    double omegae;  // 电角速度
    double theta_e; // 电气角度  d轴的alpha轴的角度
};
extern struct SychronousMachine ACSM;       // 命名：交流同步电机


void SMachine_init();
int machine_simulation();


#endif
