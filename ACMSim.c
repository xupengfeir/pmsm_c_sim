#include "ACMSim.h"
#include "controller.h"

static void rK4_dynamics(double t, double *x, double *fx);
static void rK4_line(double t, double *x, double hs);
static int isNumber(double x);


struct SychronousMachine ACSM;

/* 同步电机初始化 */
void SMachine_init(){
    for(int i = 0; i < NUM_OF_STATE; i++){
        ACSM.x[i] = 0.0;
    }

	ACSM.rpm = 0.0;
	ACSM.rpm_cmd = 0.0;
	ACSM.rpm_drive_cmd = 0.0;
	ACSM.Tload = 0.0;
	ACSM.Tem = 0.0;

    ACSM.R = 0.45;
	ACSM.Ld = 4.15*1e-3;
	ACSM.Lq = 16.74*1e-3;
	ACSM.rFlux = 0.504;
	ACSM.L0 = 0.5*(ACSM.Ld + ACSM.Lq);
	ACSM.L1 = 0.5*(ACSM.Ld - ACSM.Lq);

	ACSM.Bc = 0.008;
	ACSM.Js = 0.06;
	ACSM.npp = 2;		// 极对数    

	ACSM.Ts = MACHINE_TS;

	ACSM.id = 0.0;
	ACSM.iq = 0.0;

	ACSM.ial = 0.0;
	ACSM.ibe = 0.0;
	
	ACSM.ud = 0.0;
	ACSM.uq = 0.0;

	ACSM.ual = 0.0;
	ACSM.ube = 0.0;
	ACSM.theta_e = 0.0;    
}

/* 电机微分方程数值解算
   x[0]: d轴电流；x[1]: q轴电流；x[2]: we电气转速 
   fx[0]: d轴电流微分；fx[1]: q轴电流微分；fx[2]: 电气转速微分；fx[3]: 电气角度微分
*/
static void rK4_dynamics(double t, double *x, double *fx){
    #if MACHINE_TYPE == SYNCHRONOUS_MACHINE
        //电机电气模型  这里fx是对应的x的一阶导数
        fx[0] = (ACSM.ud - ACSM.R*x[0] + x[2]*ACSM.Lq*x[1]) / ACSM.Ld;
        fx[1] = (ACSM.uq - ACSM.R*x[1] - x[2]*(ACSM.Ld*x[0] + ACSM.rFlux)) / ACSM.Lq;
        // 电机运动模型
        ACSM.Tem = 1.5*ACSM.npp*x[1]*(x[0]*(ACSM.Ld - ACSM.Lq) + ACSM.rFlux);
        fx[2] = (ACSM.Tem - ACSM.Tload - ACSM.Bc*x[2])*ACSM.npp/ACSM.Js;
        fx[3] = x[2];   //  dΘe/dt = we
    #endif
}
static void rK4_line(double t, double *x, double hs){
    #define NS NUM_OF_STATE
        double k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
        double fx[NS];
        int i;

        rK4_dynamics(t, x, fx);
        for(i = 0; i < NS; ++i){
            k1[i] = fx[i] * hs;
            xk[i] = x[i] + k1[i]*0.5;
        }

        rK4_dynamics(t, xk, fx);
        for(i = 0; i < NS; ++i){
            k2[i] = fx[i] * hs;
            xk[i] = x[i] + k2[i]*0.5;
        }

        rK4_dynamics(t, xk, fx);
        for(i = 0; i < NS; ++i){
            k3[i] = fx[i] * hs;
            xk[i] = x[i] + k3[i];
        }

        rK4_dynamics(t, xk, fx);
        for(i = 0; i < NS; ++i){
            k4[i] = fx[i] * hs;
            x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i]) *  ONED6;
        }
    #undef NS
}

/* 电机仿真 */
int machine_simulation(){
    /* 电机微分解算 更新微分方程的4个状态量  id、iq、We，theta_e */
    rK4_line(CTRL.timebase, ACSM.x, ACSM.Ts);
    /* 更新电机完成一次仿真后的状态量 电流 转速*/
    #if MACHINE_TYPE == SYNCHRONOUS_MACHINE
        ACSM.theta_e = ACSM.x[3];
        if (ACSM.theta_e > M_PI){
            ACSM.theta_e -= 2*M_PI;
        }else if (ACSM.theta_e < -M_PI){
            ACSM.theta_e += 2*M_PI;
        }
        ACSM.x[3] = ACSM.theta_e;

        ACSM.id = ACSM.x[0];    // 微分方程解算出来的dq轴电流
        ACSM.iq = ACSM.x[1];
        ACSM.ial = DQ2A(ACSM.id, ACSM.iq, cos(ACSM.theta_e), sin(ACSM.theta_e));
        ACSM.ibe = DQ2B(ACSM.id, ACSM.iq, cos(ACSM.theta_e), sin(ACSM.theta_e));
        ACSM.rpm = ACSM.x[2]*60 / (2*M_PI*ACSM.npp); 
    #endif
    /* 判断转速值是否有效 */
    if(isNumber(ACSM.rpm))
        return false;
    else{
        printf("ACSM.rpm is %g\n", ACSM.rpm);
        return true;
    }
}

static int isNumber(double x){
	return (x == x);
}