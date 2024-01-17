#include "ACMSim.h"
#include "measure.h"


struct SmMeasure sm;

void SmMeasure_init(){
    for(int i = 0; i < 2; ++i){
        sm.us[i] = 0;
        sm.is[i] = 0;
        sm.us_curr[i] = 0;
        sm.is_curr[i] = 0;
        sm.us_prev[i] = 0;
        sm.is_prev[i] = 0;        
    }
    sm.Js = ACSM.Js;
    sm.Js_inv = 1.0/sm.Js;

    sm.R = ACSM.R;
    sm.rFlux = ACSM.rFlux;
    sm.Ld = ACSM.Ld;
    sm.Lq = ACSM.Lq;
    sm.npp = ACSM.npp;
    sm.omg = 0.0;
}
/* 无感FOC控制，扩展反电势(eemf) + 滑膜观测器(smo) + 锁相环(PLL) */
void measurement(){
    /*测量量如下：
        电流量作为控制器的反馈量 和 滑膜观测器的输入量；
        电压量作为滑膜观测器的输入量
    */
    IS_C(0) = ACSM.ial;
    IS_C(1) = ACSM.ibe;
    US_C(0) = ACSM.ual;
    US_C(1) = ACSM.ube;

}