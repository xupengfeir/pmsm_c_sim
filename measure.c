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

void measurement(){
    IS_C(0) = ACSM.ial;
    IS_C(1) = ACSM.ibe;
    sm.omg = ACSM.x[2];
    sm.theta_e = ACSM.x[3];
}